/**
 * This file is modified part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#include "geometry/g2o_types.h"

#include  "utils/converter.h"
namespace RVWO {

bool VertexPose::read(std::istream& is) {
  std::vector<Eigen::Matrix<double, 3, 3> > Rcw;
  std::vector<Eigen::Matrix<double, 3, 1> > tcw;
  std::vector<Eigen::Matrix<double, 3, 3> > Rbc;
  std::vector<Eigen::Matrix<double, 3, 1> > tbc;

  const int num_cams = _estimate.Rbc.size();
  for (int idx = 0; idx < num_cams; idx++) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) is >> Rcw[idx](i, j);
    }
    for (int i = 0; i < 3; i++) {
      is >> tcw[idx](i);
    }

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) is >> Rbc[idx](i, j);
    }
    for (int i = 0; i < 3; i++) {
      is >> tbc[idx](i);
    }

    float nextParam;
    for (size_t i = 0; i < _estimate.pCamera[idx]->size(); i++) {
      is >> nextParam;
      _estimate.pCamera[idx]->setParameter(nextParam, i);
    }
  }

  double bf;
  is >> bf;
  _estimate.SetParam(Rcw, tcw, Rbc, tbc, bf);
  updateCache();

  return true;
}

bool VertexPose::write(std::ostream& os) const {
  std::vector<Eigen::Matrix<double, 3, 3> > Rcw = _estimate.Rcw;
  std::vector<Eigen::Matrix<double, 3, 1> > tcw = _estimate.tcw;

  std::vector<Eigen::Matrix<double, 3, 3> > Rbc = _estimate.Rbc;
  std::vector<Eigen::Matrix<double, 3, 1> > tbc = _estimate.tbc;

  const size_t num_cams = tcw.size();

  for (int idx = 0; idx < num_cams; idx++) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) os << Rcw[idx](i, j) << " ";
    }
    for (int i = 0; i < 3; i++) {
      os << tcw[idx](i) << " ";
    }

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) os << Rbc[idx](i, j) << " ";
    }
    for (int i = 0; i < 3; i++) {
      os << tbc[idx](i) << " ";
    }

    for (size_t i = 0; i < _estimate.pCamera[idx]->size(); i++) {
      os << _estimate.pCamera[idx]->getParameter(i) << " ";
    }
  }

  os << _estimate.bf << " ";

  return os.good();
}

void EdgeMono::linearizeOplus() {
  const auto* VPose = static_cast<const VertexPose*>(_vertices[1]);
  const g2o::VertexSBAPointXYZ* VPoint =
      static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

  const Eigen::Matrix3d& Rcw = VPose->estimate().Rcw[cam_idx];
  const Eigen::Vector3d& tcw = VPose->estimate().tcw[cam_idx];
  const Eigen::Vector3d Xc = Rcw * VPoint->estimate() + tcw;
  const Eigen::Vector3d Xb =
      VPose->estimate().Rbc[cam_idx] * Xc + VPose->estimate().tbc[cam_idx];
  const Eigen::Matrix3d& Rcb = VPose->estimate().Rcb[cam_idx];

  const Eigen::Matrix<double, 2, 3> proj_jac =
      VPose->estimate().pCamera[cam_idx]->projectJac(Xc);
  _jacobianOplusXi = -proj_jac * Rcw;

  Eigen::Matrix<double, 3, 6> SE3deriv;
  double x = Xb(0);
  double y = Xb(1);
  double z = Xb(2);
  // clang-format off
  SE3deriv << 0.0, z, -y, 1.0, 0.0, 0.0,
              -z, 0.0, x, 0.0, 1.0, 0.0,
              y, -x, 0.0, 0.0, 0.0, 1.0;
  // clang-format on
  _jacobianOplusXj = proj_jac * Rcb * SE3deriv;  // TODO optimize this product
}

void EdgeMonoOnlyPose::linearizeOplus() {
  const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);

  const Eigen::Matrix3d& Rcw = VPose->estimate().Rcw[cam_idx];
  const Eigen::Vector3d& tcw = VPose->estimate().tcw[cam_idx];
  const Eigen::Vector3d Xc = Rcw * Xw + tcw;
  const Eigen::Vector3d Xb =
      VPose->estimate().Rbc[cam_idx] * Xc + VPose->estimate().tbc[cam_idx];
  const Eigen::Matrix3d& Rcb = VPose->estimate().Rcb[cam_idx];

  Eigen::Matrix<double, 2, 3> proj_jac =
      VPose->estimate().pCamera[cam_idx]->projectJac(Xc);

  Eigen::Matrix<double, 3, 6> SE3deriv;
  double x = Xb(0);
  double y = Xb(1);
  double z = Xb(2);
  // clang-format off
  SE3deriv << 0.0, z, -y, 1.0, 0.0, 0.0,
              -z, 0.0, x, 0.0, 1.0, 0.0,
              y, -x, 0.0, 0.0, 0.0, 1.0;
  // clang-format on
  _jacobianOplusXi =
      proj_jac * Rcb * SE3deriv;  // symbol different becasue of update mode
}

void EdgeStereo::linearizeOplus() {
  const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
  const g2o::VertexSBAPointXYZ* VPoint =
      static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

  const Eigen::Matrix3d& Rcw = VPose->estimate().Rcw[cam_idx];
  const Eigen::Vector3d& tcw = VPose->estimate().tcw[cam_idx];
  const Eigen::Vector3d Xc = Rcw * VPoint->estimate() + tcw;
  const Eigen::Vector3d Xb =
      VPose->estimate().Rbc[cam_idx] * Xc + VPose->estimate().tbc[cam_idx];
  const Eigen::Matrix3d& Rcb = VPose->estimate().Rcb[cam_idx];
  const double bf = VPose->estimate().bf;
  const double inv_z2 = 1.0 / (Xc(2) * Xc(2));

  Eigen::Matrix<double, 3, 3> proj_jac;
  proj_jac.block<2, 3>(0, 0) =
      VPose->estimate().pCamera[cam_idx]->projectJac(Xc);
  proj_jac.block<1, 3>(2, 0) = proj_jac.block<1, 3>(0, 0);
  proj_jac(2, 2) += bf * inv_z2;

  _jacobianOplusXi = -proj_jac * Rcw;

  Eigen::Matrix<double, 3, 6> SE3deriv;
  double x = Xb(0);
  double y = Xb(1);
  double z = Xb(2);
  // clang-format off
  SE3deriv << 0.0, z, -y, 1.0, 0.0, 0.0,
              -z, 0.0, x, 0.0, 1.0, 0.0,
              y, -x, 0.0, 0.0, 0.0, 1.0;
  // clang-format on
  _jacobianOplusXj = proj_jac * Rcb * SE3deriv;
}

void EdgeStereoOnlyPose::linearizeOplus() {
  const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);

  const Eigen::Matrix3d& Rcw = VPose->estimate().Rcw[cam_idx];
  const Eigen::Vector3d& tcw = VPose->estimate().tcw[cam_idx];
  const Eigen::Vector3d Xc = Rcw * Xw + tcw;
  const Eigen::Vector3d Xb =
      VPose->estimate().Rbc[cam_idx] * Xc + VPose->estimate().tbc[cam_idx];
  const Eigen::Matrix3d& Rcb = VPose->estimate().Rcb[cam_idx];
  const double bf = VPose->estimate().bf;
  const double inv_z2 = 1.0 / (Xc(2) * Xc(2));

  Eigen::Matrix<double, 3, 3> proj_jac;
  proj_jac.block<2, 3>(0, 0) =
      VPose->estimate().pCamera[cam_idx]->projectJac(Xc);
  proj_jac.block<1, 3>(2, 0) = proj_jac.block<1, 3>(0, 0);
  proj_jac(2, 2) += bf * inv_z2;

  Eigen::Matrix<double, 3, 6> SE3deriv;
  double x = Xb(0);
  double y = Xb(1);
  double z = Xb(2);
  // clang-format off
  SE3deriv << 0.0, z, -y, 1.0, 0.0, 0.0,
              -z, 0.0, x, 0.0, 1.0, 0.0,
              y, -x, 0.0, 0.0, 0.0, 1.0;
  // clang-format on
  _jacobianOplusXi = proj_jac * Rcb * SE3deriv;
}

VertexVelocity::VertexVelocity(KeyFrame* pKF) {
  setEstimate(pKF->GetVelocity().cast<double>());
}

VertexVelocity::VertexVelocity(Frame* pF) {
  setEstimate(pF->GetVelocity().cast<double>());
}

}  // namespace RVWO
