// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "robust_kernel.h"

namespace g2o {

RobustKernel::RobustKernel() : _delta(1.) {}

RobustKernel::RobustKernel(double delta) : _delta(delta) {}

void RobustKernel::setDelta(double delta) { _delta = delta; }

// RobustKernelAdaptive::RobustKernelAdaptive() : _alpha(2.0f),_c(1.0f) {}

// RobustKernelAdaptive::RobustKernelAdaptive(double alpha, double c) : _alpha(alpha),_c(c) {}

// void RobustKernelAdaptive::setAlpha(double alpha) { _alpha = alpha; }

// void RobustKernelAdaptive::setC(double c) { _c = c; }

// void RobustKernelAdaptive::robustify(double e, Eigen::Vector3d& rho) const {

//   double c2 = (_c * _c);
//   double e2 = e/c2;
//   double aux1 = e2/abs(_alpha - 2.0f) + 1.0f;
//   double aux2 = pow(aux1, _alpha/2.0f) - 1.0f;
//   if (fabs(_alpha - 2.0f) <= 1e-5f) {
//         rho[0] = e2 / 2.0f;
//         rho[1] = sqrt(e) / c2; // e = error ** 2
//         rho[2] = 1.0f / c2;
//   }
//   else if (fabs(_alpha ) <= 1e-5f) {
//         double beta = (e + 2 * c2);
//         rho[0] = log(e2/2.0f + 1.0f);
//         rho[1] = (2.0f * sqrt(e))/beta;
//         rho[2] = (4 * c2 - 2 * e)/(beta * beta);
//   }
//   else if (_alpha < -10000.0f) {
//         double beta = exp(-e2/2.0f);
//         rho[0] = 1.0f - beta;
//         rho[1] = sqrt(e)/(c2) * beta;
//         rho[2] = beta * (1/c2 - e2/c2);
//   }
//   else {
//         double beta = 1.0 + e2 / fabs(_alpha - 2);
//         double d = _alpha / 2 - 1;
//         rho[0] = (fabs(_alpha - 2.) / _alpha) * aux2;
//         rho[1] = (sqrt(e) / c2) * pow(beta, d);
//         rho[2] = (pow(beta, d) / c2) * (1 + (2 * d * e2) / (fabs(_alpha - 2) * beta));
//   }
// }

}  // end namespace g2o
