//
// Created by vuong on 2/9/22.
//
/**
 * This file is part of ORB-SLAM3
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

#include <System.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <opencv4/opencv2/core/types.hpp>

using namespace std;

/**
 *   \brief Load Images data function.
 *
 *   Function to load image data saved in ASML format
 *
 *   \param strPathFolder Path to the folder datasets.
 *   \param vTimeStamps Vector of timestamp.
 *   \param vstrImageLeft Vector of left image path.
 *   \param vstrImageRight Vector of right image path.
 *   \return Success?
 *
 **/
bool LoadImages(const string& strPathFolder, vector<string>& vstrImageLeft,
                vector<string>& vstrImageLeftSeg, vector<string>& vstrImageRight,
                vector<double>& vTimeStamps);

int main(int argc, char** argv) {
  if (argc < 4) {
    cerr << endl
         << "Usage: ./stereo_inertial_zed2 path_to_vocabulary "
            "path_to_settings path_to_sequence_folder_1 "
            "(path_to_image_folder_2 ... "
            "path_to_image_folder_N) "
         << endl;
    return EXIT_FAILURE;
  }

//  const int num_seq = ((argc - 3) / 2);
  const int num_seq = 1;
  cout << "num_seq = " << num_seq << endl;
  bool bFileName = false;
  string file_name;
  if (bFileName) {
    file_name = string(argv[argc - 1]);
    cout << "file name: " << file_name << endl;
  }

  // Load all sequences:
  int seq;
  vector<vector<string>> vstrImageLeft;
  vector<vector<string>> vstrImageRight;
  vector<vector<string>> vstrImageLeftSeg;
  vector<vector<double>> vTimestampsCam;

  vector<int> vnImages;
  vector<int> first_image(num_seq, 0);

  vstrImageLeft.resize(num_seq);
  vstrImageLeftSeg.resize(num_seq);
  vstrImageRight.resize(num_seq);
  vTimestampsCam.resize(num_seq);
  vnImages.resize(num_seq);

  int tot_images = 0;

  for (int i = 0; i < num_seq; ++i) {
    cout << "Loading data for sequence " << seq << "...\n";

    string pathSeq(argv[(2 * seq) + 6]);

    cout << "Loading Images for sequence " << seq << "...\n";
    bool bLoadImages = LoadImages(pathSeq, vstrImageLeft[seq], vstrImageLeftSeg[seq],
                                  vstrImageRight[seq], vTimestampsCam[seq]);

    if (!bLoadImages) {
      std::cerr << "ERROR: Fail to load data " << seq << std::endl;
      return EXIT_FAILURE;
    }

    if (vstrImageLeft[seq].size() != vstrImageRight[seq].size()) {
      cerr << "ERROR: Left and Right image is not equal " << seq << endl;
      std::cout << "Left: " << vstrImageLeft[seq].size()
                << " vs Right: " << vstrImageRight[seq].size() << std::endl;
      return EXIT_FAILURE;
    }

    tot_images += vnImages[seq];
    vnImages[seq] = (int)vstrImageLeft[seq].size();

    // Find the first image where have Odom and IMU
    // while (vTimestampsCam[seq][first_image[seq]] <=
    //            vTimestampsImu[seq][first_imu[seq]] ||
    //        vTimestampsCam[seq][first_image[seq]] <=
    //            vTimestampsOdom[seq][first_odom[seq]])
    //   first_image[seq]++;

    // Find first imu && odom to be considered, supposing imu measurements start
    // first

    // while (vTimestampsImu[seq][first_imu[seq]] <=
    //        vTimestampsCam[seq][first_image[seq]])
    //   first_imu[seq]++;
    // first_imu[seq]--;  // first imu measurement to be considered

    // while (vTimestampsOdom[seq][first_odom[seq]] <=
    //        vTimestampsCam[seq][first_image[seq]])
    //   first_odom[seq]++;
    // first_odom[seq]--;  // first odom measurement to be considered
  }

  // Read rectification parameters
  cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    cerr << "ERROR: Wrong path to settings" << endl;
    return -1;
  }

  // Vector for tracking time statistics
  vector<double> vTimesTrack;
  vTimesTrack.resize(tot_images);

  cout << endl << "-------" << endl;
  cout.precision(17);
  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], argv[3], argv[4], argv[5], ORB_SLAM3::System::STEREO, false);

  cv::Mat imLeft, imLeftSeg, imRight;
  for (seq = 0; seq < num_seq; seq++) {
    double t_rect = 0.f;
    double t_resize = 0.f;
    double t_track = 0.f;
    // Seq loop
    int proccIm = 0;
    for (int ni = first_image[seq]; ni < vnImages[seq]; ni++, proccIm++) {
      // Read left and right images from file
      imLeft = cv::imread(vstrImageLeft[seq][ni], cv::IMREAD_UNCHANGED);
      imLeftSeg = cv::imread(vstrImageLeftSeg[seq][ni], cv::IMREAD_UNCHANGED);
      imRight = cv::imread(vstrImageRight[seq][ni], cv::IMREAD_UNCHANGED);
      // cv::imshow("test", imLeftSeg);
      // cv::waitKey(0);

      if (imLeft.empty()) {
        cerr << endl
             << "Failed to load image at: " << string(vstrImageLeft[seq][ni])
             << endl;
        return EXIT_FAILURE;
      }

      if (imLeftSeg.empty()) {
        cerr << endl
             << "Failed to load segmented image at: " << string(vstrImageLeftSeg[seq][ni])
             << endl;
        return EXIT_FAILURE;
      }

      if (imRight.empty()) {
        cerr << endl
             << "Failed to load image at: " << string(vstrImageRight[seq][ni])
             << endl;
        return EXIT_FAILURE;
      }

      double tframe = vTimestampsCam[seq][ni];

#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t1 =
          std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t1 =
          std::chrono::monotonic_clock::now();
#endif
      // Pass the images to the SLAM system
      SLAM.TrackStereoSegment(imLeft, imLeftSeg, imRight, tframe);

#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t2 =
          std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t2 =
          std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
      t_track =
          t_rect + t_resize +
          std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
              t2 - t1)
              .count();
      SLAM.InsertTrackTime(t_track);
#endif

      double ttrack =
          std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
              .count();

      //       Wait to load the next frame
      double T = 0;
      if (ni < vnImages[seq] - 1)
        T = vTimestampsCam[seq][ni + 1] - tframe;
      else if (ni > 0)
        T = tframe - vTimestampsCam[seq][ni - 1];

      if (ttrack < T) usleep((unsigned int)((T - ttrack) * 1e6));  // 1e6
    }
    std::cout << "Finish seq " << std::endl;
    if (seq < num_seq - 1) {
      cout << "Changing the dataset" << endl;

      SLAM.ChangeDataset();
    }
    SLAM.SaveTrajectoryTUM("/home/wfram/ORB_SLAM3_VIWO/OpenLorisFrameTraj.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("/home/wfram/ORB_SLAM3_VIWO/OpenLorisKeyFrameTraj.txt");
  }
  std::cout << "Trying stop thread " << std::endl;
  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  if (bFileName) {
    std::cout << "Saved trajectory " << std::endl;
    const string kf_file = "kf_" + string(argv[argc - 1]) + ".txt";
    const string f_file = "f_" + string(argv[argc - 1]) + ".txt";
    SLAM.SaveTrajectoryEuRoC(f_file);
    SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
  } else {
    std::cout << "Saved trajectory " << std::endl;
    SLAM.SaveTrajectoryUZH("CameraTrajectory.txt");
    SLAM.SaveTrajectoryUZH("KeyFrameTrajectory.txt");
  }

  return EXIT_SUCCESS;
}

bool LoadImages(const string& strPathFolder, vector<string>& vstrImageLeft,
                vector<string>& vstrImageLeftSeg, vector<string>& vstrImageRight,
                vector<double>& vTimeStamps) {
  ifstream fTimes;
  string strPathTimesLeft = strPathFolder + "/fisheye1.txt";
  string strPathTimesLeftSeg = strPathFolder + "/fisheye1seg.txt";
  string strPathTimesRight = strPathFolder + "/fisheye2.txt";
  fTimes.open(strPathTimesLeft.c_str());

  if (!fTimes.good()) {
    std::cerr << "Left timestamp path doesn't exist\n";
    return false;
  }
  vTimeStamps.reserve(5000);
  vstrImageLeft.reserve(5000);
  vstrImageLeftSeg.reserve(5000);
  vstrImageRight.reserve(5000);
  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);
    if (s[0] == '#') continue;
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      double t;
      ss >> t;
      vTimeStamps.push_back(t);
      string strLeft;
      ss >> strLeft;
      vstrImageLeft.push_back(strPathFolder + "/" + strLeft);
    }
  }
  fTimes.close();

  fTimes.open(strPathTimesLeftSeg.c_str());

  if (!fTimes.good()) {
    std::cerr << "Left segmented timestamp path doesn't exist\n";
    return false;
  }

  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);
    if (s[0] == '#') continue;
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      double t;
      ss >> t;
      vTimeStamps.push_back(t);
      string strLeftSeg;
      ss >> strLeftSeg;
      vstrImageLeftSeg.push_back(strPathFolder + "/" + strLeftSeg);
    }
  }
  fTimes.close();

  fTimes.open(strPathTimesRight.c_str());

  if (!fTimes.good()) {
    std::cerr << "Right timestamp path doesn't exist\n";
    return false;
  }

  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);
    if (s[0] == '#') continue;
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      double t;
      ss >> t;
      vTimeStamps.push_back(t);
      string strRight;
      ss >> strRight;
      vstrImageRight.push_back(strPathFolder + "/" + strRight);
    }
  }
  return true;
}
