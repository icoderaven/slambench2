/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "../dataset-tools/include/ETHSLAM.h"

#include <io/SLAMFile.h>
#include <io/SLAMFrame.h>
#include <io/format/PointCloud.h>
#include <io/sensor/AccelerometerSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/PointCloudSensor.h>
#include <Eigen/Eigen>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

#include <fstream>
#include <iostream>

using namespace slambench::io;

/*
 *
 * The dataset folder contains :
 * > imu.txt  depth  depth.txt  groundtruth.txt  rgb  rgb.txt
 *
 */

bool analyseETHSLAMFolder(const std::string &dirname) {
  static const std::vector<std::string> requirements = {
      "imu.txt",
      "rgb.txt",
      "rgb",
      "depth.txt",
      "depth",
      "groundtruth.txt",
      "calibration.txt",
      "dims.txt"};

  try {
    if (!boost::filesystem::exists(dirname)) return false;

    boost::filesystem::directory_iterator end_itr;  // default construction yields past-the-end
    for (auto requirement : requirements) {
      bool seen = false;

      for (boost::filesystem::directory_iterator itr(dirname); itr != end_itr; ++itr) {
        if (requirement == itr->path().filename()) {
          seen = true;
        }
      }

      if (!seen) {
        std::cout << "File not found: <dataset_dir>/" << requirement << std::endl;
        return false;
      }
    }
  } catch (boost::filesystem::filesystem_error &e) {
    std::cerr << "I/O Error with directory " << dirname << std::endl;
    std::cerr << e.what() << std::endl;
    return false;
  }

  return true;
}

bool loadETHSLAMDepthData(const std::string &dirname, SLAMFile &file, const Sensor::pose_t &pose, const DepthSensor::intrinsics_t &intrinsics, const CameraSensor::distortion_coefficients_t &distortion, const DepthSensor::disparity_params_t &disparity_params, const DepthSensor::disparity_type_t &disparity_type, const int width, const int height) {
  DepthSensor *depth_sensor = new DepthSensor("Depth");
  depth_sensor->Index = 0;
  // TODO: Read from images/aux yaml file?
  depth_sensor->Width = width;
  depth_sensor->Height = height;
  depth_sensor->FrameFormat = frameformat::Raster;
  depth_sensor->PixelFormat = pixelformat::D_I_16;
  depth_sensor->DisparityType = disparity_type;
  depth_sensor->Description = "Depth";
  depth_sensor->CopyPose(pose);
  depth_sensor->CopyIntrinsics(intrinsics);
  depth_sensor->CopyDisparityParams(disparity_params);
  depth_sensor->DistortionType = slambench::io::CameraSensor::RadialTangential;
  depth_sensor->CopyRadialTangentialDistortion(distortion);
  depth_sensor->Index = file.Sensors.size();
  depth_sensor->Rate = 27.1;

  file.Sensors.AddSensor(depth_sensor);

  std::string line;

  std::ifstream infile(dirname + "/" + "depth.txt");

  boost::smatch match;

  while (std::getline(infile, line)) {
    if (line.size() == 0) {
      continue;
    } else if (boost::regex_match(line, match, boost::regex("^\\s*#.*$"))) {
      continue;
    } else if (boost::regex_match(line, match, boost::regex("^([0-9]+)[.]([0-9]+)\\s+(.*)$"))) {
      int timestampS = std::stoi(match[1]);
      int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());
      std::string depthfilename = match[3];

      ImageFileFrame *depth_frame = new ImageFileFrame();
      depth_frame->FrameSensor = depth_sensor;
      depth_frame->Timestamp.S = timestampS;
      depth_frame->Timestamp.Ns = timestampNS;

      std::stringstream frame_name;
      frame_name << dirname << "/" << depthfilename;
      depth_frame->Filename = frame_name.str();

      if (access(depth_frame->Filename.c_str(), F_OK) < 0) {
        printf("No depth image for frame (%s)\n", frame_name.str().c_str());
        perror("");
        return false;
      }

      file.AddFrame(depth_frame);

    } else {
      std::cerr << "Unknown line:" << line << std::endl;
      return false;
    }
  }
  return true;
}

bool loadETHSLAMRGBData(const std::string &dirname, SLAMFile &file, const Sensor::pose_t &pose, const CameraSensor::intrinsics_t &intrinsics, const CameraSensor::distortion_coefficients_t &distortion, const int width, const int height) {
  CameraSensor *rgb_sensor = new CameraSensor("RGB", CameraSensor::kCameraType);
  rgb_sensor->Index = 0;
  rgb_sensor->Width = width;
  rgb_sensor->Height = height;
  rgb_sensor->FrameFormat = frameformat::Raster;
  rgb_sensor->PixelFormat = pixelformat::RGB_III_888;
  rgb_sensor->Description = "RGB";
  rgb_sensor->CopyPose(pose);
  rgb_sensor->CopyIntrinsics(intrinsics);
  rgb_sensor->DistortionType = slambench::io::CameraSensor::RadialTangential;
  rgb_sensor->CopyRadialTangentialDistortion(distortion);
  rgb_sensor->Index = file.Sensors.size();
  rgb_sensor->Rate = 27.1;

  file.Sensors.AddSensor(rgb_sensor);

  std::string line;

  std::ifstream infile(dirname + "/" + "rgb.txt");

  boost::smatch match;

  while (std::getline(infile, line)) {
    if (line.size() == 0) {
      continue;
    } else if (boost::regex_match(line, match, boost::regex("^\\s*#.*$"))) {
      continue;
    } else if (boost::regex_match(line, match, boost::regex("^([0-9]+)[.]([0-9]+)\\s+(.*)$"))) {
      int timestampS = std::stoi(match[1]);
      int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());
      std::string rgbfilename = match[3];

      ImageFileFrame *rgb_frame = new ImageFileFrame();
      rgb_frame->FrameSensor = rgb_sensor;
      rgb_frame->Timestamp.S = timestampS;
      rgb_frame->Timestamp.Ns = timestampNS;

      std::stringstream frame_name;
      frame_name << dirname << "/" << rgbfilename;
      rgb_frame->Filename = frame_name.str();

      if (access(rgb_frame->Filename.c_str(), F_OK) < 0) {
        printf("No RGB image for frame (%s)\n", frame_name.str().c_str());
        perror("");
        return false;
      }

      file.AddFrame(rgb_frame);

    } else {
      std::cerr << "Unknown line:" << line << std::endl;
      return false;
    }
  }
  return true;
}

bool loadETHSLAMGreyData(const std::string &dirname, SLAMFile &file, const Sensor::pose_t &pose, const CameraSensor::intrinsics_t &intrinsics, const CameraSensor::distortion_coefficients_t &distortion, const int width, const int height) {
  CameraSensor *grey_sensor = new CameraSensor("Grey", CameraSensor::kCameraType);
  grey_sensor->Index = 0;
  grey_sensor->Width = width;
  grey_sensor->Height = height;
  grey_sensor->FrameFormat = frameformat::Raster;
  grey_sensor->PixelFormat = pixelformat::G_I_8;
  grey_sensor->Description = "Grey";

  grey_sensor->CopyPose(pose);
  grey_sensor->CopyIntrinsics(intrinsics);
  grey_sensor->CopyRadialTangentialDistortion(distortion);
  grey_sensor->DistortionType = slambench::io::CameraSensor::RadialTangential;
  grey_sensor->Index = file.Sensors.size();
  grey_sensor->Rate = 27.1;

  file.Sensors.AddSensor(grey_sensor);

  std::string line;

  std::ifstream infile(dirname + "/" + "rgb.txt");

  boost::smatch match;

  while (std::getline(infile, line)) {
    if (line.size() == 0) {
      continue;
    } else if (boost::regex_match(line, match, boost::regex("^\\s*#.*$"))) {
      continue;
    } else if (boost::regex_match(line, match, boost::regex("^([0-9]+)[.]([0-9]+)\\s+(.*)$"))) {
      int timestampS = std::stoi(match[1]);
      int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());
      std::string rgbfilename = match[3];

      ImageFileFrame *grey_frame = new ImageFileFrame();
      grey_frame->FrameSensor = grey_sensor;
      grey_frame->Timestamp.S = timestampS;
      grey_frame->Timestamp.Ns = timestampNS;

      std::stringstream frame_name;
      frame_name << dirname << "/" << rgbfilename;
      grey_frame->Filename = frame_name.str();

      if (access(grey_frame->Filename.c_str(), F_OK) < 0) {
        printf("No RGB image for frame (%s)\n", frame_name.str().c_str());
        perror("");
        return false;
      }

      file.AddFrame(grey_frame);

    } else {
      std::cerr << "Unknown line:" << line << std::endl;
      return false;
    }
  }
  return true;
}

bool loadETHSLAMGroundTruthData(const std::string &dirname, SLAMFile &file) {
  GroundTruthSensor *gt_sensor = new GroundTruthSensor("GroundTruth");
  gt_sensor->Index = file.Sensors.size();
  gt_sensor->Description = "GroundTruthSensor";
  file.Sensors.AddSensor(gt_sensor);

  if (!gt_sensor) {
    std::cout << "gt sensor not found..." << std::endl;
    return false;
  } else {
    std::cout << "gt sensor created..." << std::endl;
  }

  std::string line;

  boost::smatch match;
  std::ifstream infile(dirname + "/" + "groundtruth.txt");

  while (std::getline(infile, line)) {
    if (line.size() == 0) {
      continue;
    } else if (boost::regex_match(line, match, boost::regex("^\\s*#.*$"))) {
      continue;
    } else if (boost::regex_match(line, match, boost::regex("^([0-9]+)[.]([0-9]+)\\s+([-0-9.]+)\\s+([-0-9.]+)\\s+([-0-9.]+)\\s+([-0-9.e]+)\\s+([-0-9.e]+)\\s+([-0-9.e]+)\\s+([-0-9.e]+)$"))) {
      int timestampS = std::stoi(match[1]);
      int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());

      float tx = std::stof(match[3]);
      float ty = std::stof(match[4]);
      float tz = std::stof(match[5]);

      float QX = std::stof(match[6]);
      float QY = std::stof(match[7]);
      float QZ = std::stof(match[8]);
      float QW = std::stof(match[9]);

      Eigen::Matrix3f rotationMat = Eigen::Quaternionf(QW, QX, QY, QZ).toRotationMatrix();
      Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
      pose.block(0, 0, 3, 3) = rotationMat;

      pose.block(0, 3, 3, 1) << tx, ty, tz;

      SLAMInMemoryFrame *gt_frame = new SLAMInMemoryFrame();
      gt_frame->FrameSensor = gt_sensor;
      gt_frame->Timestamp.S = timestampS;
      gt_frame->Timestamp.Ns = timestampNS;
      gt_frame->Data = malloc(gt_frame->GetSize());

      memcpy(gt_frame->Data, pose.data(), gt_frame->GetSize());

      file.AddFrame(gt_frame);

    } else {
      std::cerr << "Unknown line:" << line << std::endl;
      return false;
    }
  }
  return true;
}

bool loadETHSLAMAccelerometerData(const std::string &dirname, SLAMFile &file) {
  AccelerometerSensor *accelerometer_sensor = new AccelerometerSensor("Accelerometer");
  accelerometer_sensor->Index = file.Sensors.size();
  accelerometer_sensor->Description = "AccelerometerSensor";
  file.Sensors.AddSensor(accelerometer_sensor);

  if (!accelerometer_sensor) {
    std::cout << "accelerometer_sensor not found..." << std::endl;
    return false;
  } else {
    std::cout << "accelerometer_sensor created..." << std::endl;
  }

  std::string line;

  boost::smatch match;
  std::ifstream infile(dirname + "/" + "imu.txt");

  while (std::getline(infile, line)) {
    if (line.size() == 0) {
      continue;
    } else if (boost::regex_match(line, match, boost::regex("^\\s*#.*$"))) {
      continue;
    } else if (boost::regex_match(line, match, boost::regex("^([0-9]+)[.]([0-9]+)\\s+([-0-9.]+)\\s+([-0-9.]+)\\s+([-0-9.]+)$"))) {
      int timestampS = std::stoi(match[1]);
      int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());
      float ax = std::stof(match[3]);
      float ay = std::stof(match[4]);
      float az = std::stof(match[5]);

      SLAMInMemoryFrame *accelerometer_frame = new SLAMInMemoryFrame();
      accelerometer_frame->FrameSensor = accelerometer_sensor;
      accelerometer_frame->Timestamp.S = timestampS;
      accelerometer_frame->Timestamp.Ns = timestampNS;
      accelerometer_frame->Data = malloc(accelerometer_frame->GetSize());
      ((float *)accelerometer_frame->Data)[0] = ax;
      ((float *)accelerometer_frame->Data)[1] = ay;
      ((float *)accelerometer_frame->Data)[2] = az;

      file.AddFrame(accelerometer_frame);

    } else {
      std::cerr << "Unknown line:" << line << std::endl;
      return false;
    }
  }
  return true;
}

SLAMFile *ETHSLAMReader::GenerateSLAMFile() {
  if (!(grey || rgb || depth)) {
    std::cerr << "No sensors defined\n";
    return nullptr;
  }

  std::string dirname = input;

  if (!analyseETHSLAMFolder(dirname)) {
    std::cerr << "Invalid folder." << std::endl;
    return nullptr;
  }

  SLAMFile *slamfilep = new SLAMFile();
  SLAMFile &slamfile = *slamfilep;

  Sensor::pose_t pose = Eigen::Matrix4f::Identity();

  CameraSensor::intrinsics_t intrinsics_rgb;
  CameraSensor::distortion_coefficients_t distortion_rgb;

  for (int i = 0; i < 5; ++i) {
    distortion_rgb[i] = 0.0f;
  }

  std::string line;
  boost::smatch match;
  std::ifstream dims_file(dirname + "/" + "dims.txt");

  int width = 0, height = 0;
  while (std::getline(dims_file, line)) {
    if (line.size() == 0) {
      continue;
    } else if (boost::regex_match(line, match, boost::regex("^\\s*#.*$"))) {
      continue;
    } else if (boost::regex_match(line, match, boost::regex("([0-9]+)\\s+([0-9]+)$"))) {
      width = std::stoi(match[1]);
      height = std::stoi(match[2]);
    } else {
      std::cerr << "Unknown line:" << line << std::endl;
    }
  }
  dims_file.close();

  std::ifstream calib_file(dirname + "/" + "calibration.txt");

  while (std::getline(calib_file, line)) {
    if (line.size() == 0) {
      continue;
    } else if (boost::regex_match(line, match, boost::regex("^\\s*#.*$"))) {
      continue;
    } else if (boost::regex_match(line, match, boost::regex("([0-9.]+)\\s+([0-9.]+)\\s+([0-9.]+)\\s+([0-9.]+)$"))) {
      // Just because this is nuts, this library divides intrinsics by width & height
      intrinsics_rgb[0] = std::stof(match[1]) / width;
      intrinsics_rgb[1] = std::stof(match[2]) / height;
      intrinsics_rgb[2] = std::stof(match[3]) / width;
      intrinsics_rgb[3] = std::stof(match[4]) / height;

    } else {
      std::cerr << "Unknown line:" << line << std::endl;
    }
  }
  calib_file.close();

  DepthSensor::disparity_params_t disparity_params = {0.001, 0.0};
  DepthSensor::disparity_type_t disparity_type = DepthSensor::affine_disparity;

  /**
	 * load Depth
	 */

  if (depth && !loadETHSLAMDepthData(dirname, slamfile, pose, intrinsics_rgb, distortion_rgb, disparity_params, disparity_type, width, height)) {
    std::cout << "Error while loading depth information." << std::endl;
    delete slamfilep;
    return nullptr;
  }

  /**
	 * load Grey
	 */

  if (grey && !loadETHSLAMGreyData(dirname, slamfile, pose, intrinsics_rgb, distortion_rgb, width, height)) {
    std::cout << "Error while loading Grey information." << std::endl;
    delete slamfilep;
    return nullptr;
  }

  /**
	 * load RGB
	 */

  if (rgb && !loadETHSLAMRGBData(dirname, slamfile, pose, intrinsics_rgb, distortion_rgb, width, height)) {
    std::cout << "Error while loading RGB information." << std::endl;
    delete slamfilep;
    return nullptr;
  }

  /**
	 * load GT
	 */
  if (gt && !loadETHSLAMGroundTruthData(dirname, slamfile)) {
    std::cout << "Error while loading gt information." << std::endl;
    delete slamfilep;
    return nullptr;
  }

  /**
	 * load Accelerometer: This one failed
	 */
  if (accelerometer && !loadETHSLAMAccelerometerData(dirname, slamfile)) {
    std::cout << "Error while loading Accelerometer information." << std::endl;
    delete slamfilep;
    return nullptr;
  }

  return slamfilep;
}
