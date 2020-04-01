/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_SCENE3D_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_SCENE3D_H_

#include <ParameterComponent.h>
#include <ParameterManager.h>
#include <Parameters.h>

#include <io/sensor/CameraSensor.h>
#include <io/sensor/DepthSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/Sensor.h>
#include "../../dataset-tools/include/DatasetReader.h"

namespace slambench {

namespace io {

class SCENE3DReader : public DatasetReader {
 
 public:
  std::string input;
  bool grey = true, rgb = true, depth = true, gt = true;

  SCENE3DReader(std::string name) : DatasetReader(name) {
    this->addParameter(TypedParameter<std::string>("i", "input-directory", "path of the SCENE3D dataset directory", &this->input, NULL));
    this->addParameter(TypedParameter<bool>("grey", "grey", "set to true or false to specify if the GREY stream need to be include in the slam file.", &this->grey, NULL));
    this->addParameter(TypedParameter<bool>("rgb", "rgb", "set to true or false to specify if the RGB stream need to be include in the slam file.", &this->rgb, NULL));
    this->addParameter(TypedParameter<bool>("depth", "depth", "set to true or false to specify if the DEPTH stream need to be include in the slam file.", &this->depth, NULL));
    this->addParameter(TypedParameter<bool>("gt", "gt", "set to true or false to specify if the GROUNDTRUTH POSE stream need to be include in the slam file.", &this->gt, NULL));
  }

  SLAMFile* GenerateSLAMFile();
};

}
}

#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_SCENE3D_H_ */
