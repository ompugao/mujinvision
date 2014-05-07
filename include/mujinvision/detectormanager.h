// -*- coding: utf-8 -*-
// Copyright (C) 2012-2014 MUJIN Inc. <rosen.diankov@mujin.co.jp>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/** \file detectormanager.h
    \brief Public headers of DetectorManager.
 */
#ifndef MUJIN_VISION_DETECTOR_MANAGER_H
#define MUJIN_VISION_DETECTOR_MANAGER_H

#include "detector.h"

namespace mujinvision {

class DetectorManager
{
public:
    DetectorManager() {
    }

    virtual ~DetectorManager(){
    }

    /** Creates object detector.
        \param objectparams_pt boost property tree describing object parameters
        \param detectionparams_pt boost property tree describing detection parameters
        \param region where objects are to be detected
        \param mColorCamera map to color cameras from names
        \param mDepthCamera map to depth cameras from names
        \param setstatusfn function the detector can use to set status
     */
    virtual ObjectDetectorPtr CreateObjectDetector(const ptree& objectparams_pt, const ptree& detectionparams_pt, RegionConstPtr region, std::map<std::string, CameraPtr> mColorCamera, std::map<std::string, CameraPtr> mDepthCamera, const boost::function<void(const std::string& msg)>& setstatusfn) = 0;
};

typedef boost::shared_ptr<DetectorManager> DetectorManagerPtr;
typedef boost::shared_ptr<DetectorManager const> DetectorManagerConstPtr;
typedef boost::weak_ptr<DetectorManager> DetectorManagerWeakPtr;

} // namespace mujinvision
#endif
