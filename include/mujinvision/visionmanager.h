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
/** \file visionmanager.h
    \brief Public headers of Vision Manager
 */
#ifndef MUJIN_VISION_MANAGER_H
#define MUJIN_VISION_MANAGER_H

#include <mujincontrollerclient/binpickingtask.h>
#include <mujinvision/visionparameters.h>
#include <boost/thread.hpp>

namespace mujinvision {

using namespace mujinclient;

/// \brief manages the bin-picking object detection process
class MUJINVISION_API VisionManager
{
public:

    /** \brief sets up vision manager
        - loads config file
     */
    VisionManager() {
    }

    virtual ~VisionManager() {
    }

    /** \brief preparation for the detection process
        - load object.conf
        - subscribe to image streams
        - connect to the mujin controller
        - initialize detection
     */
    virtual ptree Initialize(const std::string& detectorConfigurationFilename, ///< object model, detection parameters
                             const std::string& imagesubscriberConfigurationFilename, ///< subscriber parameters
                             const std::string& controllerIp,
                             const unsigned int controllerPort,
                             const std::string& controllerUsernamePass,
                             const std::string& robotControllerIp,
                             const unsigned int robotControllerPort,
                             const unsigned int binpickingTaskZmqPort,
                             const unsigned int binpickingTaskHeartbeatPort,
                             const double binpickingTaskHeartbeatTimeout,
                             const std::string& binpickingTaskScenePk,
                             const std::string& robotname,
                             const std::string& regionname
                             ) = 0;

    /** \brief Detects objects in specified region with specified cameras
        \param regionname name of the region
        \param cameranames names of the cameras
        \param detectedobjects detection results in meters in world frame
     */
    virtual ptree DetectObjects(const std::string& regionname,
                                const std::vector<std::string>& cameranames,
                                std::vector<DetectedObjectPtr>& detectedobjectsworld) = 0;

    /** \brief starts detection thread to continuously detect objects and sends detection results to mujin controller
     */
    virtual ptree StartDetectionLoop(const std::string& regionname,
                                     const std::vector<std::string>& cameranames) = 0;

    virtual ptree StopDetectionLoop() = 0;

    /** \brief Updates the point cloud obstacle and sends it to mujin controller
        \param regionname name of the region where the detection happens
        \param cameranames names of the cameras used for detection
        \param detectedobjects detection result in meters in  world frame
        \param voxelsize size of the voxel grid in meters used for simplifying the cloud
        \param pointsize size of the point in meters to be sent to the mujin controller
     */
    virtual ptree SendPointCloudObstacleToController(const std::string& regionname,
                                                     const std::vector<std::string>& cameranames,
                                                     const std::vector<DetectedObjectPtr>& detectedobjectsworld,
                                                     const double voxelsize=0.01,
                                                     const double pointsize=0.005) = 0;

    /** \brief Visualizes the raw camera point clouds on mujin controller
     */
    virtual ptree VisualizePointCloudOnController(const std::string& regionname,
                                                  const std::vector<std::string>& cameranames) = 0;

    /** \brief Clears visualization made by VisualizePointCloudOnController on mujin controller.
     */
    virtual ptree ClearVisualizationOnController()=0;

    /** \brief saves a snapshot for each sensor mapped to the region
     */
    virtual ptree SaveSnapshot(const std::string& regionname) = 0;

    /** \brief Updates the locally maintained list of the detected objects
        \param detectedobjectsworld detection result in world frame
        \param sendtocontroller whether to send the list to mujin controller
     */
    virtual ptree UpdateDetectedObjects(const std::vector<DetectedObjectPtr>& detectobjectsworld, const bool sendtocontroller=false) = 0;

    /** \brief Updates the region info from the mujin controller
        - updates position of the region
        - updates globalroi3d of the region
     */
    virtual ptree SyncRegion(const std::string& regionname) = 0;

    /** \brief Updates info about the cameras associated with the region from the mujin controller. If no cameraname is provided, then update all cameras mapped to the region.
        - updates positions of the cameras
     */
    virtual ptree SyncCameras(const std::string& regionname,
                              const std::vector<std::string>& cameranames) = 0;

protected:

    BinPickingTaskResourcePtr _pBinpickingTask;

    boost::shared_ptr<boost::thread> _pDetectionThread;
    bool _bStopDetectionLoop;

};

} // namespace mujinvision
#endif
