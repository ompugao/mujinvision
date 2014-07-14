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
/** \file visionmanagerimpl.h
    \brief Public headers of MujinVisionManager.
 */
#ifndef MUJIN_VISION_MANAGER_H
#define MUJIN_VISION_MANAGER_H

#include <mujincontrollerclient/binpickingtask.h>
#include <mujinvision/visionparameters.h>
#include <mujinvision/mujinzmq.h>
#include <mujinvision/imagesubscribermanager.h>
#include <mujinvision/detectormanager.h>

#include <queue>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

namespace mujinvision {

using namespace mujinclient;

class MUJINVISION_API MujinVisionManager
{
public:
    /** \brief sets up vision manager
        - loads config file
     */    
    MujinVisionManager(ImageSubscriberManagerPtr imagesubscribermanager, DetectorManagerPtr detectormanager, const std::string& visionmanagerConfigurationFilename);
    virtual ~MujinVisionManager();

    virtual void Destroy();

    /// \brief vision server parameters
    struct MUJINVISION_API VisionServerParameters : public ParametersBase
    {
        VisionServerParameters(const ptree& pt)
        {
            _pt = pt;
            heartbeatPort = pt.get<unsigned int>("heartbeat_port");
            configurationPort = pt.get<unsigned int>("configuration_port");
            statusPort = pt.get<unsigned int>("status_port");
            rpcPort = pt.get<unsigned int>("rpc_port");
            FOREACH(v, pt.get_child("imagestream_connections")) {
                ConnectionParametersPtr pimagestreamconnection(new ConnectionParameters(v->second));
                streamerConnections.push_back(pimagestreamconnection);
            }
            useLocalArv = pt.get<bool>("use_local_arv");
            maxPositionError = pt.get<double>("max_position_error");
            clearRadius = pt.get<double>("clear_radius");
            timeToRemember = pt.get<unsigned int>("time_to_remember");
            timeToIgnore = pt.get<unsigned int>("time_to_ignore");
            numDetectionsToKeep = pt.get<unsigned int>("num_detections_to_keep");
        }

        virtual ~VisionServerParameters() {
        }

        unsigned int heartbeatPort;
        unsigned int configurationPort;
        unsigned int statusPort;
        unsigned int rpcPort;
        std::vector<ConnectionParametersPtr > streamerConnections;
        bool useLocalArv;
        double maxPositionError; ///< in meter, max position error to consider detections the same
        double clearRadius; ///< in meter, clear detection results within the radius of the last picked locations
        unsigned int timeToRemember; ///< in millisecond, time to keep detection result before forgetting it
        unsigned int timeToIgnore; ///< in millisecond, time to ignore detection result after picking in the region
        unsigned int numDetectionsToKeep; ///< number of detection history to keep

        std::string GetJsonString()
        {
            std::stringstream ss;
            ss << "{";
            ss << "\"heartbeat_port\": " << heartbeatPort << ",";
            ss << "\"configuration_port\": " << configurationPort << ",";
            ss << "\"status_port\": " << statusPort << ",";
            ss << "\"rpc_port\": " << rpcPort << ",";
            ss << "\"imagestream_connections\": [";
            for (unsigned int i=0; i<streamerConnections.size(); i++) {
                ss << streamerConnections[i]->GetJsonString();
                if (i<streamerConnections.size()-1) {
                    ss << ",";
                }
            }
            ss << "],";
            ss << "\"use_local_arv\": ";
            if (useLocalArv) {
                ss << "true";
            } else {
                ss << "false";
            }
            ss << ",";
            ss << ParametersBase::GetJsonString("max_position_error") << ": " << maxPositionError << ",";
            ss << ParametersBase::GetJsonString("clear_radius") << ": " << clearRadius << ",";
            ss << ParametersBase::GetJsonString("time_to_remember") << ": " << timeToRemember << ",";
            ss << ParametersBase::GetJsonString("time_to_ignore") << ": " << timeToIgnore << ",";
            ss << ParametersBase::GetJsonString("num_detections_to_keep") << ": " << numDetectionsToKeep;
            ss << "}";
            return ss.str();
        }

        ptree GetPropertyTree()
        {
            if (_pt.empty()) {
                _pt.put<unsigned int>("heartbeat_port", heartbeatPort);
                _pt.put<unsigned int>("configuration_port", configurationPort);
                _pt.put<unsigned int>("status_port", statusPort);
                _pt.put<unsigned int>("rpc_port", rpcPort);
                ptree streamerConnections_pt;
                for (unsigned int i=0; i<streamerConnections.size(); i++) {
                    streamerConnections_pt.push_back(std::make_pair("", streamerConnections[i]->GetPropertyTree()));
                }
                _pt.put_child("imagestream_connections", streamerConnections_pt);
                _pt.put<bool>("use_local_arv", useLocalArv);
            }
            return _pt;
        }
    };
    typedef boost::shared_ptr<VisionServerParameters> VisionServerParametersPtr;
    typedef boost::shared_ptr<VisionServerParameters const> VisionServerParametersConstPtr;
    typedef boost::weak_ptr<VisionServerParameters> VisionServerParametersWeakPtr;

    class MUJINVISION_API StatusPublisher : public ZmqPublisher {
public:
        StatusPublisher(const unsigned int port) : ZmqPublisher(port)
        {
            _InitializeSocket();
        }

        virtual ~StatusPublisher()
        {
            _DestroySocket();
        }
    };
    typedef boost::shared_ptr<StatusPublisher> StatusPublisherPtr;
    typedef boost::shared_ptr<StatusPublisher const> StatusPublisherConstPtr;
    typedef boost::weak_ptr<StatusPublisher> StatusPublisherWeakPtr;

    class MUJINVISION_API CommandServer : public ZmqServer {
public:
        CommandServer(const unsigned int port) : ZmqServer(port)
        {
            _InitializeSocket();
        }

        virtual ~CommandServer()
        {
            _DestroySocket();
        }
    };
    typedef boost::shared_ptr<CommandServer> CommandServerPtr;
    typedef boost::shared_ptr<CommandServer const> CommandServerConstPtr;
    typedef boost::weak_ptr<CommandServer> CommandServerWeakPtr;

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
                             );

    /** \brief Detects objects in specified region with specified cameras
        \param regionname name of the region
        \param cameranames names of the cameras
        \param detectedobjects detection results in meters in world frame
     */
    virtual ptree DetectObjects(const std::string& regionname,
                                const std::vector<std::string>& cameranames,
                                std::vector<DetectedObjectPtr>& detectedobjectsworld);

    /** \brief starts detection thread to continuously detect objects and sends detection results to mujin controller
     */
    virtual ptree StartDetectionLoop(const std::string& regionname,
                                     const std::vector<std::string>& cameranames,
                                     const double voxelsize=0.01,
                                     const double pointsize=0.005);

    virtual ptree StopDetectionLoop();

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
                                                     const double pointsize=0.005);

    /** \brief Visualizes the raw camera point clouds on mujin controller
     */
    virtual ptree VisualizePointCloudOnController(const std::string& regionname,
                                                  const std::vector<std::string>& cameranames,
                                                  const double pointsize=0.005);

    /** \brief Clears visualization made by VisualizePointCloudOnController on mujin controller.
     */
    virtual ptree ClearVisualizationOnController();

    /** \brief Saves a snapshot for each sensor mapped to the region. If detection was called before, snapshots of the images used for the last detection will be saved. Images are saved to the visionmanager application directory.
     */
    virtual ptree SaveSnapshot(const std::string& regionname, const bool getlatest=true);

    /** \brief Updates the locally maintained list of the detected objects
        \param detectedobjectsworld detection result in world frame
        \param sendtocontroller whether to send the list to mujin controller
     */
    virtual ptree UpdateDetectedObjects(const std::vector<DetectedObjectPtr>& detectobjectsworld, const bool sendtocontroller=false);

    /** \brief Updates the region info from the mujin controller
        - updates position of the region
        - updates globalroi3d of the region
     */
    virtual ptree SyncRegion(const std::string& regionname);

    /** \brief Updates info about the cameras associated with the region from the mujin controller. If no cameraname is provided, then update all cameras mapped to the region.
        - updates positions of the cameras
     */
    virtual ptree SyncCameras(const std::string& regionname,
                              const std::vector<std::string>& cameranames);

    bool bShutdown;

private:

    enum ManagerStatus {
        MS_Lost=0,
        MS_Pending=1,
        MS_Active=2,
        MS_Preempting=3,
        MS_Preempted=4,
        MS_Succeeded=5,
        MS_Paused=6,
        MS_Aborted=7,
    };

    struct DetectedInfo {
        unsigned long long timestamp; ///< timestamp of part's last detection.
        unsigned int count; ///< count is the number of detections of part.
        Vector meanPosition; ///< meanPosition is the mean position of part's history of detected positions.
        Vector meanRotation; ///< meanRotation is the mean rotation of part's history of detected rotations.
        double meanScore; ///< meanScore is the mean score of part's detection.
        std::vector<Vector> positions; ///< positions is part's history of detected positions. positions[i] is the i'th history of part's XYZ position.
        std::vector<Vector> rotations; ///< rotations is part's history of detected rotations. rotation[i] is the i'th history of part's rotation.
        std::vector<double> scores; ///< scores is part's history of detection confidence. rotation[i] is the i'th history of part's score.
    };

    void _DeInitialize();
    
    void _SetStatusMessage(const std::string& msg);
    void _SetStatus(ManagerStatus status, const std::string& msg="", const bool disableInterrupt=false);

    /** \brief Executes command in json string. Returns result in json string.
     */
    void _ExecuteConfigurationCommand(const ptree& command_pt, std::stringstream& result_ss);
    void _ExecuteUserCommand(const ptree& command_pt, std::stringstream& result_ss);

    /** \brief Gets result ptree from status.
     */
    ptree _GetResultPtree(ManagerStatus status);

    /** \brief Receives and executes commands from the user and sends results back.
     */
    void _CommandThread(const unsigned int port);
    void _StartCommandServer(const unsigned int port);
    void _StopCommandServer(const unsigned int port);
    void _StartCommandThread(const unsigned int port);
    void _StopCommandThread(const unsigned int port);

    /** \brief Publishes status periodically. When there are more status messages on the status queue, then publish all of them at once, otherwise, publish the last status message.
     */
    void _StatusThread(const unsigned int port, const unsigned int ms);
    void _StartStatusThread(const unsigned int port, const unsigned int ms=100);
    void _StopStatusThread();
    void _StartStatusPublisher(const unsigned int port);
    void _PublishStopStatus();

    void _DetectionThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double voxelsize, const double pointsize);
    void _StartDetectionThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double voxelsize, const double pointsize);
    void _StopDetectionThread();


    /** \brief Gets transform of the instobject in meters.
     */
    mujinvision::Transform _GetTransform(const std::string& instobjname);

    /** \brief Updates the world transform of region from the mujin controller.
     */
    void _SyncRegion(const std::string& regionname);

    /** \brief Updates the world transform of camera from the mujin controller.
     */
    void _SyncCamera(const std::string& regionname, const std::string& cameraname);

    /** \brief Gets a color image (uncropped) from image subscriber manager.
     */
    ColorImagePtr _GetColorImage(const std::string& regionname, const std::string& cameraname);

    /** \brief Gets a depth image (uncropped) from image subscriber manager.
     */
    DepthImagePtr _GetDepthImage(const std::string& regionname, const std::string& cameraname);

    /** \brief Converts a vector detectedobjects to "objects": [detectedobject->GetJsonString()]
     */
    std::string _GetJsonString(const std::vector<DetectedObjectPtr>& detectedobjects);

    /** \brief Filters cameranames with region, so that only cameras mapped to the region are returned. If no cameranames is specified, then return all cameras mapped to the region.
     */
    std::vector<std::string> _GetCameraNames(const std::string& regionname, const std::vector<std::string>& cameranames);

    /** \brief This function wraps _GetCameraNames so that it returns only color cameras.
     */
    std::vector<std::string> _GetColorCameraNames(const std::string& regionname, const std::vector<std::string>& cameranames);

    /** \brief This function wraps _GetCameraNames so that it returns only depth cameras.
     */
    std::vector<std::string> _GetDepthCameraNames(const std::string& regionname, const std::vector<std::string>& cameranames);

    /** \brief Sends detected object list to mujin controller.
     */
    void _SendDetectedObjectsToController(const std::vector<DetectedObjectPtr>& detectedobjectsworld);

    /** \brief Converts mujinclient::Transform to Transform.
     */
    Transform _GetTransform(const mujinclient::Transform& t);

    /** \brief Get string representation of transform.
     */
    std::string _GetString(const Transform& transform);

    /** \brief Gets status json string.
     */
    std::string _GetStatusJsonString(const unsigned long long timestamp, const std::string& status, const std::string& message);

    boost::array<std::string,8> _vStatusDescriptions;

    bool _initialized;

    ControllerClientPtr _pControllerClient;
    SceneResourcePtr _pSceneResource;
    VisionServerParametersPtr _pVisionServerParameters;
    BinPickingTaskResourcePtr _pBinpickingTask;

    std::queue<ManagerStatus> _statusQueue;
    std::queue<std::string> _messageQueue;
    std::queue<unsigned long long> _timestampQueue;
    boost::mutex _mutexStatusQueue; ///< protects _statusQueue, _messageQueue, and _timestampQueue
    //boost::condition _condStatus; ///< notification when _statusqueue has data

    std::map<unsigned int, boost::shared_ptr<boost::thread> > _mPortCommandThread; ///< port -> thread
    boost::shared_ptr<boost::thread> _pStatusThread;
    boost::shared_ptr<boost::thread> _pDetectionThread;

    boost::mutex _mutexCancelCommand;
    std::map<unsigned int, CommandServerPtr> _mPortCommandServer; ///< port -> server
    boost::mutex _mutexCommandServerMap;
    StatusPublisherPtr _pStatusPublisher;

    std::map<std::string, RegionPtr > _mNameRegion; ///< name->region
    std::map<std::string, CameraParametersPtr> _mNameCameraParameters; ///< name->camera param
    std::map<std::string, CameraPtr > _mNameCamera; ///< name->camera
    std::map<std::string, CameraPtr > _mNameColorCamera; ///< name->camera
    std::map<std::string, CameraPtr > _mNameDepthCamera; ///< name->camera

    std::vector<ImageSubscriberPtr> _vSubscribers;
    unsigned int _numDepthImagesToAverage;
    ImageSubscriberManagerPtr _pImagesubscriberManager;

    ObjectDetectorPtr _pDetector;
    boost::mutex _mutexDetector;
    DetectorManagerPtr _pDetectorManager;

    std::set<unsigned long long> _sTimestamp; ///< set of saved timestamp in millisecond
    std::vector<DetectedInfo> _vDetectedInfo;
    
    bool _bStopStatusThread;
    bool _bStopDetectionThread;
    bool _bCancelCommand;
    bool _bExecutingUserCommand; ///< true if currently executing a user command
    std::map<unsigned int, bool > _mPortStopCommandThread; ///< port -> bool
};
typedef boost::shared_ptr<MujinVisionManager> MujinVisionManagerPtr;
typedef boost::shared_ptr<MujinVisionManager const> MujinVisionManagerConstPtr;
typedef boost::weak_ptr<MujinVisionManager> MujinVisionManagerWeakPtr;
} // namespace mujinvision
#endif
