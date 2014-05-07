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
#ifndef MUJIN_VISION_MANAGER_IMPL_H
#define MUJIN_VISION_MANAGER_IMPL_H

#include <mujinvision/visionmanager.h>
#include <mujinvision/mujinzmq.h>
#include <mujinvision/imagesubscribermanager.h>
#include <mujinvision/detectormanager.h>

#include <queue>
#include <boost/filesystem.hpp>

namespace mujinvision {

class MujinVisionManager : public VisionManager
{
public:
    MujinVisionManager(ImageSubscriberManagerPtr imagesubscribermanager, DetectorManagerPtr detectormanager, const std::string& visionmanagerConfigurationFilename);
    virtual ~MujinVisionManager();

    void Destroy();

    /// \brief vision server parameters
    struct VisionServerParameters : public ParametersBase
    {
        VisionServerParameters(const ptree& pt)
        {
            _pt = pt;
            heartbeatPort = pt.get<unsigned int>("heartbeat_port");
            configurationPort = pt.get<unsigned int>("configuration_port");
            statusPort = pt.get<unsigned int>("status_port");
            rpcPort = pt.get<unsigned int>("rpc_port");
            BOOST_FOREACH(const ptree::value_type &v, pt.get_child("imagestream_connections")) {
                ConnectionParametersPtr pimagestreamconnection(new ConnectionParameters(v.second));
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

    class StatusPublisher : public ZmqPublisher {
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

    class CommandServer : public ZmqServer {
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

    ptree Initialize(const std::string& detectorConfigurationFilename, ///< object model, detection parameters
                     const std::string& imagesubscriberConfigurationFilename, ///< subscriber parameters
                     const std::string& controllerIp,
                     const unsigned int controllerPort,
                     const std::string& controllerUsernamePass,
                     const std::string& robotControllerIp,
                     const unsigned int robotControllerPort,
                     const unsigned int binpickingTaskZmqPort,
                     const std::string& binpickingTaskScenePk,
                     const std::string& robotname,
                     const std::string& regionname
                     );

    ptree DetectObjects(const std::string& regionname,
                        const std::vector<std::string>& cameranames,
                        std::vector<DetectedObjectPtr>& detectedobjectsworld);

    ptree StartDetectionLoop(const std::string& regionname,
                             const std::vector<std::string>& cameranames);

    ptree StopDetectionLoop();

    ptree SendPointCloudObstacleToController(const std::string& regionname,
                                             const std::vector<std::string>& cameranames,
                                             const std::vector<DetectedObjectPtr>& detectedobjectsworld,
                                             const double voxelsize=0.01,
                                             const double pointsize=0.005);

    ptree VisualizePointCloudOnController(const std::string& regionname,
                                          const std::vector<std::string>& cameranames);

    ptree ClearVisualizationOnController();

    ptree SaveSnapshot(const std::string& regionname);

    ptree UpdateDetectedObjects(const std::vector<DetectedObjectPtr>& detectobjectsworld, const bool sendtocontroller=false);

    ptree SyncRegion(const std::string& regionname);

    ptree SyncCameras(const std::string& regionname,
                      const std::vector<std::string>& cameranames);

    /** \brief Transforms detected objects.
     */
    static void TransformDetectedObjects(const std::vector<DetectedObjectPtr>& detectedobjectsfrom, std::vector<DetectedObjectPtr>& detectedobjectsto, const Transform& worldtransformfrom, const Transform& worldtransformto);

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

    void _DetectionThread(const std::string& regionname, const std::vector<std::string>& cameranames);
    void _StartDetectionThread(const std::string& regionname, const std::vector<std::string>& cameranames);
    void _StopDetectionThread();


    /** \brief Gets transform of the instobject in meters.
     */
    mujinvision::Transform _GetTransform(const std::string& instobjname);

    /** \brief Updates the world transform of region from the mujin controller.
     */
    void _SyncRegion(const std::string& regionname);

    /** \brief Updates the world transform of camera from the mujin controller.
     */
    void _SyncCamera(const std::string& cameraname);

    /** \brief Gets a color image from image subscriber manager.
     */
    ColorImagePtr _GetColorImage(const std::string& regionname, const std::string& cameraname);

    /** \brief Gets a depth image from image subscriber manager.
     */
    DepthImagePtr _GetDepthImage(const std::string& regionname, const std::string& cameraname);

    /** \brief Converts a vector detectedobjects to "objects": [detectedobject->GetJsonString()]
     */
    std::string _GetJsonString(const std::vector<DetectedObjectPtr>& detectedobjects);

    /** \brief Filters cameranames with region, so that only cameras mapped to the region are returned. If no cameranames is specified, then return all cameras mapped to the region.
     */
    std::vector<std::string> _GetCameraNames(const std::string& regionname, const std::vector<std::string>& cameranames);

    /** \brief Sends detected object list to mujin controller.
     */
    void _SendDetectedObjectsToController(const std::vector<DetectedObjectPtr>& detectedobjectsworld);

    /** \brief Converts mujinclient::Transform to Transform.
     */
    Transform _GetTransform(const mujinclient::Transform& t);

    /** \brief Gets status json string.
     */
    std::string _GetStatusJsonString(const unsigned long long timestamp, const std::string& status, const std::string& message);

    boost::array<std::string,8> _vStatusDescriptions;

    VisionServerParametersPtr _pVisionServerParameters;

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
    std::vector<unsigned long long> _vDetectedTimestamp; ///< _vDetectedTimestamp[i] is the timestamp of part i's last detection.
    std::vector<unsigned int> _vDetectedCount; ///< _vDetectedCount[i] is the number of detections of part i.
    std::vector<Vector> _vDetectedMeanPosition; ///< _vDetectedMeanPosition[i] is the mean position of part i's history of detected positions.
    std::vector<Vector> _vDetectedMeanRotation; ///< _vDetectedMeanRotation[i] is the mean rotation of part i's history of detected rotations.
    std::vector<double> _vDetectedMeanScore; ///< _vDetectedScore[i] is the mean score of part i's detection.
    std::vector<std::vector<Vector> > _vDetectedPositions; ///< _vDetectedPositions[i] is part i's history of detected positions. _vDetectedPositions[i][j] is the j'th history of part i's XYZ position.
    std::vector<std::vector<Vector> > _vDetectedRotations; ///< _vDetectedRotations[i] is part i's history of detected rotations. _vDetectedRotation[i][j] is the j'th history of part i's rotation.
    std::vector<std::vector<double> > _vDetectedScores; ///< _vDetectedScores[i] is part i's history of detection confidence. _vDetectedRotation[i][j] is the j'th history of part i's score.

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
