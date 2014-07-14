// -*- coding: utf-8 -*-
// Copyright (C) 2012-2013 MUJIN Inc. <rosen.diankov@mujin.co.jp>
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
#include <mujinvision/mujinvisionmanager.h>

namespace mujinvision {

class CallFunctionOnDestruction
{
public:
    CallFunctionOnDestruction(const boost::function<void()>& fn) : _fn(fn) {
    }
    ~CallFunctionOnDestruction() {
        _fn();
    }
protected:
    boost::function<void()> _fn;
};

MujinVisionManager::MujinVisionManager(ImageSubscriberManagerPtr imagesubscribermanager, DetectorManagerPtr detectormanager, const std::string& visionmanagerConfigFilename)
{
    _initialized = false;
    _pImagesubscriberManager = imagesubscribermanager;
    _pDetectorManager = detectormanager;
    _vStatusDescriptions[MS_Lost] = "lost";
    _vStatusDescriptions[MS_Pending] = "pending";
    _vStatusDescriptions[MS_Active] = "active";
    _vStatusDescriptions[MS_Preempting] = "preempting";
    _vStatusDescriptions[MS_Preempted] = "preempted";
    _vStatusDescriptions[MS_Succeeded] = "succeeded";
    _vStatusDescriptions[MS_Paused] = "paused";
    _vStatusDescriptions[MS_Aborted] = "aborted";
    bShutdown = false;
    ptree pt;
    // load visionserver configuration
    if (!boost::filesystem::exists(visionmanagerConfigFilename)) {
        throw MujinVisionException(visionmanagerConfigFilename+" does not exist!", MVE_ConfigurationFileError);
    }
    read_json(visionmanagerConfigFilename, pt);
    _pVisionServerParameters.reset(new VisionServerParameters(pt.get_child("visionserver")));
    // set up regions
    std::vector<RegionParametersPtr > vRegionParameters;
    RegionParametersPtr pRegionParameters;
    FOREACH(v, pt.get_child("regions")) {
        RegionParametersPtr pregionparameters(new RegionParameters(v->second));
        vRegionParameters.push_back(pregionparameters);
        _mNameRegion[pregionparameters->instobjectname] = RegionPtr(new Region(pregionparameters));
    }
    // set up camera parameters
    FOREACH(v, pt.get_child("cameras")) {
        _mNameCameraParameters[v->first].reset(new CameraParameters(v->second));
    }
    _StartStatusThread(_pVisionServerParameters->statusPort);
    _StartCommandThread(_pVisionServerParameters->rpcPort);
    _StartCommandThread(_pVisionServerParameters->configurationPort);
}

MujinVisionManager::~MujinVisionManager()
{
    Destroy();
}

void MujinVisionManager::Destroy()
{
    std::cout << "Destroying MujinVisionManager" << std::endl;
    _StopStatusThread();
    _StopCommandThread(_pVisionServerParameters->rpcPort);
    _StopCommandThread(_pVisionServerParameters->configurationPort);
}

void MujinVisionManager::_SetStatus(ManagerStatus status, const std::string& msg, const bool disableInterrupt)
{
    if (status == MS_Preempted) {
        {
            boost::mutex::scoped_lock lock(_mutexCancelCommand);
            _bCancelCommand = false;
        }
    }
    if( _bCancelCommand && !disableInterrupt ) {
        throw UserInterruptException("Cancelling command.");
    }
    std::cout << GetMilliTime() << " " << _vStatusDescriptions.at(status) << ": " << msg << std::endl;
    boost::mutex::scoped_lock lock(_mutexStatusQueue);
    _statusQueue.push(status);
    _messageQueue.push(msg);
    _timestampQueue.push(GetMilliTime());
}

void MujinVisionManager::_SetStatusMessage(const std::string& msg)
{
    if (_statusQueue.size()>0) {
        _SetStatus(_statusQueue.front(), msg, false);
    } else {
        throw MujinVisionException("VisionManager is in invalid state.", MVE_Failed);
    }
}

void MujinVisionManager::_StartStatusPublisher(const unsigned int port)
{
    _pStatusPublisher.reset(new StatusPublisher(port));
    if (!_pStatusPublisher) {
        throw MujinVisionException("Failed to start status publisher!", MVE_Failed);
    }
    _SetStatus(MS_Pending);
}

void MujinVisionManager::_PublishStopStatus()
{
    StatusPublisherPtr pStatusPublisher = _pStatusPublisher;
    if( !!pStatusPublisher ) {
        pStatusPublisher->Publish(_GetStatusJsonString(GetMilliTime(), _vStatusDescriptions.at(MS_Lost), ""));
        std::cout << "Stopped status publisher" << std::endl;
    }
}

void MujinVisionManager::_StartStatusThread(const unsigned int port, const unsigned int ms)
{
    _bStopStatusThread = false;
    _pStatusThread.reset(new boost::thread(boost::bind(&MujinVisionManager::_StatusThread, this, port, ms)));
}

void MujinVisionManager::_StopStatusThread()
{
    if (!!_pStatusThread) {
        {
            boost::mutex::scoped_lock lock(_mutexStatusQueue);
            _bStopStatusThread = true;
        }
        _pStatusThread->join();
        _pStatusThread.reset();
        std::cout << "Stopped status thread" << std::endl;
    }
}

void MujinVisionManager::_StartCommandThread(const unsigned int port)
{
    _mPortStopCommandThread[port] = false;
    _mPortCommandThread[port].reset(new boost::thread(boost::bind(&MujinVisionManager::_CommandThread, this, port)));
}

void MujinVisionManager::_StopCommandThread(const unsigned int port)
{
    if (!_mPortStopCommandThread[port]) {
        _mPortStopCommandThread[port] = true;
        _mPortCommandThread[port]->join();
        std::cout << "Stopped command thread (port: " << port << ")." << std::endl;
    }
}

void MujinVisionManager::_StartCommandServer(const unsigned int port)
{
    {
        boost::mutex::scoped_lock lock(_mutexCommandServerMap);
        _mPortCommandServer[port].reset(new CommandServer(port));
    }
    if (!_mPortCommandServer[port]) {
        std::stringstream ss;
        ss << "Failed to start command server at port " << port << "!";
        throw MujinVisionException(ss.str(), MVE_Failed);
    }
}

void MujinVisionManager::_StopCommandServer(const unsigned int port)
{
    std::cout << "Stopped command server (port: " << port << ")." << std::endl;
}

void MujinVisionManager::_ExecuteConfigurationCommand(const ptree& command_pt, std::stringstream& result_ss)
{
    result_ss << "{";
    std::string command = command_pt.get<std::string>("command");
    if (command == "Cancel") {
        boost::mutex::scoped_lock lock(_mutexCancelCommand);
        if (_bExecutingUserCommand) { // only cancel when user command is being executed
            _bCancelCommand = true;
            _SetStatus(MS_Preempting, "", true);
        } else {
            _SetStatusMessage("No command is being excuted, do nothing.");
        }
        result_ss << ParametersBase::GetJsonString("status", _vStatusDescriptions.at(MS_Preempting));
    } else if (command == "Quit") {
        // throw exception, shutdown gracefully
        bShutdown=true;
        _StopStatusThread();
        _StopCommandThread(_pVisionServerParameters->rpcPort);
        throw UserInterruptException("User requested exit.");
    }
    result_ss << "}";
}

void MujinVisionManager::_ExecuteUserCommand(const ptree& command_pt, std::stringstream& result_ss)
{
    result_ss << "{";
    ptree result_pt;
    _SetStatus(MS_Active);
    {
        // only one command thread is running, so _bExecutingUserCommand must be false at this point, and _bCancelCommand must not be true, therefore no race condition of setting _bCancelCommand from true to false
        boost::mutex::scoped_lock lock(_mutexCancelCommand);
        _bCancelCommand = false;
        _bExecutingUserCommand = true;
    }
    std::string command = command_pt.get<std::string>("command");
    if (command == "Initialize") {
        if (_initialized) {
            _SetStatusMessage("Vision manager was initialized, de-initialize it first.");
            _DeInitialize();
        } else {
            _initialized = true;
        }
        result_pt = Initialize(command_pt.get<std::string>("detectorConfigurationFilename"),
                               command_pt.get<std::string>("imagesubscriberConfigurationFilename"),
                               command_pt.get<std::string>("mujinControllerIp"),
                               command_pt.get<unsigned int>("mujinControllerPort"),
                               command_pt.get<std::string>("mujinControllerUsernamePass"),
                               command_pt.get<std::string>("robotControllerIp"),
                               command_pt.get<unsigned int>("robotControllerPort"),
                               command_pt.get<unsigned int>("binpickingTaskZmqPort"),
                               command_pt.get<unsigned int>("binpickingTaskHeartbeatPort"),
                               command_pt.get<double>("binpickingTaskHeartbeatTimeout"),
                               command_pt.get<std::string>("binpickingTaskScenePk"),
                               command_pt.get<std::string>("robotname"),
                               command_pt.get<std::string>("regionname")
                               );
        result_ss << ParametersBase::GetJsonString("status",result_pt.get<std::string>("status"));
    } else if (command == "DetectObjects") {
        std::vector<std::string> cameranames;
        boost::optional<const ptree&> cameranames_pt(command_pt.get_child_optional("cameranames"));
        if (!!cameranames_pt) {
            FOREACH(v, *cameranames_pt) {
                cameranames.push_back(v->second.get<std::string>(""));
            }
        }
        std::vector<DetectedObjectPtr> detectedobjects;
        result_pt = DetectObjects(command_pt.get<std::string>("regionname"), cameranames, detectedobjects);
        result_ss << ParametersBase::GetJsonString("status",result_pt.get<std::string>("status"));
        result_ss << ", ";
        result_ss << _GetJsonString(detectedobjects);
    } else if (command == "StartDetectionLoop") {
        std::vector<std::string> cameranames;
        boost::optional<const ptree&> cameranames_pt(command_pt.get_child_optional("cameranames"));
        if (!!cameranames_pt) {
            FOREACH(v, *cameranames_pt) {
                cameranames.push_back(v->second.get<std::string>(""));
            }
        }
        double voxelsize = command_pt.get("voxelsize",0.01);
        double pointsize = command_pt.get("pointsize",0.005);
        result_pt = StartDetectionLoop(command_pt.get<std::string>("regionname"),
                                       cameranames,
                                       voxelsize,
                                       pointsize);
        result_ss << ParametersBase::GetJsonString("status",result_pt.get<std::string>("status"));
    } else if (command == "StopDetectionLoop") {
        result_pt = StopDetectionLoop();
        result_ss << ParametersBase::GetJsonString("status",result_pt.get<std::string>("status"));
    } else if (command == "SendPointCloudObstacleToController") {
        std::vector<std::string> cameranames;
        boost::optional<const ptree&> cameranames_pt(command_pt.get_child_optional("cameranames"));
        if (!!cameranames_pt) {
            FOREACH(v, *cameranames_pt) {
                cameranames.push_back(v->second.get<std::string>(""));
            }
        }
        std::vector<DetectedObjectPtr> detectedobjects;
        boost::optional<const ptree&> detectedobjects_pt(command_pt.get_child_optional("detectedobjects"));
        if (!!detectedobjects_pt) {
            FOREACH(v, *detectedobjects_pt) {
                detectedobjects.push_back(DetectedObjectPtr(new DetectedObject(v->second.get_child(""))));
            }
        }
        double voxelsize = command_pt.get("voxelsize",0.01);
        double pointsize = command_pt.get("pointsize",0.005);
        result_pt = SendPointCloudObstacleToController(command_pt.get<std::string>("regionname"),
                                                       cameranames,
                                                       detectedobjects,
                                                       voxelsize,
                                                       pointsize);
        result_ss << ParametersBase::GetJsonString("status",result_pt.get<std::string>("status"));
    } else if (command == "VisualizePointCloudOnController") {
        std::vector<std::string> cameranames;
        boost::optional<const ptree&> cameranames_pt(command_pt.get_child_optional("cameranames"));
        if (!!cameranames_pt) {
            FOREACH(v, *cameranames_pt) {
                cameranames.push_back(v->second.get<std::string>(""));
            }
        }
        double pointsize = command_pt.get("pointsize",0.005);
        result_pt = VisualizePointCloudOnController(command_pt.get<std::string>("regionname"),
                                                    cameranames,
                                                    pointsize);
        result_ss << ParametersBase::GetJsonString("status",result_pt.get<std::string>("status"));
    } else if (command == "ClearVisualizationOnController") {
        result_pt = ClearVisualizationOnController();
        result_ss << ParametersBase::GetJsonString("status",result_pt.get<std::string>("status"));
    } else if (command == "SaveSnapshot") {
        bool getlatest = command_pt.get("getlatest",true);
        result_pt = SaveSnapshot(command_pt.get<std::string>("regionname"), getlatest);
        result_ss << ParametersBase::GetJsonString("status",result_pt.get<std::string>("status"));
    } else if (command == "UpdateDetectedObjects") {
        std::vector<DetectedObjectPtr> detectedobjects;
        boost::optional<const ptree&> detectedobjects_pt(command_pt.get_child_optional("detectedobjects"));
        if (!!detectedobjects_pt) {
            FOREACH(v, *detectedobjects_pt) {
                detectedobjects.push_back(DetectedObjectPtr(new DetectedObject(v->second.get_child(""))));
            }
        }
        result_pt = UpdateDetectedObjects(detectedobjects,
                                          command_pt.get<bool>("sendtocontroller"));
        result_ss << ParametersBase::GetJsonString("status",result_pt.get<std::string>("status"));
    } else if (command == "SyncRegion") {
        result_pt = SyncRegion(command_pt.get<std::string>("regionname"));
        result_ss << ParametersBase::GetJsonString("status",result_pt.get<std::string>("status"));
    } else if (command == "SyncCameras") {
        std::vector<std::string> cameranames;
        boost::optional<const ptree&> cameranames_pt(command_pt.get_child_optional("cameranames"));
        if (!!cameranames_pt) {
            FOREACH(v, *cameranames_pt) {
                cameranames.push_back(v->second.get<std::string>(""));
            }
        }
        result_pt = SyncCameras(command_pt.get<std::string>("regionname"),
                                cameranames);
        result_ss << ParametersBase::GetJsonString("status",result_pt.get<std::string>("status"));
    } else {
        std::stringstream ss;
        ss << "Received unknown command " << command << ".";
        throw MujinVisionException(ss.str(), MVE_CommandNotSupported);
    }
    _SetStatus(MS_Pending);
    result_ss << "}";
}

void MujinVisionManager::_StatusThread(const unsigned int port, const unsigned int ms)
{
    _StartStatusPublisher(port);
    std::cout << "Started status thread (port: " << port << ")."<< std::endl;
    std::vector<ManagerStatus> vstatus;
    std::vector<std::string> vmessage;
    std::vector<unsigned long long> vtimestamp;
    CallFunctionOnDestruction(boost::bind(&MujinVisionManager::_PublishStopStatus, this));
    while (!_bStopStatusThread) {
        {
            boost::mutex::scoped_lock lock(_mutexStatusQueue);
            vstatus.resize(0);
            while (_statusQueue.size()>1) {
                vstatus.push_back(_statusQueue.front());
                _statusQueue.pop();
                vmessage.push_back(_messageQueue.front());
                _messageQueue.pop();
                vtimestamp.push_back(_timestampQueue.front());
                _timestampQueue.pop();
            }
            if (vstatus.size()==0) {
                vstatus.push_back(_statusQueue.front());
                vmessage.push_back(_messageQueue.front());
                vtimestamp.push_back(_timestampQueue.front());
            }
        }
        for (unsigned int i=0; i<vstatus.size(); i++) {
            _pStatusPublisher->Publish(_GetStatusJsonString(vtimestamp.at(i), _vStatusDescriptions.at(vstatus.at(i)), vmessage.at(i)));
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
    }
}

std::string MujinVisionManager::_GetStatusJsonString(const unsigned long long timestamp, const std::string& status, const std::string& message)
{
    std::stringstream ss;
    ss << "{";
    ss << ParametersBase::GetJsonString("timestamp") << ": " << timestamp << ", ";
    ss << ParametersBase::GetJsonString("status", status) << ", ";
    ss << ParametersBase::GetJsonString("message",message);
    ss << "}";
    return ss.str();
}

void MujinVisionManager::_CommandThread(const unsigned int port)
{
    _StartCommandServer(port);
    std::cout << "Started command thread (port: " << port << ")." << std::endl;
    std::string incomingmessage;
    ptree command_pt;
    std::stringstream command_ss, result_ss;
    while (!_mPortStopCommandThread[port]) {
        try {
            // receive message
            if( _mPortCommandServer[port]->Recv(incomingmessage) > 0 ) {
                std::cout << "Received command message: " << incomingmessage << "." << std::endl;
                // execute command
                command_ss.str("");
                command_ss.clear();
                command_ss.str(incomingmessage);
                read_json(command_ss, command_pt);
                result_ss.str("");
                result_ss.clear();
                try {
                    if (port == _pVisionServerParameters->configurationPort) {
                        _ExecuteConfigurationCommand(command_pt, result_ss);
                    } else if (port == _pVisionServerParameters->rpcPort) {
                        _ExecuteUserCommand(command_pt, result_ss);
                    }
                }
                catch (const UserInterruptException& ex) { // need to catch it here, otherwise zmq will be in bad state
                    if (port == _pVisionServerParameters->configurationPort) {
                        std::cerr << "User requested program exit." << std::endl;
                        throw;
                    } else {
                        _SetStatus(MS_Preempted,"",true);
                        std::cerr << "User interruped command execution." << std::endl;
                        result_ss << "{" << ParametersBase::GetJsonString("status", _vStatusDescriptions[MS_Preempted]) << "}";
                    }
                }
                catch (const MujinVisionException& e) {
                    _SetStatus(MS_Aborted,"",true);
                    std::cerr << "MujinVisionException " << e.message() << std::endl;
                    if (e.GetCode() == MVE_CommandNotSupported) {
                        result_ss << "{" << ParametersBase::GetJsonString("error", e.message()) << "}";
                    } else if (e.GetCode() == MVE_InvalidArgument) {
                        result_ss << "{" << ParametersBase::GetJsonString("error", e.message()) << "}";
                    } else if (e.GetCode() == MVE_ConfigurationFileError) {
                        result_ss << "{" << ParametersBase::GetJsonString("error", e.message()) << "}";
                    } else if (e.GetCode() == MVE_ControllerError) {
                        result_ss << "{" << ParametersBase::GetJsonString("error", e.message()) << "}";
                    } else {
                        std::cerr << "Unhandled MujinVisionException, throw. " << std::endl;
                        throw;
                    }
                }

                // send output
                _mPortCommandServer[port]->Send(result_ss.str());

            } else {
                // wait for command
                boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            }
        }
        catch (const UserInterruptException& ex) {
            throw;
        }
        catch (const mujinclient::MujinException& e) {
            _SetStatus(MS_Aborted,"",true);
            std::cerr << "mujinclient::MujinException " << e.message() << std::endl;
            throw;
        } catch (const zmq::error_t& e) {
            _SetStatus(MS_Aborted,"",true);
            std::cerr << "zmq exception " << e.what() << std::endl;
            throw;
        }
        catch (const std::exception& e) {
            _SetStatus(MS_Aborted,"",true);
            std::cerr << "std::exception " << e.what() << std::endl;
            throw;
        }
    }
    _StopCommandServer(port);
}

void MujinVisionManager::_StartDetectionThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double voxelsize, const double pointsize)
{
    _bStopDetectionThread = false;
    _pDetectionThread.reset(new boost::thread(boost::bind(&MujinVisionManager::_DetectionThread, this, regionname, cameranames, voxelsize, pointsize)));
}

void MujinVisionManager::_StopDetectionThread()
{
    _SetStatusMessage("Stopping detectoin thread.");
    if (!_bStopDetectionThread) {
        _bStopDetectionThread = true;
        if (!!_pDetectionThread) {
            _pDetectionThread->join();
            _SetStatusMessage("Stopped detection thread.");
        }
    }
}

void MujinVisionManager::_DetectionThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double voxelsize, const double pointsize)
{
    while (!_bStopDetectionThread) {
        // update picked positions
        if (_bStopDetectionThread) {
            break;
        }
        BinPickingTaskResource::ResultGetPickedPositions pickedpositions;
        _pBinpickingTask->GetPickedPositions(pickedpositions,"m");
        const unsigned int numPickedPositions = pickedpositions.transforms.size();
        std::cout << "Got " << numPickedPositions << " picked positions" << std::endl;

        // remove saved detection results near picked positions
        bool pickedRecently = false;
        Vector weights(2,2,1); // prioritize XY over Z
        for (unsigned int i=0; i<numPickedPositions; i++) {
            unsigned long long timestamp = pickedpositions.timestamps[i];
            // if timestamp is known
            if (_sTimestamp.find(timestamp)!=_sTimestamp.end()) {
                if (GetMilliTime() - timestamp < _pVisionServerParameters->timeToIgnore) {
                    std::cout << "Just picked up an object, keep ignoring detection in this region (" << (_pVisionServerParameters->timeToIgnore - (GetMilliTime()-timestamp)) << " ms left)." << std::endl;
                } else {
                    std::cout << "Already cleared picked position at timestamp " << (GetMilliTime() - timestamp) << " ms ago." << std::endl;
                    continue;
                }
            } else { // for new timestamp
                _sTimestamp.insert(timestamp);
                std::cout << "Added timestamp " << timestamp << " to cleared set." << std::endl;
                pickedRecently = true;
            }
            std::cout << "An object was picked " << (GetMilliTime()-timestamp) << " ms ago, clear known detection results that are nearby." << std::endl;
            Transform transform = _GetTransform(pickedpositions.transforms[i]);
            Vector position = transform.trans;

            if (_vDetectedInfo.size()>0) {
                for (int j = _vDetectedInfo.size() - 1; j >= 0; j--) { // have to iterate from the end to remove items from the vectors
                    double dist = std::sqrt(((position-_vDetectedInfo.at(j).meanPosition)*weights).lengthsqr3());
                    std::cout << "Part " << j << " distance to object " << dist << std::endl;
                    if (dist < _pVisionServerParameters->clearRadius) {
                        std::cout << "Part " << j << " is within the clear radius of picked position, clear its records." << std::endl;
                        _vDetectedInfo.erase(_vDetectedInfo.begin()+j);
                    }
                }
            }
        }

        // if picked recently, skip detection once to make sure there is no occlusion
        if (_bStopDetectionThread) {
            break;
        }
        if (pickedRecently) {
            std::cout << "An object was just picked, skip detection once to make sure there is no occlusion." << std::endl;
            continue;
        }

        // detect objects
        if (_bStopDetectionThread) {
            break;
        }
        std::vector<DetectedObjectPtr> detectedobjects;
        DetectObjects(regionname, cameranames, detectedobjects);

        // process results
        if (_bStopDetectionThread) {
            break;
        }
        for (unsigned int i=0; i<detectedobjects.size(); i++) {
            unsigned long long timestamp = GetMilliTime();
            double score = detectedobjects[i]->confidence;
            Transform transform = detectedobjects[i]->transform;
            TransformMatrix mat(transform);
            Vector position = transform.trans;
            Vector rotation = transform.rot;
            double minDist = 999;
            int minIndex = -1;
            // make sure the z-axis of the detected rotation's origin is pointing up in the world frame, so that upside-down flipping is considered the same
            double dotproductX = mat.m[0]+mat.m[4]+mat.m[8];
            double dotproductY = mat.m[1]+mat.m[5]+mat.m[9];
            double dotproductZ = mat.m[2]+mat.m[6]+mat.m[10];
            if (dotproductZ<0  // if z pointing down
                || (dotproductZ == 0 && dotproductX <0) // or if z pointing flat, but x pointing down
                || (dotproductZ == 0 && dotproductX == 0 && dotproductY<0) // or both z and x pointing flat, but y pointing down
                ) {
                std::cout << "Upside-down detection (" << rotation[0] << ", " << rotation[1] << ", " << rotation[2] << ", " << rotation[3] << "), flip rotation." << std::endl;
                // rotate around x axis by 180
                rotation[0] = -transform.rot[1];
                rotation[1] = transform.rot[0];
                rotation[2] = transform.rot[3];
                rotation[3] = -transform.rot[2];
            }

            for (unsigned int j=0; j<_vDetectedInfo.size(); j++) {
                double dist = std::sqrt(((position-_vDetectedInfo[j].meanPosition)*weights).lengthsqr3());
                if (dist < minDist) {
                    minDist = dist;
                    minIndex = j;
                }
            }
            if (minDist < _pVisionServerParameters->maxPositionError) {
                _vDetectedInfo.at(minIndex).count++;
                unsigned int numDetections;
                // only keep track of the last n detection results
                if (_vDetectedInfo.at(minIndex).count <= _pVisionServerParameters->numDetectionsToKeep) {
                    _vDetectedInfo.at(minIndex).positions.push_back(position);
                    _vDetectedInfo.at(minIndex).rotations.push_back(rotation);
                    _vDetectedInfo.at(minIndex).scores.push_back(score);
                    numDetections = _vDetectedInfo.at(minIndex).count;
                } else {
                    numDetections = _pVisionServerParameters->numDetectionsToKeep;
                    unsigned int newindex = _vDetectedInfo.at(minIndex).count% numDetections;
                    _vDetectedInfo.at(minIndex).positions.at(newindex) = position;
                    _vDetectedInfo.at(minIndex).rotations.at(newindex) = rotation;
                    _vDetectedInfo.at(minIndex).scores.at(newindex) = score;
                }
                std::cout << "Part " << minIndex << " is known (minDist " << minDist << "), updating its mean position averaging " << numDetections << " detections." << std::endl;

                // update timestamp
                _vDetectedInfo.at(minIndex).timestamp = timestamp;
                // update means
                Vector sumPosition(0,0,0);
                for (unsigned int j=0; j<numDetections; j++) {
                    sumPosition += _vDetectedInfo.at(minIndex).positions.at(j);
                }
                _vDetectedInfo.at(minIndex).meanPosition = sumPosition * (1.0f/numDetections);
                double minQuatDotProduct = 999;
                int minQuatIndex = -1;
                for (unsigned int j=0; j<numDetections; j++) {
                    double sum = 0;
                    for (unsigned int k=0; k<numDetections; k++) {
                        sum += 1- _vDetectedInfo.at(minIndex).rotations.at(j).dot(_vDetectedInfo.at(minIndex).rotations.at(k));
                    }
                    double quatDotProduct = sum / numDetections;
                    if (quatDotProduct < minQuatDotProduct) {
                        minQuatDotProduct = quatDotProduct;
                        minQuatIndex = j;
                    }
                }
                _vDetectedInfo.at(minIndex).meanRotation = _vDetectedInfo.at(minIndex).rotations.at(minQuatIndex);
                double sumScore=0;
                for (unsigned int j=0; j<numDetections; j++) {
                    sumScore += _vDetectedInfo.at(minIndex).scores.at(j);
                }
                _vDetectedInfo.at(minIndex).meanScore = sumScore / numDetections;
            } else { // new object is detected
                std::cout << "New object is detected at (" << rotation[0] << ", " << rotation[1] << ", " << rotation[2] << ", " << rotation[3] << " ," <<  position[0] << ", " << position[1] << ", " << position[2] << ")" << std::endl;
                std::vector<Vector> positions;
                positions.push_back(position);
                std::vector<Vector> rotations;
                rotations.push_back(rotation);
                std::vector<double> scores;
                scores.push_back(score);
                DetectedInfo info;
                info.timestamp = timestamp;
                info.count = 1;
                info.meanPosition = position;
                info.meanRotation = rotation;
                info.meanScore = score;
                info.positions = positions;
                info.rotations = rotations;
                info.scores = scores;
                _vDetectedInfo.push_back(info);
            }
        }
        if (_vDetectedInfo.size()>0) {
            // remove old detection results
            for (int i=_vDetectedInfo.size()-1; i>=0; i--) {
                if (GetMilliTime() - _vDetectedInfo.at(i).timestamp > _pVisionServerParameters->timeToRemember) {
                    std::cout << "Part " << i << " has not been seen for " << _pVisionServerParameters->timeToRemember << " ms, removing its records." << std::endl;
                    _vDetectedInfo.erase(_vDetectedInfo.begin()+i);
                }
            }
        }

        // create new results
        std::vector<DetectedObjectPtr> newdetectedobjects;
        if (detectedobjects.size()>0) {
            for (unsigned int i=0; i<_vDetectedInfo.size(); i++) {
                Transform transform;
                transform.trans = _vDetectedInfo.at(i).meanPosition;
                transform.rot = _vDetectedInfo.at(i).meanRotation;
                DetectedObjectPtr obj(new DetectedObject(detectedobjects[0]->name, transform, _vDetectedInfo.at(i).meanScore));
                newdetectedobjects.push_back(obj);
                obj->Print();
            }
        }

        // send results to mujin controller
        if (_bStopDetectionThread) {
            break;
        }
        std::cout << "Sending " << newdetectedobjects.size() << " detected objects to the mujin controller." << std::endl;
        SendPointCloudObstacleToController(regionname, cameranames, newdetectedobjects, voxelsize, pointsize);
        UpdateDetectedObjects(newdetectedobjects, true);

        // visualize results
        if (_bStopDetectionThread) {
            break;
        }
    }
}

mujinvision::Transform MujinVisionManager::_GetTransform(const std::string& instobjname)
{
    mujinclient::Transform t;
    _pBinpickingTask->GetTransform(instobjname,t,"m");
    return mujinvision::Transform(Vector(t.quaternion[0] /*w*/,t.quaternion[1] /*x*/, t.quaternion[2] /*y*/,t.quaternion[3] /*z*/), Vector(t.translate[0], t.translate[1], t.translate[2]));
}

void MujinVisionManager::_SyncCamera(const std::string& regionname, const std::string& cameraname)
{
    if (_mNameCamera.find(cameraname) == _mNameCamera.end()) {
        throw MujinVisionException("Camera "+cameraname+ " is unknown!", MVE_InvalidArgument);
    }
    Transform O_T_C = _GetTransform(cameraname); // camera transform in world frame
    _mNameCamera[cameraname]->SetWorldTransform(O_T_C);
    std::cout << "setting camera transform to: " << std::endl;
    std::cout << _GetString(_mNameCamera[cameraname]->GetWorldTransform());

    // compute image roi
    Transform O_T_B = _GetTransform(regionname); // region transform in world frame
    Transform C_T_O = O_T_C.inverse(); // world origin in camera frame
    Transform B_T_Bvertex[8]; // box vertex transforms in region frame
    Transform O_T_Bvertex[8]; // box vertex transforms in world frame
    Transform C_T_Bvertex[8]; // box vertex transforms in camera frame
    // get globalroi3d
    double globalroi3d[6]; // minx maxx miny maxy minz maxz in region frame
    globalroi3d[0] = _mNameRegion[regionname]->pRegionParameters->minx;
    globalroi3d[1] = _mNameRegion[regionname]->pRegionParameters->maxx;
    globalroi3d[2] = _mNameRegion[regionname]->pRegionParameters->miny;
    globalroi3d[3] = _mNameRegion[regionname]->pRegionParameters->maxy;
    globalroi3d[4] = _mNameRegion[regionname]->pRegionParameters->minz;
    globalroi3d[5] = _mNameRegion[regionname]->pRegionParameters->maxz;
    // update vertices with globalroi3d
    unsigned int index=0;
    //    .3--------1 z
    //  .' |      .'|
    // 7---+----5'  |
    // |   |    |   |
    // |y ,2----+---0 o
    // |.'      | .'
    // 6--------4'
    //         x
    for (unsigned int i=0; i<2; i++) {
        for (unsigned int j=2; j<4; j++) {
            for (unsigned int k=4; k<6; k++) {
                B_T_Bvertex[index].trans[0] = globalroi3d[i];
                B_T_Bvertex[index].trans[1] = globalroi3d[j];
                B_T_Bvertex[index].trans[2] = globalroi3d[k];
                O_T_Bvertex[index] = O_T_B * B_T_Bvertex[index];
                //std::cout << "vertex " << index << " (" << B_T_Bvertex[index].trans[0] << ", " << B_T_Bvertex[index].trans[1] << ", " << B_T_Bvertex[index].trans[2] << ") (" << O_T_Bvertex[index].trans[0] << ", " << O_T_Bvertex[index].trans[1] << ", " << O_T_Bvertex[index].trans[2] << ")" << std::endl;
                C_T_Bvertex[index] = C_T_O * (O_T_B * B_T_Bvertex[index]);
                index++;
            }
        }
    }
    // get sensordata
    RobotResource::AttachedSensorResource::SensorData sensordata;
    utils::GetSensorData(_pControllerClient, _pSceneResource, cameraname, sensordata);
    // project vertices into image
    std::vector<double> pxlist, pylist;
    double x,y,px,py;
    for (unsigned int i=0; i<8; i++) {
        x = C_T_Bvertex[i].trans[0] / C_T_Bvertex[i].trans[2];
        y = C_T_Bvertex[i].trans[1] / C_T_Bvertex[i].trans[2];
        px = sensordata.intrinsic[0] * x  + sensordata.intrinsic[1] * y + sensordata.intrinsic[2];
        py = sensordata.intrinsic[3] * x  + sensordata.intrinsic[4] * y + sensordata.intrinsic[5];
        pxlist.push_back(px);
        pylist.push_back(py);
    }
    // need to make sure roi is within image boundary
    const int image_width  = sensordata.image_dimensions[0];
    const int image_height  = sensordata.image_dimensions[1];
    _mNameCamera[cameraname]->pCameraParameters->minu = std::max(int(*std::min_element(pxlist.begin(),pxlist.end())), 0);
    _mNameCamera[cameraname]->pCameraParameters->maxu = std::min(int(*std::max_element(pxlist.begin(),pxlist.end())), image_width);
    _mNameCamera[cameraname]->pCameraParameters->minv = std::max(int(*std::min_element(pylist.begin(),pylist.end())), 0);
    _mNameCamera[cameraname]->pCameraParameters->maxv = std::min(int(*std::max_element(pylist.begin(),pylist.end())), image_height);
    std::cout << "image_roi: " << _mNameCamera[cameraname]->pCameraParameters->minu << ", " << _mNameCamera[cameraname]->pCameraParameters->maxu << ", " << _mNameCamera[cameraname]->pCameraParameters->minv << ", "<< _mNameCamera[cameraname]->pCameraParameters->maxv << std::endl;
}

void MujinVisionManager::_SyncRegion(const std::string& regionname)
{
    if (_mNameRegion.find(regionname) == _mNameRegion.end()) {
        throw MujinVisionException("Region "+regionname+ " is unknown!", MVE_InvalidArgument);
    }
    _mNameRegion[regionname]->SetWorldTransform(_GetTransform(regionname));
    std::cout << "setting region transform to: " << std::endl;
    std::cout << _GetString(_mNameRegion[regionname]->GetWorldTransform());
    // update globalroi3d from mujin controller
    if (!_mNameRegion[regionname]->pRegionParameters->bInitializedRoi) {
        std::cout << "Computing globalroi3d from mujin controller." << std::endl;
        // get axis aligned bounding box for region
        BinPickingTaskResource::ResultAABB raabb;
        _pBinpickingTask->GetAABB(regionname, raabb, "m");
        //                0                    2                    4
        //                x                    y                    z
        double aabb[6] = {0,raabb.extents[0]*2,0,raabb.extents[1]*2,0,raabb.extents[2]*2};
        double center[3] = {raabb.pos[0], raabb.pos[1], raabb.pos[2]};

        // need to prepare offets to calculate vertices positions
        Transform O_T_B = _GetTransform(regionname); // region origin in world frame
        TransformMatrix m_O_T_B(O_T_B);
        // top left            top right      |---> y world frame
        // bottom left         bottom right   v
        //                                    x
        if (m_O_T_B.trans[0]-center[0]<0 && m_O_T_B.trans[1]-center[1]<0) { // top left
        } else if (m_O_T_B.trans[0]-center[0]<0 && m_O_T_B.trans[1]-center[1]>0) { // top right
            // flip y
            aabb[3] = -aabb[3];
        } else if (m_O_T_B.trans[0]-center[0]>0 && m_O_T_B.trans[1]-center[1]<0) { // bottom left
            // flip x
            aabb[1] = -aabb[1];
        } else if (m_O_T_B.trans[0]-center[0]>0 && m_O_T_B.trans[1]-center[1]>0) { // bottom right
            // flip x and y
            aabb[1] = -aabb[1];
            aabb[3] = -aabb[3];
        }
        //std::cout << "trans " << m_O_T_B.trans[0] << ", " << m_O_T_B.trans[1] << ", " << m_O_T_B.trans[2] <<  " pos " << center[0] << ", " << center[1]<< ", " << center[2] << " aabb " << aabb[0] << ", " << aabb[1] << ", " << aabb[2] << ", " << aabb[3] << ", " << aabb[4] << ", " << aabb[5] << std::endl;
        Transform O_T_Bvertex[8]; // region vertices in world frame
        Transform B_T_Bvertex[8]; // region vertices in region frame
        for (unsigned int i=0; i<8; i++) {
            O_T_Bvertex[i] = O_T_B;
        }
        unsigned int index=0;
        double minx,maxx,miny,maxy,minz,maxz; // roi in region frame
        // order of vertices
        //    .3--------1 z
        //  .' |      .'|
        // 7---+----5'  |
        // |   |    |   |
        // |y ,2----+---0 o  world frame
        // |.'      | .'
        // 6--------4'
        //         x
        for (unsigned int i=0; i<2; i++) {
            for (unsigned int j=2; j<4; j++) {
                for (unsigned int k=4; k<6; k++) {
                    // update vertices positions in world frame
                    O_T_Bvertex[index].trans[0] += aabb[i];
                    O_T_Bvertex[index].trans[1] += aabb[j];
                    O_T_Bvertex[index].trans[2] += aabb[k];
                    // calculate vertices positions in region frame
                    B_T_Bvertex[index] = O_T_B.inverse() * O_T_Bvertex[index];
                    // calculate the roi in region frame
                    if (index==0) {
                        minx=B_T_Bvertex[index].trans[0];
                        maxx=B_T_Bvertex[index].trans[0];
                        miny=B_T_Bvertex[index].trans[1];
                        maxy=B_T_Bvertex[index].trans[1];
                        minz=B_T_Bvertex[index].trans[2];
                        maxz=B_T_Bvertex[index].trans[2];
                    } else {
                        minx = std::min(minx, B_T_Bvertex[index].trans[0]);
                        maxx = std::max(maxx, B_T_Bvertex[index].trans[0]);
                        miny = std::min(miny, B_T_Bvertex[index].trans[1]);
                        maxy = std::max(maxy, B_T_Bvertex[index].trans[1]);
                        minz = std::min(minz, B_T_Bvertex[index].trans[2]);
                        maxz = std::max(maxz, B_T_Bvertex[index].trans[2]);
                    }
                    index++;
                }
            }
        }
        std::cout << "initialized " << regionname << "\'s roi in region frame: " << minx << " " << maxx << " " << miny << " " << maxy << " " << minz << " " << maxz << std::endl;
        _mNameRegion[regionname]->pRegionParameters->minx = minx;
        _mNameRegion[regionname]->pRegionParameters->maxx = maxx;
        _mNameRegion[regionname]->pRegionParameters->miny = miny;
        _mNameRegion[regionname]->pRegionParameters->maxy = maxy;
        _mNameRegion[regionname]->pRegionParameters->minz = minz;
        _mNameRegion[regionname]->pRegionParameters->maxz = maxz;
        _mNameRegion[regionname]->pRegionParameters->bInitializedRoi = true;
        std::cout << _mNameRegion[regionname]->pRegionParameters->GetJsonString() << std::endl;
    }
}

ColorImagePtr MujinVisionManager::_GetColorImage(const std::string& regionname, const std::string& cameraname)
{
    ColorImagePtr colorimage;
    unsigned long long timestamp;
    bool isoccluding;
    while (!_bCancelCommand && !bShutdown) {
        colorimage = _pImagesubscriberManager->GetColorImage(cameraname,timestamp);
        if (!colorimage) {
            std::cerr << "[WARN]: Could not get color image for camera: " << cameraname << ", wait for 1 more second." << std::endl;
            boost::this_thread::sleep(boost::posix_time::seconds(1));
            continue;
        }
        _pBinpickingTask->IsRobotOccludingBody(regionname, cameraname, timestamp, timestamp, isoccluding);
        if (!isoccluding) {
            break;
        } else {
            std::cerr << "[WARN]: Region is occluded in the view of " << cameraname << ", will try again." << std::endl;
            boost::this_thread::sleep(boost::posix_time::milliseconds(200));
        }
    }
    return colorimage;
}

DepthImagePtr MujinVisionManager::_GetDepthImage(const std::string& regionname, const std::string& cameraname)
{
    DepthImagePtr depthimage;
    unsigned long long starttime, endtime;
    bool isoccluding;
    while (!_bCancelCommand && !bShutdown) {
        depthimage = _pImagesubscriberManager->GetDepthImage(cameraname, _numDepthImagesToAverage, starttime, endtime);
        if (!depthimage) {
            std::cerr << "could not get depth image for camera: " << cameraname << ", wait for 1 more second" << std::endl;
            boost::this_thread::sleep(boost::posix_time::seconds(1));
            continue;
        }
        _pBinpickingTask->IsRobotOccludingBody(regionname, cameraname, starttime, endtime, isoccluding);
        if (!isoccluding) {
            break;
        }else {
            std::cerr << "[WARN]: Region is occluded in the view of " << cameraname << ", will try again." << std::endl;
            boost::this_thread::sleep(boost::posix_time::milliseconds(200));
        }
    }
    return depthimage;
}

ptree MujinVisionManager::_GetResultPtree(ManagerStatus status)
{
    _SetStatus(status);
    std::string statusstring = _vStatusDescriptions.at(status);
    ptree pt;
    pt.put<std::string>("status", statusstring);
    return pt;
}

ptree MujinVisionManager::Initialize(const std::string& detectorConfigFilename, const std::string& imagesubscriberConfigFilename, const std::string& controllerIp, const unsigned int controllerPort, const std::string& controllerUsernamePass, const std::string& robotControllerIp, const unsigned int robotControllerPort, const unsigned int binpickingTaskZmqPort, const unsigned int binpickingTaskHeartbeatPort, const double binpickingTaskHeartbeatTimeout, const std::string& binpickingTaskScenePk, const std::string& robotname, const std::string& regionname)
{
    ptree pt;

    // connect to mujin controller
    std::stringstream url_ss;
    url_ss << "http://"<< controllerIp << ":" << controllerPort;
    ControllerClientPtr controller = CreateControllerClient(controllerUsernamePass, url_ss.str());
    _pControllerClient = controller;
    _SetStatusMessage("Connected to mujin controller at " + url_ss.str());
    SceneResourcePtr scene(new SceneResource(controller,binpickingTaskScenePk));
    _pSceneResource = scene;
    _pBinpickingTask = scene->GetOrCreateBinPickingTaskFromName_UTF8("binpickingtask1", TRO_EnableZMQ);
    _pBinpickingTask->Initialize(robotControllerIp, robotControllerPort, binpickingTaskZmqPort, binpickingTaskHeartbeatPort, binpickingTaskHeartbeatTimeout);

    // sync region
    _SetStatusMessage("Syncing region.");
    _SyncRegion(regionname);
    RegionPtr region = _mNameRegion[regionname];

    // set up cameras
    _SetStatusMessage("Setting up cameras.");
    std::string name;
    CameraParametersPtr pcameraparameters;
    FOREACH(it, _mNameCameraParameters) {
        name = it->first;
        pcameraparameters = it->second;
        RobotResource::AttachedSensorResource::SensorData sensordata;
        utils::GetSensorData(_pControllerClient, _pSceneResource, name, sensordata);

        CalibrationDataPtr calibrationdata(new CalibrationData());
        calibrationdata->fx           = sensordata.intrinsic[0];
        calibrationdata->fy           = sensordata.intrinsic[4];
        calibrationdata->pu           = sensordata.intrinsic[2];
        calibrationdata->pv           = sensordata.intrinsic[5];
        calibrationdata->s            = sensordata.intrinsic[1];
        calibrationdata->focal_length = sensordata.focal_length;
        for (size_t idceff = 0; idceff < 5; idceff++) {
            calibrationdata->distortioncoeffs[idceff] = sensordata.distortion_coeffs[idceff];
        }
        if (sensordata.extra_parameters.size()==4 && sensordata.extra_parameters[0]==1) { // TODO: reorganize
            calibrationdata->kappa = sensordata.extra_parameters[1];
        } else {
            calibrationdata->kappa = 0;
        }
        calibrationdata->image_width  = sensordata.image_dimensions[0];
        calibrationdata->image_height = sensordata.image_dimensions[1];

        if (sensordata.extra_parameters.size()==4 && sensordata.extra_parameters[0]==1) { // TODO: reorganize
            calibrationdata->pixel_width = sensordata.extra_parameters[2];
            calibrationdata->pixel_height = sensordata.extra_parameters[3];
        } else {
            calibrationdata->pixel_width  = sensordata.focal_length / sensordata.intrinsic[0];
            calibrationdata->pixel_height = sensordata.focal_length / sensordata.intrinsic[4];
        }

        if (std::find(region->pRegionParameters->cameranames.begin(), region->pRegionParameters->cameranames.end(), name) != region->pRegionParameters->cameranames.end()) {
            _mNameCamera[name] = CameraPtr(new Camera(name, pcameraparameters, calibrationdata));
            _SyncCamera(regionname, name);
            if (pcameraparameters->isColorCamera) {
                _SetStatusMessage("Loading parameters for color camera " + name +".");
                _mNameColorCamera[name] = _mNameCamera[name];
            }
            if (pcameraparameters->isDepthCamera) {
                _SetStatusMessage("Loading parameters for depth camera " + name +".");
                _mNameDepthCamera[name] = _mNameCamera[name];
            }
        }
    }

    // set up subscribers
    _SetStatusMessage("Loading subscriber configuration.");
    // load subscriber configuration
    if (!boost::filesystem::exists(imagesubscriberConfigFilename)) {
        throw MujinVisionException(imagesubscriberConfigFilename+" does not exist!", MVE_ConfigurationFileError);
    }
    read_json(imagesubscriberConfigFilename, pt);

    // set up image manager
    _SetStatusMessage("Setting up image manager.");
    for (unsigned int i=0; i<_pVisionServerParameters->streamerConnections.size(); i++) {
        _vSubscribers.push_back(_pImagesubscriberManager->CreateImageSubscriber(_pVisionServerParameters->streamerConnections[i]->ip, _pVisionServerParameters->streamerConnections[i]->port, pt.get_child("zmq_subscriber")));
    }
    _pImagesubscriberManager->Initialize(_mNameCamera, _vSubscribers);

    _numDepthImagesToAverage = pt.get_child("zmq_subscriber").get<unsigned int>("num_depth_images_to_average"); // assuming each message has one depth image


    // set up detector
    _SetStatusMessage("Setting up detector.");
    if (!boost::filesystem::exists(detectorConfigFilename)) {
        throw MujinVisionException(detectorConfigFilename+" does not exist!", MVE_ConfigurationFileError);
    }
    read_json(detectorConfigFilename, pt);
    _pDetector = _pDetectorManager->CreateObjectDetector(pt.get_child("object"),pt.get_child("detection"), region, _mNameColorCamera, _mNameDepthCamera,boost::bind(&MujinVisionManager::_SetStatusMessage,this,_1));
    return _GetResultPtree(MS_Succeeded);
}

void MujinVisionManager::_DeInitialize()
{
    _StopDetectionThread();
    _pDetector->DeInitialize();
    _pImagesubscriberManager->DeInitialize();
}

ptree MujinVisionManager::DetectObjects(const std::string& regionname, const std::vector<std::string>&cameranames, std::vector<DetectedObjectPtr>&detectedobjects)
{
    uint64_t starttime = GetMilliTime();
    // TODO: use actual cameras
    std::string colorcameraname = _GetColorCameraNames(regionname, cameranames).at(0);
    std::string depthcameraname = _GetDepthCameraNames(regionname, cameranames).at(0);
    CameraPtr colorcamera = _mNameCamera[colorcameraname];
    CameraPtr depthcamera = _mNameCamera[depthcameraname];

    // set up images
    ColorImagePtr originalcolorimage = _GetColorImage(regionname, colorcameraname);
    DepthImagePtr depthimage = _GetDepthImage(regionname, depthcameraname);
    _pDetector->SetColorImage(colorcameraname, originalcolorimage, colorcamera->pCameraParameters->minu, colorcamera->pCameraParameters->maxu, colorcamera->pCameraParameters->minv, colorcamera->pCameraParameters->maxv);
    _pDetector->mMergedDepthImage[depthcameraname] = depthimage;

    // detect objects
    _pDetector->DetectObjects(colorcameraname, depthcameraname, detectedobjects);
    std::stringstream msgss;
    msgss << "Detected " << detectedobjects.size() << " objects. Took " << (GetMilliTime()-starttime)/1000.0f << " seconds.";
    _SetStatusMessage(msgss.str());
    return _GetResultPtree(MS_Succeeded);
}

ptree MujinVisionManager::StartDetectionLoop(const std::string& regionname, const std::vector<std::string>&cameranames,const double voxelsize, const double pointsize)
{
    _StartDetectionThread(regionname, cameranames, voxelsize, pointsize);
    return _GetResultPtree(MS_Succeeded);
}

ptree MujinVisionManager::StopDetectionLoop()
{
    _StopDetectionThread();
    return _GetResultPtree(MS_Succeeded);
}

ptree MujinVisionManager::SendPointCloudObstacleToController(const std::string& regionname, const std::vector<std::string>&cameranames, const std::vector<DetectedObjectPtr>& detectedobjectsworld, const double voxelsize, const double pointsize)
{
    std::vector<std::string> cameranamestobeused = _GetDepthCameraNames(regionname, cameranames);
    for(unsigned int i=0; i<cameranamestobeused.size(); i++) {
        std::string cameraname = cameranamestobeused[i];
        // get point cloud obstacle
        std::vector<Real> points;
        _pDetector->GetPointCloudObstacle(cameraname, detectedobjectsworld, points, voxelsize);

        std::stringstream ss;
        ss <<"Sending over " << (points.size()/3) << " points.";
        _SetStatusMessage(ss.str());
        _pBinpickingTask->AddPointCloudObstacle(points, pointsize, "__dynamicobstacle__");
    }

    return _GetResultPtree(MS_Succeeded);
}

ptree MujinVisionManager::VisualizePointCloudOnController(const std::string& regionname, const std::vector<std::string>&cameranames, const double pointsize)
{
    std::vector<std::string> cameranamestobeused = _GetDepthCameraNames(regionname, cameranames);
    std::vector<std::vector<Real> > pointslist;
    std::vector<std::string> names;
    std::vector<double> points;
    for (unsigned int i=0; i<cameranamestobeused.size(); i++) {
        points.resize(0);
        _pDetector->GetCameraPointCloud(cameranamestobeused[i], _GetDepthImage(regionname, cameranamestobeused[i]), points);
        if (points.size()>0) {
            pointslist.push_back(points);
            std::stringstream name_ss;
            name_ss << "__pointcloud_" << i;
            names.push_back(name_ss.str());
        }
    }
    _pBinpickingTask->VisualizePointCloud(pointslist, pointsize*1000.0f, names); // need to convert pointsize to millimeter
    return _GetResultPtree(MS_Succeeded);
}

ptree MujinVisionManager::ClearVisualizationOnController()
{
    _pBinpickingTask->ClearVisualization();
    return _GetResultPtree(MS_Succeeded);
}

ptree MujinVisionManager::SaveSnapshot(const std::string& regionname, const bool getlatest)
{
    std::vector<std::string> cameranames;
    std::vector<std::string> cameranamestobeused = _GetCameraNames(regionname, cameranames);
    FOREACH(iter,_mNameColorCamera) {
        std::string colorcameraname = iter->first;
        if (std::find(cameranamestobeused.begin(), cameranamestobeused.end(), colorcameraname) != cameranamestobeused.end()) {
            std::stringstream filename_ss;
            filename_ss << colorcameraname << "_" << GetMilliTime() << ".png";
            ColorImagePtr colorimage;
            if (getlatest || !_pDetector->mColorImage[colorcameraname]) {
                colorimage = _GetColorImage(regionname, colorcameraname);
            } else {
                colorimage = _pDetector->mColorImage[colorcameraname];
            }
            _pImagesubscriberManager->WriteColorImage(colorimage, filename_ss.str());
        }
    }
    FOREACH(iter,_mNameDepthCamera) {
        std::string depthcameraname = iter->first;
        if (std::find(cameranamestobeused.begin(), cameranamestobeused.end(), depthcameraname) != cameranamestobeused.end()) {
            std::stringstream filename_ss;
            filename_ss << depthcameraname << "_" << GetMilliTime() << ".pcd";
            DepthImagePtr depthimage;
            if (getlatest || !_pDetector->mMergedDepthImage[depthcameraname]) {
                depthimage = _GetDepthImage(regionname, depthcameraname);
            } else {
                depthimage = _pDetector->mMergedDepthImage[depthcameraname];
            }
            _pImagesubscriberManager->WriteDepthImage(depthimage, filename_ss.str());
        }
    }
    return _GetResultPtree(MS_Succeeded);
}

ptree MujinVisionManager::UpdateDetectedObjects(const std::vector<DetectedObjectPtr>&detectobjectsworld, const bool sendtocontroller)
{
    if (detectobjectsworld.size()==0) {
        return _GetResultPtree(MS_Succeeded);
    }
    if (sendtocontroller) {
        _SendDetectedObjectsToController(detectobjectsworld);
    }
    return _GetResultPtree(MS_Succeeded);
}

ptree MujinVisionManager::SyncRegion(const std::string& regionname)
{
    _SyncRegion(regionname);
    return _GetResultPtree(MS_Succeeded);
}

ptree MujinVisionManager::SyncCameras(const std::string& regionname, const std::vector<std::string>&cameranames)
{
    std::vector<std::string> cameranamestobeused = _GetCameraNames(regionname, cameranames);
    std::cout << "updating ";
    for (unsigned int i=0; i<cameranamestobeused.size(); i++) {
        std::cout << cameranamestobeused[i];
        _SyncCamera(regionname, cameranamestobeused[i]);
        if (i<cameranamestobeused.size()-1) {
            std::cout << ", ";
        }
    }
    std::cout << std::endl;
    return _GetResultPtree(MS_Succeeded);
}

std::string MujinVisionManager::_GetJsonString(const std::vector<DetectedObjectPtr>&detectedobjects)
{
    std::stringstream ss;
    ss << ParametersBase::GetJsonString("objects") << ": [";
    for (unsigned int i=0; i<detectedobjects.size(); i++) {
        ss << detectedobjects[i]->GetJsonString();
        if (i<detectedobjects.size()-1) {
            ss << ",";
        }
    }
    ss << "]";
    return ss.str();
}

std::vector<std::string> MujinVisionManager::_GetCameraNames(const std::string& regionname, const std::vector<std::string>& cameranames)
{
    if (_mNameRegion.find(regionname) == _mNameRegion.end()) {
        throw MujinVisionException("Region "+regionname+ " is unknown!", MVE_InvalidArgument);
    }
    std::vector<std::string> cameranamestobeused;
    std::vector<std::string> mappedcameranames = _mNameRegion[regionname]->pRegionParameters->cameranames;
    if (cameranames.size()==0) {
        cameranamestobeused = mappedcameranames;
    } else {
        for (unsigned int i=0; i<cameranames.size(); i++) {
            cameranamestobeused.push_back(cameranames[i]);
        }
    }
    return cameranamestobeused;
}

std::vector<std::string> MujinVisionManager::_GetColorCameraNames(const std::string& regionname, const std::vector<std::string>& cameranames)
{
    std::vector<std::string> cameranamescandidates = _GetCameraNames(regionname, cameranames);
    std::vector<std::string> colorcameranames;
    for(std::vector<std::string>::const_iterator itr = cameranamescandidates.begin(); itr != cameranamescandidates.end(); itr++) {
        if(_mNameCameraParameters[*itr]->isColorCamera) {
            colorcameranames.push_back(*itr);
        }
    }
    return colorcameranames;
}

std::vector<std::string> MujinVisionManager::_GetDepthCameraNames(const std::string& regionname, const std::vector<std::string>& cameranames)
{
    std::vector<std::string> cameranamescandidates= _GetCameraNames(regionname, cameranames);
    std::vector<std::string> colorcameranames;
    for(std::vector<std::string>::const_iterator itr = cameranamescandidates.begin(); itr != cameranamescandidates.end(); itr++) {
        if(_mNameCameraParameters[*itr]->isDepthCamera) {
            colorcameranames.push_back(*itr);
        }
    }
    return colorcameranames;
}

void MujinVisionManager::_SendDetectedObjectsToController(const std::vector<DetectedObjectPtr>& detectedobjectsworld)
{
    std::vector<mujinclient::Transform> transformsworld;
    std::vector<double> confidence;
    for (unsigned int i=0; i<detectedobjectsworld.size(); i++) {
        mujinclient::Transform transform;
        transform.quaternion[0] = detectedobjectsworld[i]->transform.rot[0];
        transform.quaternion[1] = detectedobjectsworld[i]->transform.rot[1];
        transform.quaternion[2] = detectedobjectsworld[i]->transform.rot[2];
        transform.quaternion[3] = detectedobjectsworld[i]->transform.rot[3];
        transform.translate[0] = detectedobjectsworld[i]->transform.trans[0];
        transform.translate[1] = detectedobjectsworld[i]->transform.trans[1];
        transform.translate[2] = detectedobjectsworld[i]->transform.trans[2];
        transformsworld.push_back(transform);
        confidence.push_back(detectedobjectsworld[i]->confidence);
    }
    _pBinpickingTask->UpdateObjects(detectedobjectsworld[0]->name, transformsworld, confidence,"m");
}

Transform MujinVisionManager::_GetTransform(const mujinclient::Transform& t)
{
    Transform transform;
    for (unsigned int i=0; i<3; i++) {
        transform.trans[i] = t.translate[i];
    }
    for (unsigned int i=0; i<4; i++) {
        transform.rot[i] = t.quaternion[i];
    }
    return transform;
}

void Utils::TransformDetectedObjects(const std::vector<DetectedObjectPtr>& detectedobjectsfrom, std::vector<DetectedObjectPtr>& detectedobjectsto, const Transform& O_T_S, const Transform& O_T_G)
{
    detectedobjectsto.clear();
    if (detectedobjectsfrom.size()==0) {
        return;
    }
    Transform G_T_S = O_T_G.inverse()*O_T_S;
    const std::string name = detectedobjectsfrom[0]->name;
    for (size_t i=0; i<detectedobjectsfrom.size(); i++) {
        Transform G_T_A = G_T_S * detectedobjectsfrom[i]->transform;
        DetectedObjectPtr detectedobj(new DetectedObject(name, G_T_A, detectedobjectsfrom[i]->confidence));
        detectedobjectsto.push_back(detectedobj);
    }
    BOOST_ASSERT(detectedobjectsfrom.size() == detectedobjectsto.size());
}

std::string MujinVisionManager::_GetString(const Transform& transform)
{
    std::stringstream ss;
    TransformMatrix t(transform);
    for (unsigned int r = 0; r < 3; ++r) {
        for (unsigned int c = 0; c < 3; ++c) {
            ss << t.m[r*4+c] << " ";
        }
        ss << t.trans[r] << std::endl;
    }
    return ss.str();
}

} // namespace mujinvision
