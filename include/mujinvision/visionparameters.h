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
/** \file visionparameters.h
    \brief Common parameters definitions.
 */
#ifndef MUJIN_VISION_PARAMETERS_H
#define MUJIN_VISION_PARAMETERS_H

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/ptr_container/ptr_map.hpp>
#include <boost/function.hpp>

#include "geometry.h"
#include "visionexceptions.h"

#include <time.h>

#ifndef _WIN32
#if !(defined(CLOCK_GETTIME_FOUND) && (POSIX_TIMERS > 0 || _POSIX_TIMERS > 0))
#include <sys/time.h>
#endif
#else
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <sys/timeb.h>    // ftime(), struct timeb
inline void usleep(unsigned long microseconds) {
    Sleep((microseconds+999)/1000);
}
#endif

#ifdef _WIN32
inline uint64_t GetMilliTime()
{
    LARGE_INTEGER count, freq;
    QueryPerformanceCounter(&count);
    QueryPerformanceFrequency(&freq);
    return (uint64_t)((count.QuadPart * 1000) / freq.QuadPart);
}

inline uint64_t GetMicroTime()
{
    LARGE_INTEGER count, freq;
    QueryPerformanceCounter(&count);
    QueryPerformanceFrequency(&freq);
    return (count.QuadPart * 1000000) / freq.QuadPart;
}

inline uint64_t GetNanoTime()
{
    LARGE_INTEGER count, freq;
    QueryPerformanceCounter(&count);
    QueryPerformanceFrequency(&freq);
    return (count.QuadPart * 1000000000) / freq.QuadPart;
}

inline static uint64_t GetNanoPerformanceTime() {
    return GetNanoTime();
}

#else

inline void GetWallTime(uint32_t& sec, uint32_t& nsec)
{
#if defined(CLOCK_GETTIME_FOUND) && (POSIX_TIMERS > 0 || _POSIX_TIMERS > 0)
    struct timespec start;
    clock_gettime(CLOCK_REALTIME, &start);
    sec  = start.tv_sec;
    nsec = start.tv_nsec;
#else
    struct timeval timeofday;
    gettimeofday(&timeofday,NULL);
    sec  = timeofday.tv_sec;
    nsec = timeofday.tv_usec * 1000;
#endif
}

inline uint64_t GetMilliTimeOfDay()
{
    struct timeval timeofday;
    gettimeofday(&timeofday,NULL);
    return (uint64_t)timeofday.tv_sec*1000+(uint64_t)timeofday.tv_usec/1000;
}

inline uint64_t GetNanoTime()
{
    uint32_t sec,nsec;
    GetWallTime(sec,nsec);
    return (uint64_t)sec*1000000000 + (uint64_t)nsec;
}

inline uint64_t GetMicroTime()
{
    uint32_t sec,nsec;
    GetWallTime(sec,nsec);
    return (uint64_t)sec*1000000 + (uint64_t)nsec/1000;
}

inline uint64_t GetMilliTime()
{
    uint32_t sec,nsec;
    GetWallTime(sec,nsec);
    return (uint64_t)sec*1000 + (uint64_t)nsec/1000000;
}

inline static uint64_t GetNanoPerformanceTime()
{
#if defined(CLOCK_GETTIME_FOUND) && (POSIX_TIMERS > 0 || _POSIX_TIMERS > 0) && defined(_POSIX_MONOTONIC_CLOCK)
    struct timespec start;
    uint32_t sec, nsec;
    clock_gettime(CLOCK_MONOTONIC, &start);
    sec  = start.tv_sec;
    nsec = start.tv_nsec;
    return (uint64_t)sec*1000000000 + (uint64_t)nsec;
#else
    return GetNanoTime();
#endif
}
#endif

namespace mujinvision {

using geometry::MathTransform;
typedef MathTransform<double> Transform;
using geometry::MathTransformMatrix;
typedef MathTransformMatrix<double> TransformMatrix;
using geometry::MathVector;
typedef MathVector<double> Vector;
using boost::property_tree::ptree;

/// \brief base class of parameters
struct ParametersBase
{
    ParametersBase() {
    }
    virtual ~ParametersBase() {
    }

    virtual std::string GetJsonString() = 0;
    virtual ptree GetPropertyTree() = 0;

    void Print()
    {
        std::cout << GetJsonString() << std::endl;
    }

    static std::string GetJsonString(const std::string& str)
    {
        return "\""+str+"\"";
    }

    static std::string GetJsonString(const std::string& key, const std::string& value)
    {
        return GetJsonString(key) + ": " + GetJsonString(value);
    }

    static std::string GetJsonString(const std::string& key, const unsigned int value)
    {
        std::stringstream ss;
        ss << ParametersBase::GetJsonString(key) << ": " << value;
        return ss.str();
    }

    static std::string GetJsonString(const std::vector<std::string>& vec)
    {
        std::stringstream ss;
        ss << "[";
        for (unsigned int i = 0; i < vec.size(); i++) {
            ss << vec[i];
            if( i != vec.size()-1) {
                ss << ", ";
            }
        }
        ss << "]";
        return ss.str();
    }

    static std::string GetJsonString(const std::vector<double>& vec)
    {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<double>::digits10+1);
        ss << "[";
        for (unsigned int i = 0; i < vec.size(); i++) {
            ss << vec[i];
            if( i != vec.size()-1) {
                ss << ", ";
            }
        }
        ss << "]";
        return ss.str();
    }

    static std::string GetJsonString (const std::vector<int>& vec)
    {
        std::stringstream ss;
        ss << "[";
        for (unsigned int i = 0; i < vec.size(); i++) {
            ss << vec[i];
            if( i != vec.size()-1) {
                ss << ", ";
            }
        }
        ss << "]";
        return ss.str();
    }

    static std::string GetJsonString(const Transform& transform)
    {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<double>::digits10+1);
        // \"translation\":[%.15f, %.15f, %.15f], \"quaternion\":[%.15f, %.15f, %.15f, %.15f]
        ss << "\"translation\": [";
        for (unsigned int i=0; i<3; i++) {
            ss << transform.trans[i];
            if (i!=3-1) {
                ss << ", ";
            }
        }
        ss << "], ";
        ss << "\"quaternion\": [";
        for (unsigned int i=0; i<4; i++) {
            ss << transform.rot[i];
            if (i!=4-1) {
                ss << ", ";
            }
        }
        ss << "], ";
        return ss.str();
    }

protected:
    ptree _pt;

};

/// \brief ip and port of a connection
struct ConnectionParameters : public ParametersBase
{
    ConnectionParameters() {
    }

    ConnectionParameters(const ptree& pt)
    {
        _pt = pt;
        ip = pt.get<std::string>("ip");
        port = pt.get<int>("port");
    }

    virtual ~ConnectionParameters() {
    }

    std::string ip;
    int port;

    std::string GetJsonString()
    {
        std::stringstream ss;
        ss << "{";
        ss << "\"ip\": \"" << ip << "\", \"port\": " << port;
        ss << "}";
        return ss.str();
    }

    ptree GetPropertyTree()
    {
        if (_pt.empty()) {
            _pt.put<std::string>("ip", ip);
            _pt.put<int>("port", port);
        }
        return _pt;
    }
};
typedef boost::shared_ptr<ConnectionParameters> ConnectionParametersPtr;
typedef boost::shared_ptr<ConnectionParameters const> ConnectionParametersConstPtr;
typedef boost::weak_ptr<ConnectionParameters> ConnectionParametersWeakPtr;

/// \brief image data structure
class Image;
typedef boost::shared_ptr<Image> ImagePtr;
typedef boost::shared_ptr<Image const> ImageConstPtr;
typedef boost::weak_ptr<Image> ImageWeakPtr;

class ColorImage;
typedef boost::shared_ptr<ColorImage> ColorImagePtr;
typedef boost::shared_ptr<ColorImage const> ColorImageConstPtr;
typedef boost::weak_ptr<ColorImage> ColorImageWeakPtr;

class DepthImage;
typedef boost::shared_ptr<DepthImage> DepthImagePtr;
typedef boost::shared_ptr<DepthImage const> DepthImageConstPtr;
typedef boost::weak_ptr<DepthImage> DepthImageWeakPtr;

/// \brief information about camera
struct CameraParameters : public ParametersBase // TODO: auto compute imageroi
{
    CameraParameters() {
    }

    CameraParameters(const ptree& pt)
    {
        _pt = pt;
        //name = pt.get<std::string>("name");
        id = pt.get<std::string>("id");

        minu=maxu=minv=maxv=-1;
        std::vector<int> roi;
        boost::optional<const ptree&> imageroi_pt = pt.get_child_optional("imageroi");
        if (!!imageroi_pt) {
            BOOST_FOREACH(const ptree::value_type &rv, *imageroi_pt) {
                roi.push_back(boost::lexical_cast<double>(rv.second.data()));
            }
        }
        minu = roi[0];
        maxu = roi[1];
        minv = roi[2];
        maxv = roi[3];

        isDepthCamera = pt.get<bool>("is_depth_camera", true);
    }

    virtual ~CameraParameters() {
    }

    //std::string name;
    std::string id;
    bool isDepthCamera;
    /// \brief image roi in pixel
    int minu,maxu,minv,maxv;

    std::string GetJsonString()
    {
        std::stringstream ss;
        //ss << "\"" << name << "\": ";
        ss << "{";
        ss << "\"id\": \"" << id << "\"";
        if (minu>0) {
            ss << ", \"imageroi\": ["<< minu << ", " << maxu << ", " << minv << ", " << maxv << "]";
        }
        if (!isDepthCamera) {
            ss << ", \"is_depth_camera\": false";
        }
        ss << "}";
        return ss.str();
    }

    ptree GetPropertyTree()
    {
        if (_pt.empty()) {
            //ptree pt;
            _pt.put<std::string>("id", id);
            if (minu>0) {
                ptree imageroi_pt;
                ptree p;
                p.put("",minu);
                imageroi_pt.push_back(std::make_pair("", p));
                p.put("",maxu);
                imageroi_pt.push_back(std::make_pair("", p));
                p.put("",minv);
                imageroi_pt.push_back(std::make_pair("", p));
                p.put("",maxv);
                imageroi_pt.push_back(std::make_pair("", p));
                _pt.put_child("imageroi", imageroi_pt);
            }
            if (!isDepthCamera) {
                _pt.put<bool>("isDepthCamera", isDepthCamera);
            }
            //_pt.put_child(name, pt);
        }
        return _pt;
    }
};
typedef boost::shared_ptr<CameraParameters> CameraParametersPtr;
typedef boost::shared_ptr<CameraParameters const> CameraParametersConstPtr;
typedef boost::weak_ptr<CameraParameters> CameraParametersWeakPtr;

/// \brief sensor calibration data
struct CalibrationData : public ParametersBase
{

    CalibrationData() {
    }

    CalibrationData(const ptree& pt)
    {
        _pt = pt;
        fx = pt.get<double>("fx");
        fy = pt.get<double>("fy");
        pu = pt.get<double>("pu");
        pv = pt.get<double>("pv");
        s = pt.get<double>("s");
        focal_length = pt.get<double>("focal_length");
        kappa = pt.get<double>("kappa");
        image_width = pt.get<double>("image_width");
        image_height = pt.get<double>("image_height");
        pixel_width = pt.get<double>("pixel_width");
        pixel_height = pt.get<double>("pixel_height");
    }

    virtual ~CalibrationData() {
    }

    double fx;
    double fy;
    double pu;
    double pv;
    double s;
    double focal_length;
    double kappa;
    double image_width;
    double image_height;
    double pixel_width;
    double pixel_height;
    double distortioncoeffs[5];
    std::vector<double> extra_parameters;

    std::string GetJsonString()
    {
        std::stringstream ss;
        ss << "{";
        ss << "fx: " << fx << ", ";
        ss << "fy: " << fy << ", ";
        ss << "pu: " << pu << ", ";
        ss << "pv: " << pv << ", ";
        ss << "s: " << s << ", ";
        ss << "focal_length: " << focal_length << ", ";
        ss << "kappa: " << kappa << ", ";
        ss << "image_width: " << image_width << ", ";
        ss << "image_height: " << image_height << ", ";
        ss << "pixel_width: " << pixel_width << ", ";
        ss << "pixel_height: " << pixel_height << ", ";
        ss << "extra_parameters: [";
        for (size_t iparam = 0; iparam < extra_parameters.size(); iparam++) {
            ss << extra_parameters[iparam];
            if (iparam != extra_parameters.size()) {
                ss << ", ";
            }
        }
        ss << "]";
        ss << "}";
        return ss.str();
    }

    ptree GetPropertyTree()
    {
        if (_pt.empty()) {
            _pt.put<double>("fx", fx);
            _pt.put<double>("fy", fy);
            _pt.put<double>("pu", pu);
            _pt.put<double>("pv", pv);
            _pt.put<double>("s", s);
            _pt.put<double>("focal_length", focal_length);
            _pt.put<double>("kappa", kappa);
            _pt.put<double>("image_width", image_width);
            _pt.put<double>("image_height", image_height);
            _pt.put<double>("pixel_width", pixel_width);
            _pt.put<double>("pixel_height", pixel_height);
            ptree extrapt;
            for (unsigned int i=0; i<extra_parameters.size(); i++) {
                ptree p;
                p.put<double>("",extra_parameters[i]);
                extrapt.push_back(std::make_pair("",p));
            }
            _pt.put_child("extra_parameters", extrapt);
        }
        return _pt;
    }
};
typedef boost::shared_ptr<CalibrationData> CalibrationDataPtr;
typedef boost::shared_ptr<CalibrationData const> CalibrationDataConstPtr;
typedef boost::weak_ptr<CalibrationData> CalibrationDataWeakPtr;

/// \brief information about the detected object
struct DetectedObject : public ParametersBase
{
    DetectedObject() {
    }

    /// assume input is in milimeter
    DetectedObject(const ptree& pt)
    {
        _pt = pt;
        name = pt.get<std::string>("name");
        unsigned int i=0;
        BOOST_FOREACH(const ptree::value_type &v, pt.get_child("translation_")) {
            transform.trans[i] = boost::lexical_cast<double>(v.second.data()); // assuming in milimeter
            i++;
        }
        i=0;
        BOOST_FOREACH(const ptree::value_type &v, pt.get_child("quat_")) {
            transform.rot[i] = boost::lexical_cast<double>(v.second.data());
            i++;
        }
        confidence = pt.get<double>("confidence");
    }

    /// assume input is in meter
    DetectedObject(const std::string& n, const Transform& t, const double& c)
    {
        name = n;
        transform = t;
        confidence = c;
    }

    virtual ~DetectedObject() {
    }

    std::string name;
    Transform transform; ///< in meter
    double confidence; ///< detection score

    std::string GetJsonString()
    {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<double>::digits10+1);
        //"{\"name\": \"obj\",\"translation_\":[100,200,300],\"quat_\":[1,0,0,0],\"confidence\":0.5}"
        ss << "{";
        ss << "\"name\": \"" << name << "\", ";
        ss << "\"translation_\": [";
        for (unsigned int i=0; i<3; i++) {
            ss << transform.trans[i];
            if (i!=3-1) {
                ss << ", ";
            }
        }
        ss << "], ";
        ss << "\"quat_\": [";
        for (unsigned int i=0; i<4; i++) {
            ss << transform.rot[i];
            if (i!=4-1) {
                ss << ", ";
            }
        }
        ss << "], ";
        ss << "\"confidence\": " << confidence;
        ss << "}";
        return ss.str();
    }

    ptree GetPropertyTree()
    {
        if (_pt.empty()) {
            _pt.put<std::string>("name", name);
            ptree translation_pt;
            for (unsigned int i=0; i<3; i++) {
                ptree p;
                p.put<double>("",transform.trans[i]*1000.0f); // convert from meter to milimeter
                translation_pt.push_back(std::make_pair("",p));
            }
            _pt.put_child("translation_", translation_pt);
            ptree quat_pt;
            for (unsigned int i=0; i<4; i++) {
                ptree p;
                p.put<double>("",transform.rot[i]);
                quat_pt.push_back(std::make_pair("",p));
            }
            _pt.put_child("quat_", quat_pt);
            _pt.put<double>("confidence", confidence);
        }
        return _pt;
    }
};
typedef boost::shared_ptr<DetectedObject> DetectedObjectPtr;
typedef boost::shared_ptr<DetectedObject const> DetectedObjectConstPtr;
typedef boost::weak_ptr<DetectedObject> DetectedObjectWeakPtr;

/// \brief Specifies region where vision system performs detection with a set of cameras
struct RegionParameters : public ParametersBase
{
    RegionParameters() {
    }

    RegionParameters(const ptree& pt)
    {
        _pt = pt;
        instobjectname = pt.get<std::string>("instobjectname");
        BOOST_FOREACH(const ptree::value_type &cv, pt.get_child("cameranames")) { // TODO: replace BOOST_FOREACH with FOREACH
            cameranames.push_back(cv.second.data());
        }
        std::vector<double> roi;
        BOOST_FOREACH(const ptree::value_type &rv, pt.get_child("globalroi3d")) {
            roi.push_back(boost::lexical_cast<double>(rv.second.data()));
        }
        minx = roi[0];
        maxx = roi[1];
        miny = roi[2];
        maxy = roi[3];
        minz = roi[4];
        maxz = roi[5];
    }

    virtual ~RegionParameters() {
    }

    std::string instobjectname; // instobject name in mujin controller that defines the container of the objects to be detected
    std::vector<std::string> cameranames;

    /// \brief global roi in meter
    double minx,maxx,miny,maxy,minz,maxz;

    std::string GetJsonString()
    {
        std::stringstream ss;
        ss << "{";
        ss << "\"name\": \"" <<  instobjectname << "\", ";
        ss << "\"cameranames\": " << ParametersBase::GetJsonString(cameranames) << ",";
        ss << "\"globalroi3d\": [" << minx << ", " << maxx << ", " << miny << ", " << maxy << ", " << minz << ", " << maxz << "]";
        ss << "}";
        return ss.str();
    }

    ptree GetPropertyTree()
    {
        if (_pt.empty()) {
            _pt.put<std::string>("name", instobjectname);
            ptree cameranames_pt;
            for (size_t i=0; i<cameranames.size(); i++) {
                cameranames_pt.put<std::string>("", cameranames[i]);
            }
            _pt.put_child("cameranames", cameranames_pt);
            ptree globalroi3d_pt;
            ptree p;
            p.put("",minx);
            globalroi3d_pt.push_back(std::make_pair("", p));
            p.put("",maxx);
            globalroi3d_pt.push_back(std::make_pair("", p));
            p.put("",miny);
            globalroi3d_pt.push_back(std::make_pair("", p));
            p.put("",maxy);
            globalroi3d_pt.push_back(std::make_pair("", p));
            p.put("",minz);
            globalroi3d_pt.push_back(std::make_pair("", p));
            p.put("",maxz);
            globalroi3d_pt.push_back(std::make_pair("", p));
            _pt.put_child("globalroi3d", globalroi3d_pt);
        }
        return _pt;
    }
};
typedef boost::shared_ptr<RegionParameters> RegionParametersPtr;
typedef boost::shared_ptr<RegionParameters const> RegionParametersConstPtr;
typedef boost::weak_ptr<RegionParameters> RegionParametersWeakPtr;

/// \brief camera class
class Camera
{
public:
    Camera(const std::string& n, CameraParametersPtr params, CalibrationDataPtr calibdata)
    {
        name = n;
        pCameraParameters = params;
        pCalibrationData = calibdata;
    }

    virtual ~Camera() {
    }

    void SetWorldTransform(const Transform& worldtransform)
    {
        worldTransform = worldtransform;
    }

    const Transform& GetWorldTransform() const
    {
        return worldTransform;
    }

    std::string name;
    CameraParametersPtr pCameraParameters;

    /// \brief calibration data
    CalibrationDataPtr pCalibrationData;

    CameraParametersPtr GetCameraParameters() const
    {
        return pCameraParameters;
    }

protected:

    /// \brief camera transform in world frame
    Transform worldTransform;

    int _roiminu, _roimaxu, _roiminv, _roimaxv; ///< TODO image ROI computed automatically from the global ROI

};
typedef boost::shared_ptr<Camera> CameraPtr;
typedef boost::shared_ptr<Camera const> CameraConstPtr;
typedef boost::weak_ptr<Camera> CameraWeakPtr;

/// \brief information about the region
class Region
{
public:
    Region() {
    }

    Region(RegionParametersPtr params)
    {
        pRegionParameters = params;
        minx = params->minx;
        maxx = params->maxx;
        miny = params->miny;
        maxy = params->maxy;
        minz = params->minz;
        maxz = params->maxz;
    }

    virtual ~Region() {
    }

    void SetWorldTransform(const Transform& t)
    {
        worldTransform = t;
        toRegionTransform = worldTransform.inverse();
    }

    const Transform& GetWorldTransform() const
    {
        return worldTransform;
    }

    bool IsPointInROI(const double px, const double py, const double pz)
    {
        Vector p(px,py,pz);
        Vector v = toRegionTransform*p;

        if (v.x >= minx && v.x <= maxx && v.y >= miny && v.y <= maxy && v.z >= minz && v.z <= maxz) {
            return true;
        } else {
            return false;
        }
    }

    RegionParametersPtr pRegionParameters;

private:

    double minx,maxx,miny,maxy,minz,maxz;
    /// \brief used to transform a point in the world frame to the box frame
    Transform toRegionTransform;
    Transform worldTransform;

};
typedef boost::shared_ptr<Region> RegionPtr;
typedef boost::shared_ptr<Region const> RegionConstPtr;
typedef boost::weak_ptr<Region> RegionWeakPtr;

// enum DetectionResultStatusCode {
//     DS_Succeeded=0,
//     DS_Aborted=1,
//     DS_Interrupted=2,
// };

class MujinInterruptable
{
public:
    MujinInterruptable() {
    }

    virtual ~MujinInterruptable() {
    }

    virtual void SetSetStatusFn(const boost::function<void(const std::string& msg)>& setstatusfn) {
        _setstatusfn = setstatusfn;
    }

    virtual void SetStatus(const std::string& msg)
    {
        if( !!_setstatusfn ) {
            _setstatusfn(msg);
        }
    }

protected:
    boost::function<void(const std::string& msg)> _setstatusfn;
};

} // namespace mujinvision
#endif
