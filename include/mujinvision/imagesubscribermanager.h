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
/** \file imagesubscribermanager.h
    \brief Public headers of ImageSubscriberManager.
 */
#ifndef MUJIN_VISION_IMAGE_SUBSCRIBER_MANAGER_H
#define MUJIN_VISION_IMAGE_SUBSCRIBER_MANAGER_H

#include "imagesubscriber.h"

namespace mujinvision {

class MUJINVISION_API ImageSubscriberManager
{
public:
    ImageSubscriberManager() {
    }

    virtual ~ImageSubscriberManager() {
    }

    /** \brief Initializes the image subscriber manager.
        \param mNameCamera map to cameras the subscribers subscribe to
        \param subscribers vector of subscribers to be managed
     */
    virtual void Initialize(const std::map<std::string, CameraPtr >&mNameCamera, const std::vector<ImageSubscriberPtr>&subscribers) = 0;

    /** \brief Gets the latest color image from camera and its timestamp.
        \param cameraname name of the camera
        \param timestamp timestamp of the color image
        \return pointer to the color image
     */
    virtual ColorImagePtr GetColorImage(const std::string& cameraname, unsigned long long& timestamp) = 0;

    /** \brief Gets the latest n color images from camera and the min/max timestamps.
        \param cameraname name of the camera
        \param n number of desired images
        \param colorimages vector to which the desired images will be pushed
        \param starttime timestamp of the earliest image
        \param endtime timestamp of the latest image
     */
    virtual void GetConsecutiveColorImages(const std::string& cameraname, const unsigned int n, std::vector<ColorImagePtr>& colorimages, unsigned long long& starttime, unsigned long long& endtime) = 0;

    /** \brief Gets the depth image from the latest n images with depth data, and the min/max timestamps of the images used.
        \param cameraname name of the camera
        \param n number of desired images to use for creating the depth image
        \param starttime timestamp of the earliest image
        \param endtime timestamp of the latest image
        \return pointer to the depth image
     */
    virtual DepthImagePtr GetDepthImage(const std::string& cameraname, const unsigned int n, unsigned long long& starttime, unsigned long long& endtime) = 0;

    /** \brief Creates image susbcriber.
        \param ip ip address of the image subscriber
        \param port port of the image subscriber
        \params_pt boost property tree defining the image subscriber parameters
     */
    virtual ImageSubscriberPtr CreateImageSubscriber(const std::string& ip, const unsigned int port, const ptree& params_pt) = 0;

protected:

    std::vector<ImageSubscriberPtr> _vSubscribers;

    std::map<std::string, CameraPtr> _mNameCamera; ///< name -> camera
};

typedef boost::shared_ptr<ImageSubscriberManager> ImageSubscriberManagerPtr;
typedef boost::shared_ptr<ImageSubscriberManager const> ImageSubscriberManagerConstPtr;
typedef boost::weak_ptr<ImageSubscriberManager> ImageSubscriberManagerWeakPtr;

} // namespace mujinvision
#endif
