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
/** \file imagesubscriber.h
    \brief Public headers of ImageSubscriber.
 */
#ifndef MUJIN_IMAGE_SUBSCRIBER_H
#define MUJIN_IMAGE_SUBSCRIBER_H

#include <mujinvision/visionparameters.h>

namespace mujinvision {

enum ImageType {
    IT_Color=0,
    IT_Depth=1,
    IT_Ir=2,
    IT_Depthxyz=3,
    IT_Any=100,
};

class MUJINVISION_API ImageSubscriber
{
public:
    ImageSubscriber() {
    }
    virtual ~ImageSubscriber() {
    }

    /// \brief User-defined subscriber parameters.
    class MUJINVISION_API SubscriberParameters;

    virtual void StartCaptureThread() = 0;

    virtual void StopCaptureThread() = 0;

    /** \brief Gets images from the latest message. Blocks until one message is processed. Note that one message may contain multiple images.
        \param outimages result images will be pushed back onto this vector
     */
    virtual void GetImagesFromMessage(std::vector<ImagePtr>& outimages) = 0;

    /** \brief Gets images from the last n consecutive messages. Blocks until n messages are processed. Note that one message may contain multiple images.
        \param n number of the desired last consecutive messages
        \param outimages vector to which the result images will be pushed
     */
    virtual void GetImagesFromConsecutiveMessages(const unsigned int n, std::vector<ImagePtr>& outimages) = 0;

    /** \brief Loads image subscriber parameters.
        \param pt boost property tree defining the image subscriber parameters
     */
    virtual void LoadParameters(const ptree& pt) = 0;

    /** \brief Gets n images of specified type from buffered messages. Should not block for new messages because some subscribers might not provide images of the desired image type.
        \param cameraid id of the camera
        \param imagetype type of the image
        \param n number of desired images
        \param outimages vector to which the result images will be pushed
     */
    virtual void GetSpecificImages(const std::string& cameraid, ImageType imagetype, const unsigned int n, std::vector<ImagePtr>& outimages) = 0;

};

typedef boost::shared_ptr<ImageSubscriber> ImageSubscriberPtr;
typedef boost::shared_ptr<ImageSubscriber const> ImageSubscriberConstPtr;
typedef boost::weak_ptr<ImageSubscriber> ImageSubscriberWeakPtr;

} // namespace mujinvision
#endif
