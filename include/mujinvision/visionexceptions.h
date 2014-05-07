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
/** \file visionexceptions.h
    \brief Exception definitions.
 */
#ifndef MUJIN_VISION_EXCEPTIONS_H
#define MUJIN_VISION_EXCEPTIONS_H

namespace mujinvision {

enum MujinVisionErrorCode
{
    MVE_Failed=0,
    MVE_InvalidArgument=1,
    MVE_CommandNotSupported=2,
    MVE_ConnectionError=3,
    MVE_ImageAcquisitionError=4,
    MVE_RecognitionError=5,
    MVE_ConfigurationFileError=6,
    MVE_ControllerError=10,
};

inline const char* GetErrorCodeString(MujinVisionErrorCode error)
{
    switch(error) {
    case MVE_Failed: return "Failed";
    case MVE_InvalidArgument: return "InvalidArguments";
    case MVE_CommandNotSupported: return "CommandNotSupported";
    case MVE_ConnectionError: return "ConnectionError";
    case MVE_ImageAcquisitionError: return "ImageAcquisitionError";
    case MVE_RecognitionError: return "RecognitionError";
    case MVE_ConfigurationFileError: return "ConfigurationFileError";
    case MVE_ControllerError: return "ControllerError";
    }
    // throw an exception?
    return "";
};

class MujinVisionException : public std::exception
{
public:
    MujinVisionException() : std::exception(), _s(""), _error(MVE_Failed) {
    }

    MujinVisionException(const std::string& s, MujinVisionErrorCode error=MVE_Failed) : std::exception()
    {
        _error=error;
        _s="mujinvision (";
        _s+= GetErrorCodeString(_error);
        _s+= "): ";
        _s+= s;
    }

    virtual ~MujinVisionException() throw() {
    }

    char const* what() const throw()
    {
        return _s.c_str();
    }

    const std::string& message() const
    {
        return _s;
    }

    MujinVisionErrorCode GetCode() const
    {
        return _error;
    }

private:
    std::string _s;
    MujinVisionErrorCode _error;

};

/// \brief exception throw when user interrupts the function
class UserInterruptException : public std::exception
{
public:
    UserInterruptException() : std::exception() {
    }
    UserInterruptException(const std::string& s) : std::exception(), _s(s) {
    }
    virtual ~UserInterruptException() throw() {
    }
    char const* what() const throw() {
        return _s.c_str();
    }
    const std::string& message() const {
        return _s;
    }

private:
    std::string _s;
};

};
#endif
