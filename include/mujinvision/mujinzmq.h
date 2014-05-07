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
/** \file mujinzmq.h
    \brief Communication classes based on ZMQ.
 */
#ifndef MUJIN_ZMQ_H
#define MUJIN_ZMQ_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <zmq.hpp>
#include <sstream>

namespace mujinvision
{

#include <mujinvision/config.h>

/** \brief Base class for subscriber
 */
class MUJINVISION_API ZmqSubscriber
{
public:
    ZmqSubscriber(const std::string& host, const unsigned int port);

    virtual ~ZmqSubscriber();

protected:
    void _InitializeSocket();

    void _DestroySocket();

    boost::shared_ptr<zmq::context_t> _context;
    boost::shared_ptr<zmq::socket_t>  _socket;
    std::string _host;
    unsigned int _port;
};

/** \brief Base class for publisher
 */
class MUJINVISION_API ZmqPublisher
{
public:
    ZmqPublisher(const unsigned int port);

    virtual ~ZmqPublisher();

    bool Publish(const std::string& messagestr);

protected:
    void _InitializeSocket();

    void _DestroySocket();

    boost::shared_ptr<zmq::context_t> _context;
    boost::shared_ptr<zmq::socket_t>  _socket;
    unsigned int _port;
};

/** \brief Base class for client
 */
class MUJINVISION_API ZmqClient
{
public:
    ZmqClient(const std::string& host, const unsigned int port);

    virtual ~ZmqClient();

protected:
    void _InitializeSocket();

    void _DestroySocket();

    unsigned int _port;
    std::string _host;
    boost::shared_ptr<zmq::context_t> _context;
    boost::shared_ptr<zmq::socket_t>  _socket;
};

/** \brief Base class for server
 */
class MUJINVISION_API ZmqServer
{
public:
    ZmqServer(const unsigned int port);

    virtual ~ZmqServer();

    unsigned int Recv(std::string& data);

    void Send(const std::string& message);

protected:
    void _InitializeSocket();

    void _DestroySocket();

    zmq::message_t _reply;
    unsigned int _port;
    boost::shared_ptr<zmq::context_t> _context;
    boost::shared_ptr<zmq::socket_t>  _socket;
};

} // namespace mujinvision
#endif
