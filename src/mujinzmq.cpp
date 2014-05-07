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
#include <mujinvision/mujinzmq.h>

namespace mujinvision
{

ZmqSubscriber::ZmqSubscriber(const std::string& host, const unsigned int port)
{
    _host = host;
    _port = port;
}

ZmqSubscriber::~ZmqSubscriber()
{
    _DestroySocket();
}

void ZmqSubscriber::_InitializeSocket()
{
    _context.reset(new zmq::context_t (1));
    _socket.reset(new zmq::socket_t ((*_context.get()), ZMQ_SUB));

    std::ostringstream port_stream;
    port_stream << _port;
    _socket->connect (("tcp://" + _host + ":" + port_stream.str()).c_str());
    _socket->setsockopt(ZMQ_SUBSCRIBE, "", 0);
}

void ZmqSubscriber::_DestroySocket()
{
    if (!!_socket) {
        _socket->close();
        _socket.reset();
    }
    if( !!_context ) {
        _context->close();
        _context.reset();
    }
}

ZmqPublisher::ZmqPublisher(const unsigned int port)
{
    _port = port;
}

ZmqPublisher::~ZmqPublisher()
{
    _DestroySocket();
}

bool ZmqPublisher::Publish(const std::string& messagestr)
{
    zmq::message_t message(messagestr.size());
    memcpy(message.data(), messagestr.data(), messagestr.size());
    return _socket->send(message);
}

void ZmqPublisher::_InitializeSocket()
{
    _context.reset(new zmq::context_t (1));
    _socket.reset(new zmq::socket_t ((*(zmq::context_t*)_context.get()), ZMQ_PUB));
    std::ostringstream port_stream;
    port_stream << _port;
    _socket->bind (("tcp://*:" + port_stream.str()).c_str());
}

void ZmqPublisher::_DestroySocket()
{
    if (!!_socket) {
        _socket->close();
        _socket.reset();
    }
    if( !!_context ) {
        _context->close();
        _context.reset();
    }
}

ZmqClient::ZmqClient(const std::string& host, const unsigned int port)
{
    _host = host;
    _port = port;
}

ZmqClient::~ZmqClient()
{
    _DestroySocket();
}

void ZmqClient::_InitializeSocket()
{
    _context.reset(new zmq::context_t (1));
    _socket.reset(new zmq::socket_t ((*(zmq::context_t*)_context.get()), ZMQ_REQ));
    std::ostringstream port_stream;
    port_stream << _port;
    //std::cout << "connecting to socket at " << _host << ":" << _port << std::endl;
    _socket->connect (("tcp://" + _host + ":" + port_stream.str()).c_str());
}

void ZmqClient::_DestroySocket()
{
    if (!!_socket) {
        _socket->close();
        _socket.reset();
    }
    if( !!_context ) {
        _context->close();
        _context.reset();
    }
}

ZmqServer::ZmqServer(const unsigned int port)
{
    _port = port;
}

ZmqServer::~ZmqServer()
{
    _DestroySocket();
}

unsigned int ZmqServer::Recv(std::string& data)
{
    _socket->recv(&_reply, ZMQ_NOBLOCK);
    //std::string replystring((char *) reply.data(), (size_t) reply.size());
    data.resize(_reply.size());
    std::copy((uint8_t*)_reply.data(), (uint8_t*)_reply.data()+_reply.size(), data.begin());
    return _reply.size(); //replystring;
}

void ZmqServer::Send(const std::string& message)
{
    zmq::message_t request(message.size());
    memcpy((void *) request.data(), message.c_str(), message.size());
    _socket->send(request);
}

void ZmqServer::_InitializeSocket()
{
    _context.reset(new zmq::context_t (1));
    _socket.reset(new zmq::socket_t ((*(zmq::context_t*)_context.get()), ZMQ_REP));
    std::ostringstream port_stream;
    port_stream << _port;
    _socket->bind (("tcp://*:" + port_stream.str()).c_str());
    std::cout << "binded to tcp://*:" << _port << std::endl;
}

void ZmqServer::_DestroySocket()
{
    if (!!_socket) {
        _socket->close();
        _socket.reset();
    }
    if( !!_context ) {
        _context->close();
        _context.reset();
    }
}

} // namespace mujinvision
