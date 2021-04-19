
//////////////////////////////////////////////////////////////////////////////////
// MIT License
//
// Copyright (c) 2017 Vicon Motion Systems Ltd
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//////////////////////////////////////////////////////////////////////////////////
#pragma once


#include "Buffer.h"
#include <boost/asio.hpp>
#include <boost/function.hpp>

/// Class containing function that implement async reading and writing with an asio socket for the CG stream.
class VCGStreamAsyncReaderWriter
{
public:

  /// Asynchronously Read a CGStream block from the socket into the buffer.
  /// Calls the supplied handler with true on success or false on failure.
  /// Note that it is the callers responsibility to ensure that the socket
  /// the buffer references remain valid until the handler is called.
  static void ReadBuffer(   boost::asio::ip::tcp::socket& i_rSocket,
                            ViconCGStreamIO::VBuffer& i_rBuffer,
                            boost::function< void( bool ) > i_Handler );

  /// Asynchronously Write a CGStream block from the buffer to the socket.
  /// Calls the supplied handler with true on success or false on failure.
  /// Note that it is the callers responsibility to ensure that the socket
  /// the buffer references remain valid until the handler is called.
  static void WriteBuffer(  boost::asio::ip::tcp::socket& i_rSocket,
                            const ViconCGStreamIO::VBuffer& i_rBuffer,
                            boost::function< void( bool ) > i_Handler );

  /// Asynchronously Send a CGStream block from the buffer to the socket.
  /// Calls the supplied handler with true on success or false on failure.
  /// Note that unlike WriteBuffer the lifetime of the UDP socket is protected
  /// by the shared_ptr.
  static void SendBuffer(  std::shared_ptr< boost::asio::ip::udp::socket > i_pSocket,
                           const ViconCGStreamIO::VBuffer& i_rBuffer,
                           boost::function< void( bool ) > i_Handler );

  /// As SendBuffer but foe connectionless sockets
  static void SendBufferTo(   std::shared_ptr< boost::asio::ip::udp::socket > i_pSocket,
                              const ViconCGStreamIO::VBuffer& i_rBuffer,
                              const boost::asio::ip::udp::endpoint& i_rEndpoint,
                              boost::function< void( bool ) > i_Handler );

private:

  static void OnBufferHeaderRead( const boost::system::error_code i_Error,
                                  const std::size_t i_BytesRead,
                                  boost::asio::ip::tcp::socket& i_rSocket,
                                  ViconCGStreamIO::VBuffer& i_rBuffer,
                                  boost::function< void( bool ) > i_Handler );

  static void OnBufferBodyRead(   const boost::system::error_code i_Error,
                                  const std::size_t i_BytesRead,
                                  boost::function< void( bool ) > i_Handler );

  static void OnBufferWritten(    const boost::system::error_code i_Error,
                                  const std::size_t i_BytesRead,
                                  boost::function< void( bool ) > i_Handler );

  static void OnBufferSent(       std::shared_ptr< boost::asio::ip::udp::socket > i_pSocket,
                                  const boost::system::error_code i_Error,
                                  const std::size_t i_BytesRead,
                                  boost::function< void( bool ) > i_Handler );

};
