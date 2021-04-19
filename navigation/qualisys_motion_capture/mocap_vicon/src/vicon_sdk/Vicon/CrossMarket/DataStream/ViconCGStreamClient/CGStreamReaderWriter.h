
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

#include <StreamCommon/Buffer.h>
#include <memory>
#include <boost/asio.hpp>

// Class providing asio based socket support for the cg stream
class VCGStreamReaderWriter : public ViconCGStreamIO::VBuffer
{
public:

  VCGStreamReaderWriter( std::shared_ptr< boost::asio::ip::tcp::socket > i_pSocket );

  VCGStreamReaderWriter( std::shared_ptr< boost::asio::ip::udp::socket > i_pMulticastSocket );

  // determine if there is data ready to read. Return value indicates if an error occured
  bool DataReady( bool & o_rbDataReady ) const;

  // Close the socket
  void Close();

  // Determine if socket is open
  bool IsOpen() const;

  // Fill buffer from socket
  bool Fill();
  
  // Flush buffer to socket
  bool Flush();

  std::shared_ptr< boost::asio::ip::tcp::socket > m_pSocket;
  std::shared_ptr< boost::asio::ip::udp::socket > m_pMulticastSocket;
};
