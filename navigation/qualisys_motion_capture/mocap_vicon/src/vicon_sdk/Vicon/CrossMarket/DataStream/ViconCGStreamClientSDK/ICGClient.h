
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

#include <StreamCommon/Type.h>
#include <string>
#include <vector>

namespace ViconCGStream
{
  class VVideoFrame;
  class VFilter;
};

namespace ViconCGStreamClientSDK
{

class ICGFrameState;

class ICGClient 
{
public:

  /// Create an instance of the client
  static ICGClient * CreateCGClient();
  
  /// Destroy this instance of the client
  virtual void Destroy() = 0;

  /// Set the ipAddress to connect to and begin connection attempts
  virtual void Connect( std::string i_IPAddress, unsigned short i_Port ) = 0;

  /// Configure this CGClient to be a multicast receiver.
  /// Users should call either ReceiveMulticastData or Connect, not both.
  /// i_MulticastIPAddress is the address that the server will send data to (and may be the broadcast address).
  /// i_LocalIPAddress is the local IP address, used to specify which NIC should listen.
  virtual void ReceiveMulticastData( std::string i_MulticastIPAddress, std::string i_LocalIPAddress, unsigned short i_Port ) = 0;

  /// Stop this CGClient from receiving multicast data
  /// After calling this function users should call either ReceiveMulticastData or Connect.
  virtual void StopReceivingMulticastData( ) = 0;

  /// Return whether we are connected to a server
  virtual bool IsConnected() const = 0;

  /// Return whether we are receiving multicast data (note that broadcast counts as multicast)
  virtual bool IsMulticastReceiving() const = 0;

  /// Set the types ( ViconCGStreamType::Enum ) of data to be received from the server
  virtual void SetRequestTypes( ViconCGStreamType::Enum i_RequestedType, bool i_bEnable = true) = 0;

  /// Set the maximum number of frames you want cached
  virtual void SetBufferSize( unsigned int i_MaxFrames ) = 0;

  /// Request that video data be transcoded into BGR888
  virtual void SetDecodeVideo( bool i_bDecode ) = 0;

  /// Request that data is constantly streamed from the server, rather than sent on request.
  virtual void SetStreamMode( bool i_bStream ) = 0;

  /// Request for turning apex device feedback on or off
  virtual bool SetApexDeviceFeedback( unsigned int i_DeviceID, bool i_bOn ) = 0;

  /// Allows filtering of items in a group, e.g. only get centroids for a given camera id, etc.
  virtual void SetFilter( const ViconCGStream::VFilter & i_rFilter ) = 0;

  /// Set the server to transmit via multicast UDP
  /// Should be called AFTER calling Connect (and another instance of the ICGClient should have called ReceiveMulticastData)
  /// i_MulticastIPAddress is the address that the server will send data to (and may be the broadcast address).
  /// i_ServerIPAddress is the IP address that the server should transmit from, used to specify which NIC should be used by the server.
  /// This does not affect the connected stream from the server to this client.
  virtual void SetServerToTransmitMulticast( std::string i_MulticastIPAddress, std::string i_ServerIPAddress, unsigned short i_Port ) = 0;

  /// Signal the server to stop transmitting multicast data.
  /// This does not affect the connected stream from the server to this client.
  /// It is not necessary to call StopMulticastTransmission before calling Destroy.
  virtual void StopMulticastTransmission() = 0;

  /// Determine whether this client is acting as a Multicast controller
  virtual bool IsMulticastController() const = 0;

  /// Poll for frames. Returns immediately
  virtual bool PollFrame( ICGFrameState& o_rFrame ) = 0;
  virtual bool PollFrames( std::vector< ICGFrameState > & o_rFrames ) = 0;

  /// Wait for frames. Returns when a frame arrives or i_TimeoutMs expires
  virtual bool WaitFrame( ICGFrameState& o_rFrame, unsigned int i_TimeoutMs ) = 0;
  virtual bool WaitFrames( std::vector< ICGFrameState > & o_rFrames, unsigned int i_TimeoutMs ) = 0;

  /// Increment video frame reference count
  virtual void VideoFrameAddRef( const ViconCGStream::VVideoFrame * i_pVideoFrame ) = 0;

  /// Decrement video frame reference count
  virtual void VideoFrameRelease( const ViconCGStream::VVideoFrame * i_pVideoFrame ) = 0;

  /// Returns our current estimate of network latency
  virtual bool NetworkLatency(double & o_rLatency) const = 0;

protected:
/// Private desctructor. To delete, call the Destroy() method
  virtual ~ICGClient() {}
};

} // End of namespace ViconCGStreamClientSDK
