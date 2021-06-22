
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

#include "ICGClient.h"
#include "ICGFrameState.h"
#include <ViconCGStreamClient/IViconCGStreamClientCallback.h>
#include <ViconCGStreamClient/ViconCGStreamClient.h>
#include <boost/thread/condition.hpp>
#include <deque>

namespace ViconCGStreamClientSDK
{

class ICGFrameState;
class VCGClientCallback;

class VCGClient : public ICGClient
{
public:

  // Constructor
  VCGClient();
  virtual ~VCGClient();

  // ICGClient
  virtual void Destroy();
  virtual void Connect( std::string i_IPAddress, unsigned short i_Port );
  virtual void ReceiveMulticastData( std::string i_MulticastIPAddress, std::string i_LocalIPAddress, unsigned short i_Port );
  virtual void StopReceivingMulticastData( );

  virtual bool IsConnected() const;
  virtual bool IsMulticastReceiving() const;

  virtual void SetRequestTypes( ViconCGStreamType::Enum i_RequestedType, bool i_bEnable = true);
  virtual void SetBufferSize( unsigned int i_MaxFrames );
  virtual void SetDecodeVideo( bool i_bDecode );
  virtual void SetStreamMode( bool i_bStream );
  virtual void SetServerToTransmitMulticast( std::string i_MulticastIPAddress, std::string i_ServerIPAddress, unsigned short i_Port );
  virtual void StopMulticastTransmission();
  virtual bool IsMulticastController() const;

  virtual bool PollFrames( std::vector< ICGFrameState > & o_rFrames );
  virtual bool PollFrame( ICGFrameState& o_rFrame );

  virtual bool WaitFrames( std::vector< ICGFrameState > & o_rFrames, unsigned int i_TimeoutMs );
  virtual bool WaitFrame( ICGFrameState& o_rFrame, unsigned int i_TimeoutMs );

  virtual void VideoFrameAddRef( const ViconCGStream::VVideoFrame * i_pVideoFrame );
  virtual void VideoFrameRelease( const ViconCGStream::VVideoFrame * i_pVideoFrame );

  virtual bool NetworkLatency(double & o_rLatency) const;

  // maintains a list of devices with haptic feedback on. 
  // if on add to the list,
  // if off delete from the list if existed in the list.
  virtual bool SetApexDeviceFeedback( unsigned int i_DeviceID, bool i_bOn );

  /// Allows filtering of items in a group, e.g. only get centroids for a given camera id, etc.
  virtual void SetFilter( const ViconCGStream::VFilter & i_rFilter );

  // mirrors IViconCGStreamClientCallback
  virtual void OnConnect();
  virtual void OnStaticObjects(  std::shared_ptr< const VStaticObjects >  i_pStaticObjects );
  virtual void OnDynamicObjects( std::shared_ptr< const VDynamicObjects > i_pDynamicObjects );
  virtual void OnDisconnect();

protected:

  // Buffer
  typedef std::pair< std::shared_ptr< const VStaticObjects >, std::shared_ptr< const VDynamicObjects > > TFramePair;
  typedef std::deque< TFramePair > TFrameDeque;

  void ReadFramePair( const TFramePair& i_rPair, ICGFrameState& o_rFrameState );
  void VideoFrameAddRefInitialise( std::shared_ptr< const ViconCGStream::VVideoFrame > i_pVideoFrame );

  // The C++ client which does all of the work for us
  std::shared_ptr< VViconCGStreamClient >   m_pClient;
  std::shared_ptr< VCGClientCallback >      m_pCallback;
  bool                                      m_bConnected;
  bool                                      m_bMulticastReceiving;
  bool                                      m_bMulticastController;
  std::set< unsigned int >                  m_HapticDeviceOnList;

  // Configuration
  mutable boost::recursive_mutex      m_ClientMutex;
          ViconCGStream::VObjectEnums m_RequestedObjects;

  std::shared_ptr< const VStaticObjects >   m_pLastStaticObjects;
  TFrameDeque                               m_FrameDeque;
  unsigned int                              m_MaxBufferSize;

  boost::condition                          m_NewFramesCondition;
  
  
  class VVideoFrameReferenceCount
  {
  public:
    VVideoFrameReferenceCount()
    : m_ReferenceCount( 0 )
    {
    }
    int m_ReferenceCount;
    std::shared_ptr< const ViconCGStream::VVideoFrame > m_pVideoFrame;
  };
  
  typedef std::map< const ViconCGStream::VVideoFrame *, VVideoFrameReferenceCount > TVideoFrameReferenceCounted;
  TVideoFrameReferenceCounted m_VideoFrameReferenceCounted;
  
  bool m_bDestroyAfterVideo;
};

} // End of namespace ViconCGStreamClientSDK
