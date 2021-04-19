
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

#include <ViconCGStream/ObjectEnums.h>
#include <ViconCGStream/StreamInfo.h>
#include <ViconCGStream/CameraCalibrationHealth.h>
#include <ViconCGStream/CameraCalibrationInfo.h>
#include <ViconCGStream/CameraInfo.h>
#include <ViconCGStream/CameraWand2d.h>
#include <ViconCGStream/CameraWand3d.h>
#include <ViconCGStream/Centroids.h>
#include <ViconCGStream/CentroidTracks.h>
#include <ViconCGStream/CentroidWeights.h>
#include <ViconCGStream/DeviceInfo.h>
#include <ViconCGStream/ChannelInfo.h>
#include <ViconCGStream/VoltageFrame.h>
#include <ViconCGStream/ForceFrame.h>
#include <ViconCGStream/MomentFrame.h>
#include <ViconCGStream/CentreOfPressureFrame.h>
#include <ViconCGStream/SubjectInfo.h>
#include <ViconCGStream/SubjectTopology.h>
#include <ViconCGStream/SubjectHealth.h>
#include <ViconCGStream/ObjectQuality.h>
#include <ViconCGStream/FrameInfo.h>
#include <ViconCGStream/FrameRateInfo.h>
#include <ViconCGStream/HardwareFrameInfo.h>
#include <ViconCGStream/LatencyInfo.h>
#include <ViconCGStream/LocalSegments.h>
#include <ViconCGStream/GlobalSegments.h>
#include <ViconCGStream/LabeledRecons.h>
#include <ViconCGStream/UnlabeledRecons.h>
#include <ViconCGStream/LabeledReconRayAssignments.h>
#include <ViconCGStream/VideoFrame.h>
#include <ViconCGStream/GreyscaleBlobs.h>
#include <ViconCGStream/GreyscaleBlobsDetail.h>
#include <ViconCGStream/EdgePairs.h>
#include <ViconCGStream/Timecode.h>
#include <ViconCGStream/ForcePlateInfo.h>
#include <ViconCGStream/CameraWand2d.h>
#include <ViconCGStream/CameraWand3d.h>
#include <ViconCGStream/EyeTrackerInfo.h>
#include <ViconCGStream/EyeTrackerFrame.h>
#include <ViconCGStream/ApplicationInfo.h>
#include <ViconCGStream/FrameRateInfo.h>

#include <boost/optional.hpp>
#include <vector>

#include "ICGClient.h"

namespace ViconCGStreamClientSDK
{

class VVideoFramePtr
{
public:

  VVideoFramePtr()
  : m_pClient( 0 )
  , m_pVideoFrame( 0 )
  {
  }
  
  VVideoFramePtr( ICGClient * i_pClient, ViconCGStream::VVideoFrame * i_pVideoFrame )
  : m_pClient( i_pClient )
  , m_pVideoFrame( i_pVideoFrame )
  {
  }

  VVideoFramePtr( const VVideoFramePtr & i_rVideoFramePtr )
  : m_pClient( i_rVideoFramePtr.m_pClient )
  , m_pVideoFrame( i_rVideoFramePtr.m_pVideoFrame )
  {
    if ( m_pClient && m_pVideoFrame )
    {
      m_pClient->VideoFrameAddRef( m_pVideoFrame );
    }
  }
  
  ~VVideoFramePtr()
  {
    Clear();
  }
  
  const ViconCGStream::VVideoFrame & operator * () const
  {
    return *m_pVideoFrame;
  }
  
  ViconCGStream::VVideoFrame & operator * ()
  {
    return *m_pVideoFrame;
  }

  explicit operator bool()
  {
    return m_pClient && m_pVideoFrame;
  }

  void Clear()
  {
    if( m_pClient && m_pVideoFrame )
    {
      m_pClient->VideoFrameRelease( m_pVideoFrame );
      m_pClient = 0;
      m_pVideoFrame = 0;
    }
  }

private:

  ICGClient * m_pClient;  
  ViconCGStream::VVideoFrame * m_pVideoFrame;
};


class ICGFrameState
{
public:
  // Frame
  ViconCGStream::VStreamInfo                           m_Stream;
  ViconCGStream::VFrameInfo                            m_Frame;
  ViconCGStream::VHardwareFrameInfo                    m_HardwareFrame;
  ViconCGStream::VTimecode                             m_Timecode;
  ViconCGStream::VLatencyInfo                          m_Latency;
  boost::optional<ViconCGStream::VApplicationInfo>     m_ApplicationInfo;
  ViconCGStream::VFrameRateInfo                        m_FrameRateInfo;

  // Cameras
  ViconCGStream::VCameraCalibrationHealth              m_CameraCalibrationHealth;
  std::vector< ViconCGStream::VCameraCalibrationInfo > m_CameraCalibrations;
  std::vector< ViconCGStream::VCameraInfo >            m_Cameras;
  std::vector< ViconCGStream::VEdgePairs >             m_EdgePairs;
  std::vector< ViconCGStream::VGreyscaleBlobs >        m_GreyscaleBlobs;
  std::vector< ViconCGStream::VCentroids >             m_Centroids;
  std::vector< ViconCGStream::VCentroidTracks >        m_CentroidTracks;
  std::vector< ViconCGStream::VCentroidWeights >       m_CentroidWeights;
  std::vector< VVideoFramePtr >                        m_VideoFrames;
  std::vector< ViconCGStream::VCameraWand2d >          m_CameraWand2d;
  std::vector< ViconCGStream::VCameraWand3d >          m_CameraWand3d;

  // Reconstructions
  ViconCGStream::VUnlabeledRecons                      m_UnlabeledRecons;
  ViconCGStream::VLabeledRecons                        m_LabeledRecons;
  ViconCGStream::VLabeledReconRayAssignments           m_LabeledReconRayAssignments;

  // Devices
  std::vector< ViconCGStream::VDeviceInfo >            m_Devices;
  std::vector< ViconCGStream::VDeviceInfoExtra >       m_DevicesExtra;
  std::vector< ViconCGStream::VChannelInfo >           m_Channels;
  std::vector< ViconCGStream::VChannelInfoExtra >      m_ChannelUnits;
  std::vector< ViconCGStream::VVoltageFrame >          m_Voltages;

  // Force Plates
  std::vector< ViconCGStream::VForcePlateInfo >        m_ForcePlates;
  std::vector< ViconCGStream::VForceFrame >            m_Forces;
  std::vector< ViconCGStream::VMomentFrame >           m_Moments;
  std::vector< ViconCGStream::VCentreOfPressureFrame > m_CentresOfPressure;
  
  // Eye Trackers
  std::vector< ViconCGStream::VEyeTrackerInfo >        m_EyeTrackers;
  std::vector< ViconCGStream::VEyeTrackerFrame >       m_EyeTracks;

  // Subjects
  std::vector< ViconCGStream::VSubjectInfo >           m_Subjects;
  std::vector< ViconCGStream::VSubjectTopology >       m_SubjectTopologies;
  std::vector< ViconCGStream::VSubjectHealth >         m_SubjectHealths;
  std::vector< ViconCGStream::VObjectQuality >         m_ObjectQualities;
  std::vector< ViconCGStream::VGlobalSegments >        m_GlobalSegments;
  std::vector< ViconCGStream::VLocalSegments >         m_LocalSegments;
};

} // End of namespace ViconCGStreamClientSDK
