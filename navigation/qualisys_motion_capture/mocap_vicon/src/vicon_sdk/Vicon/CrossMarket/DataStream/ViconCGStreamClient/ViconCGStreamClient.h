
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

#include "IViconCGStreamClientCallback.h"

#include <boost/asio.hpp>

#include <ViconCGStream/Enum.h>
#include <ViconCGStream/ObjectEnums.h>

#include <ViconCGStream/Contents.h>

#include <ViconCGStream/StreamInfo.h>
#include <ViconCGStream/CameraInfo.h>
#include <ViconCGStream/CameraCalibrationInfo.h>
#include <ViconCGStream/CameraCalibrationHealth.h>
#include <ViconCGStream/SubjectInfo.h>
#include <ViconCGStream/SubjectTopology.h>
#include <ViconCGStream/SubjectHealth.h>
#include <ViconCGStream/DeviceInfo.h>
#include <ViconCGStream/ChannelInfo.h>
#include <ViconCGStream/ForcePlateInfo.h>
#include <ViconCGStream/ObjectQuality.h>

#include <ViconCGStream/FrameInfo.h>
#include <ViconCGStream/HardwareFrameInfo.h>
#include <ViconCGStream/Timecode.h>
#include <ViconCGStream/LatencyInfo.h>
#include <ViconCGStream/Centroids.h>
#include <ViconCGStream/CentroidTracks.h>
#include <ViconCGStream/CentroidWeights.h>
#include <ViconCGStream/LabeledRecons.h>
#include <ViconCGStream/UnlabeledRecons.h>
#include <ViconCGStream/LabeledReconRayAssignments.h>
#include <ViconCGStream/LocalSegments.h>
#include <ViconCGStream/GlobalSegments.h>
#include <ViconCGStream/GreyscaleBlobs.h>
#include <ViconCGStream/EdgePairs.h>
#include <ViconCGStream/ForceFrame.h>
#include <ViconCGStream/MomentFrame.h>
#include <ViconCGStream/CentreOfPressureFrame.h>
#include <ViconCGStream/VideoFrame.h>
#include <ViconCGStream/VoltageFrame.h>
#include <ViconCGStream/CameraWand2d.h>
#include <ViconCGStream/CameraWand3d.h>
#include <ViconCGStream/VideoFrame.h>
#include <ViconCGStream/EyeTrackerInfo.h>
#include <ViconCGStream/EyeTrackerFrame.h>
#include <ViconCGStream/Filter.h>
#include <ViconCGStream/ApplicationInfo.h>
#include <ViconCGStream/FrameRateInfo.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/optional.hpp>
#include <memory>
#include <string>
#include <deque>
class VCGStreamReaderWriter;

//-------------------------------------------------------------------------------------------------

class VStaticObjects
{
public:

  ViconCGStream::VStreamInfo m_StreamInfo;
  boost::optional<ViconCGStream::VApplicationInfo> m_ApplicationInfo;

  typedef std::vector< ViconCGStream::VCameraInfo >                         TCameraInfo;
  typedef std::vector< ViconCGStream::VCameraCalibrationInfo >              TCameraCalibrationInfo;
  typedef std::shared_ptr< ViconCGStream::VCameraCalibrationHealth >        TCameraCalibrationHealth;
  typedef std::vector< ViconCGStream::VSubjectInfo >                        TSubjectInfo;
  typedef std::vector< ViconCGStream::VSubjectTopology >                    TSubjectTopology;
  typedef std::vector< ViconCGStream::VSubjectHealth >                      TSubjectHealth;
  typedef std::vector< ViconCGStream::VObjectQuality >                      TObjectQuality;
  typedef std::vector< ViconCGStream::VDeviceInfo >                         TDeviceInfo;
  typedef std::vector< ViconCGStream::VDeviceInfoExtra >                    TDeviceInfoExtra;
  typedef std::vector< ViconCGStream::VChannelInfo >                        TChannelInfo;
  typedef std::vector< ViconCGStream::VChannelInfoExtra >                   TChannelInfoExtra;
  typedef std::vector< ViconCGStream::VForcePlateInfo >                     TForcePlateInfo;
  typedef std::vector< ViconCGStream::VEyeTrackerInfo >                     TEyeTrackerInfo;
  typedef std::map< ViconCGStreamType::UInt32, unsigned int >               TIndexMap;
  typedef std::pair< ViconCGStreamType::UInt32, ViconCGStreamType::UInt32 > TNestedIDPair;
  typedef std::map< TNestedIDPair, unsigned int >                           TNestedIndexMap;

  TCameraInfo              m_CameraInfo;
  TCameraCalibrationInfo   m_CameraCalibrationInfo;
  TCameraCalibrationHealth m_pCameraCalibrationHealth;
  TSubjectInfo             m_SubjectInfo;
  TSubjectTopology         m_SubjectTopology;
  TSubjectHealth           m_SubjectHealth;
  TObjectQuality           m_ObjectQuality;
  TDeviceInfo              m_DeviceInfo;
  TDeviceInfoExtra         m_DeviceInfoExtra;
  TChannelInfo             m_ChannelInfo;
  TChannelInfoExtra        m_ChannelInfoExtra;
  TForcePlateInfo          m_ForcePlateInfo;
  TEyeTrackerInfo          m_EyeTrackerInfo;
  TIndexMap                m_CameraMap;
  TIndexMap                m_CameraCalibrationMap;
  TIndexMap                m_SubjectMap;
  TNestedIndexMap          m_SegmentMap; 
  TIndexMap                m_DeviceMap;
  TNestedIndexMap          m_ChannelMap;


  ViconCGStream::VCameraInfo              & AddCameraInfo();
  ViconCGStream::VCameraCalibrationInfo   & AddCameraCalibrationInfo();
  ViconCGStream::VCameraCalibrationHealth & ResetCameraCalibrationHealth();
  ViconCGStream::VSubjectInfo             & AddSubjectInfo();
  ViconCGStream::VSubjectTopology         & AddSubjectTopology();
  ViconCGStream::VSubjectHealth           & AddSubjectHealth();
  ViconCGStream::VObjectQuality           & AddObjectQuality();
  ViconCGStream::VDeviceInfo              & AddDeviceInfo();
  ViconCGStream::VDeviceInfoExtra         & AddDeviceInfoExtra();
  ViconCGStream::VChannelInfo             & AddChannelInfo();
  ViconCGStream::VChannelInfoExtra        & AddChannelInfoExtra();
  ViconCGStream::VForcePlateInfo          & AddForcePlateInfo();
  ViconCGStream::VEyeTrackerInfo          & AddEyeTrackerInfo();

  void BuildMaps();
};

//-------------------------------------------------------------------------------------------------

class VDynamicObjects
{
public:

  ViconCGStream::VFrameInfo m_FrameInfo;
  ViconCGStream::VHardwareFrameInfo m_HardwareFrameInfo;
  ViconCGStream::VTimecode m_Timecode;
  ViconCGStream::VLatencyInfo m_LatencyInfo;
  ViconCGStream::VFrameRateInfo m_FrameRateInfo;
  ViconCGStream::VLabeledRecons m_LabeledRecons;
  ViconCGStream::VUnlabeledRecons m_UnlabeledRecons;
  ViconCGStream::VLabeledReconRayAssignments m_LabeledRayAssignments;

  std::vector< ViconCGStream::VCentroids > m_Centroids;
  std::vector< ViconCGStream::VCentroidTracks > m_CentroidTracks;
  std::vector< ViconCGStream::VCentroidWeights > m_CentroidWeights;
  std::vector< ViconCGStream::VLocalSegments > m_LocalSegments;
  std::vector< ViconCGStream::VGlobalSegments > m_GlobalSegments;
  std::vector< ViconCGStream::VGreyscaleBlobs > m_GreyscaleBlobs;
  std::vector< ViconCGStream::VEdgePairs > m_EdgePairs;
  std::vector< ViconCGStream::VForceFrame > m_ForceFrames;
  std::vector< ViconCGStream::VMomentFrame > m_MomentFrames;
  std::vector< ViconCGStream::VCentreOfPressureFrame > m_CentreOfPressureFrames;
  std::vector< ViconCGStream::VVoltageFrame > m_VoltageFrames;
  std::vector< ViconCGStream::VCameraWand2d > m_CameraWand2d;
  std::vector< ViconCGStream::VCameraWand3d > m_CameraWand3d;
  std::vector< ViconCGStream::VEyeTrackerFrame > m_EyeTrackerFrames;
  std::vector< std::shared_ptr< ViconCGStream::VVideoFrame > > m_VideoFrames;

  ViconCGStream::VCentroids & AddCentroids();
  ViconCGStream::VCentroidTracks & AddCentroidTracks();
  ViconCGStream::VCentroidWeights & AddCentroidWeights();
  ViconCGStream::VLocalSegments & AddLocalSegments();
  ViconCGStream::VGlobalSegments & AddGlobalSegments();
  ViconCGStream::VGreyscaleBlobs & AddGreyscaleBlobs();
  ViconCGStream::VEdgePairs & AddEdgePairs();
  ViconCGStream::VForceFrame & AddForceFrame();
  ViconCGStream::VMomentFrame & AddMomentFrame();
  ViconCGStream::VCentreOfPressureFrame & AddCentreOfPressureFrame();
  ViconCGStream::VVoltageFrame & AddVoltageFrame();
  ViconCGStream::VCameraWand2d & AddCameraWand2d();
  ViconCGStream::VCameraWand3d & AddCameraWand3d();
  ViconCGStream::VEyeTrackerFrame & AddEyeTrackerFrame();
  ViconCGStream::VVideoFrame & AddVideoFrame();
};

//-------------------------------------------------------------------------------------------------

class VViconCGStreamClient
{
public:

  VViconCGStreamClient( std::weak_ptr< IViconCGStreamClientCallback > i_pCallback );
  ~VViconCGStreamClient();
  
  void Connect( const std::string & i_rHost, unsigned short i_Port );
  void Disconnect();
  void ReceiveMulticastData( std::string i_MulticastIPAddress, std::string i_LocalIPAddress, unsigned short i_Port );
  void StopReceivingMulticastData( );

  void SetStreaming( bool i_bStreaming );
  void SetRequiredObjects( std::set< ViconCGStreamType::Enum > & i_rRequiredObjects );
  void SetApexDeviceFeedback( const std::set< unsigned int > &i_rDeviceList );
  void SetServerToTransmitMulticast( std::string i_MulticastIPAddress, std::string i_ServerIPAddress, unsigned short i_Port );
  void StopMulticastTransmission( );
  void SetFilter( const ViconCGStream::VFilter & i_rFilter );

  enum EVideoHint { EPassThrough, EDecode };
  void SetVideoHint( EVideoHint i_VideoHint );

  bool NetworkLatency( double & o_rLatency ) const;

protected:

  void ClientThread();
  void Run();


  bool ReadObjectEnums( VCGStreamReaderWriter & i_rReaderWriter );
  bool WriteObjects( VCGStreamReaderWriter & i_rReaderWriter );
  bool ReadObjects( VCGStreamReaderWriter & i_rReaderWriter );

  void CopyObjects( const ViconCGStream::VContents & i_rContents, const VStaticObjects & i_rStaticObjects, VStaticObjects & o_rStaticObjects ) const;
  void CopyObjects( const ViconCGStream::VContents & i_rContents, const VDynamicObjects & i_rDynamicObjects, VDynamicObjects & o_rDynamicObjects ) const;
  
  void DecodeVideo( ViconCGStream::VVideoFrame & io_rVideoFrame );

  void OnConnect() const;
  void OnStaticObjects( std::shared_ptr< const VStaticObjects > i_pStaticObjects ) const;
  void OnDynamicObjects( std::shared_ptr< const VDynamicObjects > i_pDynamicObjects ) const;
  void OnDisconnect() const;
  
  boost::asio::ip::address_v4 FirstV4AddressFromString( const std::string & i_rAddress );
  
  std::weak_ptr< IViconCGStreamClientCallback > m_pCallback;

  boost::asio::io_service m_Service;
  std::shared_ptr< boost::asio::ip::tcp::socket > m_pSocket;
  std::shared_ptr< boost::asio::ip::udp::socket > m_pMulticastSocket;

  std::shared_ptr< boost::thread > m_pClientThread;

  boost::recursive_mutex m_Mutex;
  std::shared_ptr< const VStaticObjects > m_pStaticObjects;
  std::shared_ptr< const VDynamicObjects > m_pDynamicObjects;

  ViconCGStream::VObjectEnums m_ServerObjects;
  ViconCGStream::VObjectEnums m_RequiredObjects;
  ViconCGStream::VFilter      m_Filter;
  bool m_bEnumsChanged;
  bool m_bStreamingChanged;
  bool m_bStreaming;
  bool m_bHapticChanged;
  bool m_bFilterChanged;

  EVideoHint m_VideoHint;
  std::vector< unsigned char > m_ScratchVideo;
  std::set< unsigned int > m_OnDeviceList;

  std::vector< double > m_GetFrameTimes;
};
