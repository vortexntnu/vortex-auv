
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
#ifdef _WIN32
  #include <winerror.h>
#endif

#include "ViconCGStreamClient.h"
#include "IViconCGStreamClientCallback.h"

#include "ViconCGStreamBayer.h"
#include "CGStreamReaderWriter.h"

#include <ViconCGStream/ScopedReader.h>
#include <ViconCGStream/ScopedWriter.h>
#include <ViconCGStream/RequestFrame.h>
#include <ViconCGStream/StartMulticastSender.h>
#include <ViconCGStream/StopMulticastSender.h>
#include <ViconCGStream/Contents.h>
#include <ViconCGStream/ApexHaptics.h>

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/timer/timer.hpp>
#include <boost/chrono/include.hpp>

#include <numeric>

namespace
{
  // Do delay.
  const bool s_bNoDelay = true;

  // Send buffer size.
  const size_t s_SocketBufferSize = 4 * 1024 * 1024;

  const size_t s_GetFrameMaxSize = 20;
}

namespace bc = boost::chrono;
typedef bc::high_resolution_clock hrc;

//-------------------------------------------------------------------------------------------------

ViconCGStream::VCentroids & VDynamicObjects::AddCentroids()
{
  m_Centroids.push_back( ViconCGStream::VCentroids() );
  return m_Centroids.back();
}

ViconCGStream::VCentroidTracks & VDynamicObjects::AddCentroidTracks()
{
  m_CentroidTracks.push_back( ViconCGStream::VCentroidTracks() );
  return m_CentroidTracks.back();
}

ViconCGStream::VCentroidWeights & VDynamicObjects::AddCentroidWeights()
{
  m_CentroidWeights.push_back( ViconCGStream::VCentroidWeights() );
  return m_CentroidWeights.back();
}

ViconCGStream::VLocalSegments & VDynamicObjects::AddLocalSegments()
{
  m_LocalSegments.push_back( ViconCGStream::VLocalSegments() );
  return m_LocalSegments.back();
}

ViconCGStream::VGlobalSegments & VDynamicObjects::AddGlobalSegments()
{
  m_GlobalSegments.push_back( ViconCGStream::VGlobalSegments() );
  return m_GlobalSegments.back();
}

ViconCGStream::VGreyscaleBlobs & VDynamicObjects::AddGreyscaleBlobs()
{
  m_GreyscaleBlobs.push_back( ViconCGStream::VGreyscaleBlobs() );
  return m_GreyscaleBlobs.back();
}

ViconCGStream::VEdgePairs & VDynamicObjects::AddEdgePairs()
{
  m_EdgePairs.push_back( ViconCGStream::VEdgePairs() );
  return m_EdgePairs.back();
}

ViconCGStream::VForceFrame & VDynamicObjects::AddForceFrame()
{
  m_ForceFrames.push_back( ViconCGStream::VForceFrame() );
  return m_ForceFrames.back();
}

ViconCGStream::VMomentFrame & VDynamicObjects::AddMomentFrame()
{
  m_MomentFrames.push_back( ViconCGStream::VMomentFrame() );
  return m_MomentFrames.back();
}

ViconCGStream::VCentreOfPressureFrame & VDynamicObjects::AddCentreOfPressureFrame()
{
  m_CentreOfPressureFrames.push_back( ViconCGStream::VCentreOfPressureFrame() );
  return m_CentreOfPressureFrames.back();
}

ViconCGStream::VVoltageFrame & VDynamicObjects::AddVoltageFrame()
{
  m_VoltageFrames.push_back( ViconCGStream::VVoltageFrame() );
  return m_VoltageFrames.back();
}

ViconCGStream::VCameraWand2d & VDynamicObjects::AddCameraWand2d()
{
  m_CameraWand2d.push_back( ViconCGStream::VCameraWand2d() );
  return m_CameraWand2d.back();
}

ViconCGStream::VCameraWand3d & VDynamicObjects::AddCameraWand3d()
{
  m_CameraWand3d.push_back( ViconCGStream::VCameraWand3d() );
  return m_CameraWand3d.back();
}

ViconCGStream::VEyeTrackerFrame & VDynamicObjects::AddEyeTrackerFrame()
{
  m_EyeTrackerFrames.push_back( ViconCGStream::VEyeTrackerFrame() );
  return m_EyeTrackerFrames.back();
}

ViconCGStream::VVideoFrame & VDynamicObjects::AddVideoFrame()
{
  m_VideoFrames.push_back( std::shared_ptr< ViconCGStream::VVideoFrame >( new ViconCGStream::VVideoFrame() ) );
  return *m_VideoFrames.back();
}

//-------------------------------------------------------------------------------------------------

ViconCGStream::VCameraInfo & VStaticObjects::AddCameraInfo()
{
  m_CameraInfo.resize( m_CameraInfo.size() + 1 );
  return m_CameraInfo.back();
}

ViconCGStream::VCameraCalibrationInfo & VStaticObjects::AddCameraCalibrationInfo()
{
  m_CameraCalibrationInfo.resize( m_CameraCalibrationInfo.size() + 1 );
  return m_CameraCalibrationInfo.back();
}

ViconCGStream::VCameraCalibrationHealth & VStaticObjects::ResetCameraCalibrationHealth()
{
  m_pCameraCalibrationHealth.reset( new ViconCGStream::VCameraCalibrationHealth );
  return *m_pCameraCalibrationHealth;
}

ViconCGStream::VSubjectInfo & VStaticObjects::AddSubjectInfo()
{
  m_SubjectInfo.resize( m_SubjectInfo.size() + 1 );
  return m_SubjectInfo.back();
}

ViconCGStream::VSubjectTopology & VStaticObjects::AddSubjectTopology()
{
  m_SubjectTopology.resize( m_SubjectTopology.size() + 1 );
  return m_SubjectTopology.back();
}

ViconCGStream::VSubjectHealth & VStaticObjects::AddSubjectHealth()
{
  m_SubjectHealth.resize( m_SubjectHealth.size() + 1 );
  return m_SubjectHealth.back();
}

ViconCGStream::VObjectQuality & VStaticObjects::AddObjectQuality()
{
  m_ObjectQuality.resize( m_ObjectQuality.size() + 1 );
  return m_ObjectQuality.back();
}

ViconCGStream::VDeviceInfo & VStaticObjects::AddDeviceInfo()
{
  m_DeviceInfo.resize( m_DeviceInfo.size() + 1 );
  return m_DeviceInfo.back();
}

ViconCGStream::VDeviceInfoExtra & VStaticObjects::AddDeviceInfoExtra()
{
  m_DeviceInfoExtra.resize( m_DeviceInfoExtra.size() + 1 );
  return m_DeviceInfoExtra.back();
}

ViconCGStream::VChannelInfo & VStaticObjects::AddChannelInfo()
{
  m_ChannelInfo.resize( m_ChannelInfo.size() + 1 );
  return m_ChannelInfo.back();
}

ViconCGStream::VChannelInfoExtra & VStaticObjects::AddChannelInfoExtra()
{
  m_ChannelInfoExtra.resize( m_ChannelInfoExtra.size() + 1 );
  return m_ChannelInfoExtra.back();
}

ViconCGStream::VForcePlateInfo & VStaticObjects::AddForcePlateInfo()
{
  m_ForcePlateInfo.resize( m_ForcePlateInfo.size() + 1 );
  return m_ForcePlateInfo.back();
}

ViconCGStream::VEyeTrackerInfo & VStaticObjects::AddEyeTrackerInfo()
{
  m_EyeTrackerInfo.resize( m_EyeTrackerInfo.size() + 1 );
  return m_EyeTrackerInfo.back();
}

void VStaticObjects::BuildMaps()
{
  m_CameraMap.clear();
  m_CameraCalibrationMap.clear();
  m_SubjectMap.clear();
  m_SegmentMap.clear();
  m_DeviceMap.clear();
  m_ChannelMap.clear();

  for( unsigned int CameraIndex = 0; CameraIndex != m_CameraInfo.size(); ++CameraIndex )
  {
    ViconCGStream::VCameraInfo & rCameraInfo = m_CameraInfo[ CameraIndex ];
    m_CameraMap.insert( std::make_pair( rCameraInfo.m_CameraID, CameraIndex ) );
  }

  for( unsigned int CameraCalibrationIndex = 0; CameraCalibrationIndex != m_CameraCalibrationInfo.size(); ++CameraCalibrationIndex )
  {
    ViconCGStream::VCameraCalibrationInfo & rCameraCalibrationInfo = m_CameraCalibrationInfo[ CameraCalibrationIndex ];
    m_CameraCalibrationMap.insert( std::make_pair( rCameraCalibrationInfo.m_CameraID, CameraCalibrationIndex ) );
  }

  for( unsigned int SubjectIndex = 0; SubjectIndex != m_SubjectInfo.size(); ++SubjectIndex )
  {
    ViconCGStream::VSubjectInfo & rSubjectInfo = m_SubjectInfo[ SubjectIndex ];

    m_SubjectMap.insert( std::make_pair( rSubjectInfo.m_SubjectID, SubjectIndex ) );
    
    for( unsigned int SegmentIndex = 0; SegmentIndex != rSubjectInfo.m_Segments.size(); ++SegmentIndex )
    {
      TNestedIDPair SegmentID( rSubjectInfo.m_SubjectID, rSubjectInfo.m_Segments[ SegmentIndex ].m_SegmentID );
      m_SegmentMap.insert( std::make_pair( SegmentID, SegmentIndex ) );
    }
  }

  for( unsigned int DeviceIndex = 0; DeviceIndex != m_DeviceInfo.size(); ++DeviceIndex )
  {
    ViconCGStream::VDeviceInfo & rDeviceInfo = m_DeviceInfo[ DeviceIndex ];

    m_DeviceMap.insert( std::make_pair( rDeviceInfo.m_DeviceID, DeviceIndex ) );
  }

  for( unsigned int ChannelIndex = 0; ChannelIndex != m_ChannelInfo.size(); ++ChannelIndex )
  {
    ViconCGStream::VChannelInfo & rChannelInfo = m_ChannelInfo[ ChannelIndex ];

    TNestedIDPair ChannelID( rChannelInfo.m_DeviceID, rChannelInfo.m_ChannelID );
    m_ChannelMap.insert( std::make_pair( ChannelID, ChannelIndex ) );
  }
}

//-------------------------------------------------------------------------------------------------

VViconCGStreamClient::VViconCGStreamClient( std::weak_ptr< IViconCGStreamClientCallback > i_pCallback )
: m_pCallback( i_pCallback )
, m_bEnumsChanged( false )
, m_bStreamingChanged( true )
, m_bStreaming( false )
, m_bHapticChanged( false )
, m_bFilterChanged( false )
, m_VideoHint( EPassThrough )
{
  m_pSocket.reset( new boost::asio::ip::tcp::socket( m_Service ) );
}

VViconCGStreamClient::~VViconCGStreamClient()
{
  Disconnect();
}

void VViconCGStreamClient::Connect( const std::string & i_rHost, unsigned short i_Port )
{
  if( m_pMulticastSocket )
  {
    return ;
  }

  boost::asio::ip::tcp::resolver Resolver( m_Service );
  boost::asio::ip::tcp::resolver::query Query( i_rHost, "" );
  
  boost::system::error_code Error;
  boost::asio::ip::tcp::resolver::iterator It = Resolver.resolve( Query, Error );
  boost::asio::ip::tcp::resolver::iterator End;
  
  if( Error )
  {
    OnDisconnect();
    return;
  }
  
  bool bConnected = false;
  for( ; It != End; ++It )
  {
    Error = boost::system::error_code();
    boost::asio::ip::tcp::endpoint EndPoint( *It );

    // Currently we only handle IPv4
    // This has to be explicitly handled, otherwise the socket can bind to a v6 endpoint and then fail
    // to connect to a v4 endpoint - which is a bit rubbish.
    if( ! EndPoint.address().is_v4() )
    {
      continue;
    }

    EndPoint.port( i_Port );

    m_pSocket->open( EndPoint.protocol(), Error );
    if( ! Error )
    {
      m_pSocket->set_option( boost::asio::socket_base::reuse_address( true ), Error );
    }
    if( ! Error )
    {
      m_pSocket->set_option( boost::asio::ip::tcp::no_delay( s_bNoDelay ), Error );
    }
    if( ! Error )
    {
      boost::system::error_code LocalError;
      m_pSocket->set_option( boost::asio::socket_base::receive_buffer_size( s_SocketBufferSize ), LocalError );
      // On Mac OS X, setting the buffer size resulted in an error "no_buffer_space"
#if !defined(__APPLE__)
      Error = LocalError;
#endif
    }
    if( ! Error )
    {
      m_pSocket->connect( EndPoint, Error );
    }

    if( Error )
    {
      m_pSocket->close();
      std::stringstream Strm;
      Strm << Error;
      std::string ErrorText = Strm.str();
      // std::cerr << Error << std::endl;
    }
    if( ! Error )
    {
      bConnected = true;
      break;
    }
  }
  
  if( bConnected )
  {
    OnConnect();
  }
  else
  {
    OnDisconnect();
    return;
  }

  m_pClientThread.reset(new boost::thread(boost::bind(&VViconCGStreamClient::ClientThread, this)));
}

void VViconCGStreamClient::Disconnect()
{
  boost::system::error_code DontCareError;
  m_pSocket->shutdown( boost::asio::ip::tcp::socket::shutdown_both, DontCareError );
  m_pSocket->close();
  if( m_pMulticastSocket )
  {
    boost::system::error_code DontCareError2;
    m_pMulticastSocket->shutdown( boost::asio::ip::tcp::socket::shutdown_both, DontCareError2 );
    m_pMulticastSocket->close();
  }
  
  if( m_pClientThread )
  {
    m_pClientThread->join();
    m_pClientThread.reset();
  }
  m_pMulticastSocket.reset();
}

void VViconCGStreamClient::ReceiveMulticastData( std::string i_MulticastIPAddress, std::string i_LocalIPAddress, unsigned short i_Port )
{
  boost::asio::ip::address_v4 MulticastAddress = FirstV4AddressFromString( i_MulticastIPAddress );
  boost::asio::ip::address_v4 LocalAddress = FirstV4AddressFromString( i_LocalIPAddress );
  boost::asio::ip::udp::endpoint LocalEndpoint( LocalAddress, i_Port );

  boost::system::error_code Error;

  if( m_pSocket->is_open() )
  {
    Disconnect();
  }

  if( !MulticastAddress.is_multicast() && ( MulticastAddress.to_ulong() != 0xFFFFFFFF ) )
  {
    OnDisconnect();
    return ;
  }

  std::shared_ptr< boost::asio::ip::udp::socket > pMulticastSocket(
    new boost::asio::ip::udp::socket( m_Service, LocalEndpoint.protocol() ) );
  if( Error )
  {
    OnDisconnect();
    return ;
  }
  pMulticastSocket->set_option( boost::asio::socket_base::reuse_address( true ), Error );
  if( Error )
  {
    OnDisconnect();
    return ;
  }
  pMulticastSocket->set_option( boost::asio::socket_base::receive_buffer_size( 128 * 1024 ), Error );
  if( Error )
  {
    OnDisconnect();
    return ;
  }
#ifdef WIN32
  pMulticastSocket->bind( LocalEndpoint, Error );
#else
  if( MulticastAddress.is_multicast() )
  {
    boost::asio::ip::udp::endpoint MulticastEndpoint( MulticastAddress, i_Port );
    pMulticastSocket->bind( MulticastEndpoint, Error );
  }
  else
  {
    boost::asio::ip::udp::endpoint BroadcastEndpoint( MulticastAddress, i_Port );
    pMulticastSocket->bind( BroadcastEndpoint, Error );
  }
#endif
  if( Error )
  {
    OnDisconnect();
    return ;
  }
  if( MulticastAddress.is_multicast() )
  {
    pMulticastSocket->set_option( boost::asio::ip::multicast::join_group( MulticastAddress, LocalAddress ), Error );
  }
  if( Error )
  {
    OnDisconnect();
    return ;
  }

  m_pMulticastSocket = pMulticastSocket;

  m_pClientThread.reset( new boost::thread( boost::bind( &VViconCGStreamClient::ClientThread, this ) ) );
}

void VViconCGStreamClient::StopReceivingMulticastData( )
{
  Disconnect();
}

void VViconCGStreamClient::SetStreaming( bool i_bStreaming )
{
  boost::recursive_mutex::scoped_lock Lock( m_Mutex );
  m_bStreaming = i_bStreaming;
  m_bStreamingChanged = true;
}

void VViconCGStreamClient::SetFilter( const ViconCGStream::VFilter & i_rFilter )
{
  boost::recursive_mutex::scoped_lock Lock( m_Mutex );
  m_Filter = i_rFilter;
  m_bFilterChanged = true;
}

void VViconCGStreamClient::SetRequiredObjects( std::set< ViconCGStreamType::Enum > & i_rRequiredObjects )
{
  boost::recursive_mutex::scoped_lock Lock( m_Mutex );
  m_RequiredObjects.m_Enums = i_rRequiredObjects;
  m_RequiredObjects.m_Enums.insert( ViconCGStreamEnum::Contents );
  m_bEnumsChanged = true;
}

void VViconCGStreamClient::SetApexDeviceFeedback( const std::set< unsigned int > &i_rDeviceList )
{
  if( i_rDeviceList != m_OnDeviceList )
  {
    m_bHapticChanged = true;
    m_OnDeviceList = i_rDeviceList;
  }
}

void VViconCGStreamClient::SetServerToTransmitMulticast( std::string i_MulticastIPAddress, std::string i_ServerIPAddress, unsigned short i_Port )
{
  boost::recursive_mutex::scoped_lock Lock( m_Mutex );

  VCGStreamReaderWriter ReaderWriter( m_pSocket );
  {
    ViconCGStreamIO::VScopedWriter Objects( ReaderWriter );

    ViconCGStream::VStartMulticastSender RequestMulticast;

    boost::asio::ip::address_v4 MulticastAddress = FirstV4AddressFromString( i_MulticastIPAddress );
    boost::asio::ip::address_v4 ServerAddress = FirstV4AddressFromString( i_ServerIPAddress );

    RequestMulticast.m_MulticastIpAddress = MulticastAddress.to_ulong();
    RequestMulticast.m_SourceIpAddress = ServerAddress.to_ulong();
    RequestMulticast.m_Port = i_Port;

    Objects.Write( RequestMulticast );
  }
  ReaderWriter.Flush();
}

void VViconCGStreamClient::StopMulticastTransmission( )
{
  boost::recursive_mutex::scoped_lock Lock( m_Mutex );

  VCGStreamReaderWriter ReaderWriter( m_pSocket );
  {
    ViconCGStreamIO::VScopedWriter Objects( ReaderWriter );

    ViconCGStream::VStopMulticastSender StopMulticast;

    Objects.Write( StopMulticast );
  }
  ReaderWriter.Flush();
}

void VViconCGStreamClient::SetVideoHint( EVideoHint i_VideoHint )
{
  boost::recursive_mutex::scoped_lock Lock( m_Mutex );
  m_VideoHint = i_VideoHint;  
}

static void Intersect( ViconCGStream::VObjectEnums & i_rServer, ViconCGStream::VObjectEnums & i_rClient, ViconCGStream::VObjectEnums & i_rOutput )
{
  std::set< ViconCGStreamType::Enum > & i_rA = i_rServer.m_Enums;
  std::set< ViconCGStreamType::Enum > & i_rB = i_rClient.m_Enums;
  std::set< ViconCGStreamType::Enum > & i_rC = i_rOutput.m_Enums;
  i_rC.clear();

  std::set< ViconCGStreamType::Enum >::iterator It = i_rA.begin();
  std::set< ViconCGStreamType::Enum >::iterator End = i_rA.end();
  for( ; It != End; ++It )
  {
    if( i_rB.find( *It ) != i_rB.end() )
    {
      i_rC.insert( *It );
    }
  }
}

bool VViconCGStreamClient::NetworkLatency( double & o_rLatency ) const
{
  if (m_GetFrameTimes.size() < s_GetFrameMaxSize)
  {
    return false;
  }

  o_rLatency = std::accumulate(m_GetFrameTimes.begin(), m_GetFrameTimes.end(), 0.0) / m_GetFrameTimes.size() ;

  return true;
}

void VViconCGStreamClient::ClientThread()
{
  m_pStaticObjects.reset();
  m_pDynamicObjects.reset();
  
  // We are timing our get_frame loop to determine network latency
  size_t FrameTimeIndex = 0;
  m_GetFrameTimes.resize(s_GetFrameMaxSize);

  if( m_pMulticastSocket )
  {
    // Multicast receive only
    VCGStreamReaderWriter ReaderWriter( m_pMulticastSocket );


    for( ;; )
    {
      hrc::time_point Before = hrc::now();
      
      if( !ReadObjects( ReaderWriter ) )
      {
        break;
      }

      hrc::time_point After = hrc::now();
      bc::duration< double, boost::milli > TimeMS = ( After - Before ) / 2 ;
      m_GetFrameTimes[ FrameTimeIndex++ % s_GetFrameMaxSize ] = TimeMS.count();
    }
  }
  else
  {
    // TCP read and write
    VCGStreamReaderWriter ReaderWriter( m_pSocket );
    if( !ReadObjectEnums( ReaderWriter ) )
    {
      OnDisconnect();
      return;
    }

    for( ;; )
    {

      hrc::time_point Before = hrc::now();

      if( !WriteObjects( ReaderWriter ) )
      {
        break;
      }
      if( !ReadObjects( ReaderWriter ) )
      {
        break;
      }
      hrc::time_point After = hrc::now();
      bc::duration< double, boost::milli > TimeMS = (After - Before) / 2;
      m_GetFrameTimes[FrameTimeIndex++ % s_GetFrameMaxSize] = TimeMS.count();
    }

    OnDisconnect();
  }  
}

bool VViconCGStreamClient::ReadObjectEnums( VCGStreamReaderWriter & i_rReaderWriter )
{
  if( !i_rReaderWriter.Fill() )
  {
    return false;
  }

  ViconCGStreamIO::VScopedReader Objects( i_rReaderWriter );
  if( Objects.Enum() != ViconCGStreamEnum::Objects )
  {
    return false;
  }

  while( Objects.Ok() )
  {
    ViconCGStreamIO::VScopedReader Object( i_rReaderWriter );
  
    if( Object.Enum() == ViconCGStreamEnum::ObjectEnums )
    {
      if( !Object.Read( m_ServerObjects ) )
      {
        return false;
      }
      
      m_bEnumsChanged = true;
      return true;
    }
  }

  return false;
}

bool VViconCGStreamClient::WriteObjects(  VCGStreamReaderWriter & i_rReaderWriter  )
{
  boost::recursive_mutex::scoped_lock Lock( m_Mutex );
  bool bWriteObjects = m_bStreamingChanged || m_bEnumsChanged || !m_bStreaming || m_bHapticChanged;

  if( bWriteObjects )
  {
    i_rReaderWriter.Clear();
    ViconCGStreamIO::VScopedWriter Objects( i_rReaderWriter );

    if( m_bEnumsChanged )
    {
      ViconCGStream::VObjectEnums Output;
      Intersect( m_ServerObjects, m_RequiredObjects, Output );    
      Objects.Write( Output );
    }
    
    if( !m_bStreaming || m_bStreamingChanged )
    {
      ViconCGStream::VRequestFrame RequestFrame;
      RequestFrame.m_bStreaming = m_bStreaming;
      Objects.Write( RequestFrame );
    }

    // write haptic feedback
    if( m_bHapticChanged )
    {    
      ViconCGStream::VApexHaptics HapticFeedback;
      //set haptic on and off devices
      HapticFeedback.m_OnDevicesList = m_OnDeviceList;
      Objects.Write( HapticFeedback );
      m_bHapticChanged = false;
    }

    if ( m_bFilterChanged )
    {
      Objects.Write( m_Filter );
      m_bFilterChanged = false;
    }
  }
  else
  {
    return true;
  }

  m_bEnumsChanged = false;
  m_bStreamingChanged = false;

  return i_rReaderWriter.Flush();
}

void VViconCGStreamClient::CopyObjects( const ViconCGStream::VContents & i_rContents, const VStaticObjects & i_rStaticObjects, VStaticObjects & o_rStaticObjects ) const
{
  typedef std::set< ViconCGStreamType::Enum > TEnums;
  
  TEnums::const_iterator It = i_rContents.m_EnumsUnchanged.begin();
  TEnums::const_iterator End = i_rContents.m_EnumsUnchanged.end();
  
  for( ; It != End; ++It )
  {
    switch( *It )
    {
    case ViconCGStreamEnum::ApplicationInfo:
      o_rStaticObjects.m_ApplicationInfo = i_rStaticObjects.m_ApplicationInfo;
      break;
    case ViconCGStreamEnum::StreamInfo:
      o_rStaticObjects.m_StreamInfo = i_rStaticObjects.m_StreamInfo;
      break;
    case ViconCGStreamEnum::CameraInfo:
      o_rStaticObjects.m_CameraInfo = i_rStaticObjects.m_CameraInfo;
      break;
    case ViconCGStreamEnum::CameraCalibrationInfo:
      o_rStaticObjects.m_CameraCalibrationInfo = i_rStaticObjects.m_CameraCalibrationInfo;
      break;
    case ViconCGStreamEnum::CameraCalibrationHealth:
      o_rStaticObjects.m_pCameraCalibrationHealth = i_rStaticObjects.m_pCameraCalibrationHealth;
      break;
    case ViconCGStreamEnum::SubjectInfo:
      o_rStaticObjects.m_SubjectInfo = i_rStaticObjects.m_SubjectInfo;
      break;
    case ViconCGStreamEnum::SubjectTopology:
      o_rStaticObjects.m_SubjectTopology = i_rStaticObjects.m_SubjectTopology;
      break;
    case ViconCGStreamEnum::SubjectHealth:
      o_rStaticObjects.m_SubjectHealth = i_rStaticObjects.m_SubjectHealth;
      break;
    case ViconCGStreamEnum::ObjectQuality:
      o_rStaticObjects.m_ObjectQuality = i_rStaticObjects.m_ObjectQuality;
      break;
    case ViconCGStreamEnum::DeviceInfo:
      o_rStaticObjects.m_DeviceInfo = i_rStaticObjects.m_DeviceInfo;
      break;
    case ViconCGStreamEnum::DeviceInfoExtra:
      o_rStaticObjects.m_DeviceInfoExtra = i_rStaticObjects.m_DeviceInfoExtra;
      break;
    case ViconCGStreamEnum::ChannelInfo:
      o_rStaticObjects.m_ChannelInfo = i_rStaticObjects.m_ChannelInfo;
      break;
    case ViconCGStreamEnum::ChannelInfoExtra:
      o_rStaticObjects.m_ChannelInfoExtra = i_rStaticObjects.m_ChannelInfoExtra;
      break;
    case ViconCGStreamEnum::ForcePlateInfo:
      o_rStaticObjects.m_ForcePlateInfo = i_rStaticObjects.m_ForcePlateInfo;
      break;
    case ViconCGStreamEnum::EyeTrackerInfo:
      o_rStaticObjects.m_EyeTrackerInfo = i_rStaticObjects.m_EyeTrackerInfo;
      break;
    }
  }
}

void VViconCGStreamClient::CopyObjects( const ViconCGStream::VContents & i_rContents, const VDynamicObjects & i_rDynamicObjects, VDynamicObjects & o_rDynamicObjects ) const
{
  typedef std::set< ViconCGStreamType::Enum > TEnums;
  
  TEnums::const_iterator It = i_rContents.m_EnumsUnchanged.begin();
  TEnums::const_iterator End = i_rContents.m_EnumsUnchanged.end();
  
  for( ; It != End; ++It )
  {
    switch( *It )
    {
    case ViconCGStreamEnum::FrameInfo:
      o_rDynamicObjects.m_FrameInfo = i_rDynamicObjects.m_FrameInfo;
      break;
    case ViconCGStreamEnum::HardwareFrameInfo:
      o_rDynamicObjects.m_HardwareFrameInfo = i_rDynamicObjects.m_HardwareFrameInfo;
      break;
    case ViconCGStreamEnum::Timecode:
      o_rDynamicObjects.m_Timecode = i_rDynamicObjects.m_Timecode;
      break;
    case ViconCGStreamEnum::LatencyInfo:
      o_rDynamicObjects.m_LatencyInfo = i_rDynamicObjects.m_LatencyInfo;
      break;
    case ViconCGStreamEnum::Centroids:
      o_rDynamicObjects.m_Centroids = i_rDynamicObjects.m_Centroids;
      break;
    case ViconCGStreamEnum::CentroidWeights:
      o_rDynamicObjects.m_CentroidWeights = i_rDynamicObjects.m_CentroidWeights;
      break;
    case ViconCGStreamEnum::LabeledRecons:
      o_rDynamicObjects.m_LabeledRecons = i_rDynamicObjects.m_LabeledRecons;
      break;
    case ViconCGStreamEnum::UnlabeledRecons:
      o_rDynamicObjects.m_UnlabeledRecons = i_rDynamicObjects.m_UnlabeledRecons;
      break;
    case ViconCGStreamEnum::LabeledReconRayAssignments:
      o_rDynamicObjects.m_LabeledRayAssignments = i_rDynamicObjects.m_LabeledRayAssignments;
      break;
    case ViconCGStreamEnum::GlobalSegments:
      o_rDynamicObjects.m_GlobalSegments = i_rDynamicObjects.m_GlobalSegments;
      break;
    case ViconCGStreamEnum::LocalSegments:
      o_rDynamicObjects.m_LocalSegments = i_rDynamicObjects.m_LocalSegments;
      break;
    case ViconCGStreamEnum::GreyscaleBlobs:
      o_rDynamicObjects.m_GreyscaleBlobs = i_rDynamicObjects.m_GreyscaleBlobs;
      break;
    case ViconCGStreamEnum::EdgePairs:
      o_rDynamicObjects.m_EdgePairs = i_rDynamicObjects.m_EdgePairs;
      break;
    case ViconCGStreamEnum::CameraWand2d:
      o_rDynamicObjects.m_CameraWand2d = i_rDynamicObjects.m_CameraWand2d;
      break;
    case ViconCGStreamEnum::CameraWand3d:
      o_rDynamicObjects.m_CameraWand3d = i_rDynamicObjects.m_CameraWand3d;
      break;
    case ViconCGStreamEnum::VideoFrame:
      o_rDynamicObjects.m_VideoFrames = i_rDynamicObjects.m_VideoFrames;
      break;
    case ViconCGStreamEnum::EyeTrackerFrame:
      o_rDynamicObjects.m_EyeTrackerFrames = i_rDynamicObjects.m_EyeTrackerFrames;
      break;
    case ViconCGStreamEnum::FrameRateInfo:
      o_rDynamicObjects.m_FrameRateInfo = i_rDynamicObjects.m_FrameRateInfo;
      break;
    }
  }
}

bool VViconCGStreamClient::ReadObjects( VCGStreamReaderWriter & i_rReaderWriter )
{
  if( !i_rReaderWriter.Fill() )
  {
    return false;
  }

  ViconCGStreamIO::VScopedReader Objects( i_rReaderWriter );
  if( Objects.Enum() != ViconCGStreamEnum::Objects )
  {
    return false;
  }

  std::shared_ptr< VStaticObjects > pStaticObjects;
  std::shared_ptr< VDynamicObjects > pDynamicObjects;

  bool bContents = false;
  ViconCGStream::VContents Contents;

  while( Objects.Ok() )
  {
    ViconCGStreamIO::VScopedReader Object( i_rReaderWriter );

    switch( Object.Enum() )
    {
    case ViconCGStreamEnum::Contents:
      if( !Object.Read( Contents ) )
      {
        return false;
      }
      
      bContents = true;
      
      break;
    case ViconCGStreamEnum::StreamInfo:
      if ( !pStaticObjects ) pStaticObjects.reset( new VStaticObjects() );
      if( !Object.Read( pStaticObjects->m_StreamInfo ) )
      {
        return false;
      }
      break;
    case ViconCGStreamEnum::ApplicationInfo:
    {
      ViconCGStream::VApplicationInfo AppInfo;
      if( !Object.Read( AppInfo ) )
      {
        return false;
      }

      if ( !pStaticObjects ) pStaticObjects.reset( new VStaticObjects() );
      pStaticObjects->m_ApplicationInfo = AppInfo;
      break;
    }
    case ViconCGStreamEnum::SubjectInfo:
      if ( !pStaticObjects ) pStaticObjects.reset( new VStaticObjects() );
      if( !Object.Read( pStaticObjects->AddSubjectInfo() ) )
      {
        return false;
      }

      break;
    case ViconCGStreamEnum::SubjectTopology:
      if ( !pStaticObjects ) pStaticObjects.reset( new VStaticObjects() );
      if( !Object.Read( pStaticObjects->AddSubjectTopology() ) )
      {
        return false;
      }

      break;
    case ViconCGStreamEnum::SubjectHealth:
      if ( !pStaticObjects ) pStaticObjects.reset( new VStaticObjects() );
      if( !Object.Read( pStaticObjects->AddSubjectHealth() ) )
      {
        return false;
      }

      break;
    case ViconCGStreamEnum::ObjectQuality:
      if ( !pStaticObjects ) pStaticObjects.reset( new VStaticObjects() );
      if( !Object.Read( pStaticObjects->AddObjectQuality() ) )
      {
        return false;
      }

      break;
    case ViconCGStreamEnum::CameraInfo:
      if ( !pStaticObjects ) pStaticObjects.reset( new VStaticObjects() );
      if( !Object.Read( pStaticObjects->AddCameraInfo() ) )
      {
        return false;
      }
      
      break;
    case ViconCGStreamEnum::CameraCalibrationInfo:
      if ( !pStaticObjects ) pStaticObjects.reset( new VStaticObjects() );
      if( !Object.Read( pStaticObjects->AddCameraCalibrationInfo() ) )
      {
        return false;
      }
      
      break;
    case ViconCGStreamEnum::CameraCalibrationHealth:
      if ( !pStaticObjects ) pStaticObjects.reset( new VStaticObjects() );
      if( !Object.Read( pStaticObjects->ResetCameraCalibrationHealth() ) )
      {
        return false;
      }
      
      break;      
    case ViconCGStreamEnum::DeviceInfo:
      if ( !pStaticObjects ) pStaticObjects.reset( new VStaticObjects() );
      if( !Object.Read( pStaticObjects->AddDeviceInfo() ) )
      {
        return false;
      }
      
      break;
    case ViconCGStreamEnum::DeviceInfoExtra:
      if ( !pStaticObjects ) pStaticObjects.reset( new VStaticObjects() );
      if( !Object.Read( pStaticObjects->AddDeviceInfoExtra() ) )
      {
        return false;
      }
      
      break;
    case ViconCGStreamEnum::ChannelInfo:
      if ( !pStaticObjects ) pStaticObjects.reset( new VStaticObjects() );
      if( !Object.Read( pStaticObjects->AddChannelInfo() ) )
      {
        return false;
      }

      break;
    case ViconCGStreamEnum::ChannelInfoExtra:
      if ( !pStaticObjects ) pStaticObjects.reset( new VStaticObjects() );
      if( !Object.Read( pStaticObjects->AddChannelInfoExtra() ) )
      {
        return false;
      }

      break;
    case ViconCGStreamEnum::ForcePlateInfo:
      if ( !pStaticObjects ) pStaticObjects.reset( new VStaticObjects() );
      if( !Object.Read( pStaticObjects->AddForcePlateInfo() ) )
      {
        return false;
      }

      break;
    case ViconCGStreamEnum::EyeTrackerInfo:
      if ( !pStaticObjects ) pStaticObjects.reset( new VStaticObjects() );
      if( !Object.Read( pStaticObjects->AddEyeTrackerInfo() ) )
      {
        return false;
      }

      break;    
    case ViconCGStreamEnum::FrameInfo:
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      if ( !Object.Read( pDynamicObjects->m_FrameInfo ) )
      {
        return false;
      }
  
      break;
    case ViconCGStreamEnum::HardwareFrameInfo:
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      if( !Object.Read( pDynamicObjects->m_HardwareFrameInfo ) )
      {
        return false;
      }
  
      break;
    case ViconCGStreamEnum::Timecode:
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      if( !Object.Read( pDynamicObjects->m_Timecode ) )
      {
        return false;
      }

      break;
    case ViconCGStreamEnum::LatencyInfo:
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      if( !Object.Read( pDynamicObjects->m_LatencyInfo ) )
      {
        return false;
      }
      
      break;
    case ViconCGStreamEnum::Centroids:
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      if( !Object.Read( pDynamicObjects->AddCentroids() ) )
      {
        return false;
      }
      
      break;
    case ViconCGStreamEnum::CentroidTracks:
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      if( !Object.Read( pDynamicObjects->AddCentroidTracks() ) )
      {
        return false;
      }
      
      break;
    case ViconCGStreamEnum::CentroidWeights:
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      if( !Object.Read( pDynamicObjects->AddCentroidWeights() ) )
      {
        return false;
      }

      break;
    case ViconCGStreamEnum::VideoFrame:
      {
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      ViconCGStream::VVideoFrame & rVideoFrame = pDynamicObjects->AddVideoFrame();
      if( !Object.Read( rVideoFrame ) )
      {
        return false;
      }

      if( m_VideoHint == EDecode )
      {
        DecodeVideo( rVideoFrame );
      }
      }
      break;
    case ViconCGStreamEnum::LabeledRecons:
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      if( !Object.Read( pDynamicObjects->m_LabeledRecons ) )
      {
        return false;
      }
      
      break;
    case ViconCGStreamEnum::UnlabeledRecons:
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      if( !Object.Read( pDynamicObjects->m_UnlabeledRecons ) )
      {
        return false;
      }
      
      break;
    case ViconCGStreamEnum::LabeledReconRayAssignments:
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      if( !Object.Read( pDynamicObjects->m_LabeledRayAssignments ) )
      {
        return false;
      }

      break;
    case ViconCGStreamEnum::GlobalSegments:
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      if( !Object.Read( pDynamicObjects->AddGlobalSegments() ) )
      {
        return false;
      }
      
      break;
    case ViconCGStreamEnum::LocalSegments:
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      if( !Object.Read( pDynamicObjects->AddLocalSegments() ) )
      {
        return false;
      }
      
      break;
    case ViconCGStreamEnum::GreyscaleBlobs:
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      if( !Object.Read( pDynamicObjects->AddGreyscaleBlobs() ) )
      {
        return false;
      }
            
      break;
    case ViconCGStreamEnum::EdgePairs:
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      if( !Object.Read( pDynamicObjects->AddEdgePairs() ) )
      {
        return false;
      }
      
      break;
    case ViconCGStreamEnum::ForceFrame:
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      if( !Object.Read( pDynamicObjects->AddForceFrame() ) )
      {
        return false;
      }
      
      break;
    case ViconCGStreamEnum::MomentFrame:
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      if( !Object.Read( pDynamicObjects->AddMomentFrame() ) )
      {
        return false;
      }
      
      break;
    case ViconCGStreamEnum::CentreOfPressureFrame:
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      if( !Object.Read( pDynamicObjects->AddCentreOfPressureFrame() ) )
      {
        return false;
      }
      
      break;
    case ViconCGStreamEnum::VoltageFrame:
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      if( !Object.Read( pDynamicObjects->AddVoltageFrame() ) )
      {
        return false;
      }

      break;
    case ViconCGStreamEnum::CameraWand2d:
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      if( !Object.Read( pDynamicObjects->AddCameraWand2d() ) )
      {
        return false;
      }

      break;
    case ViconCGStreamEnum::CameraWand3d:
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      if( !Object.Read( pDynamicObjects->AddCameraWand3d() ) )
      {
        return false;
      }

      break;
    case ViconCGStreamEnum::EyeTrackerFrame:
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      if( !Object.Read( pDynamicObjects->AddEyeTrackerFrame() ) )
      {
        return false;
      }

      break;

    case ViconCGStreamEnum::FrameRateInfo:
      if ( !pDynamicObjects ) pDynamicObjects.reset( new VDynamicObjects() );
      if( !Object.Read( pDynamicObjects->m_FrameRateInfo ) )
      {
        return false;
      }
      
      break;
    }
  }

  if( bContents && m_pStaticObjects && pStaticObjects )
  {
    CopyObjects( Contents, *m_pStaticObjects, *pStaticObjects );
  }

  if( bContents && m_pDynamicObjects && pDynamicObjects )
  {
    CopyObjects( Contents, *m_pDynamicObjects, *pDynamicObjects );
  }

  if ( pStaticObjects )
  {
    m_pStaticObjects = pStaticObjects;
    OnStaticObjects( pStaticObjects );
  }

  if ( pDynamicObjects )
  {
    m_pDynamicObjects = pDynamicObjects;
    OnDynamicObjects( pDynamicObjects );
  }
  return true;
}

//-------------------------------------------------------------------------------------------------

void VViconCGStreamClient::DecodeVideo( ViconCGStream::VVideoFrame & io_rVideoFrame )
{
  if( io_rVideoFrame.m_Format == ViconCGStream::VVideoFrame::EBayerGB8 )
  {
    m_ScratchVideo.resize( io_rVideoFrame.m_Width * io_rVideoFrame.m_Height * 3 );
    VViconCGStreamBayer::BayerGBToBGR( io_rVideoFrame.m_Width, io_rVideoFrame.m_Height, &io_rVideoFrame.m_VideoData[ 0 ], &m_ScratchVideo[ 0 ] );
  }
  else
  if( io_rVideoFrame.m_Format == ViconCGStream::VVideoFrame::EBayerBG8 )
  {
    m_ScratchVideo.resize( io_rVideoFrame.m_Width * io_rVideoFrame.m_Height * 3 );
    VViconCGStreamBayer::BayerBGToBGR( io_rVideoFrame.m_Width, io_rVideoFrame.m_Height, &io_rVideoFrame.m_VideoData[ 0 ], &m_ScratchVideo[ 0 ] );
  }
  else
  if( io_rVideoFrame.m_Format == ViconCGStream::VVideoFrame::EBayerRG8 )
  {
    m_ScratchVideo.resize( io_rVideoFrame.m_Width * io_rVideoFrame.m_Height * 3 );
    VViconCGStreamBayer::BayerRGToBGR( io_rVideoFrame.m_Width, io_rVideoFrame.m_Height, &io_rVideoFrame.m_VideoData[ 0 ], &m_ScratchVideo[ 0 ] );
  }
  else
  {
    return;
  }

  io_rVideoFrame.m_Format = ViconCGStream::VVideoFrame::EBGR888;
  io_rVideoFrame.m_VideoData = m_ScratchVideo;
}

//-------------------------------------------------------------------------------------------------

void VViconCGStreamClient::OnConnect() const
{
  std::shared_ptr< IViconCGStreamClientCallback > pCallback = m_pCallback.lock();
  if( pCallback )
  {
    pCallback->OnConnect();
  }
}

void VViconCGStreamClient::OnStaticObjects( std::shared_ptr< const VStaticObjects > i_pStaticObjects ) const
{
  std::shared_ptr< IViconCGStreamClientCallback > pCallback = m_pCallback.lock();
  if( pCallback )
  {
    pCallback->OnStaticObjects( i_pStaticObjects );
  }
}

void VViconCGStreamClient::OnDynamicObjects( std::shared_ptr< const VDynamicObjects > i_pDynamicObjects ) const
{
  std::shared_ptr< IViconCGStreamClientCallback > pCallback = m_pCallback.lock();
  if( pCallback )
  {
    pCallback->OnDynamicObjects( i_pDynamicObjects );
  }
}

void VViconCGStreamClient::OnDisconnect() const
{
  std::shared_ptr< IViconCGStreamClientCallback > pCallback = m_pCallback.lock();
  if( pCallback )
  {
    pCallback->OnDisconnect();
  }
}

boost::asio::ip::address_v4 VViconCGStreamClient::FirstV4AddressFromString( const std::string & i_rAddress )
{
  boost::system::error_code Error;
  boost::asio::ip::address_v4 Address = boost::asio::ip::address_v4::from_string( i_rAddress, Error );
  if( ! Error )
  {
    return Address;
  }

  boost::asio::ip::tcp::resolver Resolver( m_Service );
  boost::asio::ip::tcp::resolver::query Query( i_rAddress, "" );
  
  boost::asio::ip::tcp::resolver::iterator It = Resolver.resolve( Query, Error );
  boost::asio::ip::tcp::resolver::iterator End;
  
  if( ! Error )
  {
    for( ; It != End; ++It )
    {
      Error = boost::system::error_code();
      boost::asio::ip::tcp::endpoint EndPoint( *It );

      // Currently we only handle IPv4
      if( EndPoint.address().is_v4() )
      {
        return EndPoint.address().to_v4();
      }
    }
  }

  return boost::asio::ip::address_v4();
}
