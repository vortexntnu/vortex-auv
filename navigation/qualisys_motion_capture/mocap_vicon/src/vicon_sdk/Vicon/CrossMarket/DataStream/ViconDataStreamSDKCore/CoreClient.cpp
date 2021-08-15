
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
#include "ClientUtils.h"

#include "CoreClient.h"

#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

#include <ViconCGStreamClient/ViconCGStreamClient.h>

#include "ViconDataStreamSDKCoreVersion.h"

typedef std::pair< ViconCGStreamType::UInt64, ViconCGStreamType::UInt64 > TPeriod;


using namespace ClientUtils;

namespace
{
  // Timeout for wait-for-frame operations (in milliseconds) 
  static const unsigned int s_WaitFrameTimeout = 1000;

  // [ Start tick, End tick ) pair.
  TPeriod GetFramePeriod( const ViconCGStreamClientSDK::ICGFrameState & i_rFrame )
  {
    const ViconCGStreamType::Int64 FramePeriod = i_rFrame.m_Stream.m_FramePeriod;
    const ViconCGStreamType::Int64 FrameID = i_rFrame.m_Frame.m_FrameID;
    TPeriod Period;
    Period.first = FramePeriod * FrameID;
    Period.second = Period.first + FramePeriod;
    return Period;
  }
  
  /// Calculates the index to the next sample.
  /// e.g. given samples |  0  |  1  |  2  |
  ///                    ^ ^ ^ ^ ^ ^ ^ ^ ^ ^
  ///      returns       0 1 1 1 2 2 2 3 3 3
  /// \param i_Tick Ticks since sample 0.
  /// \param i_SamplePeriod Sample period in ticks.
  /// \return The index in device samples
  ViconCGStreamType::UInt64 GetNextSampleIndex( const ViconCGStreamType::UInt64 i_SystemTick,
                                                const ViconCGStreamType::UInt64 i_SampleTick,
                                                const ViconCGStreamType::UInt64 i_SamplePeriod )
  {
    if( i_SystemTick < i_SampleTick )
    {
      return 0;
    }
    else
    {
      return ( i_SystemTick - i_SampleTick + i_SamplePeriod - 1 ) / i_SamplePeriod;
    }
  }

  /// \return the number of samples in the given frame period.
  /// Note that we allow samples to overlap the end but not the beginning of the frame period.
  /// For instance, given:
  /// system frames | A   | B   | C   | D   |
  /// samples       | 0 | 1 | 2 | 3 | 4 | 5 |
  /// Frame: samples
  /// A: 0, 1
  /// B: 2
  /// C: 3, 4
  /// D: 5
  /// etc.
  unsigned int GetSamplesInFrame( const TPeriod & i_rSystemFramePeriod, const ViconCGStreamType::UInt64 i_SampleStartTick, const ViconCGStreamType::UInt64 i_SamplePeriod )
  {
    // Order of operations is important here for rounding.
    const ViconCGStreamType::UInt64 FirstSampleIndex = GetNextSampleIndex( i_rSystemFramePeriod.first, i_SampleStartTick, i_SamplePeriod );
    const ViconCGStreamType::UInt64 LastSampleIndex = GetNextSampleIndex( i_rSystemFramePeriod.second, i_SampleStartTick, i_SamplePeriod );
    return static_cast< unsigned int >( LastSampleIndex - FirstSampleIndex );
  }

  bool SampleIndexInRange( const unsigned int i_RequestedSample,
                           const unsigned int i_DeviceFrameID,
                           const size_t i_SamplesPerDeviceFrame,
                           const ViconCGStreamType::UInt64 i_SamplePeriod,
                           const ViconCGStreamType::UInt64 i_SampleTick,
                           const TPeriod & i_rSystemFramePeriod,
                           unsigned int & o_rSubSample )
  {
    const ViconCGStreamType::UInt64 SamplesStartIndex = i_DeviceFrameID * i_SamplesPerDeviceFrame;
    const ViconCGStreamType::UInt64 RequestedSampleIndex = i_RequestedSample + GetNextSampleIndex( i_rSystemFramePeriod.first, i_SampleTick, i_SamplePeriod);

    if( RequestedSampleIndex < SamplesStartIndex || (SamplesStartIndex + i_SamplesPerDeviceFrame) <= RequestedSampleIndex )
    {
      return false;
    }

    o_rSubSample = static_cast< unsigned int >( RequestedSampleIndex - SamplesStartIndex );
    return true;
  }

  const size_t BadIndex = -1;
  
  const ViconCGStreamType::UInt32 BadFrameValue = -1;

}

namespace ViconDataStreamSDK
{
namespace Core
{

VClient::VClient()
: m_ServerPort( 0 )
, m_bContinuePreFetch( false )
, m_bNewCachedFrame( false )
, m_bSegmentDataEnabled( false )
, m_bMarkerDataEnabled( false )
, m_bUnlabeledMarkerDataEnabled( false )
, m_bMarkerRayDataEnabled( false )
, m_bDeviceDataEnabled( false )
, m_bCentroidDataEnabled( false )
, m_bGreyscaleDataEnabled( false )
, m_bDebugDataEnabled( false )
, m_bCameraWand2dDataEnabled( false )
, m_bVideoDataEnabled( false )
, m_BufferSize( 1 )
{
  SetAxisMapping( Direction::Forward, Direction::Left, Direction::Up );

  // set the frame index to a bad value so we know it is not from the stream 
  m_LatestFrame.m_Frame.m_FrameID = BadFrameValue;
  m_CachedFrame = m_LatestFrame;
}

VClient::~VClient()
{
  Disconnect();
}

void VClient::GetVersion( unsigned int & o_rMajor, 
                          unsigned int & o_rMinor, 
                          unsigned int & o_rPoint ) const
{
  // Warning! Don't change these version numbers unless you understand the
  // impact of it. They should not change without the approval of Product
  // Management.
  // * The major version number - When this number increases we break backwards 
  //                              compatibility with previous major versions.
  // * The minor version number - When this number increases we have probably added 
  //                              new functionality to the SDK without breaking backwards 
  //                              compatibility with previous versions.
  // * The point version number - When this number increases, we have introduced a bug fix 
  //                              or performance enhancement without breaking backwards 
  //                              compatibility with previous versions.
  o_rMajor = VICONDATASTREAMSDKCORE_VERSION_MAJOR;
  o_rMinor = VICONDATASTREAMSDKCORE_VERSION_MINOR;
  o_rPoint = VICONDATASTREAMSDKCORE_VERSION_POINT;
}

Result::Enum VClient::Connect(       std::shared_ptr< ViconCGStreamClientSDK::ICGClient > i_pClient, 
                               const std::string &                                          i_rHostName )
{
  if( i_rHostName.length() == 0 )
  {
    return Result::InvalidHostName;
  }

  if( !i_pClient )
  {
    return Result::NullClient;
  }

  if( i_pClient->IsConnected() )
  {
    return Result::ClientAlreadyConnected;
  }

  // Kill any old client
  m_pClient.reset();

  // Parse the supplied host name
  std::string    HostName = i_rHostName;
  unsigned short HostPort = 801;
  {
    size_t Colon = HostName.rfind( ":" );
    if( Colon != std::string::npos )
    {
      try
      {
        HostPort = boost::lexical_cast< unsigned short >( HostName.substr( Colon+1 ) );
        HostName = HostName.substr( 0, Colon );
      }
      catch ( boost::bad_lexical_cast& )
      {
        // Revert to unparsed version
        HostName = i_rHostName;
        HostPort = 801;
      }
    }
  }  

  // here we attempt to connect to the IP address
  i_pClient->Connect( HostName, HostPort );

  if( !i_pClient->IsConnected() )
  {
    return Result::ClientConnectionFailed;
  }

  // copy the pointer if all is well
  m_pClient    = i_pClient;
  m_pClient->SetBufferSize( m_BufferSize );
  m_ServerName = HostName;
  m_ServerPort = HostPort;

  // set some default request types
  m_pClient->SetRequestTypes( ViconCGStreamEnum::Contents );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::StreamInfo );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::SubjectInfo );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::FrameInfo );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::Timecode );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::LatencyInfo );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::ApplicationInfo );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::SubjectHealth );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::ObjectQuality );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::FrameRateInfo );


  // turn everything else off
  m_pClient->SetRequestTypes( ViconCGStreamEnum::CameraCalibrationInfo, false);
  m_pClient->SetRequestTypes( ViconCGStreamEnum::CameraInfo, false );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::Centroids, false );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::CentroidWeights, false );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::VideoFrame, false );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::LabeledRecons, false );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::UnlabeledRecons, false );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::LocalSegments, false );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::GlobalSegments, false );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::SubjectTopology, false );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::ForceFrame, false );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::MomentFrame, false );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::CentreOfPressureFrame, false );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::VoltageFrame, false );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::GreyscaleBlobs, false );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::EdgePairs, false );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::DeviceInfo, false );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::DeviceInfoExtra, false );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::ForcePlateInfo, false );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::ChannelInfo, false );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::ChannelInfoExtra, false );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::EyeTrackerInfo, false );

  return Result::Success;
}

/// Connect to a multicast stream
Result::Enum VClient::ConnectToMulticast( std::shared_ptr< ViconCGStreamClientSDK::ICGClient > i_pClient, 
                                          const std::string &                                    i_rLocalIP,
                                          const std::string &                                    i_rMulticastIP )
{
  if( i_rLocalIP.length() == 0 )
  {
    return Result::InvalidHostName;
  }

  if( !i_pClient )
  {
    return Result::NullClient;
  }

  if( i_pClient->IsConnected() || i_pClient->IsMulticastReceiving() )
  {
    return Result::ClientAlreadyConnected;
  }

  // Parse the supplied multicast ip to extract port if specified
  std::string    MulticastIP = i_rMulticastIP;
  unsigned short MulticastPort = 44801;
  {
    size_t Colon = MulticastIP.rfind( ":" );
    if( Colon != std::string::npos )
    {
      try
      {
        MulticastPort = boost::lexical_cast< unsigned short >( MulticastIP.substr( Colon+1 ) );
        MulticastIP = MulticastIP.substr( 0, Colon );
      }
      catch ( boost::bad_lexical_cast& )
      {
        return Result::InvalidMulticastIP;
      }
    }
  }  

  if( !IsValidMulticastIP( MulticastIP ) )
  {
    return Result::InvalidMulticastIP;
  }

  // Kill any old client
  m_pClient.reset();

  // Parse the supplied host name to strip any port (the port cannot be specified)
  std::string    LocalIP = i_rLocalIP;
  {
    size_t Colon = LocalIP.rfind( ":" );
    if( Colon != std::string::npos )
    {
        LocalIP = LocalIP.substr( 0, Colon );
    }
  }  
  
  // here we attempt to connect to the IP address
  i_pClient->ReceiveMulticastData( MulticastIP, LocalIP, MulticastPort );

  if( !i_pClient->IsMulticastReceiving() )
  {
    return Result::ClientConnectionFailed;
  }

  // copy the pointer if all is well
  m_pClient = i_pClient;
  m_pClient->SetBufferSize( m_BufferSize );

  return Result::Success;
}

/// Disconnect client from the Vicon Data Stream
Result::Enum VClient::Disconnect()
{    
  if( !m_pClient )
  {
    return Result::NotConnected;
  }

  if( !m_pClient->IsConnected() && !m_pClient->IsMulticastReceiving() )
  {
    return Result::NotConnected;
  }  

  EndPreFetchThread();

  m_pClient.reset(); 
  m_ServerName.clear();
  m_ServerPort = 0;

  return Result::Success;
}

Result::Enum VClient::StartTransmittingMulticast( const std::string & i_rServerIP,
                                                  const std::string & i_rMulticastIP )
{
  // Parse the supplied multicast ip to extract port if specified
  std::string    MulticastIP = i_rMulticastIP;
  unsigned short MulticastPort = 44801;
  {
    size_t Colon = MulticastIP.rfind( ":" );
    if( Colon != std::string::npos )
    {
      try
      {
        MulticastPort = boost::lexical_cast< unsigned short >( MulticastIP.substr( Colon+1 ) );
        MulticastIP = MulticastIP.substr( 0, Colon );
      }
      catch ( boost::bad_lexical_cast& )
      {
        return Result::InvalidMulticastIP;
      }
    }
  }  

  // Parse the supplied host name to strip any port (the port cannot be specified)
  std::string HostName = i_rServerIP;
  {
    size_t Colon = HostName.rfind( ":" );
    if( Colon != std::string::npos )
    {
      HostName = HostName.substr( 0, Colon );
    }
  }  

  if( !m_pClient )
  {
    return Result::NotConnected;
  }

  if( !m_pClient->IsConnected() )
  {
    return Result::NotConnected;
  }

  if( !IsValidMulticastIP( MulticastIP ) )
  {
    return Result::InvalidMulticastIP;
  }

  if( m_pClient->IsMulticastController() )
  {
    return Result::ServerAlreadyTransmittingMulticast;
  }

  m_pClient->SetServerToTransmitMulticast( MulticastIP, HostName, MulticastPort );

  return Result::Success;
}

Result::Enum VClient::StopTransmittingMulticast()
{
  if( !m_pClient )
  {
    return Result::NotConnected;
  }

  if( !m_pClient->IsConnected() )
  {
    return Result::NotConnected;
  }

  if( !m_pClient->IsMulticastController() )
  {
    return Result::ServerNotTransmittingMulticast;
  }

  m_pClient->StopMulticastTransmission();

  return Result::Success;
}

void VClient::BeginPreFetchThread()
{
  m_bContinuePreFetch = true;

  // Cache a frame before we start the thread running
  FetchNextFrame();

  if (!m_pPreFetchThread)
  {
    m_pPreFetchThread.reset( new boost::thread( boost::bind( &VClient::PreFetchThreadBody, this ) ) );
  }
}

void VClient::EndPreFetchThread()
{
  if( m_pPreFetchThread )
  { 
    m_bContinuePreFetch = false;
    m_pPreFetchThread->join();
    m_pPreFetchThread.reset();
  }
}

bool VClient::IsConnected() const
{
  if (!m_pClient)
  {
    return false;
  }

  return m_pClient->IsConnected() || m_pClient->IsMulticastReceiving();
}

void VClient::SetBufferSize( unsigned int i_MaxFrames )
{
  m_BufferSize = i_MaxFrames;
  if( m_pClient )
  {
    m_pClient->SetBufferSize( m_BufferSize );
  }
}

Result::Enum VClient::GetFrame()
{
  if( !IsConnected() )
  {
    return Result::NotConnected;
  }

  // Get the next frame
  if( !m_pPreFetchThread || !m_bContinuePreFetch )
  {
    // Not in pre-fetch mode
    FetchNextFrame();
  }
  else
  {
    // In pre-fetch mode
    // The pre-fetch thread should have got a frame for us
  }

  // Is the latest frame different from the one we dispatched?
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  if( !m_bNewCachedFrame )
  {
    return Result::NoFrame;  
  }
  else
  {
    m_LatestFrame = m_CachedFrame;
    m_bNewCachedFrame = false;
    return Result::Success;
  }
}

bool VClient::InitGet( Result::Enum & o_rResult ) const
{
  o_rResult = Result::Success;

  if( !IsConnected() )
  {
    o_rResult = Result::NotConnected;
  }
  else if( !HasData() )
  {
    o_rResult = Result::NoFrame;
  }

  return (o_rResult == Result::Success);
}

Result::Enum VClient::GetFrameNumber( unsigned int & o_rFrameNumber ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( InitGet( GetResult, o_rFrameNumber ) )
  {
    o_rFrameNumber = m_LatestFrame.m_Frame.m_FrameID;
  }

  return GetResult; 
}

Result::Enum VClient::GetFrameRate( double & o_rFrameRateInHz ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( InitGet( GetResult, o_rFrameRateInHz ) )
  {
    ViconCGStreamType::Int64 Period = m_LatestFrame.m_Stream.m_FramePeriod;
    if (Period == 0)
    {
      o_rFrameRateInHz = 0.0;
    }
    else
    {
      o_rFrameRateInHz = static_cast<double>( 135000000 ) / static_cast<double>( Period );
    }
  }

  return GetResult;
}

Result::Enum VClient::GetTimecode( unsigned int           & o_rHours,
                                   unsigned int           & o_rMinutes,
                                   unsigned int           & o_rSeconds,
                                   unsigned int           & o_rFrames,
                                   unsigned int           & o_rSubFrame,
                                   bool                   & o_rbFieldFlag,
                                   TimecodeStandard::Enum & o_rTimecodeStandard,
                                   unsigned int           & o_rSubFramesPerFrame,
                                   unsigned int           & o_rUserBits ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Clear( o_rHours );
  Clear( o_rMinutes );
  Clear( o_rSeconds );
  Clear( o_rFrames );
  Clear( o_rSubFrame );
  Clear( o_rbFieldFlag );
  Clear( o_rTimecodeStandard );
  Clear( o_rSubFramesPerFrame );
  Clear( o_rUserBits );

  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult ) )
  {
    return GetResult; 
  }

  o_rHours             = m_LatestFrame.m_Timecode.m_Hours;
  o_rMinutes           = m_LatestFrame.m_Timecode.m_Minutes;
  o_rSeconds           = m_LatestFrame.m_Timecode.m_Seconds;
  o_rFrames            = m_LatestFrame.m_Timecode.m_Frames;
  o_rSubFrame          = m_LatestFrame.m_Timecode.m_Subframes;
  o_rbFieldFlag        = m_LatestFrame.m_Timecode.m_FieldFlag != 0;
  o_rSubFramesPerFrame = m_LatestFrame.m_Timecode.m_SubframesPerFrame;
  o_rUserBits          = m_LatestFrame.m_Timecode.m_UserBits;

  switch( m_LatestFrame.m_Timecode.m_Standard )
  {
  case ViconCGStream::VTimecode::ETimecodePAL      : o_rTimecodeStandard = TimecodeStandard::PAL;      break;
  case ViconCGStream::VTimecode::ETimecodeNTSC     : o_rTimecodeStandard = TimecodeStandard::NTSC;     break;
  case ViconCGStream::VTimecode::ETimecodeNTSCDrop : o_rTimecodeStandard = TimecodeStandard::NTSCDrop; break;
  case ViconCGStream::VTimecode::ETimecodeFilm     : o_rTimecodeStandard = TimecodeStandard::Film;     break;
  case ViconCGStream::VTimecode::ETimecodeNTSCFilm : o_rTimecodeStandard = TimecodeStandard::NTSCFilm; break;
  case ViconCGStream::VTimecode::ETimecodeATSC     : o_rTimecodeStandard = TimecodeStandard::ATSC;     break;
  default: o_rTimecodeStandard = TimecodeStandard::None;     break;
  }

  return Result::Success;
}

Result::Enum VClient::GetNetworkLatency(double & o_rLatency)
{
  bool bSuccess = m_pClient->NetworkLatency(o_rLatency);
  return bSuccess ? Result::Success : Result::NoFrame;
}

Result::Enum VClient::GetLatencyTotal( double & o_rLatency ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rLatency ) )
  {
    return GetResult; 
  }

  std::vector< ViconCGStreamDetail::VLatencyInfo_Sample >::const_iterator It  = m_LatestFrame.m_Latency.m_Samples.begin();
  std::vector< ViconCGStreamDetail::VLatencyInfo_Sample >::const_iterator End = m_LatestFrame.m_Latency.m_Samples.end();
  for( ; It != End ; ++It )
  {
    o_rLatency += It->m_Latency;
  }

  return Result::Success;
}

Result::Enum VClient::GetLatencySampleCount( unsigned int & o_rSampleCount ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( InitGet( GetResult, o_rSampleCount ) )
  {
    o_rSampleCount = static_cast< unsigned int >( m_LatestFrame.m_Latency.m_Samples.size() );
  }

  return GetResult;
}

Result::Enum VClient::GetLatencySampleName( const unsigned int i_SampleIndex, std::string & o_rSampleName ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rSampleName ) )
  {
    return GetResult; 
  }

  if( i_SampleIndex >= m_LatestFrame.m_Latency.m_Samples.size() )
  {
    return Result::InvalidIndex;
  }

  o_rSampleName = m_LatestFrame.m_Latency.m_Samples[ i_SampleIndex ].m_Name;

  return Result::Success;
}

Result::Enum VClient::GetLatencySampleValue( const std::string & i_rSampleName, double & o_rSampleValue ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rSampleValue ) )
  {
    return GetResult; 
  }

  std::vector< ViconCGStreamDetail::VLatencyInfo_Sample >::const_iterator It  = m_LatestFrame.m_Latency.m_Samples.begin();
  std::vector< ViconCGStreamDetail::VLatencyInfo_Sample >::const_iterator End = m_LatestFrame.m_Latency.m_Samples.end();
  for( ; It != End ; ++It )
  {
    if( It->m_Name == i_rSampleName )
    {
      o_rSampleValue = It->m_Latency;
      return Result::Success;
    }
  }

  return Result::InvalidLatencySampleName;
}

Result::Enum VClient::GetHardwareFrameNumber( unsigned int & o_rFrameNumber ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( InitGet( GetResult, o_rFrameNumber ) )
  {
    o_rFrameNumber = m_LatestFrame.m_HardwareFrame.m_HardwareFrame;
  }

  return GetResult; 
}

Result::Enum VClient::GetFrameRateCount( unsigned int & o_rFrameRateCount ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rFrameRateCount ) )
  {
    return GetResult; 
  }

  o_rFrameRateCount = static_cast< unsigned int >( m_LatestFrame.m_FrameRateInfo.m_FrameRates.size() );
  return Result::Success;
}

Result::Enum VClient::GetFrameRateName( const unsigned int i_FrameRateIndex, std::string & o_rFrameRateName ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rFrameRateName ) )
  {
    return GetResult; 
  }

  if( i_FrameRateIndex >= m_LatestFrame.m_FrameRateInfo.m_FrameRates.size()  )
  {
    return Result::InvalidIndex;
  }

  unsigned int Counter = 0;
  std::map< std::string, double >::const_iterator It= m_LatestFrame.m_FrameRateInfo.m_FrameRates.begin();
  std::map< std::string, double >::const_iterator End= m_LatestFrame.m_FrameRateInfo.m_FrameRates.end();
  for( ; It!=End; ++It, ++Counter )
  {
    if( Counter == i_FrameRateIndex )
    {
      o_rFrameRateName = It->first;
      break;
    }
  }
  return Result::Success;
}

Result::Enum VClient::GetFrameRateValue( const std::string & i_rFrameRateName, double & o_rFrameRateValue ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rFrameRateValue ) )
  {
    return GetResult; 
  }

  if( !m_LatestFrame.m_FrameRateInfo.m_FrameRates.count( i_rFrameRateName ) )
  {
    return Result::InvalidFrameRateName;
  }


  std::map<std::string, double> FrameRates = m_LatestFrame.m_FrameRateInfo.m_FrameRates;
  o_rFrameRateValue = FrameRates[i_rFrameRateName];
  return Result::Success;
}

Result::Enum VClient::SetSegmentDataEnabled( const bool i_bEnabled )
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  if( !IsConnected() )
  {
    return Result::NotConnected;
  }

  m_pClient->SetRequestTypes( ViconCGStreamEnum::LocalSegments, i_bEnabled );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::GlobalSegments, i_bEnabled );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::SubjectTopology, i_bEnabled );

  m_bSegmentDataEnabled = i_bEnabled;
  return Result::Success;
}

Result::Enum VClient::SetMarkerDataEnabled( const bool i_bEnabled )
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  if( !IsConnected() )
  {
    return Result::NotConnected;
  }

  m_pClient->SetRequestTypes( ViconCGStreamEnum::LabeledRecons, i_bEnabled );

  m_bMarkerDataEnabled = i_bEnabled;
  return Result::Success;
}

Result::Enum VClient::SetUnlabeledMarkerDataEnabled( const bool i_bEnabled )
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  if( !IsConnected() )
  {
    return Result::NotConnected;
  }

  m_pClient->SetRequestTypes( ViconCGStreamEnum::UnlabeledRecons, i_bEnabled );

  m_bUnlabeledMarkerDataEnabled = i_bEnabled;
  return Result::Success;
}

Result::Enum VClient::SetMarkerRayDataEnabled( const bool i_bEnabled )
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  if( !IsConnected() )
  {
    return Result::NotConnected;
  }

  m_pClient->SetRequestTypes( ViconCGStreamEnum::LabeledReconRayAssignments, i_bEnabled );

  m_bMarkerRayDataEnabled = i_bEnabled;
  return Result::Success;
}

Result::Enum VClient::SetDeviceDataEnabled( const bool i_bEnabled )
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  if( !IsConnected() )
  {
    return Result::NotConnected;
  }

  m_pClient->SetRequestTypes( ViconCGStreamEnum::DeviceInfo, i_bEnabled );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::DeviceInfoExtra, i_bEnabled );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::ChannelInfo, i_bEnabled );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::ChannelInfoExtra, i_bEnabled );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::ForcePlateInfo, i_bEnabled );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::EyeTrackerInfo, i_bEnabled );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::ForceFrame, i_bEnabled );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::MomentFrame, i_bEnabled );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::CentreOfPressureFrame, i_bEnabled );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::VoltageFrame, i_bEnabled );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::EyeTrackerFrame, i_bEnabled );

  m_bDeviceDataEnabled = i_bEnabled;
  return Result::Success;
}

Result::Enum VClient::SetCentroidDataEnabled( const bool i_bEnabled )
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  if( !IsConnected() )
  {
    return Result::NotConnected;
  }

  m_pClient->SetRequestTypes( ViconCGStreamEnum::CameraInfo, i_bEnabled );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::Centroids, i_bEnabled );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::CentroidWeights, i_bEnabled );

  m_bCentroidDataEnabled = i_bEnabled;
  return Result::Success;

}

Result::Enum VClient::SetGreyscaleDataEnabled( const bool i_bEnabled )
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  if( !IsConnected() )
  {
    return Result::NotConnected;
  }

  m_pClient->SetRequestTypes( ViconCGStreamEnum::CameraInfo, i_bEnabled );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::GreyscaleBlobs, i_bEnabled );

  m_bGreyscaleDataEnabled = i_bEnabled;
  return Result::Success;
}

Result::Enum VClient::SetDebugDataEnabled( const bool i_bEnabled )
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  if( !IsConnected() )
  {
    return Result::NotConnected;
  }

  m_pClient->SetRequestTypes( ViconCGStreamEnum::HardwareFrameInfo, i_bEnabled );
  m_bDebugDataEnabled = i_bEnabled;
  return Result::Success;
}

Result::Enum VClient::SetCameraWand2dDataEnabled( const bool i_bEnabled )
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  if( !IsConnected() )
  {
    return Result::NotConnected;
  }

  m_pClient->SetRequestTypes( ViconCGStreamEnum::CameraInfo, i_bEnabled );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::CameraWand2d, i_bEnabled );
  m_bCameraWand2dDataEnabled = i_bEnabled;
  return Result::Success;
}

Result::Enum VClient::SetVideoDataEnabled( const bool i_bEnabled )
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  if( !IsConnected() )
  {
    return Result::NotConnected;
  }

  m_pClient->SetRequestTypes( ViconCGStreamEnum::CameraInfo, i_bEnabled );
  m_pClient->SetRequestTypes( ViconCGStreamEnum::VideoFrame, i_bEnabled );

  m_bVideoDataEnabled = i_bEnabled;
  return Result::Success;

}

bool VClient::IsSegmentDataEnabled() const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  return m_bSegmentDataEnabled;
}

bool VClient::IsMarkerDataEnabled() const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  return m_bMarkerDataEnabled;
}

bool VClient::IsUnlabeledMarkerDataEnabled() const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  return m_bUnlabeledMarkerDataEnabled;
}

bool VClient::IsMarkerRayDataEnabled() const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  return m_bMarkerRayDataEnabled;
}

bool VClient::IsDeviceDataEnabled() const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  return m_bDeviceDataEnabled;
}

bool VClient::IsCentroidDataEnabled() const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  return m_bCentroidDataEnabled;
}

bool VClient::IsGreyscaleDataEnabled() const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  return m_bGreyscaleDataEnabled;
}

bool VClient::IsVideoDataEnabled() const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  return m_bVideoDataEnabled;
}

bool VClient::IsDebugDataEnabled() const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  return m_bDebugDataEnabled;
}

bool VClient::IsCameraWand2dDataEnabled() const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  return m_bCameraWand2dDataEnabled;
}


Result::Enum VClient::SetStreamMode( const StreamMode::Enum i_Mode )
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  if( !IsConnected() )
  {
    return Result::NotConnected;
  }

  switch( i_Mode )
  {
  case StreamMode::ServerPush         : EndPreFetchThread();
                                        m_pClient->SetStreamMode( true );
                                        break;
  case StreamMode::ClientPullPreFetch : m_pClient->SetStreamMode( false );
                                        BeginPreFetchThread();
                                        break;
  case StreamMode::ClientPull         : 
  default                             : m_pClient->SetStreamMode( false );
                                        EndPreFetchThread();
                                        break;
  }

  return Result::Success;
}

Result::Enum VClient::SetApexDeviceFeedback( const std::string& i_rDeviceName, bool i_bOn )
{
  if( !IsConnected() )
  {
    return Result::NotConnected;
  }
  if( !HasData() )
  {
    return Result::NoFrame;
  }

  unsigned int DeviceID;
  if( GetDeviceID( i_rDeviceName, DeviceID )!= Result::Success )
  {
    return Result::InvalidDeviceName;
  }

  if( !m_pClient )
  {
    return Result::NullClient;
  }

  if( m_pClient->SetApexDeviceFeedback( DeviceID, i_bOn ) )
  {
    return Result::Success;
  }

  return Result::HapticAlreadySet;
  
}

Result::Enum VClient::SetAxisMapping( const Direction::Enum i_XAxis, 
                                      const Direction::Enum i_YAxis, 
                                      const Direction::Enum i_ZAxis )
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  AxisMappingResult::Enum Result;
  std::shared_ptr< VAxisMapping > pAxisMapping = VAxisMapping::Create( Result, i_XAxis, i_YAxis, i_ZAxis );

  if( ! pAxisMapping )
  {
    switch( Result )
    {
    case AxisMappingResult::CoLinearAxes:   return Result::CoLinearAxes;
    case AxisMappingResult::LeftHandedAxes: return Result::LeftHandedAxes;
    default:                                return Result::Unknown;
    }
  }

  m_pAxisMapping = pAxisMapping;

  return Result::Success;
}

void VClient::GetAxisMapping( Direction::Enum & o_rXAxis, Direction::Enum & o_rYAxis, Direction::Enum & o_rZAxis ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
 
  m_pAxisMapping->GetAxisMapping( o_rXAxis, o_rYAxis, o_rZAxis );
}

Result::Enum VClient::GetServerOrientation( ServerOrientation::Enum & o_rServerOrientation ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  if( m_CachedFrame.m_ApplicationInfo )
  {
    switch(m_CachedFrame.m_ApplicationInfo.get().m_AxisOrientation )
    {
      case  ViconCGStream::VApplicationInfo::EYUp:
        o_rServerOrientation = ServerOrientation::YUp;
        break;
      case ViconCGStream::VApplicationInfo::EZUp:
        o_rServerOrientation = ServerOrientation::ZUp;
        break;
      default:
        o_rServerOrientation=ServerOrientation::Unknown;
        break;
    }
    return Result::Success;
  }
  else
  {
    o_rServerOrientation=ServerOrientation::Unknown;
    return Result::NotImplemented;
  }
  
}

Result::Enum VClient::GetSubjectCount( unsigned int & o_rSubjectCount ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( InitGet( GetResult, o_rSubjectCount ) )
  {
    o_rSubjectCount = static_cast< unsigned int >( m_LatestFrame.m_Subjects.size() );
  }

  return GetResult;
}

Result::Enum VClient::GetSubjectName( const unsigned int i_SubjectIndex, 
                                            std::string& o_rSubjectName ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rSubjectName ) )
  {
    return GetResult; 
  }

  if( i_SubjectIndex >= m_LatestFrame.m_Subjects.size() )
  {
    return Result::InvalidIndex;
  }

  o_rSubjectName = m_LatestFrame.m_Subjects[ i_SubjectIndex ].m_Name;
  return Result::Success;
}

Result::Enum VClient::GetSubjectRootSegmentName( const std::string & i_rSubjectName, 
                                                       std::string & o_rSegmentName ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Clear( o_rSegmentName );

  // here we have a valid frame of data. need to check for this subject and retrieve information
  Result::Enum GetResult = Result::Success;
  const ViconCGStream::VSubjectInfo * pSubjectInfo = GetSubjectInfo( i_rSubjectName, GetResult );
  if( !pSubjectInfo )
  {
    return GetResult;
  }

  std::vector< ViconCGStreamDetail::VSubjectInfo_Segment >::const_iterator SegIt  = pSubjectInfo->m_Segments.begin();
  std::vector< ViconCGStreamDetail::VSubjectInfo_Segment >::const_iterator SegEnd = pSubjectInfo->m_Segments.end();
  for( ; SegIt != SegEnd ; ++SegIt )
  {
    if( SegIt->m_ParentID == 0 )
    {
      o_rSegmentName = SegIt->m_Name;
      return Result::Success;
    }
  }

  return Result::Unknown;
}

Result::Enum VClient::GetSegmentCount( const std::string  & i_rSubjectName, 
                                             unsigned int & o_rSegmentCount ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Clear( o_rSegmentCount );

  Result::Enum GetResult = Result::Success;
  // here we have a valid frame of data. need to check for this subject and retrieve information
  const ViconCGStream::VSubjectInfo * pSubjectInfo = GetSubjectInfo( i_rSubjectName, GetResult );
  if( pSubjectInfo )
  {
    o_rSegmentCount = static_cast< unsigned int >( pSubjectInfo->m_Segments.size() );
    return Result::Success;
  }

  return GetResult;
}

Result::Enum VClient::GetSegmentName( const std::string  & i_rSubjectName, 
                                      const unsigned int   i_SegmentIndex, 
                                            std::string  & o_rSegmentName ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Clear( o_rSegmentName );

  Result::Enum GetResult = Result::Success;
  const ViconCGStream::VSubjectInfo * pSubjectInfo = GetSubjectInfo( i_rSubjectName, GetResult );
  if( pSubjectInfo )
  {
    if( i_SegmentIndex >= pSubjectInfo->m_Segments.size() )
    {
      return Result::InvalidIndex;
    }
    
    o_rSegmentName = pSubjectInfo->m_Segments[ i_SegmentIndex ].m_Name;
    return Result::Success;
  }

  return GetResult;
}

Result::Enum VClient::GetSegmentChildCount( const std::string  & i_rSubjectName, 
                                            const std::string  & i_rSegmentName, 
                                                  unsigned int & o_rSegmentCount ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rSegmentCount ) )
  {
    return GetResult; 
  }

  unsigned int SubjectID = 0;
  unsigned int SegmentID = 0;
  {
    Result::Enum result = GetSubjectAndSegmentID( i_rSubjectName, i_rSegmentName, SubjectID, SegmentID );
    if( result != Result::Success )
    {
      return result;
    }
  }

  // here we have a valid frame of data. need to check for this subject and retrieve information
  std::vector< ViconCGStream::VSubjectInfo >::const_iterator SubIt  = m_LatestFrame.m_Subjects.begin();
  std::vector< ViconCGStream::VSubjectInfo >::const_iterator SubEnd = m_LatestFrame.m_Subjects.end();
  for( ; SubIt != SubEnd ; ++SubIt )
  {
    if( SubjectID == SubIt->m_SubjectID )
    {
      unsigned int ChildCount = 0;
      std::vector< ViconCGStreamDetail::VSubjectInfo_Segment >::const_iterator SegIt  = SubIt->m_Segments.begin();
      std::vector< ViconCGStreamDetail::VSubjectInfo_Segment >::const_iterator SegEnd = SubIt->m_Segments.end();
      for( ; SegIt != SegEnd ; ++SegIt )
      {
        if( SegIt->m_ParentID == SegmentID )
        {
          ++ChildCount;
        }
      }

      o_rSegmentCount = ChildCount;
      return Result::Success;
    }
  }

  return Result::NotImplemented;
}

Result::Enum VClient::GetSegmentChildName( const std::string& i_rSubjectName, const std::string& i_rSegmentName, unsigned int i_SegmentIndex, std::string& o_rSegmentName ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rSegmentName ) )
  {
    return GetResult; 
  }

  unsigned int SubjectID = 0;
  unsigned int SegmentID = 0;
  {
    Result::Enum result = GetSubjectAndSegmentID( i_rSubjectName, i_rSegmentName, SubjectID, SegmentID );
    if( result != Result::Success )
    {
      return result;
    }
  }

  // here we have a valid frame of data. need to check for this subject and retrieve information
  std::vector< ViconCGStream::VSubjectInfo >::const_iterator SubIt  = m_LatestFrame.m_Subjects.begin();
  std::vector< ViconCGStream::VSubjectInfo >::const_iterator SubEnd = m_LatestFrame.m_Subjects.end();
  for( ; SubIt != SubEnd ; ++SubIt )
  {
    if( SubjectID == SubIt->m_SubjectID )
    {
      unsigned int ChildCount = 0;
      std::vector< ViconCGStreamDetail::VSubjectInfo_Segment >::const_iterator SegIt  = SubIt->m_Segments.begin();
      std::vector< ViconCGStreamDetail::VSubjectInfo_Segment >::const_iterator SegEnd = SubIt->m_Segments.end();
      for( ; SegIt != SegEnd ; ++SegIt )
      {
        if( SegIt->m_ParentID == SegmentID )
        {
          if( ChildCount == i_SegmentIndex )
          {
            o_rSegmentName = SegIt->m_Name;
            return Result::Success;
          }
          else
          {
            ++ChildCount;
          }
        }
      }
      return Result::InvalidIndex;
    }
  }

  return Result::NotImplemented;
}

Result::Enum VClient::GetSegmentParentName( const std::string& i_rSubjectName, const std::string& i_rSegmentName, std::string& o_rSegmentName ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rSegmentName ) )
  {
    return GetResult; 
  }

  unsigned int SubjectID = 0;
  unsigned int SegmentID = 0;
  {
    Result::Enum result = GetSubjectAndSegmentID( i_rSubjectName, i_rSegmentName, SubjectID, SegmentID );
    if( result != Result::Success )
    {
      return result;
    }
  }

  // here we have a valid frame of data. need to check for this subject and retrieve information
  std::vector< ViconCGStream::VSubjectInfo >::const_iterator SubIt  = m_LatestFrame.m_Subjects.begin();
  std::vector< ViconCGStream::VSubjectInfo >::const_iterator SubEnd = m_LatestFrame.m_Subjects.end();
  for( ; SubIt != SubEnd ; ++SubIt )
  {
    if( SubjectID == SubIt->m_SubjectID )
    {
      unsigned int ParentID = 0;
      {
        std::vector< ViconCGStreamDetail::VSubjectInfo_Segment >::const_iterator SegIt  = SubIt->m_Segments.begin();
        std::vector< ViconCGStreamDetail::VSubjectInfo_Segment >::const_iterator SegEnd = SubIt->m_Segments.end();
        for( ; SegIt != SegEnd ; ++SegIt )
        {
          if( SegIt->m_SegmentID == SegmentID )
          {
            ParentID = SegIt->m_ParentID;
            break;
          }
        }
      }

      // Get the parent name from the ID
      {
        std::vector< ViconCGStreamDetail::VSubjectInfo_Segment >::const_iterator SegIt  = SubIt->m_Segments.begin();
        std::vector< ViconCGStreamDetail::VSubjectInfo_Segment >::const_iterator SegEnd = SubIt->m_Segments.end();
        for( ; SegIt != SegEnd ; ++SegIt )
        {
          if( SegIt->m_SegmentID == ParentID )
          {
            o_rSegmentName = SegIt->m_Name;
            return Result::Success;
          }
        }

        return Result::Unknown;
      }
    }
  }

  return Result::NotImplemented;
}

Result::Enum VClient::GetSegmentStaticTranslation( const std::string&   i_rSubjectName, 
                                                   const std::string&   i_rSegmentName, 
                                                         double       (&o_rThreeVector)[3] ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rThreeVector ) )
  {
    return GetResult; 
  }

  unsigned int SubjectID = 0;
  unsigned int SegmentID = 0;
  {
    Result::Enum result = GetSubjectAndSegmentID( i_rSubjectName, i_rSegmentName, SubjectID, SegmentID );
    if( result != Result::Success )
    {
      return result;
    }
  }

  const ViconCGStream::VSubjectTopology * pSubjectTopology = GetSubjectTopology( SubjectID );
  if( !pSubjectTopology )
  {
    return Result::InvalidSubjectName;
  }

  std::vector< ViconCGStreamDetail::VSubjectTopology_Segment >::const_iterator It  = pSubjectTopology->m_Segments.begin();
  std::vector< ViconCGStreamDetail::VSubjectTopology_Segment >::const_iterator End = pSubjectTopology->m_Segments.end();
  for( ; It != End ; ++It )
  {
    if( It->m_SegmentID == SegmentID )
    {
      CopyAndTransformT( It->m_Translation, o_rThreeVector );
      return Result::Success;
    }
  }

  return Result::InvalidSegmentName;
}

Result::Enum VClient::GetObjectQuality( const std::string& i_rObjectName, double& o_rQuality ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  Result::Enum GetResult = Result::Success;

  // here we have a valid frame of data. first check whether we have a valid subject
  const ViconCGStream::VSubjectInfo * pSubjectInfo = GetSubjectInfo( i_rObjectName, GetResult );
  if( pSubjectInfo )
  {
    // See if we have an object quality associated with the subject ID.
    const ViconCGStream::VObjectQuality * pObjectQuality = GetObjectQuality( pSubjectInfo->m_SubjectID );
    if( !pObjectQuality )
    {
      // Not all servers will support ObjectQuality. Currently only Tracker.
      return Result::NotImplemented;
    }

    o_rQuality = pObjectQuality->m_ObjectQuality;
    return Result::Success;
  }

  return GetResult;
}

Result::Enum VClient::GetMarkerCount( const std::string  & i_rSubjectName,
                                            unsigned int & o_rMarkerCount ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  // here we have a valid frame of data. need to check for this subject and retrieve information
  const ViconCGStream::VSubjectInfo * pSubjectInfo = GetSubjectInfo( i_rSubjectName, GetResult );
  if( pSubjectInfo )
  {
    o_rMarkerCount = static_cast< unsigned int >( pSubjectInfo->m_Markers.size() );
  }

  return GetResult;
}

Result::Enum VClient::GetMarkerName( const std::string & i_rSubjectName, 
                                     const unsigned int  i_MarkerIndex, 
                                           std::string & o_rMarkerName ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Clear( o_rMarkerName );

  Result::Enum GetResult = Result::Success;
  const ViconCGStream::VSubjectInfo * pSubjectInfo = GetSubjectInfo( i_rSubjectName, GetResult );
  if( pSubjectInfo )
  {
    // check that index is correct
    if( i_MarkerIndex >= pSubjectInfo->m_Markers.size() )
    {
      return Result::InvalidIndex;
    }

    o_rMarkerName = pSubjectInfo->m_Markers[ i_MarkerIndex ].m_Name;
  }

  return GetResult;
}

Result::Enum VClient::GetMarkerParentName( const std::string & i_rSubjectName, const std::string & i_rMarkerName, std::string & o_rSegmentName ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  Clear( o_rSegmentName );

  Result::Enum GetResult = Result::Success;
  const ViconCGStream::VSubjectInfo * pSubjectInfo = GetSubjectInfo( i_rSubjectName, GetResult );
  if ( !pSubjectInfo )
  {
    return GetResult;
  }

  unsigned int MarkerID  = 0;
  GetResult = GetMarkerID( *pSubjectInfo, i_rMarkerName, MarkerID );
  if ( GetResult != Result::Success )
  {
    return GetResult;
  }

  unsigned int SegmentID = 0;

  std::vector< ViconCGStreamDetail::VSubjectInfo_Attachment >::const_iterator AttachIt  = pSubjectInfo->m_Attachments.begin();
  std::vector< ViconCGStreamDetail::VSubjectInfo_Attachment >::const_iterator AttachEnd = pSubjectInfo->m_Attachments.end();
  for( ; AttachIt != AttachEnd ; ++AttachIt )
  {
    if( AttachIt->m_MarkerID == MarkerID )
    {
      SegmentID = AttachIt->m_SegmentID;
      break;
    }
  }

  std::vector< ViconCGStreamDetail::VSubjectInfo_Segment >::const_iterator SegmentIt  = pSubjectInfo->m_Segments.begin();
  std::vector< ViconCGStreamDetail::VSubjectInfo_Segment >::const_iterator SegmentEnd = pSubjectInfo->m_Segments.end();
  for( ; SegmentIt != SegmentEnd ; ++SegmentIt )
  {
    if( SegmentIt->m_SegmentID == SegmentID )
    {
      o_rSegmentName = SegmentIt->m_Name;
      return Result::Success;
    }
  }

  return Result::InvalidMarkerName;
}

Result::Enum VClient::GetMarkerGlobalTranslation( const std::string & i_rSubjectName, 
                                                  const std::string & i_rMarkerName, 
                                                        double     (& o_rThreeVector)[3], 
                                                        bool        & o_rbOccludedFlag ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rThreeVector, o_rbOccludedFlag ) )
  {
    return GetResult; 
  }

  unsigned int SubjectID = 0;
  unsigned int MarkerID  = 0;
  Result::Enum _Result = GetSubjectAndMarkerID( i_rSubjectName, i_rMarkerName, SubjectID, MarkerID );
  if( Result::Success != _Result )
  {
    return _Result;
  }

  // go through the frame's reconstructions and find its position in this frame
  for( unsigned int i = 0 ; i < m_LatestFrame.m_LabeledRecons.m_LabeledRecons.size() ; ++i )
  {
    const ViconCGStreamDetail::VLabeledRecons_LabeledRecon& rRecon = m_LatestFrame.m_LabeledRecons.m_LabeledRecons[i];
    if( rRecon.m_SubjectID == SubjectID && rRecon.m_MarkerID == MarkerID )
    {  
      CopyAndTransformT( rRecon.m_Position, o_rThreeVector );
      return Result::Success;
    }
  }

  // if we're here then they entered correct subject and marker info,
  // but the data was not in the frame. This is okay but we must
  // set the occluded flag and return true
  o_rbOccludedFlag = true;
  return Result::Success;
}

Result::Enum VClient::GetMarkerRayAssignmentCount( const std::string& i_rSubjectName, const std::string& i_rMarkerName, unsigned int &o_rAssignmentCount ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  Result::Enum GetResult = Result::Success;
  if( !InitGet( GetResult, o_rAssignmentCount ) )
  {
    return GetResult;
  }

  std::vector< unsigned int > RayAssignments;
  std::vector< unsigned int > CentroidIndices;
  GetResult = GetReconRayAssignments( i_rSubjectName, i_rMarkerName, RayAssignments, CentroidIndices );
  if( GetResult == Result::Success )
  {
    o_rAssignmentCount = static_cast< unsigned int >( RayAssignments.size() );
  }

  return GetResult;
}

Result::Enum VClient::GetMarkerRayAssignment( const std::string& i_rSubjectName, const std::string& i_rMarkerName, int i_AssignmentIndex, unsigned int & o_rCameraID, unsigned int & o_rCentroidIndex ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  Result::Enum GetResult = Result::Success;
  if( !InitGet( GetResult, o_rCameraID ) )
  {
    return GetResult;
  }

  std::vector< unsigned int > RayAssignments;
  std::vector< unsigned int > CentroidIndices;
  GetResult = GetReconRayAssignments( i_rSubjectName, i_rMarkerName, RayAssignments, CentroidIndices );
  if( GetResult == Result::Success )
  {
    if( i_AssignmentIndex < static_cast< int >( RayAssignments.size() ) )
    {
      o_rCameraID = RayAssignments[i_AssignmentIndex];
      o_rCentroidIndex = CentroidIndices[ i_AssignmentIndex ];
      return Result::Success;
    }
    return Result::InvalidIndex;
  }

  return GetResult;
}

Result::Enum VClient::GetUnlabeledMarkerCount( unsigned int & o_rMarkerCount ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( InitGet( GetResult, o_rMarkerCount ) )
  {
    o_rMarkerCount = static_cast< unsigned int >( m_LatestFrame.m_UnlabeledRecons.m_UnlabeledRecons.size() );
  }

  return GetResult;
}

Result::Enum VClient::GetUnlabeledMarkerGlobalTranslation( const unsigned int   i_MarkerIndex,
                                                                 double      (& o_rTranslation)[3] ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rTranslation ) )
  {
    return GetResult; 
  }

  if( i_MarkerIndex >= m_LatestFrame.m_UnlabeledRecons.m_UnlabeledRecons.size() )
  {
    return Result::InvalidIndex;
  }

  CopyAndTransformT( m_LatestFrame.m_UnlabeledRecons.m_UnlabeledRecons[ i_MarkerIndex ].m_Position, o_rTranslation );
  return Result::Success;
}

Result::Enum VClient::GetLabeledMarkerCount( unsigned int & o_rMarkerCount ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  Result::Enum GetResult = Result::Success;
  if ( InitGet( GetResult, o_rMarkerCount ) )
  {
    o_rMarkerCount = static_cast< unsigned int >( m_LatestFrame.m_LabeledRecons.m_LabeledRecons.size() );
  }

  return GetResult;
}

Result::Enum VClient::GetLabeledMarkerGlobalTranslation( const unsigned int   i_MarkerIndex,
                                                               double      (& o_rTranslation)[3] ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rTranslation ) )
  {
    return GetResult;
  }

  if( i_MarkerIndex >= m_LatestFrame.m_LabeledRecons.m_LabeledRecons.size() )
  {
    return Result::InvalidIndex;
  }

  CopyAndTransformT( m_LatestFrame.m_LabeledRecons.m_LabeledRecons[ i_MarkerIndex ].m_Position, o_rTranslation );
  return Result::Success;
}

Result::Enum VClient::GetSegmentGlobalTranslation( const std::string & i_rSubjectName, 
                                                   const std::string & i_rSegmentName, 
                                                         double    ( & o_rThreeVector )[3], 
                                                         bool        & o_rbOccludedFlag ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rThreeVector, o_rbOccludedFlag ) )
  {
    return GetResult; 
  }

  // Look up the ids for the subject and segment
  unsigned int SubjectID = 0;
  unsigned int SegmentID = 0;
  Result::Enum _Result = GetSubjectAndSegmentID( i_rSubjectName, i_rSegmentName, SubjectID, SegmentID );
  if( Result::Success != _Result )
  {
    return _Result;
  }

  // go through the frame's segment data and find its position in this frame
  for (unsigned int i = 0; i < m_LatestFrame.m_GlobalSegments.size(); i++)
  {
    const ViconCGStream::VGlobalSegments& rSegments = m_LatestFrame.m_GlobalSegments[i];
    if (rSegments.m_SubjectID == SubjectID)
    {
      // now look through its segments
      for (unsigned int j = 0; j < rSegments.m_Segments.size(); j++)
      {
        const ViconCGStreamDetail::VGlobalSegments_Segment& rSegment = rSegments.m_Segments[j];
        if (rSegment.m_SegmentID == SegmentID)
        {
          CopyAndTransformT( rSegment.m_Translation, o_rThreeVector );
          return Result::Success;
        }       
      }
    }
  }

  // if we're here then they entered correct subject and marker info,
  // but the data was not in the frame. This is okay but we must
  // set the occluded flag and return true
  o_rbOccludedFlag = true;
  return Result::Success;
}

Result::Enum VClient::GetSegmentGlobalRotationHelical( const std::string & i_rSubjectName, 
                                                       const std::string & i_rSegmentName, 
                                                             double    ( & o_rThreeVector)[3], 
                                                             bool        & o_rbOccluded ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Clear( o_rThreeVector );
  Clear( o_rbOccluded );
  // InitGet is called by GetSegmentGlobalRotationMatrix.

  // Get the answer as a rotation matrix
  double RotationArray[ 9 ];
  const Result::Enum _Result = GetSegmentGlobalRotationMatrix( i_rSubjectName, i_rSegmentName, RotationArray, o_rbOccluded );

  if( Result::Success == _Result && !o_rbOccluded )
  {
    MatrixToHelical( RotationArray, o_rThreeVector );
  }

  return _Result;
}

Result::Enum VClient::GetSegmentGlobalRotationMatrix( const std::string & i_rSubjectName,
                                                      const std::string & i_rSegmentName,
                                                            double     (& o_rRotation)[9],
                                                            bool        & o_rbOccluded ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rRotation, o_rbOccluded ) )
  {
    return GetResult; 
  }

  // Look up the ids for the subject and segment
  unsigned int SubjectID = 0;
  unsigned int SegmentID = 0;
  Result::Enum _Result = GetSubjectAndSegmentID( i_rSubjectName, i_rSegmentName, SubjectID, SegmentID );
  if( Result::Success != _Result )
  {
    return _Result;
  }

  // go through the frame's segment data and find its position in this frame
  std::vector< ViconCGStream::VGlobalSegments >::const_iterator SegmentIt  = m_LatestFrame.m_GlobalSegments.begin();
  std::vector< ViconCGStream::VGlobalSegments >::const_iterator SegmentEnd = m_LatestFrame.m_GlobalSegments.end();
  for( ; SegmentIt != SegmentEnd ; ++SegmentIt )
  {
    const ViconCGStream::VGlobalSegments & rSegments( *SegmentIt );
    if( rSegments.m_SubjectID == SubjectID )
    {
      // now look through the segment detail
      std::vector< ViconCGStreamDetail::VGlobalSegments_Segment >::const_iterator SegmentDetailIt  = rSegments.m_Segments.begin();
      std::vector< ViconCGStreamDetail::VGlobalSegments_Segment >::const_iterator SegmentDetailEnd = rSegments.m_Segments.end();
      for( ; SegmentDetailIt != SegmentDetailEnd ; ++SegmentDetailIt )
      {
        const ViconCGStreamDetail::VGlobalSegments_Segment & rSegment( *SegmentDetailIt );
        if( rSegment.m_SegmentID == SegmentID )
        {
          // copy out the answer
          CopyAndTransformR( rSegment.m_Rotation, o_rRotation );
          o_rbOccluded = false;
          return Result::Success;
        }       
      }
    }
  }

  // if we're here then they entered correct subject and marker info,
  // but the data was not in the frame. This is okay but we must
  // set the occluded flag and return true
  o_rbOccluded = true;
  return Result::Success;
}

Result::Enum VClient::GetSegmentGlobalRotationQuaternion( const std::string & i_rSubjectName, 
                                                          const std::string & i_rSegmentName, 
                                                                double    ( & o_rFourVector)[4], 
                                                                bool        & o_rbOccluded ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Clear( o_rFourVector );
  Clear( o_rbOccluded );
  // InitGet is called by GetSegmentGlobalRotationMatrix.

  // Get the answer as a rotation matrix
  double RotationArray[ 9 ];
  const Result::Enum _Result = GetSegmentGlobalRotationMatrix( i_rSubjectName, i_rSegmentName, RotationArray, o_rbOccluded );

  if( Result::Success == _Result && !o_rbOccluded )
  { 
    MatrixToQuaternion( RotationArray, o_rFourVector );
  }

  return _Result;
}

Result::Enum VClient::GetSegmentGlobalRotationEulerXYZ( const std::string & i_rSubjectName, 
                                                        const std::string & i_rSegmentName, 
                                                              double    ( & o_rThreeVector)[3], 
                                                              bool        & o_rbOccluded ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  Clear( o_rThreeVector );
  Clear( o_rbOccluded );
  // InitGet is called by GetSegmentGlobalRotationMatrix.
  
  // Get the answer as a rotation matrix
  double RotationArray[ 9 ];
  const Result::Enum _Result = GetSegmentGlobalRotationMatrix( i_rSubjectName, i_rSegmentName, RotationArray, o_rbOccluded );

  if( Result::Success == _Result && !o_rbOccluded )
  { 
    MatrixToEulerXYZ( RotationArray, o_rThreeVector );
  }

  return _Result;
}

Result::Enum VClient::GetSegmentStaticRotationHelical( const std::string & i_rSubjectName, 
                                                       const std::string & i_rSegmentName, 
                                                             double    ( & o_rThreeVector)[3] ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Clear( o_rThreeVector );

  // Get the answer as a rotation matrix
  double RotationArray[ 9 ];
  const Result::Enum _Result = GetSegmentStaticRotationMatrix( i_rSubjectName, i_rSegmentName, RotationArray );

  if( Result::Success == _Result )
  {
    MatrixToHelical( RotationArray, o_rThreeVector );
  }

  return _Result;
}

Result::Enum VClient::GetSegmentStaticRotationMatrix( const std::string & i_rSubjectName,
                                                      const std::string & i_rSegmentName,
                                                            double     (& o_rRotation)[9] ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rRotation ) )
  {
    return GetResult; 
  }

  // Look up the ids for the subject and segment
  unsigned int SubjectID = 0;
  unsigned int SegmentID = 0;
  Result::Enum _Result = GetSubjectAndSegmentID( i_rSubjectName, i_rSegmentName, SubjectID, SegmentID );
  if( Result::Success != _Result )
  {
    return _Result;
  }

  const ViconCGStream::VSubjectTopology * pSubjectTopology = GetSubjectTopology( SubjectID );
  if( !pSubjectTopology )
  {
    return Result::InvalidSubjectName;
  }

  // go through the frame's segment data and find its position in this frame
  std::vector< ViconCGStreamDetail::VSubjectTopology_Segment >::const_iterator SegmentIt  = pSubjectTopology->m_Segments.begin();
  std::vector< ViconCGStreamDetail::VSubjectTopology_Segment >::const_iterator SegmentEnd = pSubjectTopology->m_Segments.end();
  for( ; SegmentIt != SegmentEnd ; ++SegmentIt )
  {
    const ViconCGStreamDetail::VSubjectTopology_Segment & rSegment( *SegmentIt );
    if( rSegment.m_SegmentID == SegmentID )
    {
      // copy out the answer
      CopyAndTransformR( rSegment.m_Rotation, o_rRotation );
      return Result::Success;
    }
  }

  return Result::Unknown;
}

Result::Enum VClient::GetSegmentStaticRotationQuaternion( const std::string & i_rSubjectName, 
                                                          const std::string & i_rSegmentName, 
                                                                double    ( & o_rFourVector)[4] ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Clear( o_rFourVector );

  // Get the answer as a rotation matrix
  double RotationArray[ 9 ];
  const Result::Enum _Result = GetSegmentStaticRotationMatrix( i_rSubjectName, i_rSegmentName, RotationArray );

  if( Result::Success == _Result )
  {
    MatrixToQuaternion( RotationArray, o_rFourVector );
  }

  return _Result;
}

Result::Enum VClient::GetSegmentStaticRotationEulerXYZ( const std::string & i_rSubjectName, 
                                                        const std::string & i_rSegmentName, 
                                                              double    ( & o_rThreeVector)[3] ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Clear( o_rThreeVector );

  // Get the answer as a rotation matrix
  double RotationArray[ 9 ];
  const Result::Enum _Result = GetSegmentStaticRotationMatrix( i_rSubjectName, i_rSegmentName, RotationArray );

  if( Result::Success == _Result )
  {
    MatrixToEulerXYZ( RotationArray, o_rThreeVector );
  }

  return _Result;
}

Result::Enum VClient::GetSegmentLocalTranslation( const std::string & i_rSubjectName, 
                                                  const std::string & i_rSegmentName, 
                                                        double    ( & o_rThreeVector)[3], 
                                                        bool        & o_rbOccluded ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rThreeVector, o_rbOccluded ) )
  {
    return GetResult; 
  }

  // Look up the ids for the subject and segment
  unsigned int SubjectID = 0;
  unsigned int SegmentID = 0;
  Result::Enum _Result = GetSubjectAndSegmentID( i_rSubjectName, i_rSegmentName, SubjectID, SegmentID );
  if( Result::Success != _Result )
  {
    return _Result;
  }

  // go through the frame's segment data and find its position in this frame
  for (unsigned int i = 0; i < m_LatestFrame.m_LocalSegments.size(); i++)
  {
    const ViconCGStream::VLocalSegments& rSegments = m_LatestFrame.m_LocalSegments[i];
    if (rSegments.m_SubjectID == SubjectID)
    {
      // now look through its segments
      for (unsigned int j = 0; j < rSegments.m_Segments.size(); j++)
      {
        const ViconCGStreamDetail::VLocalSegments_Segment& rSegment = rSegments.m_Segments[j];
        if (rSegment.m_SegmentID == SegmentID)
        {
          CopyAndTransformT( rSegment.m_Translation, o_rThreeVector );
          return Result::Success;
        }       
      }
    }
  }

  // if we're here then they entered correct subject and marker info,
  // but the data was not in the frame. This is okay but we must
  // set the occluded flag and return true
  o_rbOccluded = true;
  return Result::Success;
}

Result::Enum VClient::GetSegmentLocalRotationHelical( const std::string & i_rSubjectName, 
                                                      const std::string & i_rSegmentName, 
                                                            double    ( & o_rThreeVector)[3], 
                                                            bool        & o_rbOccluded ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Clear( o_rThreeVector );
  Clear( o_rbOccluded );

  // Get the answer as a rotation matrix
  double RotationArray[ 9 ];
  const Result::Enum _Result = GetSegmentLocalRotationMatrix( i_rSubjectName, i_rSegmentName, RotationArray, o_rbOccluded );

  if( Result::Success == _Result && !o_rbOccluded )
  {
    MatrixToHelical( RotationArray, o_rThreeVector );
  }

  return _Result;
}

Result::Enum VClient::GetSegmentLocalRotationMatrix( const std::string & i_rSubjectName,
                                                     const std::string & i_rSegmentName,
                                                           double     (& o_rRotation)[9],
                                                           bool        & o_rbOccluded ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rRotation, o_rbOccluded ) )
  {
    return GetResult; 
  }

  // Look up the ids for the subject and segment
  unsigned int SubjectID = 0;
  unsigned int SegmentID = 0;
  Result::Enum _Result = GetSubjectAndSegmentID( i_rSubjectName, i_rSegmentName, SubjectID, SegmentID );
  if( Result::Success != _Result )
  {
    return _Result;
  }

  // go through the frame's segment data and find its position in this frame
  std::vector< ViconCGStream::VLocalSegments >::const_iterator SegmentIt  = m_LatestFrame.m_LocalSegments.begin();
  std::vector< ViconCGStream::VLocalSegments >::const_iterator SegmentEnd = m_LatestFrame.m_LocalSegments.end();
  for( ; SegmentIt != SegmentEnd ; ++SegmentIt )
  {
    const ViconCGStream::VLocalSegments & rSegments( *SegmentIt );
    if( rSegments.m_SubjectID == SubjectID )
    {
      // now look through the segment detail
      std::vector< ViconCGStreamDetail::VLocalSegments_Segment >::const_iterator SegmentDetailIt  = rSegments.m_Segments.begin();
      std::vector< ViconCGStreamDetail::VLocalSegments_Segment >::const_iterator SegmentDetailEnd = rSegments.m_Segments.end();
      for( ; SegmentDetailIt != SegmentDetailEnd ; ++SegmentDetailIt )
      {
        const ViconCGStreamDetail::VLocalSegments_Segment & rSegment( *SegmentDetailIt );
        if( rSegment.m_SegmentID == SegmentID )
        {
          // copy out the answer
          CopyAndTransformR( rSegment.m_Rotation, o_rRotation );
          o_rbOccluded = false;
          return Result::Success;
        }       
      }
    }
  }

  // if we're here then they entered correct subject and marker info,
  // but the data was not in the frame. This is okay but we must
  // set the occluded flag and return true
  o_rbOccluded = true;
  return Result::Success;
}

Result::Enum VClient::GetSegmentLocalRotationQuaternion( const std::string & i_rSubjectName, 
                                                         const std::string & i_rSegmentName, 
                                                               double    ( & o_rFourVector)[4], 
                                                               bool        & o_rbOccluded ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Clear( o_rFourVector );
  Clear( o_rbOccluded );

  // Get the answer as a rotation matrix
  double RotationArray[ 9 ];
  const Result::Enum _Result = GetSegmentLocalRotationMatrix( i_rSubjectName, i_rSegmentName, RotationArray, o_rbOccluded );

  if( Result::Success == _Result && !o_rbOccluded )
  { 
    MatrixToQuaternion( RotationArray, o_rFourVector );
  }

  return _Result;
}

Result::Enum VClient::GetSegmentLocalRotationEulerXYZ( const std::string & i_rSubjectName, 
                                                       const std::string & i_rSegmentName, 
                                                             double    ( & o_rThreeVector)[3], 
                                                             bool        & o_rbOccluded ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Clear( o_rThreeVector );
  Clear( o_rbOccluded );

  // Get the answer as a rotation matrix
  double RotationArray[ 9 ];
  const Result::Enum _Result = GetSegmentLocalRotationMatrix( i_rSubjectName, i_rSegmentName, RotationArray, o_rbOccluded );

  if( Result::Success == _Result && !o_rbOccluded )
  { 
    MatrixToEulerXYZ( RotationArray, o_rThreeVector );
  }

  return _Result;
}

Result::Enum VClient::GetMarkerID( const ViconCGStream::VSubjectInfo & i_rSubjectInfo, const std::string& i_rMarkerName, unsigned int& o_rMarkerID ) const
{
  if( i_rMarkerName.empty() )
  {
    return Result::InvalidMarkerName;
  }

  const std::vector< ViconCGStreamDetail::VSubjectInfo_Marker > & rMarkers = i_rSubjectInfo.m_Markers;
  for( unsigned int j = 0 ; j < rMarkers.size() ; ++j )
  {
    const ViconCGStreamDetail::VSubjectInfo_Marker & rMarker = rMarkers[j];
    if( i_rMarkerName == rMarker.m_Name )
    {
      o_rMarkerID = rMarker.m_MarkerID;
      return Result::Success;
    }
  }

  return Result::InvalidMarkerName;
}

Result::Enum VClient::GetSubjectAndMarkerID( const std::string  & i_rSubjectName, 
                                             const std::string  & i_rMarkerName, 
                                                   unsigned int & o_rSubjectID, 
                                                   unsigned int & o_rMarkerID ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Clear( o_rSubjectID );
  Clear( o_rMarkerID );

  Result::Enum GetResult = Result::Success;
  const ViconCGStream::VSubjectInfo * pSubjectInfo = GetSubjectInfo( i_rSubjectName, GetResult );
  if ( pSubjectInfo )
  {
    GetResult = GetMarkerID( *pSubjectInfo, i_rMarkerName, o_rMarkerID );
    if ( GetResult == Result::Success )
    {
      o_rSubjectID = pSubjectInfo->m_SubjectID;
    }
  }
  return GetResult;
}

Result::Enum VClient::GetSegmentID( const ViconCGStream::VSubjectInfo & i_rSubjectInfo, const std::string& i_rSegmentName, unsigned int& o_rSegmentID ) const
{
  if( i_rSegmentName.empty() )
  {
    return Result::InvalidSegmentName;
  }

  const std::vector< ViconCGStreamDetail::VSubjectInfo_Segment > & rSegments = i_rSubjectInfo.m_Segments;
  for ( unsigned int j = 0; j < rSegments.size(); ++j )
  {
    const ViconCGStreamDetail::VSubjectInfo_Segment & rSegment = rSegments[j];
    if (i_rSegmentName == rSegment.m_Name)
    {
      o_rSegmentID = rSegment.m_SegmentID;
      return Result::Success;
    }
  }
  return Result::InvalidSegmentName;
}

Result::Enum VClient::GetSubjectAndSegmentID( const std::string  & i_rSubjectName, 
                                              const std::string  & i_rSegmentName, 
                                                    unsigned int & o_rSubjectID, 
                                                    unsigned int & o_rSegmentID ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Clear( o_rSubjectID );
  Clear( o_rSegmentID );

  Result::Enum GetResult = Result::Success;
  const ViconCGStream::VSubjectInfo * pSubjectInfo = GetSubjectInfo( i_rSubjectName, GetResult );
  if ( pSubjectInfo )
  {
    GetResult = GetSegmentID( *pSubjectInfo, i_rSegmentName, o_rSegmentID );
    if ( GetResult == Result::Success )
    {
      o_rSubjectID = pSubjectInfo->m_SubjectID;
    }
  }
  return GetResult;
}

const ViconCGStream::VDeviceInfo * VClient::GetDevice( const std::string & i_rDeviceName, Result::Enum & o_rResult ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  const auto DeviceIt =
    std::find_if( m_LatestFrame.m_Devices.begin(), m_LatestFrame.m_Devices.end(),
      [&i_rDeviceName]( const ViconCGStream::VDeviceInfo& rDevice )
        { return i_rDeviceName == AdaptDeviceName( rDevice.m_Name, rDevice.m_DeviceID ); } 
    );
  
  if( DeviceIt != m_LatestFrame.m_Devices.end() )
  {
    o_rResult = Result::Success;
    return &(*DeviceIt);
  }
  o_rResult = Result::InvalidDeviceName;
  return nullptr;
}

Result::Enum VClient::GetDeviceID( const std::string & i_rDeviceName, unsigned int & o_rDeviceID ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  Clear( o_rDeviceID );

  Result::Enum GetResult = Result::Success;
  const ViconCGStream::VDeviceInfo * pDevice = GetDevice( i_rDeviceName, GetResult );
  if ( pDevice )
  {
    o_rDeviceID = pDevice->m_DeviceID;
  }
  return GetResult;
}

Result::Enum VClient::GetReconRayAssignments( const std::string& i_rSubjectName, const std::string& i_rMarkerName, std::vector< unsigned int >& o_rCameraIDs, std::vector< unsigned int >& o_rCentroidIndex ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  Result::Enum GetResult = Result::Success;
  if( !InitGet( GetResult, o_rCameraIDs ) )
  {
    return GetResult;
  }

  unsigned int SubjectID = 0;
  unsigned int MarkerID = 0;
  Result::Enum _Result = GetSubjectAndMarkerID( i_rSubjectName, i_rMarkerName, SubjectID, MarkerID );
  if( Result::Success != _Result )
  {
    return _Result;
  }

  // go through the frame's reconstructions and find the ray contributions
  for( unsigned int i = 0; i < m_LatestFrame.m_LabeledReconRayAssignments.m_ReconRayAssignments.size(); ++i )
  {
    const ViconCGStreamDetail::VReconRayAssignments& rReconAssignments = m_LatestFrame.m_LabeledReconRayAssignments.m_ReconRayAssignments[i];
    if( rReconAssignments.m_SubjectID == SubjectID && rReconAssignments.m_MarkerID == MarkerID )
    {
      for( const auto & rReconRay : rReconAssignments.m_ReconRays )
      {
        o_rCameraIDs.push_back( rReconRay.m_CameraID );
        o_rCentroidIndex.push_back( rReconRay.m_CentroidIndex );
      }
    }
  }
  return Result::Success;
}

const ViconCGStream::VSubjectInfo * VClient::GetSubjectInfo( const std::string  i_rSubjectName, Result::Enum & o_rResult ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  if ( !InitGet( o_rResult ) )
  {
    return NULL; 
  }
  
  o_rResult = Result::InvalidSubjectName;

  if( i_rSubjectName.empty() )
  {
    return NULL;
  }

  std::vector< ViconCGStream::VSubjectInfo >::const_iterator It  = m_LatestFrame.m_Subjects.begin();
  std::vector< ViconCGStream::VSubjectInfo >::const_iterator End = m_LatestFrame.m_Subjects.end();
  for( ; It != End ; ++It )
  {
    if( i_rSubjectName == It->m_Name )
    {
      o_rResult = Result::Success;
      return &(*It);
    }
  }

  return NULL;
}

const ViconCGStream::VSubjectTopology * VClient::GetSubjectTopology( const unsigned int i_SubjectID ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  std::vector< ViconCGStream::VSubjectTopology >::const_iterator It  = m_LatestFrame.m_SubjectTopologies.begin();
  std::vector< ViconCGStream::VSubjectTopology >::const_iterator End = m_LatestFrame.m_SubjectTopologies.end();
  for( ; It != End ; ++It )
  {
    if( i_SubjectID == It->m_SubjectID )
    {
      return &(*It);
    }
  }

  return 0;
}

const ViconCGStream::VObjectQuality * VClient::GetObjectQuality( const unsigned int i_SubjectID ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  std::vector< ViconCGStream::VObjectQuality >::const_iterator It = m_LatestFrame.m_ObjectQualities.begin();
  std::vector< ViconCGStream::VObjectQuality >::const_iterator End = m_LatestFrame.m_ObjectQualities.end();
  for( ; It != End; ++It )
  {
    if( i_SubjectID == It->m_SubjectID )
    {
      return &( *It );
    }
  }

  return 0;
}

const ViconCGStream::VCentroids * VClient::GetCentroidSet( const unsigned int i_CameraID, Result::Enum & o_rResult ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  const auto rCentroidSetIt = std::find_if( m_LatestFrame.m_Centroids.begin(), m_LatestFrame.m_Centroids.end(),
                                            [&i_CameraID]( const ViconCGStream::VCentroids & rSet )
                                            {
                                              return rSet.m_CameraID == i_CameraID;
                                            } );

  if( rCentroidSetIt != m_LatestFrame.m_Centroids.end() )
  {
    o_rResult = Result::Success;
    return &(*rCentroidSetIt);
  }

  o_rResult = Result::InvalidIndex;
  return nullptr;
}

const ViconCGStream::VCentroidWeights * VClient::GetCentroidWeightSet( const unsigned int i_CameraID, Result::Enum & o_rResult ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  const auto rCentroidWeightSetIt = std::find_if( m_LatestFrame.m_CentroidWeights.begin(), m_LatestFrame.m_CentroidWeights.end(),
    [&i_CameraID]( const ViconCGStream::VCentroidWeights & rSet )
  {
    return rSet.m_CameraID == i_CameraID;
  } );

  if( rCentroidWeightSetIt != m_LatestFrame.m_CentroidWeights.end() )
  {
    o_rResult = Result::Success;
    return &( *rCentroidWeightSetIt );
  }

  o_rResult = Result::InvalidIndex;
  return nullptr;
}

const ViconCGStream::VGreyscaleBlobs  * VClient::GetGreyscaleBlobs( const unsigned int i_CameraID, Result::Enum & o_rResult ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  const auto rGreyscaleBlobIt = std::find_if( m_LatestFrame.m_GreyscaleBlobs.begin(), m_LatestFrame.m_GreyscaleBlobs.end(),
                                              [&i_CameraID]( const ViconCGStream::VGreyscaleBlobs & rSet )
                                              {
                                                return rSet.m_CameraID == i_CameraID;
                                              } );

  if( rGreyscaleBlobIt != m_LatestFrame.m_GreyscaleBlobs.end() )
  {
    o_rResult = Result::Success;
    return &(*rGreyscaleBlobIt);
  }

  o_rResult = Result::InvalidIndex;
  return nullptr;
}

void VClient::GetVideoFrame( const unsigned int i_CameraID, Result::Enum & o_rResult, ViconCGStreamClientSDK::VVideoFramePtr & o_rVideoFramePtr ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  const auto rVideoFramePtrIt = std::find_if( m_LatestFrame.m_VideoFrames.begin(), m_LatestFrame.m_VideoFrames.end(),
                                              [&i_CameraID]( const ViconCGStreamClientSDK::VVideoFramePtr & rPtr )
                                              {
                                                return (*rPtr).m_CameraID == i_CameraID;
                                              } );

  if( rVideoFramePtrIt != m_LatestFrame.m_VideoFrames.end() )
  {
    o_rResult = Result::Success;
    o_rVideoFramePtr = *rVideoFramePtrIt;
  }
  else
  {
    o_rResult = Result::InvalidIndex;
    o_rVideoFramePtr.Clear();
  }
}

bool VClient::IsForcePlateDevice( unsigned int i_DeviceID ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  unsigned int RelevantChannels = 0;

  // check for any channel information that would mean this is a forceplate
  for (unsigned int j = 0; j < m_LatestFrame.m_Channels.size(); j++)
  {
    const ViconCGStream::VChannelInfo& rChannel = m_LatestFrame.m_Channels[j];

    if (i_DeviceID == rChannel.m_DeviceID && IsForcePlateCoreChannel(rChannel))
    {
      RelevantChannels++;
    }
  }

  return 3 == RelevantChannels;
}

// the function below determines whether the device channel is one of the "core" forceplate
// data channels that are always present, namely Forces, Moments and Centre of Pressure.
// There might be other channels for forceplates, but they would always have one of these.
bool VClient::IsForcePlateCoreChannel( const ViconCGStream::VChannelInfo & rChannel ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  return IsForcePlateForceChannel(  rChannel ) ||
         IsForcePlateMomentChannel( rChannel ) ||
         IsForcePlateCoPChannel(    rChannel );
}

bool VClient::IsForcePlateForceChannel( const ViconCGStream::VChannelInfo & rChannel ) const
{
  const std::vector< std::string > & rNames = rChannel.m_ComponentNames;
  
  return ( rNames.size() == 3    ) &&
         ( rNames[ 0 ]   == "Fx" ) &&
         ( rNames[ 1 ]   == "Fy" ) &&
         ( rNames[ 2 ]   == "Fz" );
}

bool VClient::IsForcePlateMomentChannel( const ViconCGStream::VChannelInfo & rChannel ) const
{
  const std::vector< std::string > & rNames = rChannel.m_ComponentNames;
  
  return ( rNames.size() == 3    ) &&
         ( rNames[ 0 ]   == "Mx" ) &&
         ( rNames[ 1 ]   == "My" ) &&
         ( rNames[ 2 ]   == "Mz" );
}

bool VClient::IsForcePlateCoPChannel( const ViconCGStream::VChannelInfo & rChannel ) const
{
  const std::vector< std::string > & rNames = rChannel.m_ComponentNames;
  
  return ( rNames.size() == 3    ) &&
         ( rNames[ 0 ]   == "Cx" ) &&
         ( rNames[ 1 ]   == "Cy" ) &&
         ( rNames[ 2 ]   == "Cz" );
}

Result::Enum VClient::GetForcePlateCount( unsigned int & o_rCount ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rCount ) )
  {
    return GetResult; 
  }

  unsigned int ForcePlates = 0;

  for( unsigned int i = 0; i < m_LatestFrame.m_Devices.size(); i++ )
  {
    if( IsForcePlateDevice( m_LatestFrame.m_Devices[i].m_DeviceID ) )
    {
      ForcePlates++;
    }
  }

  o_rCount = ForcePlates;
  return Result::Success;
}

Result::Enum VClient::GetForcePlateID( unsigned int i_ZeroIndexedPlateIndex, unsigned int& o_rPlateID ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rPlateID ) )
  {
    return GetResult; 
  }

  unsigned int ForcePlates = 0;

  for( unsigned int i = 0 ; i < m_LatestFrame.m_Devices.size() ; ++i )
  {
    if( !IsForcePlateDevice( m_LatestFrame.m_Devices[i].m_DeviceID ) )
    {
      continue;
    }
    
    if( ForcePlates == i_ZeroIndexedPlateIndex )
    {
      // this is our forceplate
      o_rPlateID = m_LatestFrame.m_Devices[i].m_DeviceID;
      return Result::Success;
    }
    else
    {
      ++ForcePlates;
    }
  }

  return Result::InvalidIndex;
}

bool VClient::ForcePlateDeviceIndex( const unsigned int i_DeviceID, unsigned int & o_rZeroBasedIndex ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  for(unsigned int j = 0; j < m_LatestFrame.m_ForcePlates.size(); ++j )
  {
    const ViconCGStream::VForcePlateInfo & rForcePlate = m_LatestFrame.m_ForcePlates[j];

    if( i_DeviceID == rForcePlate.m_DeviceID )
    {
      o_rZeroBasedIndex = j;
      return true;
    }
  }

  return false;
}

// Internal access for the subsamples.
Result::Enum VClient::ForcePlateSubsamples( unsigned int i_PlateID, unsigned int & o_rForcePlateSubsamples ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rForcePlateSubsamples ) )
  {
    return GetResult; 
  }

  if( !IsForcePlateDevice( i_PlateID ) )
  {
    return Result::InvalidIndex;
  }

  const ViconCGStreamType::UInt64 DevicePeriod = GetDevicePeriod( i_PlateID );
  const ViconCGStreamType::UInt64 DeviceStartTick = GetDeviceStartTick( i_PlateID );
  const TPeriod FramePeriod = GetFramePeriod( m_LatestFrame );

  for( unsigned int i = 0 ; i < m_LatestFrame.m_Forces.size() ; ++i )
  {
    const ViconCGStream::VForceFrame& rForces = m_LatestFrame.m_Forces[i];
    if( rForces.m_DeviceID == i_PlateID )
    {
      const size_t NumSamples = rForces.m_Samples.size() / 3;
      if ( !NumSamples || DevicePeriod % NumSamples != 0 )
      {
        return Result::Unknown;
      }

      const ViconCGStreamType::UInt64 SamplePeriod = DevicePeriod / NumSamples;

      o_rForcePlateSubsamples = GetSamplesInFrame( FramePeriod, DeviceStartTick, SamplePeriod );
      return Result::Success;
    }
  }

  return Result::Unknown;
}

template < typename T >
Result::Enum ViconDataStreamSDK::Core::VClient::GetForcePlateVector( const unsigned int i_PlateID,
                                                                     const unsigned int i_ForcePlateSubsamples,
                                                                     const std::vector< T > & i_rFrameVector,
                                                                     boost::array< double, 3 > & o_rForcePlateVector ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult ) )
  {
    return GetResult; 
  }

  if( !IsForcePlateDevice( i_PlateID ) )
  {
    return Result::InvalidIndex;
  }

  const ViconCGStreamType::UInt64 DevicePeriod = GetDevicePeriod( i_PlateID );

  if ( !DevicePeriod )
  {
    return Result::InvalidIndex;
  }

  const ViconCGStreamType::UInt64 DeviceOffset = GetDeviceStartTick( i_PlateID );

  const TPeriod FramePeriod = GetFramePeriod( m_LatestFrame );

  for( unsigned int i = 0 ; i < i_rFrameVector.size() ; ++i )
  {
    const T& rFrame = i_rFrameVector[i];
    if( rFrame.m_DeviceID == i_PlateID )
    {
      const size_t NumSamples = rFrame.m_Samples.size() / 3;
      
      if( NumSamples == 0 || DevicePeriod % NumSamples != 0 )
      {
        return Result::Unknown;
      }

      const ViconCGStreamType::UInt64 SamplePeriod = DevicePeriod / NumSamples;

      if ( i_ForcePlateSubsamples >= GetSamplesInFrame( FramePeriod, DeviceOffset, SamplePeriod ) )
      {
        // Out of range for the entire frame.
        return Result::InvalidIndex;
      }

      unsigned int SubSample = 0;
      if( !SampleIndexInRange( i_ForcePlateSubsamples, rFrame.m_FrameID, NumSamples, SamplePeriod, DeviceOffset, FramePeriod, SubSample ) )
      {
        // Not in the current set of samples - might be in a later one though.
        continue;
      }

      o_rForcePlateVector[0] = rFrame.m_Samples[ 0 + SubSample ];
      o_rForcePlateVector[1] = rFrame.m_Samples[ NumSamples + SubSample ];
      o_rForcePlateVector[2] = rFrame.m_Samples[ NumSamples * 2 + SubSample ];

      return Result::Success;
    }   
  }

  return Result::Unknown;
}

// Internal function used by local and global force functions.
Result::Enum VClient::GetForceVector( const unsigned int i_PlateID, 
                                      const unsigned int i_ForcePlateSubsamples,
                                      boost::array< double, 3 > & o_rForceVector ) const
{
  return GetForcePlateVector( i_PlateID, i_ForcePlateSubsamples, m_LatestFrame.m_Forces, o_rForceVector );
}

// Internal function used by local and global moment functions.
Result::Enum VClient::GetMomentVector( const unsigned int i_PlateID, 
                                       const unsigned int i_ForcePlateSubsamples,
                                       boost::array< double, 3 > & o_rMomentVector ) const
{
  return GetForcePlateVector( i_PlateID, i_ForcePlateSubsamples, m_LatestFrame.m_Moments, o_rMomentVector );
}

// Internal function used by local and global CoP functions.
Result::Enum VClient::GetCentreOfPressure( const unsigned int i_PlateID, 
                                           const unsigned int i_ForcePlateSubsamples,
                                           boost::array< double, 3 > & o_rLocation ) const
{
  return GetForcePlateVector( i_PlateID, i_ForcePlateSubsamples, m_LatestFrame.m_CentresOfPressure, o_rLocation );
}

Result::Enum VClient::GetForceVector( const unsigned int i_PlateID, 
                                      const unsigned int i_Subsample, 
                                      double ( & o_rForceVector)[3] ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Clear( o_rForceVector );

  boost::array< double, 3 > ForceVector;
  Result::Enum Result = GetForceVector( i_PlateID, i_Subsample, ForceVector );

  if( Result == Result::Success )
  {
    CopyAndTransformT( ForceVector.data(), o_rForceVector );
  }

  return Result;
}

Result::Enum VClient::GetMomentVector( const unsigned int i_PlateID, 
                                       const unsigned int i_Subsample, 
                                       double ( & o_rMomentVector )[3] ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Clear( o_rMomentVector );

  boost::array< double, 3 > MomentVector;
  Result::Enum Result = GetMomentVector( i_PlateID, i_Subsample, MomentVector );

  if( Result == Result::Success )
  {
    CopyAndTransformT( MomentVector.data(), o_rMomentVector );
  }

  return Result;
}

Result::Enum VClient::GetCentreOfPressure( const unsigned int i_PlateID, 
                                           const unsigned int i_Subsample, 
                                           double ( & o_rLocation )[3] ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Clear( o_rLocation );

  boost::array< double, 3 > Location;
  Result::Enum Result = GetCentreOfPressure( i_PlateID, i_Subsample, Location );

  if( Result == Result::Success )
  {
    CopyAndTransformT( Location.data(), o_rLocation );
  }

  return Result;
}

Result::Enum VClient::GetGlobalForceVector( const unsigned int i_PlateID, 
                                            const unsigned int i_Subsample, 
                                            double ( & o_rForceVector)[3] ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Clear( o_rForceVector );

  boost::array< double, 3 > ForceVector;

  Result::Enum Result = GetForceVector( i_PlateID, i_Subsample, ForceVector );

  if( Result == Result::Success )
  {
    // Get the plate index.

    unsigned int PlateIndex;
    if( !ForcePlateDeviceIndex( i_PlateID, PlateIndex ) )
    {
      return Result::Unknown;
    }

    // Transform result to global coordinates by rotating by plate orientation.

    const ViconCGStream::VForcePlateInfo & rForcePlate = m_LatestFrame.m_ForcePlates[ PlateIndex ];

    boost::array< double, 3 * 3 > WorldRotation;
    std::copy( rForcePlate.m_WorldRotation, rForcePlate.m_WorldRotation + 9, WorldRotation.begin() );

    boost::array< double, 3 > WorldForceVector;

    WorldForceVector = WorldRotation * ForceVector;

    CopyAndTransformT( WorldForceVector.data(), o_rForceVector );
  }

  return Result;
}

Result::Enum VClient::GetGlobalMomentVector( const unsigned int i_PlateID, 
                                             const unsigned int i_Subsample, 
                                             double ( & o_rMomentVector )[3] ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Clear( o_rMomentVector );

  boost::array< double, 3 > MomentVector;
  Result::Enum Result = GetMomentVector( i_PlateID, i_Subsample, MomentVector );

  if( Result == Result::Success )
  {
    unsigned int PlateIndex;
    if( !ForcePlateDeviceIndex( i_PlateID, PlateIndex ) )
    {
      return Result::Unknown;
    }

    const ViconCGStream::VForcePlateInfo & rForcePlate = m_LatestFrame.m_ForcePlates[ PlateIndex ];

    boost::array< double, 3 * 3 > WorldRotation;
    std::copy( rForcePlate.m_WorldRotation, rForcePlate.m_WorldRotation + 9, WorldRotation.begin() );

    boost::array< double, 3 > WorldMomentVector;

    WorldMomentVector = WorldRotation * MomentVector;

    CopyAndTransformT( WorldMomentVector.data(), o_rMomentVector );
  }

  return Result;
}

Result::Enum VClient::GetGlobalCentreOfPressure( const unsigned int i_PlateID, 
                                                 const unsigned int i_Subsample, 
                                                 double ( & o_rLocation )[3] ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Clear( o_rLocation );

  boost::array< double, 3 > LocationVector;
  Result::Enum Result = GetCentreOfPressure( i_PlateID, i_Subsample, LocationVector );

  if( Result == Result::Success )
  {
    unsigned int PlateIndex;
    if( !ForcePlateDeviceIndex( i_PlateID, PlateIndex ) )
    {
      return Result::Unknown;
    }

    const ViconCGStream::VForcePlateInfo & rForcePlate = m_LatestFrame.m_ForcePlates[ PlateIndex ];

    boost::array< double, 3 * 3 > WorldRotation;
    std::copy( rForcePlate.m_WorldRotation, rForcePlate.m_WorldRotation + 9, WorldRotation.begin() );

    boost::array< double, 3 > WorldTranslation;
    std::copy( rForcePlate.m_WorldTranslation, rForcePlate.m_WorldTranslation + 3, WorldTranslation.begin() );

    // Force Plate Info is in mm.  Result CoP is in meters.
    WorldTranslation /= 1000.0;

    boost::array< double, 3 > WorldLocationVector;

    WorldLocationVector = WorldRotation * LocationVector + WorldTranslation;

    CopyAndTransformT( WorldLocationVector.data(), o_rLocation );
  }

  return Result;
}

Result::Enum VClient::GetForcePlateSubsamples( const unsigned int i_PlateID, unsigned int& o_rForcePlateSubsamples ) const
{
  return ForcePlateSubsamples( i_PlateID, o_rForcePlateSubsamples );
}

Result::Enum VClient::GetForceVector( const unsigned int i_PlateID, double ( & o_rForceVector)[3] ) const
{
  return GetForceVector( i_PlateID, 0, o_rForceVector );
}

Result::Enum VClient::GetMomentVector( const unsigned int i_PlateID, double ( & o_rMomentVector)[3] ) const
{
  return GetMomentVector( i_PlateID, 0, o_rMomentVector );
}

Result::Enum VClient::GetCentreOfPressure( const unsigned int i_PlateID, double ( & o_rLocation )[3] ) const
{
  return GetCentreOfPressure( i_PlateID, 0, o_rLocation );
}

Result::Enum VClient::GetGlobalForceVector( const unsigned int i_PlateID, double ( & o_rForceVector)[3] ) const
{
  return GetGlobalForceVector( i_PlateID, 0, o_rForceVector );
}

Result::Enum VClient::GetGlobalMomentVector( const unsigned int i_PlateID, double ( & o_rMomentVector)[3] ) const
{
  return GetGlobalMomentVector( i_PlateID, 0, o_rMomentVector );
}

Result::Enum VClient::GetGlobalCentreOfPressure( const unsigned int i_PlateID, double ( & o_rLocation )[3] ) const
{
  return GetGlobalCentreOfPressure( i_PlateID, 0, o_rLocation );
}

// Comment explaining about analog components
Result::Enum VClient::GetNumberOfAnalogChannels( const unsigned int i_PlateID, unsigned int& o_rChannelCount ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rChannelCount ) )
  {
    return GetResult; 
  }

  if( !IsForcePlateDevice( i_PlateID ) )
  {
    return Result::InvalidIndex;
  }

  // now find channel information that is not
  for( size_t j = 0 ; j < m_LatestFrame.m_Channels.size() ; ++j )
  {
    const ViconCGStream::VChannelInfo& rChannel = m_LatestFrame.m_Channels[j];

    if( i_PlateID == rChannel.m_DeviceID && !IsForcePlateCoreChannel( rChannel ) )
    {
      // we assume it is an analogue channel
      o_rChannelCount = static_cast< unsigned int >( rChannel.m_ComponentNames.size() );
      return Result::Success;
    }
  }
  return Result::Unknown;
}

Result::Enum VClient::GetAnalogChannelVoltage( const unsigned int i_PlateID, 
                                               const unsigned int i_ChannelIndex, 
                                               double & o_rVoltage) const
{ 
  return GetAnalogChannelVoltage( i_PlateID, i_ChannelIndex, 0, o_rVoltage );
}

Result::Enum VClient::GetAnalogChannelVoltage( const unsigned int i_PlateID, 
                                               const unsigned int i_ChannelIndex, 
                                               const unsigned int i_Subsample,
                                               double & o_rVoltage) const
{ 
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rVoltage ) )
  {
    return GetResult; 
  }

  if( !IsForcePlateDevice( i_PlateID ) )
  {
    return Result::InvalidIndex;
  }

  const ViconCGStreamType::UInt64 DevicePeriod = GetDevicePeriod( i_PlateID );

  if ( !DevicePeriod )
  {
    return Result::InvalidIndex;
  }

  unsigned int ChannelID = 0;
  unsigned int VoltageComponentsPerSample = 0;
  bool bFoundChannelID = false;

  // get the channel ID for the voltage channel of this plate
  for( size_t j = 0 ; j < m_LatestFrame.m_Channels.size() ; ++j )
  {
    const ViconCGStream::VChannelInfo& rChannel = m_LatestFrame.m_Channels[j];

    if( i_PlateID == rChannel.m_DeviceID && !IsForcePlateCoreChannel( rChannel ) )
    {
      ChannelID = rChannel.m_ChannelID;
      VoltageComponentsPerSample = static_cast< unsigned int >( rChannel.m_ComponentNames.size() );
      bFoundChannelID = true;
      break;
    }
  }

  if( !bFoundChannelID )
  {
    return Result::InvalidIndex;
  }

  const TPeriod FramePeriod = GetFramePeriod( m_LatestFrame );
  const ViconCGStreamType::UInt64 DeviceStartTick = GetDeviceStartTick( i_PlateID );

  // now look through the voltage channels for this ID
  // subfactor the voltage values by "VoltageComponentsPerSample"

  for( size_t i = 0 ; i < m_LatestFrame.m_Voltages.size() ; ++i )
  {
    const ViconCGStream::VVoltageFrame& rVoltages = m_LatestFrame.m_Voltages[i];

    if( rVoltages.m_ChannelID == ChannelID )
    {
      // get the voltages out of here
      size_t NumberOfVoltageSets = rVoltages.m_Samples.size() / VoltageComponentsPerSample;

      if( 0 == NumberOfVoltageSets || DevicePeriod % NumberOfVoltageSets != 0 )
      {
        return Result::Unknown;
      }

      const ViconCGStreamType::UInt64 SamplePeriod = DevicePeriod / NumberOfVoltageSets;

      if ( i_Subsample >= GetSamplesInFrame( FramePeriod, DeviceStartTick, SamplePeriod ) )
      {
        // Out of range for the entire frame.
        return Result::InvalidIndex;
      }

      unsigned int SubSample = 0;
      if( !SampleIndexInRange( i_Subsample, rVoltages.m_FrameID, NumberOfVoltageSets, SamplePeriod, DeviceStartTick, FramePeriod, SubSample ) )
      {
        // Not in the current set of samples - might be in a later one though.
        continue;
      }

      // Look up the sample we want.
      o_rVoltage = rVoltages.m_Samples[ i_ChannelIndex * NumberOfVoltageSets + SubSample ];
      return Result::Success;
    }
  }

  return Result::Unknown;

}

Result::Enum VClient::GetEyeTrackerCount( unsigned int & o_rCount ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( InitGet( GetResult, o_rCount ) )
  {
    o_rCount = static_cast<unsigned int>( m_LatestFrame.m_EyeTrackers.size() );
  }
  return GetResult;
}

Result::Enum VClient::GetEyeTrackerID( const unsigned int i_EyeTrackerIndex, unsigned int& o_rEyeTrackerID ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rEyeTrackerID ) )
  {
    return GetResult; 
  }

  if( i_EyeTrackerIndex < m_LatestFrame.m_EyeTrackers.size() )
  {
    o_rEyeTrackerID = m_LatestFrame.m_EyeTrackers[ i_EyeTrackerIndex ].m_DeviceID;
    return Result::Success;
  }

  return Result::InvalidIndex;
}

Result::Enum VClient::GetEyeTrackerGlobalPosition( const unsigned int i_EyeTrackerID, double (&o_rThreeVector)[3], bool& o_rbOccludedFlag ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rThreeVector, o_rbOccludedFlag ) )
  {
    return GetResult; 
  }

  size_t EyeTrackerIndex = -1;

  for( size_t i = 0; i < m_LatestFrame.m_EyeTrackers.size(); i++ )
  {
    if( m_LatestFrame.m_EyeTrackers[ i ].m_DeviceID == i_EyeTrackerID )
    {
      EyeTrackerIndex = i;
    }
  }

  if( EyeTrackerIndex == BadIndex )
  {
    return Result::InvalidIndex;
  }

  const ViconCGStream::VEyeTrackerInfo & rEyeTracker = m_LatestFrame.m_EyeTrackers[ EyeTrackerIndex ];

  // Look up the ids for the subject and segment
  unsigned int SubjectID = rEyeTracker.m_SubjectID;
  unsigned int SegmentID = rEyeTracker.m_SegmentID;

  // go through the frame's segment data and find its position in this frame
  for( unsigned int i = 0; i < m_LatestFrame.m_GlobalSegments.size(); i++ )
  {
    const ViconCGStream::VGlobalSegments& rSegments = m_LatestFrame.m_GlobalSegments[i];
    if( rSegments.m_SubjectID == SubjectID )
    {
      // now look through its segments
      for( unsigned int j = 0; j < rSegments.m_Segments.size(); j++ )
      {
        const ViconCGStreamDetail::VGlobalSegments_Segment& rSegment = rSegments.m_Segments[j];
        if( rSegment.m_SegmentID == SegmentID )
        {
          // Use the segment to calculate global eye location.

          boost::array< double, 3 * 3 > WorldRotation;
          std::copy( rSegment.m_Rotation, rSegment.m_Rotation + 9, WorldRotation.begin() );

          boost::array< double, 3 > WorldTranslation;
          std::copy( rSegment.m_Translation, rSegment.m_Translation + 3, WorldTranslation.begin() );

          boost::array< double, 3 > EyeTranslation;
          std::copy( rEyeTracker.m_LocalTranslation, rEyeTracker.m_LocalTranslation + 3, EyeTranslation.begin() );

          const boost::array< double, 3 > WorldEyeTranslation = WorldRotation * EyeTranslation + WorldTranslation;

          CopyAndTransformT( WorldEyeTranslation.data(), o_rThreeVector );
          return Result::Success;
        }       
      }
    }
  }

  // If we fail to find the segment it is probably just failed to fit.
  // We mark this as an occluded success.

  o_rbOccludedFlag = true;
  return Result::Success;
}

Result::Enum VClient::GetEyeTrackerGlobalGazeVector( const unsigned int i_EyeTrackerID, double (&o_rThreeVector)[3], bool& o_rbOccludedFlag ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rThreeVector, o_rbOccludedFlag ) )
  {
    return GetResult; 
  }

  size_t EyeTrackerIndex = -1;

  for( size_t i = 0; i < m_LatestFrame.m_EyeTrackers.size(); i++ )
  {
    if( m_LatestFrame.m_EyeTrackers[ i ].m_DeviceID == i_EyeTrackerID )
    {
      EyeTrackerIndex = i;
    }
  }

  if( EyeTrackerIndex == BadIndex )
  {
    return Result::InvalidIndex;
  }

  const ViconCGStream::VEyeTrackerInfo & rEyeTracker = m_LatestFrame.m_EyeTrackers[ EyeTrackerIndex ];

  size_t EyeTrackIndex = -1;

  for( size_t i = 0; i < m_LatestFrame.m_EyeTracks.size(); i++ )
  {
    if( m_LatestFrame.m_EyeTracks[ i ].m_DeviceID == i_EyeTrackerID )
    {
      EyeTrackIndex = i;
    }
  }

  if( EyeTrackIndex == BadIndex )
  {
    // If we fail to find the eye track information then the 
    // the user is probably blinking or similar.
    // This is a successfull call for an occluded sample.
    o_rbOccludedFlag = true;
    return Result::Success;
  }

  const ViconCGStream::VEyeTrackerFrame & rEyeTrack = m_LatestFrame.m_EyeTracks[ EyeTrackIndex ];

  // Look up the ids for the subject and segment
  unsigned int SubjectID = rEyeTracker.m_SubjectID;
  unsigned int SegmentID = rEyeTracker.m_SegmentID;

  // go through the frame's segment data and find its position in this frame
  for( unsigned int i = 0; i < m_LatestFrame.m_GlobalSegments.size(); i++ )
  {
    const ViconCGStream::VGlobalSegments& rSegments = m_LatestFrame.m_GlobalSegments[i];
    if( rSegments.m_SubjectID == SubjectID )
    {
      // now look through its segments
      for( unsigned int j = 0; j < rSegments.m_Segments.size(); j++ )
      {
        const ViconCGStreamDetail::VGlobalSegments_Segment& rSegment = rSegments.m_Segments[j];
        if( rSegment.m_SegmentID == SegmentID )
        {
          // Use the segment to calculate global eye location.

          boost::array< double, 3 * 3 > WorldRotation;
          std::copy( rSegment.m_Rotation, rSegment.m_Rotation + 9, WorldRotation.begin() );

          boost::array< double, 3 * 3 > EyeRotation;
          std::copy( rEyeTracker.m_LocalRotation, rEyeTracker.m_LocalRotation + 9, EyeRotation.begin() );

          boost::array< double, 3 > EyeGaze;
          std::copy( rEyeTrack.m_GazeVector, rEyeTrack.m_GazeVector + 3, EyeGaze.begin() );

          const boost::array< double, 3 > WorldGazeVector = ( WorldRotation * EyeRotation ) * EyeGaze;

          CopyAndTransformT( WorldGazeVector.data(), o_rThreeVector );
          return Result::Success;
        }
      }
    }
  }

  // No present head segment.
  o_rbOccludedFlag = true;
  return Result::Success;
}

bool VClient::IsEyeTrackerDevice(unsigned int i_DeviceID) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  for( unsigned int i = 0; i < m_LatestFrame.m_EyeTrackers.size(); i++ )
  {
    if( m_LatestFrame.m_EyeTrackers[ i ].m_DeviceID == i_DeviceID )
    {
      return true;
    }
  }

  return false;
}

bool VClient::HasData() const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  return BadFrameValue != m_LatestFrame.m_Frame.m_FrameID;
}

void VClient::PreFetchThreadBody()
{
  while( m_bContinuePreFetch )
  {
    FetchNextFrame();   
  }
}

void VClient::FetchNextFrame()
{
  if( !m_pClient )
  {
    return;
  }

  std::vector< ViconCGStreamClientSDK::ICGFrameState > LoadedFrames;

  ViconCGStreamClientSDK::ICGFrameState Frame;
  if( m_pClient->WaitFrame( Frame, s_WaitFrameTimeout ) )
  {
    // copy out the last frame
    m_bNewCachedFrame = true;
    m_CachedFrame = Frame;
  }
} 

Result::Enum VClient::GetDeviceCount( unsigned int & o_rDeviceCount ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( InitGet( GetResult, o_rDeviceCount ) )
  {
    o_rDeviceCount = static_cast< unsigned int >( m_LatestFrame.m_Devices.size() );
  }
  return GetResult;
}

Result::Enum VClient::GetDeviceName( const unsigned int       i_DeviceIndex,
                                           std::string      & o_rDeviceName,
                                           DeviceType::Enum & o_rDeviceType ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rDeviceName, o_rDeviceType ) )
  {
    return GetResult; 
  }

  if( i_DeviceIndex >= m_LatestFrame.m_Devices.size() )
  {
    return Result::InvalidIndex;
  }

  const ViconCGStream::VDeviceInfo & rDevice( m_LatestFrame.m_Devices[ i_DeviceIndex ] );
  o_rDeviceName = AdaptDeviceName( rDevice.m_Name, rDevice.m_DeviceID );
  if( IsForcePlateDevice( rDevice.m_DeviceID ) )
  {
    o_rDeviceType = DeviceType::ForcePlate;
  }
  else if( IsEyeTrackerDevice( rDevice.m_DeviceID ) )
  {
    o_rDeviceType = DeviceType::EyeTracker;
  }
  else
  {
    o_rDeviceType = DeviceType::Unknown;
  }

  return Result::Success;
}

Result::Enum VClient::GetDeviceOutputCount( const std::string  & i_rDeviceName,
                                                  unsigned int & o_rDeviceOutputCount ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rDeviceOutputCount ) )
  {
    return GetResult; 
  }

  // Get the device id
  unsigned int DeviceID = 0;
  Result::Enum _Result = GetDeviceID( i_rDeviceName, DeviceID );
  if( Result::Success != _Result )
  {
    return _Result;
  }

  // Iterate over the channels for this device
  std::vector< ViconCGStream::VChannelInfo >::const_iterator ChannelIt  = m_LatestFrame.m_Channels.begin();
  std::vector< ViconCGStream::VChannelInfo >::const_iterator ChannelEnd = m_LatestFrame.m_Channels.end();
  for( ; ChannelIt != ChannelEnd ; ++ChannelIt )
  {
    const ViconCGStream::VChannelInfo & rChannel( *ChannelIt );

    if( rChannel.m_DeviceID == DeviceID )
    {
      o_rDeviceOutputCount += static_cast< unsigned int >( rChannel.m_ComponentNames.size() );
    }
  }

  return Result::Success;
}

namespace
{
  template< typename TFrame >
  bool GetSamples( const std::vector< TFrame > & i_rFrames,
                   const ViconCGStream::VChannelInfo & i_rChannel,
                   const unsigned int            i_Subsample,
                   const ViconCGStreamType::UInt64 i_DevicePeriod,
                   const ViconCGStreamType::UInt64 i_DeviceStartTick,
                   const TPeriod               & i_rFramePeriod,
                   std::vector< double >       & o_rSamples,
                         bool                  & o_rbOccluded )
  {
    o_rSamples.clear();
    o_rbOccluded = true;

    typename std::vector< TFrame >::const_iterator It  = i_rFrames.begin();
    typename std::vector< TFrame >::const_iterator End = i_rFrames.end();

    const size_t NumComponents = i_rChannel.m_ComponentNames.size();
    for( ; It != End ; ++It )
    {
      const TFrame & rFrame( *It );
      if( rFrame.m_DeviceID       == i_rChannel.m_DeviceID  &&
          rFrame.m_ChannelID      == i_rChannel.m_ChannelID &&
          rFrame.m_Samples.size() >= NumComponents && 
          NumComponents != 0 )
      {
        const size_t Stride = rFrame.m_Samples.size() / NumComponents;

        assert( i_DevicePeriod % Stride == 0 );

        const ViconCGStreamType::UInt64 SamplePeriod = i_DevicePeriod / Stride;

        if ( i_Subsample >= GetSamplesInFrame( i_rFramePeriod, i_DeviceStartTick, SamplePeriod ) )
        {
          // Out of range for the entire frame.
          return false;
        }

        unsigned int SubSample = 0;
        if( !SampleIndexInRange( i_Subsample, rFrame.m_FrameID, Stride, SamplePeriod, i_DeviceStartTick, i_rFramePeriod, SubSample ) )
        {
          // Not in the current set of samples - might be in a later one though.
          continue;
        }

        for( size_t i = 0 ; i < NumComponents ; ++i )
        {
          o_rSamples.push_back( rFrame.m_Samples[ i * Stride + SubSample ] );
        }

        o_rbOccluded = false;
        break;
      }
    }

    return true;
  }

  template< typename TFrame, unsigned int N >
  bool GetSamples( const std::vector< TFrame > & i_rFrames,
                   const ViconCGStream::VChannelInfo & i_rChannel,
                   const unsigned int            i_Subsample,
                   const ViconCGStreamType::UInt64 i_DevicePeriod,
                   const ViconCGStreamType::UInt64 i_DeviceStartTick,
                   const TPeriod               & i_rFramePeriod,
                         double               (& o_rSamples)[ N ],
                         bool                  & o_rbOccluded )
  {
    assert( i_rChannel.m_ComponentNames.size() == N );
    std::vector< double > Samples;
    bool bOK = GetSamples< TFrame >( i_rFrames, i_rChannel, i_Subsample, i_DevicePeriod, i_DeviceStartTick, i_rFramePeriod, Samples, o_rbOccluded );

    if( !o_rbOccluded )
    {
      assert( Samples.size() == N );
      std::copy( Samples.begin(), Samples.begin() + N, o_rSamples );
    }

    return bOK;
  }

  template< typename TFrame >
  bool GetSampleCount( const std::vector< TFrame > & i_rFrames,
                       const ViconCGStream::VChannelInfo & i_rChannel,
                       const ViconCGStreamType::UInt64 i_DevicePeriod,
                       const ViconCGStreamType::UInt64 i_DeviceStartTick,
                       const TPeriod               & i_rFramePeriod,
                             unsigned int          & o_rCount )
  {
    typename std::vector< TFrame >::const_iterator It  = i_rFrames.begin();
    typename std::vector< TFrame >::const_iterator End = i_rFrames.end();
    o_rCount = 0;
    const size_t Components = i_rChannel.m_ComponentNames.size();
    for( ; It != End ; ++It )
    {
      const TFrame & rFrame( *It );
      const size_t NumberOfSampleComponents = rFrame.m_Samples.size();
      if( rFrame.m_DeviceID       == i_rChannel.m_DeviceID  &&
          rFrame.m_ChannelID      == i_rChannel.m_ChannelID &&
          NumberOfSampleComponents >= Components &&
          Components != 0 )
      {
        assert( NumberOfSampleComponents % Components == 0 );

        const size_t NumSamples = NumberOfSampleComponents / Components;

        assert( i_DevicePeriod % NumSamples == 0 );
        const ViconCGStreamType::Int64 SamplePeriod = i_DevicePeriod / NumSamples;

        o_rCount = GetSamplesInFrame( i_rFramePeriod, i_DeviceStartTick, SamplePeriod );
        return true;
      }
    }
    return false;
  }
}

Result::Enum VClient::GetDeviceOutputName( const std::string  & i_rDeviceName,
                                           const unsigned int   i_DeviceOutputIndex,
                                                 std::string  & o_rDeviceOutputName,
                                                 Unit::Enum   & o_rDeviceOutputUnit ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rDeviceOutputName, o_rDeviceOutputUnit ) )
  {
    return GetResult; 
  }

  // Get the device id
  unsigned int DeviceID = 0;
  Result::Enum _Result = GetDeviceID( i_rDeviceName, DeviceID );
  if( Result::Success != _Result )
  {
    return _Result;
  }

  // Is this a forceplate?
  const bool bIsForcePlate( IsForcePlateDevice( DeviceID ) );

  // Iterate over the channels for this device
  unsigned int CurrentDeviceOutputIndex = 0;
  std::vector< ViconCGStream::VChannelInfo >::const_iterator ChannelIt  = m_LatestFrame.m_Channels.begin();
  const std::vector< ViconCGStream::VChannelInfo >::const_iterator ChannelEnd = m_LatestFrame.m_Channels.end();
  for( ; ChannelIt != ChannelEnd ; ++ChannelIt )
  {
    const ViconCGStream::VChannelInfo & rChannel( *ChannelIt );

    // Wrong device
    if( rChannel.m_DeviceID != DeviceID )
    {
      continue;
    }

    std::vector< std::string >::const_iterator ComponentIt  = rChannel.m_ComponentNames.begin();
    std::vector< std::string >::const_iterator ComponentEnd = rChannel.m_ComponentNames.end();
    for( ; ComponentIt != ComponentEnd ; ++ComponentIt )
    {
      if( CurrentDeviceOutputIndex == i_DeviceOutputIndex )
      {
        o_rDeviceOutputName = AdaptDeviceOutputName( *ComponentIt, CurrentDeviceOutputIndex );

        if( bIsForcePlate )
        {
          // Determine where the data lives in this channel
          if( IsForcePlateForceChannel( rChannel ) )
          {
            o_rDeviceOutputUnit = Unit::Newton;
          }
          else if( IsForcePlateMomentChannel( rChannel ) )
          {
            o_rDeviceOutputUnit = Unit::NewtonMeter;
          }
          else if( IsForcePlateCoPChannel( rChannel ) )
          {
            o_rDeviceOutputUnit = Unit::Meter;
          }
          else
          {
            o_rDeviceOutputUnit = Unit::Volt;
          }
        }
        else
        {
          // Not a forceplate.
          o_rDeviceOutputUnit = Unit::Unknown;

          // Look for information in the extra channel information.

          std::vector< ViconCGStream::VChannelInfoExtra >::const_iterator ChannelUnitIt  = m_LatestFrame.m_ChannelUnits.begin();
          const std::vector< ViconCGStream::VChannelInfoExtra >::const_iterator ChannelUnitEnd = m_LatestFrame.m_ChannelUnits.end();
          for( ; ChannelUnitIt != ChannelUnitEnd ; ++ChannelUnitIt )
          {
            const ViconCGStream::VChannelInfoExtra & rChannelInfoExtra( *ChannelUnitIt );

            // Wrong Device
            if( rChannelInfoExtra.m_DeviceID != DeviceID )
            {
              continue;
            }

            // Wrong Channel
            if( rChannelInfoExtra.m_ChannelID != rChannel.m_ChannelID )
            {
              continue;
            }

            if( rChannelInfoExtra.m_Unit == "meter" )
            {
              o_rDeviceOutputUnit = Unit::Meter;
            }
            else if( rChannelInfoExtra.m_Unit == "kilogram" )
            {
              o_rDeviceOutputUnit = Unit::Kilogram;
            }
            else if( rChannelInfoExtra.m_Unit == "second" )
            {
              o_rDeviceOutputUnit = Unit::Second;
            }
            else if( rChannelInfoExtra.m_Unit == "ampere" )
            {
              o_rDeviceOutputUnit = Unit::Ampere;
            }
            else if( rChannelInfoExtra.m_Unit == "kelvin" )
            {
              o_rDeviceOutputUnit = Unit::Kelvin;
            }
            else if( rChannelInfoExtra.m_Unit == "mole" )
            {
              o_rDeviceOutputUnit = Unit::Mole;
            }
            else if( rChannelInfoExtra.m_Unit == "candela" )
            {
              o_rDeviceOutputUnit = Unit::Candela;
            }
            else if( rChannelInfoExtra.m_Unit == "radian" )
            {
              o_rDeviceOutputUnit = Unit::Radian;
            }
            else if( rChannelInfoExtra.m_Unit == "steradian" )
            {
              o_rDeviceOutputUnit = Unit::Steradian;
            }
            else if( rChannelInfoExtra.m_Unit == "meter squared" )
            {
              o_rDeviceOutputUnit = Unit::MeterSquared;
            }
            else if( rChannelInfoExtra.m_Unit == "meter cubed" )
            {
              o_rDeviceOutputUnit = Unit::MeterCubed;
            }
            else if( rChannelInfoExtra.m_Unit == "meter per second" )
            {
              o_rDeviceOutputUnit = Unit::MeterPerSecond;
            }
            else if( rChannelInfoExtra.m_Unit == "meter per second squared" )
            {
              o_rDeviceOutputUnit = Unit::MeterPerSecondSquared;
            }
            else if( rChannelInfoExtra.m_Unit == "radian per second" )
            {
              o_rDeviceOutputUnit = Unit::RadianPerSecond;
            }
            else if( rChannelInfoExtra.m_Unit == "radian per second squared" )
            {
              o_rDeviceOutputUnit = Unit::RadianPerSecondSquared;
            }
            else if( rChannelInfoExtra.m_Unit == "hertz" )
            {
              o_rDeviceOutputUnit = Unit::Hertz;
            }
            else if( rChannelInfoExtra.m_Unit == "newton" )
            {
              o_rDeviceOutputUnit = Unit::Newton;
            }
            else if( rChannelInfoExtra.m_Unit == "joule" )
            {
              o_rDeviceOutputUnit = Unit::Joule;
            }
            else if( rChannelInfoExtra.m_Unit == "watt" )
            {
              o_rDeviceOutputUnit = Unit::Watt;
            }
            else if( rChannelInfoExtra.m_Unit == "pascal" )
            {
              o_rDeviceOutputUnit = Unit::Pascal;
            }
            else if( rChannelInfoExtra.m_Unit == "lumen" )
            {
              o_rDeviceOutputUnit = Unit::Lumen;
            }
            else if( rChannelInfoExtra.m_Unit == "lux" )
            {
              o_rDeviceOutputUnit = Unit::Lux;
            }
            else if( rChannelInfoExtra.m_Unit == "coulomb" )
            {
              o_rDeviceOutputUnit = Unit::Coulomb;
            }
            else if( rChannelInfoExtra.m_Unit == "volt" )
            {
              o_rDeviceOutputUnit = Unit::Volt;
            }
            else if( rChannelInfoExtra.m_Unit == "ohm" )
            {
              o_rDeviceOutputUnit = Unit::Ohm;
            }
            else if( rChannelInfoExtra.m_Unit == "farad" )
            {
              o_rDeviceOutputUnit = Unit::Farad;
            }
            else if( rChannelInfoExtra.m_Unit == "weber" )
            {
              o_rDeviceOutputUnit = Unit::Weber;
            }
            else if( rChannelInfoExtra.m_Unit == "tesla" )
            {
              o_rDeviceOutputUnit = Unit::Tesla;
            }
            else if( rChannelInfoExtra.m_Unit == "henry" )
            {
              o_rDeviceOutputUnit = Unit::Henry;
            }
            else if( rChannelInfoExtra.m_Unit == "siemens" )
            {
              o_rDeviceOutputUnit = Unit::Siemens;
            }
            else if( rChannelInfoExtra.m_Unit == "becquerel" )
            {
              o_rDeviceOutputUnit = Unit::Becquerel;
            }
            else if( rChannelInfoExtra.m_Unit == "gray" )
            {
              o_rDeviceOutputUnit = Unit::Gray;
            }
            else if( rChannelInfoExtra.m_Unit == "sievert" )
            {
              o_rDeviceOutputUnit = Unit::Sievert;
            }
            else if( rChannelInfoExtra.m_Unit == "katal" )
            {
              o_rDeviceOutputUnit = Unit::Katal;
            }

          }
        }

        return Result::Success;
      }
      else
      {
        ++CurrentDeviceOutputIndex;
      }
    }
  }
  return Result::InvalidIndex;
}

Result::Enum VClient::GetDeviceOutputValue( const std::string & i_rDeviceName,
                                            const std::string & i_rDeviceOutputName,
                                                  double      & o_rValue,
                                                  bool        & o_rbOccluded ) const
{
  return GetDeviceOutputValue( i_rDeviceName, i_rDeviceOutputName, 0, o_rValue, o_rbOccluded );
}



Result::Enum VClient::GetDeviceOutputSubsamples( const std::string  & i_rDeviceName, 
                                                 const std::string  & i_rDeviceOutputName, 
                                                       unsigned int & o_rDeviceOutputSubsamples,
                                                       bool         & o_rbOccluded ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rDeviceOutputSubsamples ) )
  {
    return GetResult; 
  }

  const ViconCGStream::VDeviceInfo * pDevice = GetDevice( i_rDeviceName, GetResult );
  if ( !pDevice )
  {
    return GetResult; 
  }
  const unsigned int DeviceID = pDevice->m_DeviceID;
  const ViconCGStreamType::UInt64 DevicePeriod = pDevice->m_FramePeriod;

  ViconCGStreamType::Int64 DeviceStartTick = GetDeviceStartTick( DeviceID );

  const TPeriod FramePeriod = GetFramePeriod( m_LatestFrame );

  // Iterate over the channels for this device
  unsigned int CurrentDeviceOutputIndex = 0;
  std::vector< ViconCGStream::VChannelInfo >::const_iterator ChannelIt  = m_LatestFrame.m_Channels.begin();
  std::vector< ViconCGStream::VChannelInfo >::const_iterator ChannelEnd = m_LatestFrame.m_Channels.end();
  for( ; ChannelIt != ChannelEnd ; ++ChannelIt )
  {
    const ViconCGStream::VChannelInfo & rChannel( *ChannelIt );

    // Wrong device
    if( rChannel.m_DeviceID != DeviceID )
    {
      continue;
    }

    if( rChannel.m_ComponentNames.empty() )
    {
      return Result::NoFrame;
    }

    std::vector< std::string >::const_iterator ComponentIt  = rChannel.m_ComponentNames.begin();
    std::vector< std::string >::const_iterator ComponentEnd = rChannel.m_ComponentNames.end();
    for( unsigned int ComponentIndex = 0 ; ComponentIt != ComponentEnd ; ++ComponentIt, ++ComponentIndex, ++CurrentDeviceOutputIndex )
    {
      if( i_rDeviceOutputName == AdaptDeviceOutputName( *ComponentIt, CurrentDeviceOutputIndex ) )
      {
        // This is the correct component.
        // Find the correct data array.

        if( IsForcePlateForceChannel( rChannel ) )
        {
          o_rbOccluded = !GetSampleCount( m_LatestFrame.m_Forces, rChannel, DevicePeriod, DeviceStartTick, FramePeriod, o_rDeviceOutputSubsamples );
        }
        else if( IsForcePlateMomentChannel( rChannel ) )
        {
          o_rbOccluded = !GetSampleCount( m_LatestFrame.m_Moments, rChannel, DevicePeriod, DeviceStartTick, FramePeriod, o_rDeviceOutputSubsamples );
        }
        else if( IsForcePlateCoPChannel( rChannel ) )
        {
          o_rbOccluded = !GetSampleCount( m_LatestFrame.m_CentresOfPressure, rChannel, DevicePeriod, DeviceStartTick, FramePeriod, o_rDeviceOutputSubsamples );
        }
        else
        {
          o_rbOccluded = !GetSampleCount( m_LatestFrame.m_Voltages, rChannel, DevicePeriod, DeviceStartTick, FramePeriod, o_rDeviceOutputSubsamples );
        }

        return Result::Success;
      }
    }
  }

  return Result::InvalidDeviceOutputName;
}

Result::Enum VClient::GetDeviceOutputValue( const std::string  & i_rDeviceName,
                                            const std::string  & i_rDeviceOutputName,
                                                  unsigned int   i_Subsample,
                                                  double       & o_rValue,
                                                  bool         & o_rbOccluded ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rValue, o_rbOccluded ) )
  {
    return GetResult; 
  }

  const ViconCGStream::VDeviceInfo * pDevice = GetDevice( i_rDeviceName, GetResult );
  if ( !pDevice )
  {
    return GetResult; 
  }
  const unsigned int DeviceID = pDevice->m_DeviceID;
  const ViconCGStreamType::UInt64 DevicePeriod = pDevice->m_FramePeriod;


  ViconCGStreamType::Int64 DeviceStartTick = GetDeviceStartTick( DeviceID );

  const TPeriod FramePeriod = GetFramePeriod( m_LatestFrame );

  // Try and find the channel which contains this device output
  unsigned int CurrentDeviceOutputIndex = 0;
  std::vector< ViconCGStream::VChannelInfo >::const_iterator ChannelIt  = m_LatestFrame.m_Channels.begin();
  std::vector< ViconCGStream::VChannelInfo >::const_iterator ChannelEnd = m_LatestFrame.m_Channels.end();
  for( ; ChannelIt != ChannelEnd ; ++ChannelIt )
  {
    const ViconCGStream::VChannelInfo & rChannel( *ChannelIt );

    // Wrong device
    if( rChannel.m_DeviceID != DeviceID )
    {
      continue;
    }

    std::vector< std::string >::const_iterator ComponentIt  = rChannel.m_ComponentNames.begin();
    std::vector< std::string >::const_iterator ComponentEnd = rChannel.m_ComponentNames.end();
    for( unsigned int ComponentIndex = 0 ; ComponentIt != ComponentEnd ; ++ComponentIt, ++ComponentIndex, ++CurrentDeviceOutputIndex )
    {
      if( i_rDeviceOutputName == AdaptDeviceOutputName( *ComponentIt, CurrentDeviceOutputIndex ) )
      {
        // This is the correct component. Now figure out where to get the data from

        // Force frame
        if( IsForcePlateForceChannel( rChannel ) )
        {
          if( 3 != rChannel.m_ComponentNames.size() )
          {
            return Result::Unknown;
          }

          double Samples[ 3 ];
          if( !GetSamples( m_LatestFrame.m_Forces,
                           rChannel,
                           i_Subsample,
                           DevicePeriod,
                           DeviceStartTick,
                           FramePeriod,
                           Samples,
                           o_rbOccluded ) )
          {
            return Result::InvalidIndex;
          }

          if( !o_rbOccluded )
          {
            double TransformedSamples[ 3 ];
            CopyAndTransformT( Samples, TransformedSamples );
            o_rValue = TransformedSamples[ ComponentIndex ];
          }

          return Result::Success;
        }

        if( IsForcePlateMomentChannel( rChannel ) )
        {
          if( 3 != rChannel.m_ComponentNames.size() )
          {
            return Result::Unknown;
          }

          double Samples[ 3 ];
          if( !GetSamples( m_LatestFrame.m_Moments,
                           rChannel,
                           i_Subsample,
                           DevicePeriod,
                           DeviceStartTick,
                           FramePeriod,
                           Samples,
                           o_rbOccluded ) )
          {
            return Result::InvalidIndex;
          }

          if( !o_rbOccluded )
          {
            double TransformedSamples[ 3 ];
            CopyAndTransformT( Samples, TransformedSamples );
            o_rValue = TransformedSamples[ ComponentIndex ];
          }

          return Result::Success;
        }

        if( IsForcePlateCoPChannel( rChannel ) )
        {
          if( 3 != rChannel.m_ComponentNames.size() )
          {
            return Result::Unknown;
          }

          double Samples[ 3 ];
          if( !GetSamples( m_LatestFrame.m_CentresOfPressure,
                           rChannel,
                           i_Subsample,
                           DevicePeriod,
                           DeviceStartTick,
                           FramePeriod,
                           Samples,
                           o_rbOccluded ) )
          {
            return Result::InvalidIndex;
          }

          if( !o_rbOccluded )
          {
            double TransformedSamples[ 3 ];
            CopyAndTransformT( Samples, TransformedSamples );
            o_rValue = TransformedSamples[ ComponentIndex ];
          }

          return Result::Success;
        }

        // Voltage
        {
          std::vector< double > Samples;
          if( !GetSamples( m_LatestFrame.m_Voltages,
                           rChannel,
                           i_Subsample,
                           DevicePeriod,
                           DeviceStartTick,
                           FramePeriod,
                           Samples,
                           o_rbOccluded ) )
          {
            return Result::InvalidIndex;
          }

          if( !o_rbOccluded )
          {
            if( Samples.size() % rChannel.m_ComponentNames.size() != 0 )
            {
              return Result::Unknown;
            }

            o_rValue = Samples[ ComponentIndex ];
            return Result::Success;
          }
        }

        // Other
        {
          // If we didn't find any data for it then we must be occluded
          o_rbOccluded = true;
          return Result::Success;
        }
      }
    }
  }

  return Result::InvalidDeviceOutputName;
}

Result::Enum VClient::GetCameraCount( unsigned int & o_rCount ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( InitGet( GetResult, o_rCount ) )
  {
    o_rCount = static_cast< unsigned int >( m_LatestFrame.m_Cameras.size() );
  }
  return GetResult;
}

Result::Enum VClient::GetCameraName( const unsigned int i_CameraIndex, std::string  & o_rCameraName ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( !InitGet( GetResult, o_rCameraName ) )
  {
    return GetResult; 
  }

  if( i_CameraIndex >= m_LatestFrame.m_Cameras.size() )
  {
    return Result::InvalidIndex;
  }

  const ViconCGStream::VCameraInfo & rCamera( m_LatestFrame.m_Cameras[ i_CameraIndex ] );
  o_rCameraName = AdaptCameraName( rCamera.m_Name, rCamera.m_DisplayType, rCamera.m_CameraID );

  return GetResult;

}

const ViconCGStream::VCameraInfo * VClient::GetCamera( const std::string & i_rCameraName, Result::Enum & o_rResult ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  const auto rCameraIt = 
    std::find_if( m_LatestFrame.m_Cameras.begin(), m_LatestFrame.m_Cameras.end(),
      [&i_rCameraName]( const ViconCGStream::VCameraInfo & rCamera )
        { return AdaptCameraName( rCamera.m_Name, rCamera.m_DisplayType, rCamera.m_CameraID ) == i_rCameraName; }
    );

  if( rCameraIt != m_LatestFrame.m_Cameras.end() )
  {
    o_rResult = Result::Success;
    return &(*rCameraIt);
  }

  o_rResult = Result::InvalidCameraName;
  return nullptr;
}


Result::Enum VClient::GetCameraID( const std::string & i_rCameraName, unsigned int & o_rCameraID ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  Clear( o_rCameraID );

  Result::Enum GetResult = Result::Success;
  const ViconCGStream::VCameraInfo * pCamera = GetCamera( i_rCameraName, GetResult );
  if ( pCamera )
  {
    o_rCameraID = pCamera->m_CameraID;
  }
  return GetResult;
}

Result::Enum VClient::GetCameraUserID( const std::string & i_rCameraName, unsigned int & o_rUserID ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  Clear( o_rUserID );

  Result::Enum GetResult = Result::Success;
  const ViconCGStream::VCameraInfo * pCamera = GetCamera( i_rCameraName, GetResult );
  if ( pCamera )
  {
    o_rUserID = pCamera->m_UserID;
  }
  return GetResult;
}

Result::Enum VClient::GetCameraType( const std::string & i_rCameraName, std::string & o_rCameraType ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
 
  Result::Enum GetResult = Result::Success;
  const ViconCGStream::VCameraInfo * pCamera = GetCamera( i_rCameraName, GetResult );
  if ( pCamera )
  {
    o_rCameraType = pCamera->m_DisplayType;
  }

  return GetResult;
}


Result::Enum VClient::GetCameraDisplayName( const std::string & i_rCameraName, std::string  & o_rCameraDisplayName ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  const ViconCGStream::VCameraInfo * pCamera = GetCamera( i_rCameraName, GetResult );
  if ( pCamera )
  {
    o_rCameraDisplayName = pCamera->m_Name;
  }

  return GetResult;
}


Result::Enum VClient::GetCameraResolution( const std::string & i_rCameraName, unsigned int & o_rResolutionX, unsigned int & o_rResolutionY ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  Result::Enum GetResult = Result::Success;
  if( InitGet( GetResult, o_rResolutionX, o_rResolutionY ) )
  {
    const ViconCGStream::VCameraInfo* pCamera = GetCamera( i_rCameraName, GetResult );
    if( !pCamera )
    {
      return GetResult;
    }

    o_rResolutionX = pCamera->m_ResolutionX;
    o_rResolutionY = pCamera->m_ResolutionY;
  }

  return GetResult;
}

Result::Enum VClient::GetIsVideoCamera( const std::string & i_rCameraName, bool & o_rIsVideoCamera ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  Result::Enum GetResult = Result::Success;
  if( InitGet( GetResult, o_rIsVideoCamera ) )
  {
    const ViconCGStream::VCameraInfo* pCamera = GetCamera( i_rCameraName, GetResult );
    if( !pCamera )
    {
      return GetResult;
    }

    o_rIsVideoCamera = pCamera->m_bIsVideoCamera;
  }

  return GetResult;
}


Result::Enum VClient::GetCentroidCount( const std::string & i_rCameraName, unsigned int & o_rCount ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  Result::Enum GetResult = Result::Success;
  if( InitGet( GetResult, o_rCount ) )
  {
    const ViconCGStream::VCameraInfo* pCamera = GetCamera( i_rCameraName, GetResult );
    if( !pCamera )
    {
      return GetResult;
    }

    const ViconCGStream::VCentroids* pCentroidSet = GetCentroidSet( pCamera->m_CameraID, GetResult );
    if( pCentroidSet )
    {
      o_rCount = static_cast< unsigned int >( pCentroidSet->m_Centroids.size() );
    }
  }
  return GetResult;
}

Result::Enum VClient::GetCentroidPosition( const std::string & i_rCameraName,
                                           const unsigned int i_CentroidIndex,
                                           double (&o_rPosition)[2],
                                           double & o_rRadius /*,
                                           double & o_rAccuracy */ ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );
  
  Result::Enum GetResult = Result::Success;
  if ( InitGet( GetResult, o_rPosition, o_rRadius /*, o_rAccuracy */ ) )
  {
    const ViconCGStream::VCameraInfo* pCamera = GetCamera( i_rCameraName, GetResult );
    if( !pCamera )
    {
      return GetResult;
    }

    const ViconCGStream::VCentroids* pCentroidSet = GetCentroidSet( pCamera->m_CameraID, GetResult );
    if( pCentroidSet )
    {
      if( i_CentroidIndex < pCentroidSet->m_Centroids.size() )
      {
        const ViconCGStreamDetail::VCentroids_Centroid & rCentroid = 
          pCentroidSet->m_Centroids[ i_CentroidIndex ];

        o_rPosition[0] = rCentroid.m_Position[0];
        o_rPosition[1] = rCentroid.m_Position[1];
        o_rRadius = rCentroid.m_Radius;
//        o_rAccuracy = rCentroid.m_Accuracy;
      }
      else
      {
        GetResult = Result::InvalidIndex;
      }
    }
  }
  return GetResult;
}

Result::Enum VClient::GetCentroidWeight( const std::string & i_rCameraName,
  const unsigned int i_CentroidIndex,
  double & o_rWeight ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  Result::Enum GetResult = Result::Success;
  if( InitGet( GetResult, o_rWeight ) )
  {
    const ViconCGStream::VCameraInfo* pCamera = GetCamera( i_rCameraName, GetResult );
    if( !pCamera )
    {
      return GetResult;
    }

    const ViconCGStream::VCentroidWeights* pCentroidWeightSet = GetCentroidWeightSet( pCamera->m_CameraID, GetResult );
    if( pCentroidWeightSet )
    {
      if( i_CentroidIndex < pCentroidWeightSet->m_Weights.size() )
      {
        o_rWeight = pCentroidWeightSet->m_Weights[i_CentroidIndex];;
      }
      else
      {
        GetResult = Result::InvalidIndex;
      }
    }
  }
  return GetResult;
}

Result::Enum VClient::GetGreyscaleBlobCount( const std::string & i_rCameraName, unsigned int & o_rCount ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  Result::Enum GetResult = Result::Success;
  if( InitGet( GetResult, o_rCount ) )
  {
    const ViconCGStream::VCameraInfo* pCamera = GetCamera( i_rCameraName, GetResult );
    if( !pCamera )
    {
      return GetResult;
    }

    const ViconCGStream::VGreyscaleBlobs * pBlobSet = GetGreyscaleBlobs( pCamera->m_CameraID, GetResult );
    if( pBlobSet )
    {
      o_rCount = static_cast< unsigned int >( pBlobSet->m_GreyscaleBlobs.size() );
    }
  }
  return GetResult;
}


Result::Enum VClient::GetGreyscaleBlob( const std::string & i_rCameraName,
                                        const unsigned int i_BlobIndex,
                                        std::vector< unsigned int > & o_rLineXPositions,
                                        std::vector< unsigned int > & o_rLineYPositions,
                                        std::vector< std::vector< unsigned char > > & o_rLinePixelValues ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  Result::Enum GetResult = Result::Success;
  if ( InitGet( GetResult, o_rLineXPositions, o_rLineYPositions, o_rLinePixelValues ) )
  {
    const ViconCGStream::VCameraInfo* pCamera = GetCamera( i_rCameraName, GetResult );
    if( !pCamera )
    {
      return GetResult;
    }

    const ViconCGStream::VGreyscaleBlobs* pBlobSet = GetGreyscaleBlobs( pCamera->m_CameraID, GetResult );
    if( pBlobSet )
    {
      if( i_BlobIndex < pBlobSet->m_GreyscaleBlobs.size() )
      {
        const ViconCGStreamDetail::VGreyscaleBlobs_GreyscaleBlob & rBlob = pBlobSet->m_GreyscaleBlobs[ i_BlobIndex ];

        o_rLineXPositions.reserve( rBlob.m_GreyscaleLines.size() );
        o_rLineYPositions.reserve( rBlob.m_GreyscaleLines.size() );
        o_rLinePixelValues.reserve( rBlob.m_GreyscaleLines.size() );

        for ( auto line : rBlob.m_GreyscaleLines )
        {
          o_rLineXPositions.push_back( line.m_Position[0] );
          o_rLineYPositions.push_back( line.m_Position[1] );
          o_rLinePixelValues.push_back( line.m_Greyscale);
        }

      }
      else
      {
        GetResult = Result::InvalidIndex;
      }
    }
  }

  return GetResult;
}

Result::Enum VClient::GetVideoFrame( const std::string & i_rCameraName, ViconCGStreamClientSDK::VVideoFramePtr & o_rVideoFramePtr ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  Result::Enum GetResult = Result::Success;
  if ( InitGet( GetResult ) )
  {
    const ViconCGStream::VCameraInfo* pCamera = GetCamera( i_rCameraName, GetResult );
    if( !pCamera )
    {
      return GetResult;
    }

    GetVideoFrame( pCamera->m_CameraID, GetResult, o_rVideoFramePtr );
  }

  return GetResult;
}

Result::Enum VClient::SetCameraFilter( const std::vector< unsigned int > & i_rCameraIdsForCentroids,
                                       const std::vector< unsigned int > & i_rCameraIdsForBlobs,
                                       const std::vector< unsigned int > & i_rCameraIdsForVideo )
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  // Clear any current filters on supported camera data types
  m_Filter.Clear( ViconCGStreamEnum::Centroids );
  m_Filter.Clear( ViconCGStreamEnum::GreyscaleBlobs );
  m_Filter.Clear( ViconCGStreamEnum::VideoFrame );

  // Add new filters for the given cameras
  for ( unsigned int Id : i_rCameraIdsForCentroids )
  {
    m_Filter.Add( ViconCGStreamEnum::Centroids, Id );
  }

  for ( unsigned int Id : i_rCameraIdsForBlobs )
  {
    m_Filter.Add( ViconCGStreamEnum::GreyscaleBlobs, Id );
  }

  for ( unsigned int Id : i_rCameraIdsForVideo )
  {
    m_Filter.Add( ViconCGStreamEnum::VideoFrame, Id );
  }

  // Items with no filter set in the CGServer are allowed by default.  If any of the input
  // vectors are empty, the client request is that no data is sent for that type.  We have
  // to add a dummy id in this case to activate the filter and circumvent its default pass
  m_Filter.Add( ViconCGStreamEnum::Centroids, -1 );
  m_Filter.Add( ViconCGStreamEnum::GreyscaleBlobs, -1 );
  m_Filter.Add( ViconCGStreamEnum::VideoFrame, -1 );

  m_pClient->SetFilter( m_Filter );

  return Result::Success;
}


ViconCGStreamClientSDK::ICGFrameState& VClient::LatestFrame()
{ 
  return m_LatestFrame; 
}

ViconCGStreamClientSDK::ICGFrameState& VClient::CachedFrame()
{ 
  return m_CachedFrame; 
}

void VClient::CopyAndTransformT( const double i_Translation[ 3 ], double ( & io_Translation )[ 3 ] ) const
{
  if (m_pAxisMapping)
  {

    Direction::Enum RequestedX, RequestedY, RequestedZ;
    m_pAxisMapping->GetAxisMapping(RequestedX, RequestedY, RequestedZ);

    Direction::Enum ServerX, ServerY, ServerZ;

    AxisMappingResult::Enum Error = AxisMappingResult::Success;
    std::shared_ptr< VAxisMapping > pServerAxisMapping;

    // If we've got information about our stream type from the server
    if (m_CachedFrame.m_ApplicationInfo && m_CachedFrame.m_ApplicationInfo.get().m_AxisOrientation == ViconCGStream::VApplicationInfo::EYUp)
    {
      ServerX = Direction::Forward;
      ServerY = Direction::Up;
      ServerZ = Direction::Right;

      // Only create the server axis mapping if we need to modify the data from the server
      pServerAxisMapping = VAxisMapping::Create(Error, Direction::Forward, Direction::Up, Direction::Right);
    }
    else
    {
      // We either know it's Z-up, or we assume it's Z-up due to lack of contrary information.
      ServerX = Direction::Forward;
      ServerY = Direction::Left;
      ServerZ = Direction::Up;
    }

    // We will avoid the mapping in favour of a pure copy if the input and requested output are the same. 
    if (RequestedX == ServerX && RequestedY == ServerY && RequestedZ == ServerZ)
    {
      std::copy(i_Translation, i_Translation + 3, io_Translation);
    }
    else
    {
      if (pServerAxisMapping && Error == AxisMappingResult::Success)
      {
        double Q[9];
        pServerAxisMapping->GetTransformationMatrix(Q);
        m_pAxisMapping->CopyAndTransformT(i_Translation, Q, io_Translation);
      }
      else
      {
        // We don't need to do a server axis mapping
        m_pAxisMapping->CopyAndTransformT(i_Translation, io_Translation);
      }
    }
  }
  else
  {
    // Just do a pure copy if there's no axis mapping at all.
    std::copy( i_Translation, i_Translation + 3, io_Translation );
  }
}

void VClient::CopyAndTransformR( const double i_Rotation[ 9 ], double ( & io_Rotation )[ 9 ] ) const
{
  if (m_pAxisMapping)
  {

    Direction::Enum RequestedX, RequestedY, RequestedZ;
    m_pAxisMapping->GetAxisMapping(RequestedX, RequestedY, RequestedZ);

    Direction::Enum ServerX, ServerY, ServerZ;

    AxisMappingResult::Enum Error = AxisMappingResult::Success;
    std::shared_ptr< VAxisMapping > pServerAxisMapping;

    // If we've got information about our stream type from the server
    if (m_CachedFrame.m_ApplicationInfo && m_CachedFrame.m_ApplicationInfo.get().m_AxisOrientation == ViconCGStream::VApplicationInfo::EYUp)
    {
      ServerX = Direction::Forward;
      ServerY = Direction::Up;
      ServerZ = Direction::Right;

      // Only create the server axis mapping if we need to modify the data from the server
      pServerAxisMapping = VAxisMapping::Create(Error, Direction::Forward, Direction::Up, Direction::Right);
    }
    else
    {
      // We either know it's Z-up, or we assume it's Z-up due to lack of contrary information.
      ServerX = Direction::Forward;
      ServerY = Direction::Left;
      ServerZ = Direction::Up;
    }

    // We will avoid the mapping in favour of a pure copy if the input and requested output are the same. 
    if (RequestedX == ServerX && RequestedY == ServerY && RequestedZ == ServerZ)
    {
      std::copy(i_Rotation, i_Rotation + 9, io_Rotation);
    }
    else
    {
      if (pServerAxisMapping && Error == AxisMappingResult::Success)
      {
        double Q[9];
        pServerAxisMapping->GetTransformationMatrix(Q);
        m_pAxisMapping->CopyAndTransformR(i_Rotation, Q, io_Rotation);
      }
      else
      {
        // We don't need to do a server axis mapping
        m_pAxisMapping->CopyAndTransformR(i_Rotation, io_Rotation);
      }
    }
  }
  else
  {
    // Just do a pure copy if there's no axis mapping at all.
    std::copy(i_Rotation, i_Rotation + 9, io_Rotation);
  }
  
}

ViconCGStreamType::UInt64 VClient::GetDevicePeriod( const unsigned int i_DeviceID ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  std::vector< ViconCGStream::VDeviceInfo >::const_iterator It  = m_LatestFrame.m_Devices.begin();
  std::vector< ViconCGStream::VDeviceInfo >::const_iterator End = m_LatestFrame.m_Devices.end();
  for( ; It != End ; ++It )
  {
    const ViconCGStream::VDeviceInfo & rDevice( *It );
    if ( rDevice.m_DeviceID == i_DeviceID )
    {
      return rDevice.m_FramePeriod;
    }
  }
  return 0;
}

ViconCGStreamType::UInt64 VClient::GetDeviceStartTick( const unsigned int i_DeviceID ) const
{
  boost::recursive_mutex::scoped_lock Lock( m_FrameMutex );

  std::vector< ViconCGStream::VDeviceInfoExtra >::const_iterator It  = m_LatestFrame.m_DevicesExtra.begin();
  std::vector< ViconCGStream::VDeviceInfoExtra >::const_iterator End = m_LatestFrame.m_DevicesExtra.end();
  for( ; It != End ; ++It )
  {
    const ViconCGStream::VDeviceInfoExtra & rDevice( *It );
    if ( rDevice.m_DeviceID == i_DeviceID )
    {
      return rDevice.m_FrameStartTick;
    }
  }
  return 0;
}
} // End of namespace Core
} // End of namespace ViconDataStreamSDK
