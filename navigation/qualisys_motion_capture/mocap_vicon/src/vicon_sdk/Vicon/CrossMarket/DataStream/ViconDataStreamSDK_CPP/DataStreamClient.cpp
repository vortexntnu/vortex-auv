
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

// We are going to export these from a DLL
#define _EXPORTING

#include "DataStreamClient.h"
#include <memory>
#include <ViconDataStreamSDKCore/CoreClient.h>
#include "StringFactory.h"

#include "CoreAdapters.h"

namespace ViconDataStreamSDK
{
namespace CPP
{



  // This class just wraps something whose identity we don't want to expose
  // in our header file
  class ClientImpl
  {
  public:
    ClientImpl()
    : m_pCoreClient( new ViconDataStreamSDK::Core::VClient() )
    , m_pStringFactory( new VStringFactory() )
    {
    }

    std::shared_ptr< ViconDataStreamSDK::Core::VClient > m_pCoreClient;
    std::shared_ptr< VStringFactory                    > m_pStringFactory;
  };


  // Constructor
  CLASS_DECLSPEC
  Client::Client()
  : m_pClientImpl( new ClientImpl() )
  {
  }

  // Destructor
  CLASS_DECLSPEC
  Client::~Client()
  {
    delete m_pClientImpl;
    m_pClientImpl = 0;
  }

  // GetVersion
  CLASS_DECLSPEC
  Output_GetVersion Client::GetVersion() const
  {
    Output_GetVersion Output;
    m_pClientImpl->m_pCoreClient->GetVersion( Output.Major, 
                                              Output.Minor, 
                                              Output.Point );

    return Output;
  }

  // Connect
  CLASS_DECLSPEC
  Output_Connect Client::Connect( const String & HostName )
  {
    std::shared_ptr< ViconCGStreamClientSDK::ICGClient > pClient( ViconCGStreamClientSDK::ICGClient::CreateCGClient(), 
                                                                    boost::bind( &ViconCGStreamClientSDK::ICGClient::Destroy, _1 ) );
    Output_Connect Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->Connect( pClient, HostName ) );
    
    return Output;
  }

  // ConnectToMulticast
  CLASS_DECLSPEC
  Output_ConnectToMulticast Client::ConnectToMulticast( const String & LocalIP,
                                                        const String & MulticastIP )
  {
    std::shared_ptr< ViconCGStreamClientSDK::ICGClient > pClient( ViconCGStreamClientSDK::ICGClient::CreateCGClient(), 
                                                                    boost::bind( &ViconCGStreamClientSDK::ICGClient::Destroy, _1 ) );
    Output_ConnectToMulticast Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->ConnectToMulticast( pClient, LocalIP, MulticastIP ) );
    
    return Output;
  }

  // Disconnect
  CLASS_DECLSPEC
  Output_Disconnect Client::Disconnect()
  {
    Output_Disconnect Output;

    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->Disconnect() );

    return Output;
  }

  // IsConnected
  CLASS_DECLSPEC
  Output_IsConnected Client::IsConnected() const
  {
    Output_IsConnected Output;
    Output.Connected = m_pClientImpl->m_pCoreClient->IsConnected();

    return Output;
  }

  // StartTransmittingMulticast
  CLASS_DECLSPEC
  Output_StartTransmittingMulticast Client::StartTransmittingMulticast( const String & ServerIP,
                                                                        const String & MulticastIP )
  {
    Output_StartTransmittingMulticast Output;

    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->StartTransmittingMulticast( ServerIP, MulticastIP ) );

    return Output;
  }

  // StopTransmittingMulticast
  CLASS_DECLSPEC
  Output_StopTransmittingMulticast Client::StopTransmittingMulticast()
  {
    Output_StopTransmittingMulticast Output;

    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->StopTransmittingMulticast() );

    return Output;
  }

  // SetBufferSize
  CLASS_DECLSPEC
  void Client::SetBufferSize( unsigned int i_BufferSize )
  {
    m_pClientImpl->m_pCoreClient->SetBufferSize( i_BufferSize );
  }
  
  // EnableSegmentData
  CLASS_DECLSPEC
  Output_EnableSegmentData Client::EnableSegmentData()
  {
    Output_EnableSegmentData Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->EnableSegmentData() );

    return Output;
  }

  // EnableMarkerData
  CLASS_DECLSPEC
  Output_EnableMarkerData Client::EnableMarkerData()
  {
    Output_EnableMarkerData Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->EnableMarkerData() );

    return Output;
  }

  // EnableUnlabeledMarkerData
  CLASS_DECLSPEC
  Output_EnableUnlabeledMarkerData Client::EnableUnlabeledMarkerData()
  {
    Output_EnableUnlabeledMarkerData Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->EnableUnlabeledMarkerData() );

    return Output;
  }

  // EnableMarkerRayData
  CLASS_DECLSPEC
  ViconDataStreamSDK::CPP::Output_EnableMarkerRayData Client::EnableMarkerRayData()
  {
    Output_EnableMarkerRayData Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->EnableMarkerRayData() );

    return Output;
  }

  // EnableDeviceData
  CLASS_DECLSPEC
  Output_EnableDeviceData Client::EnableDeviceData()
  {
    Output_EnableDeviceData Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->EnableDeviceData() );

    return Output;
  }

  // Enable centroid data
  CLASS_DECLSPEC
  Output_EnableCentroidData Client::EnableCentroidData()
  {
    Output_EnableCentroidData Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->EnableCentroidData() );
    return Output;
  }

  // Enable greyscale data
  CLASS_DECLSPEC
  Output_EnableGreyscaleData Client::EnableGreyscaleData()
  {
    Output_EnableGreyscaleData Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->EnableGreyscaleData() );
    return Output;
  }

  // Enable video data
  CLASS_DECLSPEC
  Output_EnableVideoData Client::EnableVideoData()
  {
    Output_EnableVideoData Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->EnableVideoData() );
    return Output;
  }

  // Enable debug data
  CLASS_DECLSPEC
  Output_EnableDebugData Client::EnableDebugData()
  {
    Output_EnableDebugData Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->EnableDebugData() );
    return Output;
  }

  // DisableSegmentData
  CLASS_DECLSPEC
  Output_DisableSegmentData Client::DisableSegmentData()
  {
    Output_DisableSegmentData Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->DisableSegmentData() );

    return Output;
  }

  // DisableMarkerData
  CLASS_DECLSPEC
  Output_DisableMarkerData Client::DisableMarkerData()
  {
    Output_DisableMarkerData Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->DisableMarkerData() );

    return Output;
  }

  // DisableUnlabeledMarkerData
  CLASS_DECLSPEC
  Output_DisableUnlabeledMarkerData Client::DisableUnlabeledMarkerData()
  {
    Output_DisableUnlabeledMarkerData Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->DisableUnlabeledMarkerData() );

    return Output;
  }

  // DisableMarkerRayData
  CLASS_DECLSPEC
  ViconDataStreamSDK::CPP::Output_DisableMarkerRayData Client::DisableMarkerRayData()
  {
    Output_DisableMarkerRayData Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->DisableMarkerRayData() );

    return Output;
  }

  // DisableDeviceData
  CLASS_DECLSPEC
  Output_DisableDeviceData Client::DisableDeviceData()
  {
    Output_DisableDeviceData Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->DisableDeviceData() );

    return Output;
  }

  // Disable centroid data
  CLASS_DECLSPEC
  Output_DisableCentroidData Client::DisableCentroidData()
  {
    Output_DisableCentroidData Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->DisableCentroidData() );
    return Output;
  }

  // Disable greyscale data
  CLASS_DECLSPEC
  Output_DisableGreyscaleData Client::DisableGreyscaleData()
  {
    Output_DisableGreyscaleData Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->DisableGreyscaleData() );
    return Output;
  }

  // Disable video data
  CLASS_DECLSPEC
  Output_DisableVideoData Client::DisableVideoData()
  {
    Output_DisableVideoData Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->DisableVideoData() );
    return Output;
  }

  // Disable debug data
  CLASS_DECLSPEC
  Output_DisableDebugData Client::DisableDebugData()
  {
    Output_DisableDebugData Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->DisableDebugData() );
    return Output;
  }


  // IsSegmentDataEnabled
  CLASS_DECLSPEC
  Output_IsSegmentDataEnabled Client::IsSegmentDataEnabled() const
  {
    Output_IsSegmentDataEnabled Output;
    Output.Enabled = m_pClientImpl->m_pCoreClient->IsSegmentDataEnabled();

    return Output;
  }

  // IsMarkerDataEnabled
  CLASS_DECLSPEC
  Output_IsMarkerDataEnabled Client::IsMarkerDataEnabled() const
  {
    Output_IsMarkerDataEnabled Output;
    Output.Enabled = m_pClientImpl->m_pCoreClient->IsMarkerDataEnabled();

    return Output;
  }

  // IsUnlabeledMarkerDataEnabled
  CLASS_DECLSPEC
  Output_IsUnlabeledMarkerDataEnabled Client::IsUnlabeledMarkerDataEnabled() const
  {
    Output_IsUnlabeledMarkerDataEnabled Output;
    Output.Enabled = m_pClientImpl->m_pCoreClient->IsUnlabeledMarkerDataEnabled();

    return Output;
  }

  // IsMarkerRayDataEnabled
  CLASS_DECLSPEC
  ViconDataStreamSDK::CPP::Output_IsMarkerRayDataEnabled Client::IsMarkerRayDataEnabled() const
  {
    Output_IsMarkerRayDataEnabled Output;
    Output.Enabled = m_pClientImpl->m_pCoreClient->IsMarkerRayDataEnabled();

    return Output;
  }

  // IsDeviceDataEnabled
  CLASS_DECLSPEC
  Output_IsDeviceDataEnabled Client::IsDeviceDataEnabled() const
  {
    Output_IsDeviceDataEnabled Output;
    Output.Enabled = m_pClientImpl->m_pCoreClient->IsDeviceDataEnabled();

    return Output;
  }

  // IsCentroidDataEnabled
  CLASS_DECLSPEC
  Output_IsCentroidDataEnabled Client::IsCentroidDataEnabled() const
  {
    Output_IsCentroidDataEnabled Output;
    Output.Enabled = m_pClientImpl->m_pCoreClient->IsCentroidDataEnabled();

    return Output;
  }

  // IsGreyscaleDataEnabled
  CLASS_DECLSPEC
  Output_IsGreyscaleDataEnabled Client::IsGreyscaleDataEnabled() const
  {
    Output_IsGreyscaleDataEnabled Output;
    Output.Enabled = m_pClientImpl->m_pCoreClient->IsGreyscaleDataEnabled();

    return Output;
  }

  // IsVideoDataEnabled
  CLASS_DECLSPEC
  Output_IsVideoDataEnabled Client::IsVideoDataEnabled() const
  {
    Output_IsVideoDataEnabled Output;
    Output.Enabled = m_pClientImpl->m_pCoreClient->IsVideoDataEnabled();

    return Output;
  }

  // IsDebugDataEnabled
  CLASS_DECLSPEC
  Output_IsDebugDataEnabled Client::IsDebugDataEnabled() const
  {
    Output_IsDebugDataEnabled Output;
    Output.Enabled = m_pClientImpl->m_pCoreClient->IsDebugDataEnabled();

    return Output;
  }

  // SetStreamMode
  CLASS_DECLSPEC
  Output_SetStreamMode Client::SetStreamMode( const StreamMode::Enum Mode )
  {
    Output_SetStreamMode Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->SetStreamMode( Adapt( Mode ) ) );

    return Output;
  }

  ViconDataStreamSDK::CPP::Output_SetApexDeviceFeedback Client::SetApexDeviceFeedback( const String& i_rDeviceName, bool i_bOn )
  {
    Output_SetApexDeviceFeedback Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->SetApexDeviceFeedback( i_rDeviceName, i_bOn ) );

    return Output;
  }

  // SetAxisMapping
  CLASS_DECLSPEC
  Output_SetAxisMapping Client::SetAxisMapping( const Direction::Enum XAxis, 
                                                const Direction::Enum YAxis, 
                                                const Direction::Enum ZAxis )
  {
    Output_SetAxisMapping Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->SetAxisMapping( Adapt( XAxis ), 
                                                                         Adapt( YAxis ), 
                                                                         Adapt( ZAxis ) ) );

    return Output;
  }

  // GetAxisMapping
  CLASS_DECLSPEC
  Output_GetAxisMapping Client::GetAxisMapping() const
  {
    ViconDataStreamSDK::Core::Direction::Enum _XAxis;
    ViconDataStreamSDK::Core::Direction::Enum _YAxis;
    ViconDataStreamSDK::Core::Direction::Enum _ZAxis;
    
    m_pClientImpl->m_pCoreClient->GetAxisMapping( _XAxis, _YAxis, _ZAxis );

    Output_GetAxisMapping Output;
    Output.XAxis = Adapt( _XAxis );
    Output.YAxis = Adapt( _YAxis );
    Output.ZAxis = Adapt( _ZAxis );

    return Output;
  }

  ViconDataStreamSDK::CPP::Output_GetServerOrientation Client::GetServerOrientation() const
  {
    Output_GetServerOrientation Output;
    ViconDataStreamSDK::Core::ServerOrientation::Enum Orientation;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetServerOrientation( Orientation ) ); 
    Output.Orientation = Adapt( Orientation );

    return Output;
  }

  // GetFrame
  CLASS_DECLSPEC
  Output_GetFrame Client::GetFrame()
  {
    Output_GetFrame Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetFrame() );

    return Output;
  }

  // GetFrameNumber
  CLASS_DECLSPEC
  Output_GetFrameNumber Client::GetFrameNumber() const
  {
    Output_GetFrameNumber Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetFrameNumber( Output.FrameNumber ) );

    return Output;
  }

  // GetTimecode
  CLASS_DECLSPEC
  Output_GetTimecode Client::GetTimecode() const
  {
    Output_GetTimecode Output;
    ViconDataStreamSDK::Core::TimecodeStandard::Enum _TimecodeStandard;

    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetTimecode( Output.Hours,
                                                                      Output.Minutes,
                                                                      Output.Seconds,
                                                                      Output.Frames,
                                                                      Output.SubFrame,
                                                                      Output.FieldFlag,
                                                                      _TimecodeStandard,
                                                                      Output.SubFramesPerFrame,
                                                                      Output.UserBits ) );

    Output.Standard = Adapt( _TimecodeStandard );
    
    return Output;
  }

  // GetFrameRate
  CLASS_DECLSPEC
  Output_GetFrameRate Client::GetFrameRate() const
  {
    Output_GetFrameRate Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetFrameRate( Output.FrameRateHz ) );
    
    return Output;
  }

  // GetLatencySampleCount
  CLASS_DECLSPEC
  Output_GetLatencySampleCount Client::GetLatencySampleCount() const
  {
    Output_GetLatencySampleCount Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetLatencySampleCount( Output.Count ) );

    return Output;
  }

  // GetLatencySampleName
  CLASS_DECLSPEC
  Output_GetLatencySampleName Client::GetLatencySampleName( const unsigned int LatencySampleIndex ) const
  {
    Output_GetLatencySampleName Output;
    std::string Name;

    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetLatencySampleName( LatencySampleIndex,
                                                                               Name ) );

    Output.Name.Set( Name.c_str(), *m_pClientImpl->m_pStringFactory.get() );
    return Output;
  }

  // GetLatencySampleValue
  CLASS_DECLSPEC
  Output_GetLatencySampleValue Client::GetLatencySampleValue( const String & LatencySampleName ) const
  {
    Output_GetLatencySampleValue Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetLatencySampleValue( LatencySampleName,
                                                                                Output.Value ) );

    return Output;
  }

  // GetLatencyTotal
  CLASS_DECLSPEC
  Output_GetLatencyTotal Client::GetLatencyTotal() const
  {
    Output_GetLatencyTotal Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetLatencyTotal( Output.Total ) );

    return Output;
  }

  // GetHardwareFrameNumber
  CLASS_DECLSPEC
  Output_GetHardwareFrameNumber Client::GetHardwareFrameNumber() const
  {
    Output_GetHardwareFrameNumber Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetHardwareFrameNumber( Output.HardwareFrameNumber ) );

    return Output;
  }

  Output_GetFrameRateCount  Client::GetFrameRateCount() const
  {
    Output_GetFrameRateCount Output;
    Output.Result = Adapt ( m_pClientImpl->m_pCoreClient->GetFrameRateCount( Output.Count ) );

    return Output;
  }

  Output_GetFrameRateName   Client::GetFrameRateName( const unsigned int FrameRateIndex ) const
  {
    Output_GetFrameRateName Output;
    std::string Name;

    Output.Result = Adapt ( m_pClientImpl->m_pCoreClient->GetFrameRateName( FrameRateIndex, Name ) );

    Output.Name.Set( Name.c_str(), *m_pClientImpl->m_pStringFactory.get() );
    return Output;
  }

  Output_GetFrameRateValue  Client::GetFrameRateValue( const String & FrameRateName ) const
  {
    Output_GetFrameRateValue Output;
    Output.Result = Adapt ( m_pClientImpl->m_pCoreClient->GetFrameRateValue( FrameRateName, Output.Value ) );

    return Output;
  }

  // GetSubjectCount
  CLASS_DECLSPEC
  Output_GetSubjectCount Client::GetSubjectCount() const
  {
    Output_GetSubjectCount Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetSubjectCount( Output.SubjectCount ) );

    return Output;
  }

  // GetSubjectName
  CLASS_DECLSPEC
  Output_GetSubjectName Client::GetSubjectName( const unsigned int SubjectIndex ) const
  {
    Output_GetSubjectName Output;

    std::string SubjectName;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetSubjectName( SubjectIndex, SubjectName ) );
    Output.SubjectName.Set( SubjectName.c_str(), *m_pClientImpl->m_pStringFactory.get() );

    return Output;
  }

  // GetSubjectRootSegmentName
  CLASS_DECLSPEC
  Output_GetSubjectRootSegmentName Client::GetSubjectRootSegmentName( const String & SubjectName ) const
  {
    Output_GetSubjectRootSegmentName Output;
    std::string SegmentName;

    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetSubjectRootSegmentName( SubjectName, SegmentName ) );

    Output.SegmentName.Set( SegmentName.c_str(), *m_pClientImpl->m_pStringFactory.get() );
    return Output;
  }

  // GetSegmentCount
  CLASS_DECLSPEC
  Output_GetSegmentCount Client::GetSegmentCount( const String & SubjectName ) const
  {
    Output_GetSegmentCount Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetSegmentCount( SubjectName, Output.SegmentCount ) );

    return Output;
  }

  // GetSegmentName
  CLASS_DECLSPEC
  Output_GetSegmentName Client::GetSegmentName( const String       & SubjectName,
                                                const unsigned int   SegmentIndex ) const
  {
    Output_GetSegmentName Output;
    std::string SegmentName;

    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetSegmentName( SubjectName, SegmentIndex, SegmentName ) );

    Output.SegmentName.Set( SegmentName.c_str(), *m_pClientImpl->m_pStringFactory.get() );
    return Output;
  }

  // GetSegmentChildCount
  CLASS_DECLSPEC
  Output_GetSegmentChildCount Client::GetSegmentChildCount( const String & SubjectName,
                                                            const String & SegmentName ) const
  {
    Output_GetSegmentChildCount Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetSegmentChildCount( SubjectName, SegmentName, Output.SegmentCount ) );

    return Output;
  }

  // GetSegmentChildName
  CLASS_DECLSPEC
  Output_GetSegmentChildName Client::GetSegmentChildName( const String       & SubjectName,
                                                          const String       & SegmentName,
                                                          const unsigned int   SegmentIndex ) const
  {
    Output_GetSegmentChildName Output;
    std::string SegmentChildName;

    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetSegmentChildName( SubjectName, SegmentName, SegmentIndex, SegmentChildName ) );

    Output.SegmentName.Set( SegmentChildName.c_str(), *m_pClientImpl->m_pStringFactory.get() );
    return Output;
  }

  // GetSegmentParentName
  CLASS_DECLSPEC
  Output_GetSegmentParentName Client::GetSegmentParentName( const String       & SubjectName,
                                                            const String       & SegmentName ) const
  {
    Output_GetSegmentParentName Output;
    std::string SegmentParentName;

    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetSegmentParentName( SubjectName, SegmentName, SegmentParentName ) );

    Output.SegmentName.Set( SegmentParentName.c_str(), *m_pClientImpl->m_pStringFactory.get() );
    return Output;
  }

  // GetSegmentGlobalTranslation
  CLASS_DECLSPEC
  Output_GetSegmentGlobalTranslation Client::GetSegmentGlobalTranslation( const String & SubjectName,
                                                                          const String & SegmentName ) const
  {
    Output_GetSegmentGlobalTranslation Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetSegmentGlobalTranslation( SubjectName, 
                                                                                      SegmentName, 
                                                                                      Output.Translation, 
                                                                                      Output.Occluded ) );

    return Output;
  }

  // GetSegmentGlobalRotationHelical
  CLASS_DECLSPEC
  Output_GetSegmentGlobalRotationHelical Client::GetSegmentGlobalRotationHelical( const String & SubjectName,
                                                                                  const String & SegmentName ) const
  {
    Output_GetSegmentGlobalRotationHelical Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetSegmentGlobalRotationHelical( SubjectName, 
                                                                                          SegmentName, 
                                                                                          Output.Rotation, 
                                                                                          Output.Occluded ) );

    return Output;
  }

  // GetSegmentGlobalRotationMatrix
  CLASS_DECLSPEC
  Output_GetSegmentGlobalRotationMatrix Client::GetSegmentGlobalRotationMatrix( const String & SubjectName,
                                                                                const String & SegmentName ) const
  {
    Output_GetSegmentGlobalRotationMatrix Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetSegmentGlobalRotationMatrix( SubjectName, 
                                                                                         SegmentName, 
                                                                                         Output.Rotation, 
                                                                                         Output.Occluded ) );

    return Output;
  }

  // GetSegmentGlobalRotationQuaternion
  CLASS_DECLSPEC
  Output_GetSegmentGlobalRotationQuaternion Client::GetSegmentGlobalRotationQuaternion( const String & SubjectName,
                                                                                        const String & SegmentName ) const
  {
    Output_GetSegmentGlobalRotationQuaternion Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetSegmentGlobalRotationQuaternion( SubjectName, 
                                                                                             SegmentName, 
                                                                                             Output.Rotation, 
                                                                                             Output.Occluded ) );

    return Output;
  }

  // GetSegmentGlobalRotationEulerXYZ
  CLASS_DECLSPEC
  Output_GetSegmentGlobalRotationEulerXYZ Client::GetSegmentGlobalRotationEulerXYZ( const String & SubjectName,
                                                                                    const String & SegmentName ) const
  {
    Output_GetSegmentGlobalRotationEulerXYZ Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetSegmentGlobalRotationEulerXYZ( SubjectName, 
                                                                                           SegmentName, 
                                                                                           Output.Rotation, 
                                                                                           Output.Occluded ) );

    return Output;
  }

  // GetSegmentStaticTranslation
  CLASS_DECLSPEC
  Output_GetSegmentStaticTranslation Client::GetSegmentStaticTranslation( const String & SubjectName,
                                                                          const String & SegmentName ) const
  {
    Output_GetSegmentStaticTranslation Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetSegmentStaticTranslation( SubjectName, 
                                                                                      SegmentName, 
                                                                                      Output.Translation ) );

    return Output;
  }

  // GetSegmentStaticRotationHelical
  CLASS_DECLSPEC
  Output_GetSegmentStaticRotationHelical Client::GetSegmentStaticRotationHelical( const String & SubjectName,
                                                                                  const String & SegmentName ) const
  {
    Output_GetSegmentStaticRotationHelical Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetSegmentStaticRotationHelical( SubjectName, 
                                                                                          SegmentName, 
                                                                                          Output.Rotation ) );

    return Output;
  }

  // GetSegmentStaticRotationMatrix
  CLASS_DECLSPEC
  Output_GetSegmentStaticRotationMatrix Client::GetSegmentStaticRotationMatrix( const String & SubjectName,
                                                                                const String & SegmentName ) const
  {
    Output_GetSegmentStaticRotationMatrix Output;
    Output.Result = Adapt(m_pClientImpl->m_pCoreClient->GetSegmentStaticRotationMatrix(SubjectName,
      SegmentName,
      Output.Rotation));

    return Output;
  }

  // GetSegmentStaticRotationQuaternion
  CLASS_DECLSPEC
  Output_GetSegmentStaticRotationQuaternion Client::GetSegmentStaticRotationQuaternion( const String & SubjectName,
                                                                                        const String & SegmentName ) const
  {
    Output_GetSegmentStaticRotationQuaternion Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetSegmentStaticRotationQuaternion( SubjectName, 
                                                                                             SegmentName, 
                                                                                             Output.Rotation ) );

    return Output;
  }

  // GetSegmentStaticRotationEulerXYZ
  CLASS_DECLSPEC
  Output_GetSegmentStaticRotationEulerXYZ Client::GetSegmentStaticRotationEulerXYZ( const String & SubjectName,
                                                                                    const String & SegmentName ) const
  {
    Output_GetSegmentStaticRotationEulerXYZ Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetSegmentStaticRotationEulerXYZ( SubjectName, 
                                                                                           SegmentName, 
                                                                                           Output.Rotation ) );

    return Output;
  }

  // GetSegmentLocalTranslation
  CLASS_DECLSPEC
  Output_GetSegmentLocalTranslation Client::GetSegmentLocalTranslation( const String & SubjectName,
                                                                        const String & SegmentName ) const
  {
    Output_GetSegmentLocalTranslation Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetSegmentLocalTranslation( SubjectName, 
                                                                                     SegmentName, 
                                                                                     Output.Translation, 
                                                                                     Output.Occluded ) );

    return Output;
  }

  // GetSegmentLocalRotationHelical
  CLASS_DECLSPEC
  Output_GetSegmentLocalRotationHelical Client::GetSegmentLocalRotationHelical( const String & SubjectName,
                                                                                const String & SegmentName ) const
  {
    Output_GetSegmentLocalRotationHelical Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetSegmentLocalRotationHelical( SubjectName, 
                                                                                         SegmentName, 
                                                                                         Output.Rotation, 
                                                                                         Output.Occluded ) );

    return Output;
  }

  // GetSegmentLocalRotationMatrix
  CLASS_DECLSPEC
  Output_GetSegmentLocalRotationMatrix Client::GetSegmentLocalRotationMatrix( const String & SubjectName,
                                                                              const String & SegmentName ) const
  {
    Output_GetSegmentLocalRotationMatrix Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetSegmentLocalRotationMatrix( SubjectName, 
                                                                                        SegmentName, 
                                                                                        Output.Rotation, 
                                                                                        Output.Occluded ) );

    return Output;
  }

  // GetSegmentLocalRotationQuaternion
  CLASS_DECLSPEC
  Output_GetSegmentLocalRotationQuaternion Client::GetSegmentLocalRotationQuaternion( const String & SubjectName,
                                                                                      const String & SegmentName ) const
  {
    Output_GetSegmentLocalRotationQuaternion Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetSegmentLocalRotationQuaternion( SubjectName, 
                                                                                            SegmentName, 
                                                                                            Output.Rotation, 
                                                                                            Output.Occluded ) );

    return Output;
  }

  // GetSegmentLocalRotationEulerXYZ
  CLASS_DECLSPEC
  Output_GetSegmentLocalRotationEulerXYZ Client::GetSegmentLocalRotationEulerXYZ( const String & SubjectName,
                                                                                  const String & SegmentName ) const
  {
    Output_GetSegmentLocalRotationEulerXYZ Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetSegmentLocalRotationEulerXYZ( SubjectName, 
                                                                                          SegmentName, 
                                                                                          Output.Rotation, 
                                                                                          Output.Occluded ) );

    return Output;
  }
  
  // GetObjectQuality
  CLASS_DECLSPEC
  Output_GetObjectQuality Client::GetObjectQuality( const String & ObjectName ) const
  {
    Output_GetObjectQuality Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetObjectQuality( ObjectName, Output.Quality ) );
    return Output;
  }

  // GetMarkerCount
  CLASS_DECLSPEC
  Output_GetMarkerCount Client::GetMarkerCount( const String  & SubjectName ) const
  {
    Output_GetMarkerCount Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetMarkerCount( SubjectName, Output.MarkerCount ) );

    return Output;
  }

  // GetMarkerName
  CLASS_DECLSPEC
  Output_GetMarkerName Client::GetMarkerName( const String      & SubjectName,
                                              const unsigned int  MarkerIndex ) const
  {
    Output_GetMarkerName Output;
    std::string MarkerName;

    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetMarkerName( SubjectName, MarkerIndex, MarkerName ) );

    Output.MarkerName.Set( MarkerName.c_str(), *m_pClientImpl->m_pStringFactory.get() );
    return Output;
  }

  // GetMarkerParentName
  CLASS_DECLSPEC
  Output_GetMarkerParentName Client::GetMarkerParentName( const String & SubjectName,
                                                          const String & MarkerName ) const
  {
    Output_GetMarkerParentName Output;
    std::string MarkerParentName;

    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetMarkerParentName( SubjectName, MarkerName, MarkerParentName ) );

    Output.SegmentName.Set( MarkerParentName.c_str(), *m_pClientImpl->m_pStringFactory.get() );
    return Output;
  }

  // GetMarkerGlobalTranslation
  CLASS_DECLSPEC
  Output_GetMarkerGlobalTranslation Client::GetMarkerGlobalTranslation( const String & SubjectName,
                                                                        const String & MarkerName ) const
  {
    Output_GetMarkerGlobalTranslation Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetMarkerGlobalTranslation( SubjectName, 
                                                                                     MarkerName, 
                                                                                     Output.Translation, 
                                                                                     Output.Occluded ) );

    return Output;
  }

  // GetMarkerRayContributionCount
  CLASS_DECLSPEC
  ViconDataStreamSDK::CPP::Output_GetMarkerRayContributionCount Client::GetMarkerRayContributionCount( const String & SubjectName, const String & MarkerName ) const
  {
    Output_GetMarkerRayContributionCount Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetMarkerRayAssignmentCount( SubjectName, MarkerName, Output.RayContributionsCount ) );

    return Output;
  }

  // GetMarkerRayContribution
  CLASS_DECLSPEC
  ViconDataStreamSDK::CPP::Output_GetMarkerRayContribution Client::GetMarkerRayContribution( const String & SubjectName, const String & MarkerName, unsigned int MarkerRayContributionIndex ) const
  {
    Output_GetMarkerRayContribution Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetMarkerRayAssignment( SubjectName, MarkerName, MarkerRayContributionIndex, Output.CameraID, Output.CentroidIndex ) );

    return Output;
  }

  // GetUnlabeledMarkerCount
  CLASS_DECLSPEC
  Output_GetUnlabeledMarkerCount Client::GetUnlabeledMarkerCount() const
  {
    Output_GetUnlabeledMarkerCount Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetUnlabeledMarkerCount( Output.MarkerCount ) );

    return Output;
  }

  // GetUnlabeledMarkerGlobalTranslation
  CLASS_DECLSPEC
  Output_GetUnlabeledMarkerGlobalTranslation Client::GetUnlabeledMarkerGlobalTranslation( const unsigned int MarkerIndex ) const
  {
    Output_GetUnlabeledMarkerGlobalTranslation Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetUnlabeledMarkerGlobalTranslation( MarkerIndex, Output.Translation ) );

    return Output;
  }

  // GetLabeledMarkerCount
  CLASS_DECLSPEC
  Output_GetLabeledMarkerCount Client::GetLabeledMarkerCount() const
  {
    Output_GetLabeledMarkerCount Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetLabeledMarkerCount( Output.MarkerCount ) );

    return Output;
  }

  // GetLabeledMarkerGlobalTranslation
  CLASS_DECLSPEC
  Output_GetLabeledMarkerGlobalTranslation Client::GetLabeledMarkerGlobalTranslation( const unsigned int MarkerIndex ) const
  {
    Output_GetLabeledMarkerGlobalTranslation Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetLabeledMarkerGlobalTranslation( MarkerIndex, Output.Translation ) );

    return Output;
  }

  // GetDeviceCount
  CLASS_DECLSPEC
  Output_GetDeviceCount Client::GetDeviceCount() const
  {
    Output_GetDeviceCount Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetDeviceCount( Output.DeviceCount ) );

    return Output;
  }

  // GetDeviceName
  CLASS_DECLSPEC
  Output_GetDeviceName Client::GetDeviceName( const unsigned int DeviceIndex ) const
  {
    Output_GetDeviceName Output;
    std::string _DeviceName;
    ViconDataStreamSDK::Core::DeviceType::Enum _DeviceType;
    
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetDeviceName( DeviceIndex, 
                                                                        _DeviceName,
                                                                        _DeviceType ) );

    Output.DeviceName.Set( _DeviceName.c_str(),* m_pClientImpl->m_pStringFactory.get() );
    Output.DeviceType = Adapt( _DeviceType );
    return Output;
  }

  // GetDeviceOutputCount
  CLASS_DECLSPEC
  Output_GetDeviceOutputCount Client::GetDeviceOutputCount( const String  & DeviceName ) const
  {
    Output_GetDeviceOutputCount Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetDeviceOutputCount( DeviceName, Output.DeviceOutputCount ) );

    return Output;
  }

  // GetDeviceOutputName
  CLASS_DECLSPEC
  Output_GetDeviceOutputName Client::GetDeviceOutputName( const String  & DeviceName,
                                                          const unsigned int   DeviceOutputIndex ) const
  {
    Output_GetDeviceOutputName Output;
    std::string                          _DeviceOutputName;
    ViconDataStreamSDK::Core::Unit::Enum _DeviceOutputUnit;
    Output.Result =  Adapt( m_pClientImpl->m_pCoreClient->GetDeviceOutputName( DeviceName, 
                                                                               DeviceOutputIndex, 
                                                                               _DeviceOutputName, 
                                                                               _DeviceOutputUnit ) );

    Output.DeviceOutputName.Set( _DeviceOutputName.c_str(), *m_pClientImpl->m_pStringFactory.get() );
    Output.DeviceOutputUnit = Adapt( _DeviceOutputUnit );
    
    return Output;
  }

  // GetDeviceOutputValue
  CLASS_DECLSPEC
  Output_GetDeviceOutputValue Client::GetDeviceOutputValue( const String & DeviceName,
                                                            const String & DeviceOutputName ) const
  {
    Output_GetDeviceOutputValue Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetDeviceOutputValue( DeviceName, 
                                                                               DeviceOutputName, 
                                                                               Output.Value, 
                                                                               Output.Occluded ) );

    return Output;
  }

  // GetDeviceOutputSubsamples
  CLASS_DECLSPEC
  Output_GetDeviceOutputSubsamples Client::GetDeviceOutputSubsamples( const String & DeviceName,
                                                                      const String & DeviceOutputName ) const
  {
    Output_GetDeviceOutputSubsamples Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetDeviceOutputSubsamples( DeviceName, 
                                                                                    DeviceOutputName, 
                                                                                    Output.DeviceOutputSubsamples, 
                                                                                    Output.Occluded ) );

    return Output;
  }

  // GetDeviceOutputValue
  CLASS_DECLSPEC
  Output_GetDeviceOutputValue Client::GetDeviceOutputValue( const String & DeviceName,
                                                            const String & DeviceOutputName, 
                                                            const unsigned int Subsample ) const
  {
    Output_GetDeviceOutputValue Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetDeviceOutputValue( DeviceName, 
                                                                               DeviceOutputName,
                                                                               Subsample,
                                                                               Output.Value, 
                                                                               Output.Occluded ) );

    return Output;
  }

  // GetForcePlateCount
  CLASS_DECLSPEC
  Output_GetForcePlateCount Client::GetForcePlateCount() const
  {
    Output_GetForcePlateCount Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetForcePlateCount( Output.ForcePlateCount ) );

    return Output;
  }

  // GetGlobalForceVector
  CLASS_DECLSPEC
  Output_GetGlobalForceVector Client::GetGlobalForceVector( const unsigned int ForcePlateIndex ) const
  {
    return GetGlobalForceVector( ForcePlateIndex, 0 );
  }

  // GetGlobalMomentVector
  CLASS_DECLSPEC
  Output_GetGlobalMomentVector Client::GetGlobalMomentVector( const unsigned int ForcePlateIndex ) const
  {
    return GetGlobalMomentVector( ForcePlateIndex, 0 );
  }

  // GetGlobalCentreOfPressure
  CLASS_DECLSPEC
  Output_GetGlobalCentreOfPressure Client::GetGlobalCentreOfPressure( const unsigned int ForcePlateIndex ) const
  {
    return GetGlobalCentreOfPressure( ForcePlateIndex, 0 );
  }

  // ForcePlateSubsamples
  CLASS_DECLSPEC
  Output_GetForcePlateSubsamples Client::GetForcePlateSubsamples( const unsigned int ForcePlateIndex ) const
  {
    Output_GetForcePlateSubsamples Output;
    Output.ForcePlateSubsamples = 0;

    unsigned int ForcePlateID;
    
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetForcePlateID( ForcePlateIndex, ForcePlateID ) );

    if( Output.Result == Result::Success )
    {
      Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetForcePlateSubsamples( ForcePlateID, Output.ForcePlateSubsamples ) );
    }

    return Output;
  }

  // GetGlobalForceVector
  CLASS_DECLSPEC
  Output_GetGlobalForceVector Client::GetGlobalForceVector( const unsigned int ForcePlateIndex, const unsigned int Subsample ) const
  {
    Output_GetGlobalForceVector Output;
    unsigned int ForcePlateID;
    
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetForcePlateID( ForcePlateIndex, ForcePlateID ) );

    if( Output.Result == Result::Success )
    {
      Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetGlobalForceVector( ForcePlateID, Subsample, Output.ForceVector ) );
    }
    else
    {
      Output.ForceVector[ 0 ] = 0.0;
      Output.ForceVector[ 1 ] = 0.0;
      Output.ForceVector[ 2 ] = 0.0;
    }

    return Output;
  }

  // GetGlobalMomentVector
  CLASS_DECLSPEC
  Output_GetGlobalMomentVector Client::GetGlobalMomentVector( const unsigned int ForcePlateIndex, const unsigned int Subsample ) const
  {
    Output_GetGlobalMomentVector Output;
    unsigned int ForcePlateID;
    
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetForcePlateID( ForcePlateIndex, ForcePlateID ) );

    if( Output.Result == Result::Success )
    {
      Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetGlobalMomentVector( ForcePlateID, Subsample, Output.MomentVector ) );
    }
    else
    {
      Output.MomentVector[ 0 ] = 0.0;
      Output.MomentVector[ 1 ] = 0.0;
      Output.MomentVector[ 2 ] = 0.0;
    }

    return Output;
  }

  // GetGlobalCentreOfPressure
  CLASS_DECLSPEC
  Output_GetGlobalCentreOfPressure Client::GetGlobalCentreOfPressure( const unsigned int ForcePlateIndex, const unsigned int Subsample ) const
  {
    Output_GetGlobalCentreOfPressure Output;
    unsigned int ForcePlateID;
    
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetForcePlateID( ForcePlateIndex, ForcePlateID ) );

    if( Output.Result == Result::Success )
    {
      Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetGlobalCentreOfPressure( ForcePlateID, Subsample, Output.CentreOfPressure ) );
    }
    else
    {
      Output.CentreOfPressure[ 0 ] = 0.0;
      Output.CentreOfPressure[ 1 ] = 0.0;
      Output.CentreOfPressure[ 2 ] = 0.0;
    }

    return Output;
  }


  // GetEyeTrackerCount
  CLASS_DECLSPEC
  Output_GetEyeTrackerCount Client::GetEyeTrackerCount() const
  {
    Output_GetEyeTrackerCount Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetEyeTrackerCount( Output.EyeTrackerCount ) );

    return Output;
  }

  // GetEyeTrackerGlobalPosition
  CLASS_DECLSPEC
  Output_GetEyeTrackerGlobalPosition Client::GetEyeTrackerGlobalPosition( const unsigned int EyeTrackerIndex ) const
  {
    Output_GetEyeTrackerGlobalPosition Output;
    unsigned int EyeTrackerID;
    
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetEyeTrackerID( EyeTrackerIndex, EyeTrackerID ) );

    if( Output.Result == Result::Success )
    {
      Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetEyeTrackerGlobalPosition( EyeTrackerID, Output.Position, Output.Occluded ) );
    }
    else
    {
      Output.Position[ 0 ] = 0.0;
      Output.Position[ 1 ] = 0.0;
      Output.Position[ 2 ] = 0.0;
      Output.Occluded = false;
    }

    return Output;
  }

  // GetEyeTrackerGlobalGazeVector
  CLASS_DECLSPEC
  Output_GetEyeTrackerGlobalGazeVector Client::GetEyeTrackerGlobalGazeVector( const unsigned int EyeTrackerIndex ) const
  {
    Output_GetEyeTrackerGlobalGazeVector Output;
    unsigned int EyeTrackerID;
    
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetEyeTrackerID( EyeTrackerIndex, EyeTrackerID ) );

    if( Output.Result == Result::Success )
    {
      Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetEyeTrackerGlobalGazeVector( EyeTrackerID, Output.GazeVector, Output.Occluded ) );
    }
    else
    {
      Output.GazeVector[ 0 ] = 0.0;
      Output.GazeVector[ 1 ] = 0.0;
      Output.GazeVector[ 2 ] = 0.0;
      Output.Occluded = false;
    }

    return Output;
  }

  CLASS_DECLSPEC
  Output_GetCameraCount Client::GetCameraCount() const
  {
    Output_GetCameraCount Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetCameraCount( Output.CameraCount ) );

    return Output;
  }

  CLASS_DECLSPEC
  Output_GetCameraName Client::GetCameraName( unsigned int i_CameraIndex ) const
  {
    Output_GetCameraName Output;
    std::string _CameraName;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetCameraName( i_CameraIndex, _CameraName ) );
    Output.CameraName.Set( _CameraName.c_str(),* m_pClientImpl->m_pStringFactory.get() );

    return Output;
  }

  CLASS_DECLSPEC
  Output_GetCameraId Client::GetCameraId( const std::string & i_rCameraName ) const
  {
    Output_GetCameraId Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetCameraID( i_rCameraName, Output.CameraId ) );

    return Output;
  }

  CLASS_DECLSPEC
  Output_GetCameraUserId Client::GetCameraUserId( const std::string & i_rCameraName ) const
  {
    Output_GetCameraUserId Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetCameraUserID( i_rCameraName, Output.CameraUserId ) );

    return Output;
  }

  CLASS_DECLSPEC
  Output_GetCameraType Client::GetCameraType( const std::string & i_rCameraName ) const
  {
    Output_GetCameraType Output;
    std::string _CameraType;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetCameraType( i_rCameraName, _CameraType ) );
    Output.CameraType.Set( _CameraType.c_str(),* m_pClientImpl->m_pStringFactory.get() );

    return Output;
  }

  CLASS_DECLSPEC
  Output_GetCameraDisplayName Client::GetCameraDisplayName( const std::string & i_rCameraName ) const
  {
    Output_GetCameraDisplayName Output;
    std::string _CameraDisplayName;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetCameraDisplayName( i_rCameraName, _CameraDisplayName ) );
    Output.CameraDisplayName.Set( _CameraDisplayName.c_str(),* m_pClientImpl->m_pStringFactory.get() );

    return Output;
  }

  CLASS_DECLSPEC
  Output_GetCameraResolution Client::GetCameraResolution( const std::string & i_rCameraName ) const
  {
    Output_GetCameraResolution Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetCameraResolution( i_rCameraName, Output.ResolutionX, Output.ResolutionY ) );

    return Output;
  }

  CLASS_DECLSPEC
  Output_GetIsVideoCamera Client::GetIsVideoCamera( const std::string & i_rCameraName ) const
  {
    Output_GetIsVideoCamera Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetIsVideoCamera( i_rCameraName, Output.IsVideoCamera ) );

    return Output;
  }

  CLASS_DECLSPEC
  Output_GetCentroidCount Client::GetCentroidCount( const std::string & i_rCameraName ) const
  {
    Output_GetCentroidCount Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetCentroidCount( i_rCameraName, Output.CentroidCount ) );

    return Output;
  }

  CLASS_DECLSPEC
  Output_GetCentroidPosition Client::GetCentroidPosition( const std::string & i_rCameraName, const unsigned int i_CentroidIndex ) const
  {
    Output_GetCentroidPosition Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetCentroidPosition( i_rCameraName, i_CentroidIndex, Output.CentroidPosition, Output.Radius /*, Output.Accuracy */ ) );

    return Output;
  }

  CLASS_DECLSPEC
  Output_GetCentroidWeight Client::GetCentroidWeight( const std::string & i_rCameraName, const unsigned int i_CentroidIndex ) const
  {
    Output_GetCentroidWeight Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetCentroidWeight( i_rCameraName, i_CentroidIndex, Output.Weight ) );

    return Output;
  }
  
  CLASS_DECLSPEC
  Output_GetGreyscaleBlobCount Client::GetGreyscaleBlobCount( const std::string & i_rCameraName ) const
  {
    Output_GetGreyscaleBlobCount Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetGreyscaleBlobCount( i_rCameraName, Output.BlobCount ) );

    return Output;
  }

  CLASS_DECLSPEC
  Output_GetGreyscaleBlob Client::GetGreyscaleBlob( const std::string & i_rCameraName, const unsigned int i_BlobIndex ) const
  {
    Output_GetGreyscaleBlob Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetGreyscaleBlob( i_rCameraName, i_BlobIndex, Output.BlobLinePositionsX, Output.BlobLinePositionsY, Output.BlobLinePixelValues ) );

    return Output;
  }

  CLASS_DECLSPEC
  Output_GetVideoFrame Client::GetVideoFrame( const std::string & i_rCameraName ) const
  {

    ViconCGStreamClientSDK::VVideoFramePtr VideoFramePtr;

    Output_GetVideoFrame Output;
    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->GetVideoFrame( i_rCameraName, VideoFramePtr ) );
    if ( Output.Result == Result::Enum::Success )
    {
      Output.m_FrameID = (*VideoFramePtr).m_FrameID;
      Output.m_Format  = (*VideoFramePtr).m_Format;
      Output.m_Width   = (*VideoFramePtr).m_Width;
      Output.m_Height  = (*VideoFramePtr).m_Height;

      Output.m_Data.reset( new std::vector< unsigned char >( (*VideoFramePtr).m_VideoData.size() ) );
      try
      {
        *Output.m_Data = (*VideoFramePtr).m_VideoData;
      }
      catch ( const std::length_error & )
      {
        // Problem rerieving the data from the CG stream
        Output.Result = Result::Unknown;
      }
    }

    return Output;
  }

  CLASS_DECLSPEC
  Output_SetCameraFilter Client::SetCameraFilter( const std::vector< unsigned int > & i_rCameraIdsForCentroids,
                                                  const std::vector< unsigned int > & i_rCameraIdsForBlobs,
                                                  const std::vector< unsigned int > & i_rCameraIdsForVideo )
  {
    Output_SetCameraFilter Output;

    Output.Result = Adapt( m_pClientImpl->m_pCoreClient->SetCameraFilter( i_rCameraIdsForCentroids, i_rCameraIdsForBlobs, i_rCameraIdsForVideo ) );
    return Output;
  }

} // End of namespace CPP
} // End of namespace ViconDataStreamSDK
