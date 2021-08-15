
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

#include "Constants.h"
#include "AxisMapping.h"
#include "ClientUtils.h"
#include "RetimingClient.h"
#include <memory>
#include <boost/array.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <ViconCGStreamClientSDK/ICGClient.h>
#include <ViconCGStreamClientSDK/CGClient.h>
#include <ViconCGStreamClientSDK/ICGFrameState.h>


namespace ViconDataStreamSDK
{
namespace Core
{

class VAxisMapping;

class VClient
{
public:

  VClient();
  virtual ~VClient();

  // Get the version of the Vicon Data Stream SDK 
  void GetVersion( unsigned int & o_rMajor, 
                   unsigned int & o_rMinor, 
                   unsigned int & o_rPoint ) const;

  // Connect client to the Vicon Data Stream
  Result::Enum Connect(       std::shared_ptr< ViconCGStreamClientSDK::ICGClient > i_pClient, 
                        const std::string &                                        i_rHostName );

  // Connect client to the Vicon Data Stream
  Result::Enum ConnectToMulticast(       std::shared_ptr< ViconCGStreamClientSDK::ICGClient > i_pClient, 
                                   const std::string &                                        i_rLocalIP,
                                   const std::string &                                        i_rMulticastIP );

  // Disconnect from the Vicon Data Stream
  Result::Enum Disconnect();

  // Multicast transmission
  Result::Enum StartTransmittingMulticast( const std::string & i_rServerIP,
                                           const std::string & i_rMulticastIP );

  Result::Enum StopTransmittingMulticast();

  // states whether the stream is connected or not
  bool IsConnected() const;

  // Control how many frames are buffered by the client (default is one)
  void SetBufferSize( unsigned int i_MaxFrames );

  Result::Enum GetFrame();
  Result::Enum GetFrameNumber( unsigned int & o_rFrameNumber ) const;
  Result::Enum GetFrameRate( double & o_rFrameRateInHz ) const;

  Result::Enum GetTimecode( unsigned int           & o_rHours,
                            unsigned int           & o_rMinutes,
                            unsigned int           & o_rSeconds,
                            unsigned int           & o_rFrames,
                            unsigned int           & o_rSubFrame,
                            bool                   & o_rbFieldFlag,
                            TimecodeStandard::Enum & o_rTimecodeStandard,
                            unsigned int           & o_rSubFramesPerFrame,
                            unsigned int           & o_rUserBits ) const;


  // Latency reporting
  Result::Enum GetNetworkLatency(double & o_rLatency);
  Result::Enum GetLatencyTotal( double & o_rLatency ) const;
  Result::Enum GetLatencySampleCount( unsigned int & o_rSampleCount ) const;
  Result::Enum GetLatencySampleName( const unsigned int i_SampleIndex, std::string & o_rSampleName ) const;
  Result::Enum GetLatencySampleValue( const std::string & i_rSampleName, double & o_rSampleValue ) const;

  // Get native hardware frame (for debugging purposes)
  Result::Enum GetHardwareFrameNumber( unsigned int & o_rFrameNumber ) const;

  // Get framerates information
  Result::Enum GetFrameRateCount( unsigned int & o_rFrameRateCount ) const;
  Result::Enum GetFrameRateName( const unsigned int i_FrameRateIndex, std::string & o_rFrameRateName ) const;
  Result::Enum GetFrameRateValue( const std::string & i_rFrameRateName, double & o_rFrameRateValue ) const;

  Result::Enum SetSegmentDataEnabled( const bool i_i_bEnabled );
  Result::Enum SetMarkerDataEnabled( const bool i_bEnabled );
  Result::Enum SetUnlabeledMarkerDataEnabled( const bool i_bEnabled );
  Result::Enum SetMarkerRayDataEnabled( const bool i_bEnabled );
  Result::Enum SetDeviceDataEnabled( const bool i_bEnabled );
  Result::Enum SetCentroidDataEnabled( const bool i_bEnabled );
  Result::Enum SetGreyscaleDataEnabled( const bool i_bEnabled );
  Result::Enum SetDebugDataEnabled( const bool i_bEnabled );
  Result::Enum SetCameraWand2dDataEnabled( const bool i_bEnabled );
  Result::Enum SetVideoDataEnabled( const bool i_bEnabled );

  Result::Enum EnableSegmentData()         { return SetSegmentDataEnabled( true ); }
  Result::Enum EnableMarkerData()          { return SetMarkerDataEnabled( true ); }
  Result::Enum EnableUnlabeledMarkerData() { return SetUnlabeledMarkerDataEnabled( true ); }
  Result::Enum EnableMarkerRayData()       { return SetMarkerRayDataEnabled( true ); }
  Result::Enum EnableDeviceData()          { return SetDeviceDataEnabled( true ); }
  Result::Enum EnableCentroidData()        { return SetCentroidDataEnabled( true ); }
  Result::Enum EnableGreyscaleData()       { return SetGreyscaleDataEnabled( true ); }
  Result::Enum EnableDebugData()           { return SetDebugDataEnabled( true ); }
  Result::Enum EnableCameraWand2dData()    { return SetCameraWand2dDataEnabled( true ); }
  Result::Enum EnableVideoData()           { return SetVideoDataEnabled( true ); }

  Result::Enum DisableSegmentData()         { return SetSegmentDataEnabled( false ); }
  Result::Enum DisableMarkerData()          { return SetMarkerDataEnabled( false ); }
  Result::Enum DisableUnlabeledMarkerData() { return SetUnlabeledMarkerDataEnabled( false ); }
  Result::Enum DisableMarkerRayData()       { return SetMarkerRayDataEnabled( false ); }
  Result::Enum DisableDeviceData()          { return SetDeviceDataEnabled( false ); }
  Result::Enum DisableCentroidData()        { return SetCentroidDataEnabled( false ); }
  Result::Enum DisableGreyscaleData()       { return SetGreyscaleDataEnabled( false ); }
  Result::Enum DisableDebugData()           { return SetDebugDataEnabled( false ); }
  Result::Enum DisableCameraWand2dData()    { return SetCameraWand2dDataEnabled( false ); }
  Result::Enum DisableVideoData()           { return SetVideoDataEnabled( false ); }

  bool IsSegmentDataEnabled() const;
  bool IsMarkerDataEnabled() const;
  bool IsUnlabeledMarkerDataEnabled() const;
  bool IsMarkerRayDataEnabled() const;
  bool IsDeviceDataEnabled() const;
  bool IsCentroidDataEnabled() const;
  bool IsGreyscaleDataEnabled() const;
  bool IsDebugDataEnabled() const;
  bool IsCameraWand2dDataEnabled() const;
  bool IsVideoDataEnabled() const;

  Result::Enum SetStreamMode( const StreamMode::Enum i_Mode );

  Result::Enum SetApexDeviceFeedback( const std::string& i_rDeviceName, bool i_bOn );

  Result::Enum SetAxisMapping( const Direction::Enum   i_XAxis,  const Direction::Enum   i_YAxis,  const Direction::Enum   i_ZAxis );
  void         GetAxisMapping( Direction::Enum & o_rXAxis, Direction::Enum & o_rYAxis, Direction::Enum & o_rZAxis ) const;

  Result::Enum GetServerOrientation( ServerOrientation::Enum & o_rServerOrientation ) const;
  Result::Enum GetSubjectCount( unsigned int & o_rSubjectCount ) const;
  Result::Enum GetSubjectName( const unsigned int i_SubjectIndex, std::string& o_rSubjectName ) const;
  Result::Enum GetSubjectRootSegmentName( const std::string & i_rSubjectName, std::string & o_rSegmentName ) const;
  
  Result::Enum GetSegmentCount(const std::string& i_rSubjectName, unsigned int& o_rSegmentCount) const;
  Result::Enum GetSegmentName( const std::string& i_rSubjectName, const unsigned int i_SegmentIndex, std::string& o_rSegmentName) const;
  Result::Enum GetSegmentChildCount( const std::string& i_rSubjectName, const std::string& i_rSegmentName, unsigned int & o_rSegmentCount ) const; 
  Result::Enum GetSegmentChildName( const std::string& i_rSubjectName, const std::string& i_rSegmentName, unsigned int i_SegmentIndex, std::string& o_rSegmentName ) const; 
  Result::Enum GetSegmentParentName( const std::string& i_rSubjectName, const std::string& i_rSegmentName, std::string& o_rSegmentName ) const; 

  Result::Enum GetSegmentStaticTranslation(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double (&o_rThreeVector)[3] ) const;
  Result::Enum GetSegmentStaticRotationHelical(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double (&o_rThreeVector)[3] ) const;
  Result::Enum GetSegmentStaticRotationMatrix( const std::string & i_rSubjectName, const std::string & i_rSegmentName, double (& o_rRotation)[9] ) const;
  Result::Enum GetSegmentStaticRotationQuaternion(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double (&o_rFourVector)[4] ) const;
  Result::Enum GetSegmentStaticRotationEulerXYZ(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double (&o_rThreeVector)[3] ) const;

  Result::Enum GetSegmentGlobalTranslation(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double (&o_rThreeVector)[3], bool& o_rbOccludedFlag) const;
  Result::Enum GetSegmentGlobalRotationHelical(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double (&o_rThreeVector)[3], bool& o_rbOccludedFlag) const;
  Result::Enum GetSegmentGlobalRotationMatrix( const std::string & i_rSubjectName, const std::string & i_rSegmentName, double (& o_rRotation)[9], bool & o_rbOccluded ) const;
  Result::Enum GetSegmentGlobalRotationQuaternion(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double (&o_rFourVector)[4], bool& o_rbOccluded ) const;
  Result::Enum GetSegmentGlobalRotationEulerXYZ(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double (&o_rThreeVector)[3], bool& o_rbOccludedFlag) const;

  Result::Enum GetSegmentLocalTranslation(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double (&o_pThreeVector)[3], bool& o_rbOccludedFlag) const;
  Result::Enum GetSegmentLocalRotationHelical(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double (&o_pThreeVector)[3], bool& o_rbOccludedFlag) const;
  Result::Enum GetSegmentLocalRotationMatrix( const std::string & i_rSubjectName, const std::string & i_rSegmentName, double (& o_rRotation)[9], bool & o_rbOccluded ) const;
  Result::Enum GetSegmentLocalRotationQuaternion(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double (&o_pFourVector)[4], bool& o_rbOccludedFlag) const;
  Result::Enum GetSegmentLocalRotationEulerXYZ(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double (&o_pThreeVector)[3], bool& o_rbOccludedFlag) const;

  Result::Enum GetObjectQuality( const std::string& i_rObjectName, double& o_rQuality ) const;
  Result::Enum GetMarkerCount( const std::string& i_rSubjectName, unsigned int& o_rMarkerCount ) const;
  Result::Enum GetMarkerName(const std::string& i_rSubjectName, const unsigned int i_MarkerIndex, std::string& o_rMarkerName) const;
  Result::Enum GetMarkerParentName( const std::string & i_rSubjectName, const std::string & i_rMarkerName, std::string & o_rSegmentName ) const;
  Result::Enum GetMarkerGlobalTranslation( const std::string& i_rSubjectName, const std::string& i_rMarkerName, double (&o_rThreeVector)[3], bool& o_rbOccludedFlag ) const;
  Result::Enum GetMarkerRayAssignmentCount( const std::string& i_rSubjectName, const std::string& i_rMarkerName, unsigned int & o_rAssignmentCount ) const;
  Result::Enum GetMarkerRayAssignment( const std::string& i_rSubjectName , const std::string& i_rMarkerName , int i_rAssignmentIndex , unsigned int & o_rCameraID , unsigned int & o_rCentroidIndex ) const;

  Result::Enum GetUnlabeledMarkerCount( unsigned int & o_rMarkerCount ) const;
  Result::Enum GetUnlabeledMarkerGlobalTranslation( const unsigned int   i_MarkerIndex,
                                                          double      (& o_rTranslation)[3] ) const;

  Result::Enum GetLabeledMarkerCount( unsigned int & o_rMarkerCount ) const;
  Result::Enum GetLabeledMarkerGlobalTranslation( const unsigned int   i_MarkerIndex,
                                                        double      (& o_rTranslation)[3] ) const;

  Result::Enum GetDeviceCount( unsigned int            & o_rDeviceCount ) const;

  // Empty device names are automatically assigned a name 'Unnamed Device N' where N is a 1 based device number.

  Result::Enum GetDeviceName( const unsigned int         i_DeviceIndex,
                                    std::string        & o_rDeviceName,
                                    DeviceType::Enum   & o_rDeviceType ) const;

  Result::Enum GetDeviceOutputCount( const std::string  & i_rDeviceName,
                                           unsigned int & o_rDeviceOutputCount ) const;

  // Empty device output names are automatically assigned a name 'Unnamed Device Output N' where N is a 1 based device output index.

  Result::Enum GetDeviceOutputName( const std::string  & i_rDeviceName,
                                    const unsigned int   i_DeviceOutputIndex,
                                          std::string  & o_rDeviceOutputName,
                                          Unit::Enum   & o_rDeviceOutputUnit ) const;

  // Get the first subsample for the device output.

  Result::Enum GetDeviceOutputValue( const std::string & i_rDeviceName,
                                     const std::string & i_rDeviceOutputName,
                                           double      & o_rValue,
                                           bool        & o_rbOccluded ) const;
  
  // Get the number of subsamples for this device output.

  Result::Enum GetDeviceOutputSubsamples( const std::string  & i_rDeviceName, 
                                          const std::string  & i_rDeviceOutputName, 
                                                unsigned int & o_rDeviceOutputSubsamples,
                                                bool         & o_rbOccluded ) const;

  // Get a specific subsample.

  Result::Enum GetDeviceOutputValue( const std::string  & i_rDeviceName,
                                     const std::string  & i_rDeviceOutputName,
                                           unsigned int   i_Subsample,
                                           double       & o_rValue,
                                           bool         & o_rbOccluded ) const;

  Result::Enum GetForcePlateCount( unsigned int & o_rCount ) const;
  Result::Enum GetForcePlateID( const unsigned int i_ZeroIndexedPlateIndex, unsigned int& o_rPlateID ) const;

  // This set of functions return the first subsample only.

  Result::Enum GetForceVector( const unsigned int i_PlateID, double ( & o_rForceVector)[3] ) const;  
  Result::Enum GetMomentVector( const unsigned int i_PlateID, double ( & o_rMomentVector)[3] ) const; 
  Result::Enum GetCentreOfPressure( const unsigned int i_PlateID, double ( & o_rLocation )[3] ) const;
  Result::Enum GetGlobalForceVector( const unsigned int i_PlateID, double ( & o_rForceVector)[3] ) const;  
  Result::Enum GetGlobalMomentVector( const unsigned int i_PlateID, double ( & o_rMomentVector)[3] ) const; 
  Result::Enum GetGlobalCentreOfPressure( const unsigned int i_PlateID, double ( & o_rLocation )[3] ) const;

  // Returns the number of subsamples available for the plate.

  Result::Enum GetForcePlateSubsamples( const unsigned int i_PlateID, unsigned int& o_rForcePlateSubsamples ) const;

  Result::Enum GetForceVector( const unsigned int i_PlateID, const unsigned int i_Subsample, double ( & o_rForceVector)[3] ) const;  
  Result::Enum GetMomentVector( const unsigned int i_PlateID, const unsigned int i_Subsample, double ( & o_rMomentVector)[3] ) const; 
  Result::Enum GetCentreOfPressure( const unsigned int i_PlateID, const unsigned int i_Subsample, double ( & o_rLocation )[3] ) const;
  Result::Enum GetGlobalForceVector( const unsigned int i_PlateID, const unsigned int i_Subsample, double ( & o_rForceVector)[3] ) const;  
  Result::Enum GetGlobalMomentVector( const unsigned int i_PlateID, const unsigned int i_Subsample, double ( & o_rMomentVector)[3] ) const; 
  Result::Enum GetGlobalCentreOfPressure( const unsigned int i_PlateID, const unsigned int i_Subsample, double ( & o_rLocation )[3] ) const;

  // Get the raw analogue associated with the plate.

  Result::Enum GetNumberOfAnalogChannels( const unsigned int i_PlateID, unsigned int& o_rChannelCount ) const;
  Result::Enum GetAnalogChannelVoltage( const unsigned int i_PlateID, const unsigned int i_ZeroIndexedChannelIndex, double& o_rVoltage ) const;
  Result::Enum GetAnalogChannelVoltage( const unsigned int i_PlateID, const unsigned int i_ZeroIndexedChannelIndex, const unsigned int i_Subsample, double& o_rVoltage ) const;

  Result::Enum GetEyeTrackerCount( unsigned int & o_rCount ) const;
  Result::Enum GetEyeTrackerID( const unsigned int i_EyeTrackerIndex, unsigned int& o_rEyeTrackerID ) const;
  Result::Enum GetEyeTrackerGlobalPosition( const unsigned int i_EyeTrackerID, double (&o_rThreeVector)[3], bool& o_rbOccludedFlag ) const;
  Result::Enum GetEyeTrackerGlobalGazeVector( const unsigned int i_EyeTrackerID, double (&o_rThreeVector)[3], bool& o_rbOccludedFlag ) const;

  Result::Enum GetCameraCount( unsigned int & o_rCount ) const;
  Result::Enum GetCameraName( const unsigned int i_CameraIndex, std::string & o_rCameraName ) const;
  Result::Enum GetCameraID( const std::string & i_rCameraName, unsigned int & o_rCameraID ) const;
  Result::Enum GetCameraUserID( const std::string & i_rCameraName, unsigned int & o_rCameraID ) const;
  Result::Enum GetCameraType( const std::string & i_rCameraName, std::string & o_rCameraType ) const;
  Result::Enum GetCameraDisplayName( const std::string & i_rCameraName, std::string & o_rCameraDisplayName ) const;

  Result::Enum GetCameraResolution( const std::string & i_rCameraName, unsigned int & o_rResolutionX, unsigned int & o_rResolutionY ) const;
  Result::Enum GetIsVideoCamera( const std::string & i_rCameraName, bool & o_rIsVideoCamera ) const;

  Result::Enum GetCentroidCount( const std::string & i_rCameraName, unsigned int & o_rCount ) const;
  Result::Enum GetCentroidPosition( const std::string & i_rCameraName,
                                    const unsigned int i_CentroidIndex,
                                    double (&o_rPosition)[2],
                                    double & o_rRadius /*,
                                    double & o_rAccuracy */ ) const;
  Result::Enum GetCentroidWeight( const std::string & i_rCameraName,
                                  const unsigned int i_CentroidIndex,
                                  double & o_rWeight ) const;

  Result::Enum GetGreyscaleBlobCount( const std::string & i_rCameraName, unsigned int & o_rCount ) const;
  Result::Enum GetGreyscaleBlob( const std::string & i_rCameraName,
                                 const unsigned int i_BlobIndex,
                                 std::vector< unsigned int > & o_rLineXPositions,
                                 std::vector< unsigned int > & o_rLineYPositions,
                                 std::vector< std::vector< unsigned char > > & o_rLinePixelValues ) const;

  Result::Enum GetVideoFrame( const std::string & i_rCameraName, ViconCGStreamClientSDK::VVideoFramePtr & o_rVideoFramePtr ) const;

  // Only stream camera data for the given cameras
  Result::Enum SetCameraFilter( const std::vector< unsigned int > & i_rCameraIdsForCentroids,
                                const std::vector< unsigned int > & i_rCameraIdsForBlobs,
                                const std::vector< unsigned int > & i_rCameraIdsForVideo );

  ViconCGStreamClientSDK::ICGFrameState& LatestFrame();
  ViconCGStreamClientSDK::ICGFrameState& CachedFrame();

private:
  Result::Enum GetSubjectAndMarkerID(const std::string& i_rSubjectName, const std::string& i_rMarkerName, unsigned int& o_rSubjectID, unsigned int& o_rMarkerID) const;
  Result::Enum GetSubjectAndSegmentID(const std::string& i_rSubjectName, const std::string& i_rSegmentName, unsigned int& o_rSubjectID, unsigned int& o_rSegmentID) const;
  Result::Enum GetDeviceID( const std::string & i_rDeviceName, unsigned int & o_rDeviceID ) const;
  Result::Enum GetReconRayAssignments( const std::string& i_rSubjectName , const std::string& i_rMarkerName , std::vector< unsigned int >& o_rCameraIDs , std::vector< unsigned int >& o_rCentroidIndex ) const;

  const ViconCGStream::VSubjectInfo     * GetSubjectInfo( const std::string  i_rSubjectName, Result::Enum & o_rResult ) const;
  const ViconCGStream::VSubjectTopology * GetSubjectTopology( const unsigned int i_SubjectID ) const;
  const ViconCGStream::VObjectQuality   * GetObjectQuality( const unsigned int i_SubjectID ) const;
  const ViconCGStream::VDeviceInfo      * GetDevice( const std::string & i_rDeviceName, Result::Enum & o_rResult ) const;
  const ViconCGStream::VCameraInfo      * GetCamera( const std::string & i_rCameraName, Result::Enum & o_rResult ) const;
  const ViconCGStream::VCentroids       * GetCentroidSet( const unsigned int i_CameraID, Result::Enum & o_rResult ) const;
  const ViconCGStream::VCentroidWeights * GetCentroidWeightSet( const unsigned int i_CameraID, Result::Enum & o_rResult ) const;
  const ViconCGStream::VGreyscaleBlobs  * GetGreyscaleBlobs( const unsigned int i_CameraID, Result::Enum & o_rResult ) const;
  void  GetVideoFrame( const unsigned int i_CameraID, Result::Enum & o_rResult, ViconCGStreamClientSDK::VVideoFramePtr & o_rVideoFramePtr ) const;

  Result::Enum GetMarkerID( const ViconCGStream::VSubjectInfo & i_rSubjectInfo, const std::string& i_rMarkerName, unsigned int& o_rMarkerID ) const;
  Result::Enum GetSegmentID( const ViconCGStream::VSubjectInfo & i_rSubjectInfo, const std::string& i_rSegmentName, unsigned int& o_rSegmentID ) const;

  bool IsForcePlateCoreChannel(const ViconCGStream::VChannelInfo& rChannel) const;
  bool IsForcePlateForceChannel(const ViconCGStream::VChannelInfo& rChannel) const;
  bool IsForcePlateMomentChannel(const ViconCGStream::VChannelInfo& rChannel) const;
  bool IsForcePlateCoPChannel(const ViconCGStream::VChannelInfo& rChannel) const;
  bool IsForcePlateDevice(unsigned int i_DeviceID) const;

  bool ForcePlateDeviceIndex( unsigned int i_DeviceID, unsigned int & o_rZeroBasedIndex ) const;

  Result::Enum ForcePlateSubsamples( unsigned int i_PlateID, unsigned int & o_rForcePlateSubsamples ) const;
  template < typename T > Result::Enum GetForcePlateVector( const unsigned int i_PlateID,
                                                            const unsigned int i_ForcePlateSubsamples,
                                                            const std::vector< T > & i_rFrameVector,
                                                            boost::array< double, 3 > & o_rForcePlateVector ) const;
  Result::Enum GetForceVector( const unsigned int i_PlateID, const unsigned int i_ForcePlateSubsamples, boost::array< double, 3 > & o_rForceVector ) const;  
  Result::Enum GetMomentVector( const unsigned int i_PlateID, const unsigned int i_ForcePlateSubsamples, boost::array< double, 3 > & o_rMomentVector ) const; 
  Result::Enum GetCentreOfPressure( const unsigned int i_PlateID, const unsigned int i_ForcePlateSubsamples, boost::array< double, 3 > & o_rLocation ) const;

  bool InitGet( Result::Enum & o_rResult ) const;
  template < typename T > bool InitGet( Result::Enum & o_rResult, T & o_rOutput ) const;
  template < typename T1, typename T2 > bool InitGet( Result::Enum & o_rResult, T1 & o_rOutput1, T2 & o_rOutput2  ) const;
  template < typename T1, typename T2, typename T3 > bool InitGet( Result::Enum & o_rResult, T1 & o_rOutput1, T2 & o_rOutput2, T3 & o_rOutput3  ) const;

  bool IsEyeTrackerDevice(unsigned int i_DeviceID) const;

  bool HasData() const;

  void FetchNextFrame();
  void PreFetchThreadBody();

  void BeginPreFetchThread();
  void EndPreFetchThread();

  void CopyAndTransformT( const double i_Translation[ 3 ], double ( & io_Translation )[ 3 ] ) const;
  void CopyAndTransformR( const double i_Rotation[ 9 ], double ( & io_Rotation )[ 9 ] ) const;

  ViconCGStreamType::UInt64 GetDevicePeriod( const unsigned int i_DeviceID ) const;
  ViconCGStreamType::UInt64 GetDeviceStartTick( const unsigned int i_DeviceID ) const;

  // reference the client
  std::shared_ptr< ViconCGStreamClientSDK::ICGClient > m_pClient;
  std::string                                          m_ServerName;
  unsigned short                                       m_ServerPort;

  std::shared_ptr< boost::thread > m_pPreFetchThread;
  bool m_bContinuePreFetch;

  ViconCGStreamClientSDK::ICGFrameState m_LatestFrame;
  ViconCGStreamClientSDK::ICGFrameState m_CachedFrame;
  bool                                  m_bNewCachedFrame;

  mutable boost::recursive_mutex m_FrameMutex;

  // What data is being requested
  bool m_bSegmentDataEnabled;
  bool m_bMarkerDataEnabled;
  bool m_bUnlabeledMarkerDataEnabled;
  bool m_bMarkerRayDataEnabled;
  bool m_bDeviceDataEnabled;
  bool m_bCentroidDataEnabled;
  bool m_bGreyscaleDataEnabled;
  bool m_bDebugDataEnabled;
  bool m_bCameraWand2dDataEnabled;
  bool m_bFrameRateInfoDataEnabled;
  bool m_bVideoDataEnabled;

  // Axis mapping object
  std::shared_ptr< VAxisMapping > m_pAxisMapping;

  // Current data filter
  ViconCGStream::VFilter m_Filter;

  std::shared_ptr< VRetimingClient > m_pRetimingClient;

  unsigned int m_BufferSize;
};

/* Needs VC12 :(
template< typename T, typename... TArgs >
bool ViconDataStreamSDK::Core::VClient::InitGet( Result::Enum & o_rResult, T & o_rOutput TArgs &... o_rArgs ) const
{
  Clear( o_rOutput );
  return InitGet( o_rResult, o_rArgs );
}
*/

template < typename T >
bool ViconDataStreamSDK::Core::VClient::InitGet( Result::Enum & o_rResult, T & o_rOutput ) const
{
  ClientUtils::Clear( o_rOutput );
  return InitGet( o_rResult );
}

template < typename T1, typename T2 >
bool ViconDataStreamSDK::Core::VClient::InitGet( Result::Enum & o_rResult, T1 & o_rOutput1, T2 & o_rOutput2 ) const
{
  ClientUtils::Clear( o_rOutput1 );
  ClientUtils::Clear( o_rOutput2 );
  return InitGet( o_rResult );
}

template < typename T1, typename T2, typename T3 >
bool ViconDataStreamSDK::Core::VClient::InitGet( Result::Enum & o_rResult, T1 & o_rOutput1, T2 & o_rOutput2, T3 & o_rOutput3 ) const
{
  ClientUtils::Clear( o_rOutput1 );
  ClientUtils::Clear( o_rOutput2 );
  ClientUtils::Clear( o_rOutput3 );
  return InitGet( o_rResult );
}

} // End of namespace Core
} // End of namespace ViconDataStreamSDK
