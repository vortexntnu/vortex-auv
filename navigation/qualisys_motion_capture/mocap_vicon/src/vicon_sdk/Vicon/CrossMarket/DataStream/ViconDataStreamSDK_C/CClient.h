
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
/**
* @file CClient.h
* @brief The C implementation of the Vicon DataStream SDK
*/

#ifdef WIN32

#ifdef _EXPORTING
#define CDLL_EXPORT    __declspec(dllexport)
#else
#define CDLL_EXPORT    __declspec(dllimport)
#endif // _EXPORTING

#elif defined( __GNUC__ )

#if __GNUC__ < 4
#error gcc 4 is required.
#endif
#define CDLL_EXPORT     __attribute__((visibility("default")))

#else

#define CDLL_EXPORT

#endif



#ifdef __cplusplus
extern "C" { 
#endif

#include "CTypeDefs.h"

  /** Construction.
  *  You can create many instances of the Vicon DataStream Client which can connect to multiple Vicon DataStream Servers.
  *  .NET is object oriented, so use the class constructor.
  *
  *  Because objects are lazily garbage collected, your instance may outlive the last reference to it for some time.
  *
  *  If the instance is pre-fetching frame data for you, then it can still use CPU and network bandwidth.
  *
  *  Consider explicitly disconnecting prior to destruction.
  *
  *       ViconDataStreamSDK.DotNET.Client pHeapClient = new ViconDataStreamSDK.DotNET.Client();
  *       Output_SomeFunction Output = pHeapClient.SomeFunction(InputParam);
  *
  *       // Signal to the garbage collector that it can clean up
  *       pHeapClient.Disconnect();
  *       pHeapClient = null;
  */
  CDLL_EXPORT CClient* Client_Create(void);

/** Destruction.
*  You can create many instances of the Vicon DataStream Client which can connect to multiple Vicon DataStream Servers.
*  .NET is object oriented, so use the class constructor.
*
*  Because objects are lazily garbage collected, your instance may outlive the last reference to it for some time.
*
*  If the instance is pre-fetching frame data for you, then it can still use CPU and network bandwidth.
*
*  Consider explicitly disconnecting prior to destruction.
*
*       ViconDataStreamSDK.DotNET.Client pHeapClient = new ViconDataStreamSDK.DotNET.Client();
*       Output_SomeFunction Output = pHeapClient.SomeFunction(InputParam);
*
*       // Signal to the garbage collector that it can clean up
*       pHeapClient.Disconnect();
*       pHeapClient = null;
*/
CDLL_EXPORT void Client_Destroy(CClient* client);

CDLL_EXPORT void  Client_GetVersion( CClient* client, COutput_GetVersion* outptr);


CDLL_EXPORT CBool Client_Connect(CClient* client, CString HostName );
CDLL_EXPORT CEnum Client_ConnectToMulticast(CClient* client, CString LocalIP, CString MulticastIP );
CDLL_EXPORT CEnum Client_Disconnect(CClient* client);
CDLL_EXPORT CBool Client_IsConnected(CClient* client);
  
CDLL_EXPORT CEnum Client_StartTransmittingMulticast(CClient* client, CString ServerIP,
                                                              CString MulticastIP );

CDLL_EXPORT CEnum Client_StopTransmittingMulticast(CClient* client);

CDLL_EXPORT CEnum Client_EnableSegmentData(CClient* client);
CDLL_EXPORT CEnum Client_EnableMarkerData(CClient* client);
CDLL_EXPORT CEnum Client_EnableUnlabeledMarkerData(CClient* client);
CDLL_EXPORT CEnum Client_EnableDeviceData(CClient* client);

CDLL_EXPORT CEnum Client_DisableSegmentData(CClient* client);
CDLL_EXPORT CEnum Client_DisableMarkerData(CClient* client);
CDLL_EXPORT CEnum Client_DisableUnlabeledMarkerData(CClient* client);
CDLL_EXPORT CEnum Client_DisableDeviceData(CClient* client);

CDLL_EXPORT CBool Client_IsSegmentDataEnabled(CClient* client);
CDLL_EXPORT CBool Client_IsMarkerDataEnabled(CClient* client);
CDLL_EXPORT CBool Client_IsUnlabeledMarkerDataEnabled(CClient* client);
CDLL_EXPORT CBool Client_IsDeviceDataEnabled(CClient* client);
    
CDLL_EXPORT CEnum Client_SetStreamMode(CClient* client, CEnum Mode );

CDLL_EXPORT CEnum Client_SetApexDeviceFeedback(CClient* client, CString i_rDeviceName, CBool i_bOn );

CDLL_EXPORT CEnum Client_SetAxisMapping(CClient* client, CEnum XAxis, CEnum YAxis, CEnum ZAxis );
CDLL_EXPORT void Client_GetAxisMapping(CClient* client, COutput_GetAxisMapping* outptr);

CDLL_EXPORT CEnum Client_GetFrame(CClient* client);
CDLL_EXPORT void Client_GetFrameNumber(CClient* client, COutput_GetFrameNumber* outptr);

CDLL_EXPORT void Client_GetTimecode(CClient* client, COutput_GetTimecode* outptr);

CDLL_EXPORT void Client_GetFrameRate(CClient* client, COutput_GetFrameRate* outptr);

CDLL_EXPORT void Client_GetLatencySampleCount(CClient* client, COutput_GetLatencySampleCount* outptr);
CDLL_EXPORT CEnum  Client_GetLatencySampleName(CClient* client, unsigned int LatencySampleIndex, 
                        int sizeOfBuffer, char* outstr );
CDLL_EXPORT void Client_GetLatencySampleValue(CClient* client, CString LatencySampleName, COutput_GetLatencySampleValue* outptr );
CDLL_EXPORT void Client_GetLatencyTotal(CClient* client, COutput_GetLatencyTotal* outptr);

CDLL_EXPORT void Client_GetSubjectCount(CClient* client, COutput_GetSubjectCount* outptr);
CDLL_EXPORT CEnum Client_GetSubjectName(CClient* client, unsigned int SubjectIndex, int sizeOfBuffer, char* outstr );

CDLL_EXPORT CEnum Client_GetSubjectRootSegmentName(CClient* client, CString SubjectName, int sizeOfBuffer, char* outstr );

CDLL_EXPORT void Client_GetSegmentCount( CClient* client,CString SubjectName,COutput_GetSegmentCount* outptr );
    
CDLL_EXPORT CEnum Client_GetSegmentName(CClient* client, CString SubjectName,
            unsigned int   SegmentIndex, int sizeOfBuffer, char* outstr );

CDLL_EXPORT void Client_GetSegmentChildCount(CClient* client, CString SubjectName,
                                                  CString SegmentName, COutput_GetSegmentChildCount* outptr );

CDLL_EXPORT CEnum Client_GetSegmentChildName(CClient* client, CString SubjectName,
                                                CString SegmentName,
                                                unsigned int   SegmentIndex, int sizeOfBuffer, char* outstr );

CDLL_EXPORT CEnum Client_GetSegmentParentName(CClient* client, CString SubjectName,
                              CString SegmentName, int sizeOfBuffer, char* outstr );

CDLL_EXPORT void Client_GetSegmentStaticTranslation(CClient* client, CString SubjectName,
                                                                CString SegmentName, COutput_GetSegmentStaticTranslation* outptr );

CDLL_EXPORT void Client_GetSegmentStaticRotationHelical(CClient* client, CString  SubjectName,
                                                        CString  SegmentName, COutput_GetSegmentStaticRotationHelical* outptr );

CDLL_EXPORT void Client_GetSegmentStaticRotationMatrix(CClient* client, CString  SubjectName,
                                                        CString  SegmentName, COutput_GetSegmentStaticRotationMatrix* outptr );

CDLL_EXPORT void Client_GetSegmentStaticRotationQuaternion(CClient* client, CString  SubjectName,
                                                            CString  SegmentName, COutput_GetSegmentStaticRotationQuaternion* outptr );

CDLL_EXPORT void Client_GetSegmentStaticRotationEulerXYZ(CClient* client, CString  SubjectName,
                                                                          CString  SegmentName, COutput_GetSegmentStaticRotationEulerXYZ* outptr );

CDLL_EXPORT void Client_GetSegmentGlobalTranslation(CClient* client, CString  SubjectName,
                                                    CString  SegmentName, COutput_GetSegmentGlobalTranslation* outptr );

CDLL_EXPORT void Client_GetSegmentGlobalRotationHelical(CClient* client, CString  SubjectName,
                                                        CString  SegmentName, COutput_GetSegmentGlobalRotationHelical* outptr );

CDLL_EXPORT void Client_GetSegmentGlobalRotationMatrix(CClient* client, CString  SubjectName,
                                                        CString  SegmentName, COutput_GetSegmentGlobalRotationMatrix* outptr );

CDLL_EXPORT void Client_GetSegmentGlobalRotationQuaternion(CClient* client, CString  SubjectName, CString  SegmentName,
                              COutput_GetSegmentGlobalRotationQuaternion* outptr);

CDLL_EXPORT void Client_GetSegmentGlobalRotationEulerXYZ(CClient* client, CString  SubjectName,
                                                          CString  SegmentName, COutput_GetSegmentGlobalRotationEulerXYZ* outptr );

CDLL_EXPORT void Client_GetSegmentLocalTranslation(CClient* client, CString  SubjectName,
                                                    CString  SegmentName, COutput_GetSegmentLocalTranslation* outptr );

CDLL_EXPORT void Client_GetSegmentLocalRotationHelical(CClient* client, CString  SubjectName,
                                                        CString  SegmentName, COutput_GetSegmentLocalRotationHelical* outptr );

CDLL_EXPORT void Client_GetSegmentLocalRotationMatrix(CClient* client, CString  SubjectName,
                                                      CString  SegmentName, COutput_GetSegmentLocalRotationMatrix* outptr );

CDLL_EXPORT void Client_GetSegmentLocalRotationQuaternion(CClient* client, CString  SubjectName,
                                                          CString  SegmentName, COutput_GetSegmentLocalRotationQuaternion* outptr );

CDLL_EXPORT void Client_GetSegmentLocalRotationEulerXYZ(CClient* client, CString  SubjectName,
                                                        CString  SegmentName, COutput_GetSegmentLocalRotationEulerXYZ* outptr );

CDLL_EXPORT void Client_GetMarkerCount(CClient* client, CString SubjectName, COutput_GetMarkerCount* outptr );

CDLL_EXPORT CEnum Client_GetMarkerName(CClient* client, CString  SubjectName,
                        unsigned int  MarkerIndex, int sizeOfBuffer, char* outstr );

CDLL_EXPORT CEnum Client_GetMarkerParentName(CClient* client, CString  SubjectName,
                            CString  MarkerName, int sizeOfBuffer, char* outstr );

CDLL_EXPORT void Client_GetMarkerGlobalTranslation(CClient* client, CString  SubjectName,
                                                    CString  MarkerName, COutput_GetMarkerGlobalTranslation* outptr );

CDLL_EXPORT void Client_GetUnlabeledMarkerCount(CClient* client, COutput_GetUnlabeledMarkerCount* outptr);
    
CDLL_EXPORT void Client_GetUnlabeledMarkerGlobalTranslation(CClient* client, unsigned int MarkerIndex, 
                      COutput_GetUnlabeledMarkerGlobalTranslation* outptr );

CDLL_EXPORT void Client_GetDeviceCount(CClient* client, COutput_GetDeviceCount* outptr);
CDLL_EXPORT CEnum  Client_GetDeviceName(CClient* client, unsigned int DeviceIndex, 
                  int sizeOfBuffer, char* outstr, CEnum* DeviceType );

CDLL_EXPORT void Client_GetDeviceOutputCount(CClient* client, CString DeviceName, COutput_GetDeviceOutputCount* outptr );

CDLL_EXPORT CEnum Client_GetDeviceOutputName(CClient* client, CString   DeviceName,
                      unsigned int   DeviceOutputIndex, int sizeOfBuffer, char* outstr, CEnum* DeviceOutputUnit );
    
CDLL_EXPORT void Client_GetDeviceOutputValue(CClient* client, CString  DeviceName,
                                              CString  DeviceOutputName, COutput_GetDeviceOutputValue* outptr );

CDLL_EXPORT void Client_GetDeviceOutputSubsamples(CClient* client, CString  DeviceName,
                                                  CString  DeviceOutputName, COutput_GetDeviceOutputSubsamples* outptr );

CDLL_EXPORT void Client_GetDeviceOutputValueForSubsample(CClient* client, CString  DeviceName,
                                                  CString  DeviceOutputName,
                                                  unsigned int Subsample,
                        COutput_GetDeviceOutputValue* outptr);

CDLL_EXPORT void Client_GetForcePlateCount(CClient* client, COutput_GetForcePlateCount* outptr);

CDLL_EXPORT void Client_GetGlobalForceVector(CClient* client,  unsigned int ForcePlateIndex, COutput_GetGlobalForceVector* outptr );
CDLL_EXPORT void Client_GetGlobalMomentVector(CClient* client,  unsigned int ForcePlateIndex, COutput_GetGlobalMomentVector* outptr );
CDLL_EXPORT void Client_GetGlobalCentreOfPressure(CClient* client,  unsigned int ForcePlateIndex, COutput_GetGlobalCentreOfPressure* outptr );

CDLL_EXPORT void Client_GetForcePlateSubsamples(CClient* client,  unsigned int ForcePlateIndex, COutput_GetForcePlateSubsamples* outptr );

CDLL_EXPORT void Client_GetGlobalForceVectorForSubsample(CClient* client, unsigned int ForcePlateIndex, unsigned int Subsample,
                            COutput_GetGlobalForceVector* outptr);
CDLL_EXPORT void Client_GetGlobalMomentVectorForSubsample(CClient* client, unsigned int ForcePlateIndex, unsigned int Subsample,
                            COutput_GetGlobalMomentVector* outptr);
CDLL_EXPORT void Client_GetGlobalCentreOfPressureForSubsample(CClient* client,  unsigned int ForcePlateIndex, unsigned int Subsample,
                              COutput_GetGlobalCentreOfPressure* outptr);

CDLL_EXPORT void Client_GetEyeTrackerCount(CClient* client, COutput_GetEyeTrackerCount* outptr);

CDLL_EXPORT void Client_GetEyeTrackerGlobalPosition(CClient* client,  unsigned int EyeTrackerIndex,
                        COutput_GetEyeTrackerGlobalPosition* outptr);
CDLL_EXPORT void Client_GetEyeTrackerGlobalGazeVector(CClient* client,  unsigned int EyeTrackerIndex,
                          COutput_GetEyeTrackerGlobalGazeVector* outptr);

CDLL_EXPORT CEnum Client_EnableMarkerRayData( CClient* client );
CDLL_EXPORT CEnum Client_EnableCentroidData( CClient* client );
CDLL_EXPORT CEnum Client_EnableGreyscaleData( CClient* client );
CDLL_EXPORT CEnum Client_EnableDebugData( CClient* client );
CDLL_EXPORT CEnum Client_DisableMarkerRayData( CClient* client );
CDLL_EXPORT CEnum Client_DisableCentroidData( CClient* client );
CDLL_EXPORT CEnum Client_DisableGreyscaleData( CClient* client );
CDLL_EXPORT CEnum Client_DisableDebugData( CClient* client );
CDLL_EXPORT CBool Client_IsMarkerRayDataEnabled( CClient* client );
CDLL_EXPORT CBool Client_IsCentroidDataEnabled( CClient* client );
CDLL_EXPORT CBool Client_IsGreyscaleDataEnabled( CClient* client );
CDLL_EXPORT CBool Client_IsDebugDataEnabled( CClient* client );

CDLL_EXPORT void Client_GetServerOrientation( CClient* client, COutput_GetServerOrientation* outptr );
CDLL_EXPORT void Client_GetHardwareFrameNumber( CClient* client, COutput_GetHardwareFrameNumber* outptr );
CDLL_EXPORT void Client_GetFrameRateCount( CClient* client, COutput_GetFrameRateCount* outptr );
CDLL_EXPORT CEnum Client_GetFrameRateName( CClient* client, unsigned int FrameRateIndex, int sizeOfBuffer, char* outstr );
CDLL_EXPORT void Client_GetFrameRateValue( CClient* client, CString FrameRateName, COutput_GetFrameRateValue* outptr );
CDLL_EXPORT void Client_GetObjectQuality( CClient* client, CString ObjectName, COutput_GetObjectQuality* outptr );
CDLL_EXPORT void Client_GetMarkerRayContributionCount( CClient* client, CString SubjectName, CString MarkerName, COutput_GetMarkerRayContributionCount* outptr );
CDLL_EXPORT void Client_GetMarkerRayContribution( CClient* client, CString SubjectName, CString MarkerName, unsigned int MarkerRayContributionIndex, COutput_GetMarkerRayContribution* outptr );
CDLL_EXPORT void Client_GetLabeledMarkerCount( CClient* client, COutput_GetLabeledMarkerCount* outptr );
CDLL_EXPORT void Client_GetLabeledMarkerGlobalTranslation( CClient* client, unsigned int MarkerIndex, COutput_GetLabeledMarkerGlobalTranslation* outptr );
CDLL_EXPORT void Client_GetCameraCount( CClient* client, COutput_GetCameraCount* outptr );
CDLL_EXPORT CEnum Client_GetCameraName( CClient* client, unsigned int i_CameraIndex, int sizeOfBuffer, char* outstr );
CDLL_EXPORT void Client_GetCameraId( CClient* client, CString i_rCameraName, COutput_GetCameraId* outptr );
CDLL_EXPORT void Client_GetCameraUserId( CClient* client, CString i_rCameraName, COutput_GetCameraUserId* outptr );
CDLL_EXPORT CEnum Client_GetCameraType( CClient* client, CString i_rCameraName, int sizeOfBuffer, char* outstr );
CDLL_EXPORT CEnum Client_GetCameraDisplayName( CClient* client, CString i_rCameraName, int sizeOfBuffer, char* outstr );
CDLL_EXPORT void Client_GetCameraResolution( CClient* client, CString i_rCameraName, COutput_GetCameraResolution* outptr );
CDLL_EXPORT void Client_GetIsVideoCamera( CClient* client, CString i_rCameraName, COutput_GetIsVideoCamera* outptr );
CDLL_EXPORT void Client_GetCentroidCount( CClient* client, CString i_rCameraName, COutput_GetCentroidCount* outptr );
CDLL_EXPORT void Client_GetCentroidPosition( CClient* client, CString i_rCameraName, unsigned int i_CentroidIndex, COutput_GetCentroidPosition* outptr );
CDLL_EXPORT void Client_GetCentroidWeight( CClient* client, CString i_rCameraName, unsigned int i_CentroidIndex, COutput_GetCentroidWeight* outptr );


#ifdef __cplusplus
}
#endif
