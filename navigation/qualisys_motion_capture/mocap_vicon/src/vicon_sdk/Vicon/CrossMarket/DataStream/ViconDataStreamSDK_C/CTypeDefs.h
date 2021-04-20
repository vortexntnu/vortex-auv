
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

typedef double CReal;
typedef void CClient;
typedef void CRetimingClient;
typedef const char* CString;
typedef int CBool;
typedef int CEnum;

static const int CSuccess = 2;

#pragma pack(push, 1)

/** @private */
typedef struct COutput_GetVersion
{
  unsigned int Major;
  unsigned int Minor;
  unsigned int Point;
} COutput_GetVersion;

/** @private */
typedef struct COutput_Connect
{
  CEnum Result;
} COutput_Connect;

/** @private */
typedef struct COutput_ConnectToMulticast
{
  CEnum Result;
} COutput_ConnectToMulticast;

/** @private */
typedef struct COutput_Disconnect
{
  CEnum Result;
} COutput_Disconnect;

/** @private */
typedef struct COutput_IsConnected
{
  CBool Connected;
} COutput_IsConnected;

/** @private */
typedef struct COutput_StartTransmittingMulticast
{
  CEnum Result;
} COutput_StartTransmittingMulticast;

/** @private */
typedef struct COutput_StopTransmittingMulticast
{
  CEnum Result;
} COutput_StopTransmittingMulticast;

/** @private */
typedef struct COutput_EnableSegmentData
{
  CEnum Result;
} COutput_EnableSegmentData;

/** @private */
typedef struct COutput_EnableMarkerData
{
  CEnum Result;
} COutput_EnableMarkerData;

/** @private */
typedef struct COutput_EnableUnlabeledMarkerData
{
  CEnum Result;
} COutput_EnableUnlabeledMarkerData;

/** @private */
typedef struct COutput_EnableDeviceData
{
  CEnum Result;
} COutput_EnableDeviceData;

/** @private */
typedef struct COutput_DisableSegmentData
{
  CEnum Result;
} COutput_DisableSegmentData;

/** @private */
typedef struct COutput_DisableMarkerData
{
  CEnum Result;
} COutput_DisableMarkerData;

/** @private */
typedef struct COutput_DisableUnlabeledMarkerData
{
  CEnum Result;
} COutput_DisableUnlabeledMarkerData;

/** @private */
typedef struct COutput_DisableDeviceData
{
  CEnum Result;
} COutput_DisableDeviceData;

/** @private */
typedef struct COutput_IsSegmentDataEnabled
{
  CBool Enabled;
} COutput_IsSegmentDataEnabled;

/** @private */
typedef struct COutput_IsMarkerDataEnabled
{
  CBool Enabled;
} COutput_IsMarkerDataEnabled;

/** @private */
typedef struct COutput_IsUnlabeledMarkerDataEnabled
{
  CBool Enabled;
} COutput_IsUnlabeledMarkerDataEnabled;

/** @private */
typedef struct COutput_IsDeviceDataEnabled
{
  CBool Enabled;
} COutput_IsDeviceDataEnabled;

/** @private */
typedef struct COutput_SetStreamMode
{
  CEnum Result;
} COutput_SetStreamMode;

/** @private */
typedef struct COutput_SetApexDeviceFeedback
{
  CEnum Result;
} COutput_SetApexDeviceFeedback;

/** @private */
typedef struct COutput_SetAxisMapping
{
  CEnum Result;
} COutput_SetAxisMapping;

/** @private */
typedef struct COutput_GetAxisMapping
{
  CEnum XAxis;
  CEnum YAxis;
  CEnum ZAxis;
} COutput_GetAxisMapping;

/** @private */
typedef struct COutput_GetFrame
{
  CEnum Result;
} COutput_GetFrame;

/** @private */
typedef struct COutput_GetFrameNumber
{
  CEnum Result;
  unsigned int FrameNumber;
} COutput_GetFrameNumber;

/** @private */
typedef struct COutput_GetTimecode
{
  CEnum          Result;
  unsigned int           Hours;
  unsigned int           Minutes;
  unsigned int           Seconds;
  unsigned int           Frames;
  unsigned int           SubFrame;
  CBool                  FieldFlag;
  CEnum          Standard;
  unsigned int           SubFramesPerFrame;
  unsigned int           UserBits;
} COutput_GetTimecode;

/** @private */
typedef struct COutput_GetFrameRate
{
  CEnum Result;
  double FrameRateHz;
} COutput_GetFrameRate;

/** @private */
typedef struct COutput_GetLatencySampleCount
{
  CEnum Result;
  unsigned int Count;
} COutput_GetLatencySampleCount;

/** @private */
typedef struct COutput_GetLatencySampleValue
{
  CEnum Result;
  double       Value;
} COutput_GetLatencySampleValue;

/** @private */
typedef struct COutput_GetLatencyTotal
{
  CEnum Result;
  double       Total;
} COutput_GetLatencyTotal;

/** @private */
typedef struct COutput_GetSubjectCount
{
  CEnum Result;
  unsigned int SubjectCount;
} COutput_GetSubjectCount;

/** @private */
typedef struct COutput_GetSegmentChildCount
{
  CEnum Result;
  unsigned int SegmentCount;
} COutput_GetSegmentChildCount;

/** @private */
typedef struct COutput_GetSegmentCount
{
  CEnum Result;
  unsigned int SegmentCount;
} COutput_GetSegmentCount;

/** @private */
typedef struct COutput_GetSegmentStaticTranslation
{
  CEnum Result;
  double       Translation[3];
} COutput_GetSegmentStaticTranslation;

/** @private */
typedef struct COutput_GetSegmentStaticRotationHelical
{
  CEnum Result;
  double       Rotation[3];
} COutput_GetSegmentStaticRotationHelical;

/** @private */
typedef struct COutput_GetSegmentStaticRotationMatrix
{
  CEnum Result;
  double       Rotation[9];
} COutput_GetSegmentStaticRotationMatrix;

/** @private */
typedef struct COutput_GetSegmentStaticRotationQuaternion
{
  CEnum Result;
  double       Rotation[4];
} COutput_GetSegmentStaticRotationQuaternion;

/** @private */
typedef struct COutput_GetSegmentStaticRotationEulerXYZ
{
  CEnum Result;
  double       Rotation[3];
} COutput_GetSegmentStaticRotationEulerXYZ;

/** @private */
typedef struct COutput_GetSegmentGlobalTranslation
{
  CEnum Result;
  double       Translation[3];
  CBool         Occluded;
} COutput_GetSegmentGlobalTranslation;

/** @private */
typedef struct COutput_GetSegmentGlobalRotationHelical
{
  CEnum Result;
  double       Rotation[3];
  CBool         Occluded;
} COutput_GetSegmentGlobalRotationHelical;

/** @private */
typedef struct COutput_GetSegmentGlobalRotationMatrix
{
  CEnum Result;
  double       Rotation[9];
  CBool         Occluded;
} COutput_GetSegmentGlobalRotationMatrix;

/** @private */
typedef struct COutput_GetSegmentGlobalRotationQuaternion
{
  CEnum Result;
  double       Rotation[4];
  CBool         Occluded;
} COutput_GetSegmentGlobalRotationQuaternion;

/** @private */
typedef struct COutput_GetSegmentGlobalRotationEulerXYZ
{
  CEnum Result;
  double       Rotation[3];
  CBool         Occluded;
} COutput_GetSegmentGlobalRotationEulerXYZ;

/** @private */
typedef struct COutput_GetSegmentLocalTranslation
{
  CEnum Result;
  double       Translation[3];
  CBool         Occluded;
} COutput_GetSegmentLocalTranslation;

/** @private */
typedef struct COutput_GetSegmentLocalRotationHelical
{
  CEnum Result;
  double       Rotation[3];
  CBool         Occluded;
} COutput_GetSegmentLocalRotationHelical;

/** @private */
typedef struct COutput_GetSegmentLocalRotationMatrix
{
  CEnum Result;
  double       Rotation[9];
  CBool         Occluded;
} COutput_GetSegmentLocalRotationMatrix;

/** @private */
typedef struct COutput_GetSegmentLocalRotationQuaternion
{
  CEnum Result;
  double       Rotation[4];
  CBool         Occluded;
} COutput_GetSegmentLocalRotationQuaternion;

/** @private */
typedef struct COutput_GetSegmentLocalRotationEulerXYZ
{
  CEnum Result;
  double       Rotation[3];
  CBool         Occluded;
} COutput_GetSegmentLocalRotationEulerXYZ;

/** @private */
typedef struct COutput_GetMarkerCount
{
  CEnum Result;
  unsigned int MarkerCount;
} COutput_GetMarkerCount;

/** @private */
typedef struct COutput_GetMarkerGlobalTranslation
{
  CEnum Result;
  double       Translation[3];
  CBool         Occluded;
} COutput_GetMarkerGlobalTranslation;

/** @private */
typedef struct COutput_GetUnlabeledMarkerCount
{
  CEnum Result;
  unsigned int MarkerCount;
} COutput_GetUnlabeledMarkerCount;

/** @private */
typedef struct COutput_GetUnlabeledMarkerGlobalTranslation
{
  CEnum Result;
  double       Translation[3];
} COutput_GetUnlabeledMarkerGlobalTranslation;

/** @private */
typedef struct COutput_GetDeviceCount
{
  CEnum Result;
  unsigned int DeviceCount;
} COutput_GetDeviceCount;

/** @private */
typedef struct COutput_GetDeviceOutputCount
{
  CEnum Result;
  unsigned int DeviceOutputCount;
} COutput_GetDeviceOutputCount;

/** @private */
typedef struct COutput_GetDeviceOutputValue
{
  CEnum Result;
  double       Value;
  CBool         Occluded;
} COutput_GetDeviceOutputValue;

/** @private */
typedef struct COutput_GetDeviceOutputSubsamples
{
  CEnum Result;
  unsigned int DeviceOutputSubsamples;
  CBool         Occluded;
} COutput_GetDeviceOutputSubsamples;

/** @private */
typedef struct COutput_GetForcePlateCount
{
  CEnum Result;
  unsigned int ForcePlateCount;
} COutput_GetForcePlateCount;

/** @private */
typedef struct COutput_GetGlobalForceVector
{
  CEnum Result;
  double       ForceVector[3];
} COutput_GetGlobalForceVector;

/** @private */
typedef struct COutput_GetGlobalMomentVector
{
  CEnum Result;
  double       MomentVector[3];
} COutput_GetGlobalMomentVector;

/** @private */
typedef struct COutput_GetGlobalCentreOfPressure
{
  CEnum Result;
  double       CentreOfPressure[3];
} COutput_GetGlobalCentreOfPressure;

/** @private */
typedef struct COutput_GetForcePlateSubsamples
{
  CEnum Result;
  unsigned int ForcePlateSubsamples;
} COutput_GetForcePlateSubsamples;

/** @private */
typedef struct COutput_GetEyeTrackerCount
{
  CEnum Result;
  unsigned int EyeTrackerCount;
} COutput_GetEyeTrackerCount;

/** @private */
typedef struct COutput_GetEyeTrackerGlobalPosition
{
  CEnum Result;
  double       Position[3];
  CBool         Occluded;
} COutput_GetEyeTrackerGlobalPosition;

/** @private */
typedef struct COutput_GetEyeTrackerGlobalGazeVector
{
  CEnum Result;
  double       GazeVector[3];
  CBool         Occluded;
} COutput_GetEyeTrackerGlobalGazeVector;

/** @private */
typedef struct COutput_GetCentroidCount
{
  CEnum Result;
  unsigned int CentroidCount;
} COutput_GetCentroidCount;

/** @private */
typedef struct COutput_GetFrameRateValue
{
  CEnum Result;
  double       Value;
} COutput_GetFrameRateValue;

/** @private */
typedef struct COutput_GetLabeledMarkerCount
{
  CEnum Result;
  unsigned int MarkerCount;
} COutput_GetLabeledMarkerCount;

/** @private */
typedef struct COutput_GetIsVideoCamera
{
  CEnum Result;
  CBool IsVideoCamera;
} COutput_GetIsVideoCamera;

/** @private */
typedef struct COutput_GetCameraId
{
  CEnum Result;
  unsigned int CameraId;
} COutput_GetCameraId;

/** @private */
typedef struct COutput_GetHardwareFrameNumber
{
  CEnum Result;
  unsigned int HardwareFrameNumber;
} COutput_GetHardwareFrameNumber;

/** @private */
typedef struct COutput_GetCameraUserId
{
  CEnum Result;
  unsigned int CameraUserId;
} COutput_GetCameraUserId;

/** @private */
typedef struct COutput_GetCameraCount
{
  CEnum Result;
  unsigned int CameraCount;
} COutput_GetCameraCount;

/** @private */
typedef struct COutput_GetCentroidPosition
{
  CEnum Result;
  double CentroidPosition[2];
  double Radius;
} COutput_GetCentroidPosition;

/** @private */
typedef struct COutput_GetMarkerRayContributionCount
{
  CEnum Result;
  unsigned int RayContributionsCount;
} COutput_GetMarkerRayContributionCount;

/** @private */
typedef struct COutput_GetObjectQuality
{
  CEnum Result;
  double Quality;
} COutput_GetObjectQuality;

/** @private */
typedef struct COutput_GetCameraResolution
{
  CEnum Result;
  unsigned int ResolutionX;
  unsigned int ResolutionY;
} COutput_GetCameraResolution;

/** @private */
typedef struct COutput_GetGreyscaleBlob
{
  CEnum Result;
  unsigned int * BlobLinePositionsX;
  unsigned int BlobLinePositionsXSize;
  unsigned int * BlobLinePositionsY;
  unsigned int BlobLinePositionsYSize;
  unsigned char * BlobLinePixelValues;
} COutput_GetGreyscaleBlob;

/** @private */
typedef struct COutput_GetFrameRateCount
{
  CEnum Result;
  unsigned int Count;
} COutput_GetFrameRateCount;

/** @private */
typedef struct COutput_GetMarkerRayContribution
{
  CEnum Result;
  unsigned int CameraID;
  unsigned int CentroidIndex;
} COutput_GetMarkerRayContribution;

/** @private */
typedef struct COutput_GetServerOrientation
{
  CEnum Result;
  CEnum Orientation;
} COutput_GetServerOrientation;

/** @private */
typedef struct COutput_GetLabeledMarkerGlobalTranslation
{
  CEnum Result;
  double       Translation[3];
} COutput_GetLabeledMarkerGlobalTranslation;

/** @private */
typedef struct COutput_GetCentroidWeight
{
  CEnum Result;
  double Weight;
} COutput_GetCentroidWeight;

/** @private */
typedef struct COutput_GetGreyscaleBlobCount
{
  CEnum Result;
  unsigned int BlobCount;
} COutput_GetGreyscaleBlobCount;

/** @private */
typedef enum
{
  Up,
  Down,
  Left,
  Right,
  Forward,
  Backward
} CDirection;

/** @private */
typedef enum
{
  UnknownServerOrientation,
  YUp,
  ZUp,
} CServerOrientation;

/** @private */
typedef enum
{
  ClientPull,
  ClientPullPreFetch,
  ServerPush
} CStreamMode;

/** @private */
typedef enum
{
  None,
  PAL,
  NTSC,
  NTSCDrop,
  Film,
  NTSCFilm,
  ATSC
} CTimecodeStandard;

/** @private */
typedef enum
{
  UnknownDeviceType,
  ForcePlate,
  EyeTracker
} CDeviceType;

/** @private */
typedef enum
{
  UnknownUnit,
  Volt,
  Newton,
  NewtonMeter,
  Meter,
  Kilogram,
  Second,
  Ampere,
  Kelvin,
  Mole,
  Candela,
  Radian,
  Steradian,
  MeterSquared,
  MeterCubed,
  MeterPerSecond,
  MeterPerSecondSquared,
  RadianPerSecond,
  RadianPerSecondSquared,
  Hertz,
  Joule,
  Watt,
  Pascal,
  Lumen,
  Lux,
  Coulomb,
  Ohm,
  Farad,
  Weber,
  Tesla,
  Henry,
  Siemens,
  Becquerel,
  Gray,
  Sievert,
  Katal
} CUnit;

/** 
 * The Result code indicates the success or failure of a function. 
 */
typedef enum
{
  UnknownResult, /**< The result is unknown. Treat it as a failure. */
  NotImplemented, /**< The function called has not been implemented in this version of the SDK.*/
  Success, /**< The function call succeeded.*/
  InvalidHostName, /**< The "HostName" parameter passed to Connect() is invalid.*/
  InvalidMulticastIP, /**< The "MulticastIP" parameter was not in the range "224.0.0.0" - "239.255.255.255"*/
  ClientAlreadyConnected, /**< Connect() was called whilst already connected to a DataStream.*/
  ClientConnectionFailed, /**< Connect() could not establish a connection to the DataStream server.*/
  ServerAlreadyTransmittingMulticast, /**< StartTransmittingMulticast() was called when the current DataStream server was already transmitting multicast on behalf of this client.*/
  ServerNotTransmittingMulticast, /**< StopTransmittingMulticast() was called when the current DataStream server was not transmitting multicast on behalf of this client.*/
  NotConnected, /**< You have called a function which requires a connection to the DataStream server, but do not have a connection.*/
  NoFrame, /**< You have called a function which requires a frame to be fetched from the DataStream server, but do not have a frame.*/
  InvalidIndex, /**< An index you have passed to a function is out of range.*/
  InvalidCameraName, /**< The Camera Name you passed to a function is invalid in this frame.*/
  InvalidSubjectName, /**< The Subject Name you passed to a function is invalid in this frame.*/
  InvalidSegmentName, /**< The Segment Name you passed to a function is invalid in this frame.*/
  InvalidMarkerName, /**< The Marker Name you passed to a function is invalid in this frame.*/
  InvalidDeviceName, /**< The Device Name you passed to a function is invalid in this frame.*/
  InvalidDeviceOutputName, /**< The Device Output Name you passed to a function is invalid in this frame.*/
  InvalidLatencySampleName, /**< The Latency Sample Name you passed to a function is invalid in this frame.*/
  CoLinearAxes, /**< The directions passed to SetAxisMapping() contain input which would cause two or more axes to lie along the same line, e.g. "Up" and "Down" are on the same line.*/
  LeftHandedAxes, /**< The directions passed to SetAxisMapping() would result in a left-handed coordinate system. This is not supported in the SDK.*/
  HapticAlreadySet, /**< Haptic feedback is already set.*/
  EarlyDataRequested, /**< Re-timed data requested is from before the first time sample we still have. */
  LateDataRequested /**< Re-timed data requested is too far into the future to be predicted. */
} CResult;

#pragma pack(pop)
