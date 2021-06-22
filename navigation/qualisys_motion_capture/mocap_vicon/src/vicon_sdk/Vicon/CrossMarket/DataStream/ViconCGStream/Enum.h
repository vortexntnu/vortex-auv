
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

/// \file
/// Contains the declaration of the ViconCGStreamEnum namespace.

#include <StreamCommon/Type.h>

/** 
 * This namespace holds the enumerated values used to identify the various types of object found
 * in the stream. Each packet consists of a header, followed by one or more object blocks. 
 *
 * The packet header comprises of the \c PacketID tag (with the value of ViconCGStreamEnum::Objects), 
 * followed by the \c PacketLength value. Both the \c PacketID and \c PacketLength fields are 
 * ViconCGStreamType::UInt32 values.
 *
 * Each object (stored after the \c PacketLength field) has a block header, followed by the data
 * specific to that object. The object header is similar to the packet header, in that it consists
 * of an \c ObjectID, and the \c ObjectLength. The \c ObjectID value uniquely identifies the kind of 
 * object present.
 */
namespace ViconCGStreamEnum
{
//-------------------------------------------------------------------------------------------------

/// Explicitely invalid value (can be used for initialisation)
const ViconCGStreamType::Enum Invalid = -1; 

/// This value appears at the start of an object packet.
const ViconCGStreamType::Enum Objects = 0x1a2b0001; 

/// See ViconCGStream::VObjectEnums.
const ViconCGStreamType::Enum ObjectEnums = 100001;

/// See ViconCGStream::VRequestFrame.
const ViconCGStreamType::Enum RequestFrame = 100002;

/// See ViconCGStream::VKeepAlive.
const ViconCGStreamType::Enum KeepAlive = 100003;

/// See ViconCGStream::VContents.
const ViconCGStreamType::Enum Contents = 100004;

/// See ViconCGStream::VStartMulticastSender.
const ViconCGStreamType::Enum StartMulticastSender = 100005;

/// See ViconCGStream::VStopMulticastSender.
const ViconCGStreamType::Enum StopMulticastSender = 100006;

/// See ViconCGStream::VFilter.
const ViconCGStreamType::Enum Filter = 100007;

/// See ViconCGStream::VStreamInfo.
const ViconCGStreamType::Enum StreamInfo = 1;

/// See ViconCGStream::VSubjectInfo.
const ViconCGStreamType::Enum SubjectInfo = 2; 

/// See ViconCGStream::VSubjectHealth.
const ViconCGStreamType::Enum SubjectHealth = 29; 

/// See ViconCGStream::VDeviceInfo.
const ViconCGStreamType::Enum DeviceInfo = 3; 

/// See ViconCGStream::VChannelInfo.
const ViconCGStreamType::Enum ChannelInfo = 4; 

/// See ViconCGStream::VFrameInfo.
const ViconCGStreamType::Enum FrameInfo = 5; 

/// See ViconCGStream::VLabeledRecons.
const ViconCGStreamType::Enum LabeledRecons = 6; 

/// See ViconCGStream::VUnlabeledRecons.
const ViconCGStreamType::Enum UnlabeledRecons = 7; 

/// See ViconCGStream::VLocalSegments.
const ViconCGStreamType::Enum LocalSegments = 8; 

/// See ViconCGStream::VLocalSegments.
const ViconCGStreamType::Enum GlobalSegments = 9; 

/// See ViconCGStream::VForceFrame.
const ViconCGStreamType::Enum ForceFrame = 10; 

/// See ViconCGStream::VMomentFrame.
const ViconCGStreamType::Enum MomentFrame = 11; 

/// See ViconCGStream::VCentreOfPressureFrame.
const ViconCGStreamType::Enum CentreOfPressureFrame = 12; 

/// See ViconCGStream::VVoltageFrame.
const ViconCGStreamType::Enum VoltageFrame = 13; 

/// See ViconCGStream::VCameraInfo.
const ViconCGStreamType::Enum CameraInfo = 14; 

/// See ViconCGStream::VCameraInfo2.
const ViconCGStreamType::Enum CameraTimingInfo = 65001; 

/// See ViconCGStream::VCentroidTracks.
const ViconCGStreamType::Enum CentroidTracks = 15; 

/// See ViconCGStream::VCentroids.
const ViconCGStreamType::Enum Centroids = 16; 

/// See ViconCGStream::VGreyScaleBlobs.
const ViconCGStreamType::Enum GreyscaleBlobs = 17;

/// See ViconCGStream::VEdgePairs.
const ViconCGStreamType::Enum EdgePairs = 18;

/// See ViconCGStream:VCameraCalibrationInfo.
const ViconCGStreamType::Enum CameraCalibrationInfo = 19;

/// See ViconCGStream::VForcePlateInfo.
const ViconCGStreamType::Enum ForcePlateInfo = 20;

/// See ViconCGStream::VVideoFrame.
const ViconCGStreamType::Enum VideoFrame = 22; 

/// See ViconCGStream::VTimecode.
const ViconCGStreamType::Enum Timecode = 23; 

/// See ViconCGStream::VUnlabeledReconsRayAssignments
const ViconCGStreamType::Enum UnlabeledReconRayAssignments = 24;

/// See ViconCGStream::VLabeledReconsRayAssignments
const ViconCGStreamType::Enum LabeledReconRayAssignments = 25;

/// See ViconCGStream::VLatencyInfo
const ViconCGStreamType::Enum LatencyInfo = 26;

/// See ViconCGStream::VCameraCalibrationHealth
const ViconCGStreamType::Enum CameraCalibrationHealth = 27; 

/// See ViconCGStream::VSubjectTopology
const ViconCGStreamType::Enum SubjectTopology = 28;

/// See ViconCGStream:VCameraWand2d.
const ViconCGStreamType::Enum CameraWand2d = 30;

/// See ViconCGStream:VCameraWand3d.
const ViconCGStreamType::Enum CameraWand3d = 31;

/// See ViconCGStream::VEyeTrackerFrame.
const ViconCGStreamType::Enum EyeTrackerFrame = 32;

/// See ViconCGStream::VEyeTrackerInfo.
const ViconCGStreamType::Enum EyeTrackerInfo = 33;

/// See ViconCGStream::VChannelInfoExtra.
const ViconCGStreamType::Enum ChannelInfoExtra = 34; 

/// See ViconCGStream::VApexHaptics.
const ViconCGStreamType::Enum ApexHaptics = 35;

/// See ViconCGStream::VHardwareFrameInfo.
const ViconCGStreamType::Enum HardwareFrameInfo = 36; 

/// See ViconCGStream::VApplicationInfo.
const ViconCGStreamType::Enum ApplicationInfo = 37;

/// See ViconCGStream::VDeviceInfoExtra.
const ViconCGStreamType::Enum DeviceInfoExtra = 38; 

/// See ViconCGStream::VDeviceInfoExtra.
const ViconCGStreamType::Enum FrameRateInfo = 39; 

/// See ViconCGStream::VObjectQuality.
const ViconCGStreamType::Enum ObjectQuality = 40;

/// See ViconCGStream::VObjectQuality.
const ViconCGStreamType::Enum CentroidWeights = 41;
//-------------------------------------------------------------------------------------------------
};
