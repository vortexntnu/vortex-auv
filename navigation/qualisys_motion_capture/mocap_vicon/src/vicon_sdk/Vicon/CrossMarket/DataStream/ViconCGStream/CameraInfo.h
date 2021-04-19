
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

#include "Item.h"
#include <string>

namespace ViconCGStream
{
//-------------------------------------------------------------------------------------------------

/// Contains the camera identifier, frame period, name, 
class VCameraInfo : public VItem
{
public:

  /// Camera identifier
  ViconCGStreamType::UInt32 m_CameraID;

  /// Camera frame period. Defined as the numbers of ticks per frame on a 135Mhz clock.
  ViconCGStreamType::UInt64 m_FramePeriod;

  /// Camera type
  std::string m_Type;

  /// Camera display type
  std::string m_DisplayType;

  /// Camera name
  std::string m_Name;

  /// Resolution X
  ViconCGStreamType::UInt32 m_ResolutionX;

  /// Resolution Y
  ViconCGStreamType::UInt32 m_ResolutionY;

  /// Aspect ratio
  ViconCGStreamType::Double m_AspectRatio;

  /// Minimum circularity
  ViconCGStreamType::Double m_CircularityThreshold;

  /// Is the camera tracking centroids
  bool m_bCentroidTracking;

  /// Is this a video camera
  bool m_bIsVideoCamera;

  /// User friendly camera number.
  ViconCGStreamType::UInt32 m_UserID;

  /// Marker velocity
  ViconCGStreamType::Double m_MarkerVelocity;

  /// Equality function
  bool IsEqual( const VCameraInfo & i_rOther ) const
  {
    return m_CameraID == i_rOther.m_CameraID &&
           m_FramePeriod == i_rOther.m_FramePeriod &&
           m_Type == i_rOther.m_Type &&
           m_DisplayType == i_rOther.m_DisplayType &&
           m_Name == i_rOther.m_Name &&
           m_ResolutionX == i_rOther.m_ResolutionX &&
           m_ResolutionY == i_rOther.m_ResolutionY &&
           m_AspectRatio == i_rOther.m_AspectRatio &&
           m_CircularityThreshold == i_rOther.m_CircularityThreshold &&
           m_bCentroidTracking == i_rOther.m_bCentroidTracking &&
           m_bIsVideoCamera == i_rOther.m_bIsVideoCamera &&
           m_UserID == i_rOther.m_UserID &&
           m_MarkerVelocity == i_rOther.m_MarkerVelocity;
  }

  /// Equality operator
  bool operator == ( const VCameraInfo & i_rOther ) const
  {
    return IsEqual( i_rOther );
  }

  /// Object type enum.
  virtual ViconCGStreamType::Enum TypeID() const
  {
    return ViconCGStreamEnum::CameraInfo;
  }

  /// Filter ID
  virtual ViconCGStreamType::UInt32 FilterID() const
  {
    return m_CameraID;
  }

  /// Read function.  
  virtual bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_CameraID ) &&
           i_rBuffer.Read( m_FramePeriod ) &&
           i_rBuffer.Read( m_Type ) &&
           i_rBuffer.Read( m_DisplayType ) &&
           i_rBuffer.Read( m_Name ) &&
           i_rBuffer.Read( m_ResolutionX ) &&
           i_rBuffer.Read( m_ResolutionY ) &&
           i_rBuffer.Read( m_AspectRatio ) &&
           i_rBuffer.Read( m_CircularityThreshold ) &&
           i_rBuffer.Read( m_bCentroidTracking ) &&
           i_rBuffer.Read( m_bIsVideoCamera ) &&
           i_rBuffer.Read( m_UserID ) &&
           i_rBuffer.Read( m_MarkerVelocity );
  }

  /// Write function.  
  virtual void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_CameraID );
    i_rBuffer.Write( m_FramePeriod );
    i_rBuffer.Write( m_Type );
    i_rBuffer.Write( m_DisplayType );
    i_rBuffer.Write( m_Name );
    i_rBuffer.Write( m_ResolutionX );
    i_rBuffer.Write( m_ResolutionY );
    i_rBuffer.Write( m_AspectRatio );
    i_rBuffer.Write( m_CircularityThreshold );
    i_rBuffer.Write( m_bCentroidTracking );
    i_rBuffer.Write( m_bIsVideoCamera );
    i_rBuffer.Write( m_UserID );
    i_rBuffer.Write( m_MarkerVelocity );
  }

};

//-------------------------------------------------------------------------------------------------
};

