
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

class VCameraCalibrationInfo : public VItem
{
public:

  /// Camera identifier (device id)
  ViconCGStreamType::UInt32 m_CameraID;

  /// Pose Translation
  ViconCGStreamType::Double m_PoseTranslation[ 3 ];

  /// Pose Rotation
  ViconCGStreamType::Double m_PoseRotation[ 9 ];

  /// Principal Point
  ViconCGStreamType::Double m_PrincipalPoint[ 2 ];

  /// Radial Distortion
  ViconCGStreamType::Double m_RadialDistortion[ 2 ];

  /// Focal Length
  ViconCGStreamType::Double m_FocalLength;

  /// Skew
  ViconCGStreamType::Double m_Skew;

  /// Image Error
  ViconCGStreamType::Double m_ImageError;

  /// Equality operator
  bool operator == ( const VCameraCalibrationInfo& i_rOther ) const
  {
    return m_CameraID == i_rOther.m_CameraID &&
           ViconCGStreamDetail::IsEqual( m_PoseTranslation, i_rOther.m_PoseTranslation ) &&
           ViconCGStreamDetail::IsEqual( m_PoseRotation, i_rOther.m_PoseRotation ) &&
           ViconCGStreamDetail::IsEqual( m_PrincipalPoint, i_rOther.m_PrincipalPoint ) &&
           ViconCGStreamDetail::IsEqual( m_RadialDistortion, i_rOther.m_RadialDistortion ) &&
           m_FocalLength == i_rOther.m_FocalLength &&
           m_Skew == i_rOther.m_Skew &&
           m_ImageError == i_rOther.m_ImageError;
  }

  /// Object type enum.
  virtual ViconCGStreamType::Enum TypeID() const
  {
    return ViconCGStreamEnum::CameraCalibrationInfo;
  }

  //// Filter ID
  virtual ViconCGStreamType::UInt32 FilterID() const
  {
    return m_CameraID;
  }
  
  /// Read function.
  virtual bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_CameraID ) &&
           i_rBuffer.Read( m_PoseTranslation ) && 
           i_rBuffer.Read( m_PoseRotation ) &&
           i_rBuffer.Read( m_PrincipalPoint ) &&
           i_rBuffer.Read( m_RadialDistortion ) &&
           i_rBuffer.Read( m_FocalLength ) &&
           i_rBuffer.Read( m_Skew ) &&
           i_rBuffer.Read( m_ImageError );
  }

  /// Write function.
  virtual void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_CameraID );
    i_rBuffer.Write( m_PoseTranslation );
    i_rBuffer.Write( m_PoseRotation );
    i_rBuffer.Write( m_PrincipalPoint );
    i_rBuffer.Write( m_RadialDistortion );
    i_rBuffer.Write( m_FocalLength );
    i_rBuffer.Write( m_Skew );
    i_rBuffer.Write( m_ImageError );
  }

};

//-------------------------------------------------------------------------------------------------
};

