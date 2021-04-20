
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
/// Contains the declaration of the ViconCGStream::VCameraCalibrationHealth class.

#include "Item.h"
#include "CameraCalibrationHealthDetail.h"
#include <vector>

namespace ViconCGStream
{
//-------------------------------------------------------------------------------------------------

/// Contains a overall system Camera calibration health indicator as well as an individual indicator for each camera
/// Camera Health indicators vary from 0.0 (bad) to 1.0 (good)
class VCameraCalibrationHealth : public VItem
{
public:

  /// Calibration Health for entire system
  ViconCGStreamType::Double m_SystemHealth;

  /// Individual Calibration Health for cameras
  std::vector< ViconCGStreamDetail::VCameraCalibrationHealth_Camera > m_Cameras;

  VCameraCalibrationHealth()
  : m_SystemHealth( 0.0 )
  {
  }

  /// Equality operator
  bool operator == ( const VCameraCalibrationHealth & i_rOther ) const
  {
    return m_SystemHealth == i_rOther.m_SystemHealth &&
           m_Cameras      == i_rOther.m_Cameras;
  }
  
  /// Object type enum.
  virtual ViconCGStreamType::Enum TypeID() const
  {
    return ViconCGStreamEnum::CameraCalibrationHealth;
  }

  /// Filter ID
  virtual ViconCGStreamType::UInt32 FilterID() const
  {
    return FILTER_NA;
  }
  
  /// Read function.
  virtual bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_SystemHealth ) && 
           i_rBuffer.Read( m_Cameras );
  }

  /// Write function.
  virtual void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_SystemHealth );
    i_rBuffer.Write( m_Cameras );
  }

};

//-------------------------------------------------------------------------------------------------
};

