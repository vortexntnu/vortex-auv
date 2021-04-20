
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

/// Contains the camera timing info.
class VCameraTimingInfo : public VItem
{
public:

  /// Camera identifier
  ViconCGStreamType::UInt32 m_CameraID;

  /// Camera frame period. Defined as the numbers of ticks per frame on a 135Mhz clock.
  ViconCGStreamType::UInt64 m_FramePeriod;

  /// Camera frame offset. Tick offset in time of frame zero.
  ViconCGStreamType::Int64 m_FrameOffset;

  /// Equality function
  bool IsEqual( const VCameraTimingInfo & i_rOther ) const
  {
    return m_CameraID == i_rOther.m_CameraID &&
           m_FramePeriod == i_rOther.m_FramePeriod &&
           m_FrameOffset == i_rOther.m_FrameOffset;
  }

  /// Equality operator
  bool operator == ( const VCameraTimingInfo & i_rOther ) const
  {
    return IsEqual( i_rOther );
  }

  /// Object type enum.
  virtual ViconCGStreamType::Enum TypeID() const
  {
    return ViconCGStreamEnum::CameraTimingInfo;
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
           i_rBuffer.Read( m_FrameOffset );
  }

  /// Write function.  
  virtual void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_CameraID );
    i_rBuffer.Write( m_FramePeriod );
    i_rBuffer.Write( m_FrameOffset );
  }

};

//-------------------------------------------------------------------------------------------------
};

