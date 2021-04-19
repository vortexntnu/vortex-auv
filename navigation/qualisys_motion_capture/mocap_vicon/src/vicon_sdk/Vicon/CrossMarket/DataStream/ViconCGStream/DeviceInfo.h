
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
/// Contains the declaration of the ViconCGStream::VDeviceInfo class.

#include "Item.h"
#include <string>

namespace ViconCGStream
{
//-------------------------------------------------------------------------------------------------

/// Contains the device identifier, frame period, and name of a newly connected or updated device.
class VDeviceInfo : public VItem
{
public:

  /// Device identifier
  ViconCGStreamType::UInt32 m_DeviceID;

  /// Device frame period. Defined as the numbers of ticks per frame on a 135Mhz clock.
  ViconCGStreamType::UInt64 m_FramePeriod;

  /// Device name
  std::string m_Name;

  /// Equality function
  bool IsEqual( const VDeviceInfo & i_rOther ) const
  {
    return m_DeviceID == i_rOther.m_DeviceID &&
           m_FramePeriod == i_rOther.m_FramePeriod &&
           m_Name == i_rOther.m_Name;  
  }

  /// Equality operator
  bool operator == ( const VDeviceInfo & i_rOther ) const
  {
    return IsEqual( i_rOther );
  }

  /// Object type enum.
  virtual ViconCGStreamType::Enum TypeID() const
  {
    return ViconCGStreamEnum::DeviceInfo;
  }

  /// Filter ID
  virtual ViconCGStreamType::UInt32 FilterID() const
  {
    return m_DeviceID;
  }

  /// Read function.
  virtual bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_DeviceID ) &&
           i_rBuffer.Read( m_FramePeriod ) &&
           i_rBuffer.Read( m_Name );
  }

  /// Write function.
  virtual void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_DeviceID );
    i_rBuffer.Write( m_FramePeriod );
    i_rBuffer.Write( m_Name );
  }

};

/// Contains the device identifier, frame start tick.
class VDeviceInfoExtra : public VItem
{
public:

  /// Device identifier
  ViconCGStreamType::UInt32 m_DeviceID;

  /// Device first device frame start tick offset from 0, on a 135Mhz clock.
  ViconCGStreamType::UInt64 m_FrameStartTick;

  /// Equality function
  bool IsEqual( const VDeviceInfoExtra & i_rOther ) const
  {
    return m_DeviceID == i_rOther.m_DeviceID &&
           m_FrameStartTick == i_rOther.m_FrameStartTick;
  }

  /// Equality operator
  bool operator == ( const VDeviceInfoExtra  & i_rOther ) const
  {
    return IsEqual( i_rOther );
  }

  /// Object type enum.
  virtual ViconCGStreamType::Enum TypeID() const
  {
    return ViconCGStreamEnum::DeviceInfoExtra;
  }

  /// Filter ID
  virtual ViconCGStreamType::UInt32 FilterID() const
  {
    return m_DeviceID;
  }

  /// Read function.
  virtual bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_DeviceID ) &&
           i_rBuffer.Read( m_FrameStartTick );
  }

  /// Write function.
  virtual void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_DeviceID );
    i_rBuffer.Write( m_FrameStartTick );
  }

};

//-------------------------------------------------------------------------------------------------
};

