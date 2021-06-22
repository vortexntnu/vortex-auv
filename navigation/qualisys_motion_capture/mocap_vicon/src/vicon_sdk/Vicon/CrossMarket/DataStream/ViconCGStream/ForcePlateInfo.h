
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
/// Contains the declaration of the ViconCGStream::VForcePlateInfo class.

#include "Item.h"
#include <string>

namespace ViconCGStream
{
//-------------------------------------------------------------------------------------------------

/// Contains the device identifier, world and local transformations, and bounds information.
/// Dimensions are in mm.
class VForcePlateInfo : public VItem
{
public:

  /// Device identifier
  ViconCGStreamType::UInt32 m_DeviceID;

  /// World translation
  ViconCGStreamType::Float m_WorldTranslation[ 3 ];

  /// World rotation matrix (row major format)
  ViconCGStreamType::Float m_WorldRotation[ 9 ];

  /// Device axis aligned bounds (min and max)
  ViconCGStreamType::Float m_Bounds[ 6 ];

  /// Local translation from the device origin in mm
  ViconCGStreamType::Float m_LocalTranslation[ 3 ];

  /// Local rotation matrix (row major format)
  ViconCGStreamType::Float m_LocalRotation[ 9 ];

  /// Equality function
  bool IsEqual( const VForcePlateInfo & i_rOther ) const
  {
    return m_DeviceID == i_rOther.m_DeviceID &&
           ViconCGStreamDetail::IsEqual( m_WorldTranslation, i_rOther.m_WorldTranslation ) &&
           ViconCGStreamDetail::IsEqual( m_WorldRotation, i_rOther.m_WorldRotation ) &&
           ViconCGStreamDetail::IsEqual( m_Bounds, i_rOther.m_Bounds ) &&
           ViconCGStreamDetail::IsEqual( m_LocalTranslation, i_rOther.m_LocalTranslation ) &&
           ViconCGStreamDetail::IsEqual( m_LocalRotation, i_rOther.m_LocalRotation );
  }

  /// Equality operator
  bool operator == ( const VForcePlateInfo & i_rOther ) const
  {
    return IsEqual( i_rOther );
  }

  /// Object type enum.
  virtual ViconCGStreamType::Enum TypeID() const
  {
    return ViconCGStreamEnum::ForcePlateInfo;
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
           i_rBuffer.Read( m_WorldTranslation ) &&
           i_rBuffer.Read( m_WorldRotation ) &&
           i_rBuffer.Read( m_Bounds ) &&
           i_rBuffer.Read( m_LocalTranslation ) &&
           i_rBuffer.Read( m_LocalRotation );
  }

  /// Write function.
  virtual void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_DeviceID );
    i_rBuffer.Write( m_WorldTranslation );
    i_rBuffer.Write( m_WorldRotation );
    i_rBuffer.Write( m_Bounds );
    i_rBuffer.Write( m_LocalTranslation );
    i_rBuffer.Write( m_LocalRotation );
  }

};

//-------------------------------------------------------------------------------------------------
};

