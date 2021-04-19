
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
/// Contains the declaration of the ViconCGStream::VEyeTrackerInfo class.

#include "Item.h"
#include <string>

namespace ViconCGStream
{
//-------------------------------------------------------------------------------------------------

/// Contains the device identifier, head segment information and local transformations 
/// for the eye tracker.
/// Initially this is supporting the Ergoneers Dikablis device.

class VEyeTrackerInfo : public VItem
{
public:

  /// Device identifier
  ViconCGStreamType::UInt32 m_DeviceID;

  /// Head subject identifier
  /// The head subject is the one that has the head segment on it.
  /// May be -1 if a segment is not set or is set incorrectly.
  ViconCGStreamType::UInt32 m_SubjectID;

  /// Head segment identifier
  /// The head segment is the one that has the eye tracker attached to it.
  /// May be -1 if a segment is not set or is set incorrectly.
  ViconCGStreamType::UInt32 m_SegmentID;

  /// Local translation from the head segment origin in mm.
  /// This is the location of the centre of the eye-ball
  /// with respect to the head.
  ViconCGStreamType::Float m_LocalTranslation[ 3 ];

  /// Local rotation matrix (row major format)
  /// This is the rotation matrix that aligns the basic eye coordinate system.
  /// The Z axis is forwards.
  ViconCGStreamType::Float m_LocalRotation[ 9 ];

  /// Equality function
  bool IsEqual( const VEyeTrackerInfo & i_rOther ) const
  {
    return m_DeviceID == i_rOther.m_DeviceID &&
           m_SubjectID == i_rOther.m_SubjectID &&
           m_SegmentID == i_rOther.m_SegmentID &&
           ViconCGStreamDetail::IsEqual( m_LocalTranslation, i_rOther.m_LocalTranslation ) &&
           ViconCGStreamDetail::IsEqual( m_LocalRotation, i_rOther.m_LocalRotation );
  }

  /// Equality operator
  bool operator == ( const VEyeTrackerInfo & i_rOther ) const
  {
    return IsEqual( i_rOther );
  }

  /// Object type enum.
  virtual ViconCGStreamType::Enum TypeID() const
  {
    return ViconCGStreamEnum::EyeTrackerInfo;
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
           i_rBuffer.Read( m_SubjectID ) &&
           i_rBuffer.Read( m_SegmentID ) &&
           i_rBuffer.Read( m_LocalTranslation ) &&
           i_rBuffer.Read( m_LocalRotation );
  }

  /// Write function.
  virtual void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_DeviceID );
    i_rBuffer.Write( m_SubjectID );
    i_rBuffer.Write( m_SegmentID );
    i_rBuffer.Write( m_LocalTranslation );
    i_rBuffer.Write( m_LocalRotation );
  }

};

//-------------------------------------------------------------------------------------------------
};

