
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
/// Contains the detail declaration of the ViconCGStream::VLocalSegments class.

#include "Enum.h"
#include <StreamCommon/Buffer.h>
#include "IsEqual.h"

namespace ViconCGStreamDetail
{
//-------------------------------------------------------------------------------------------------

/**
  * Contains a segment translation from the parent origin, and rotation matrix (row major format).
  */
class VLocalSegments_Segment
{
public:
  /// Segment identifier
  ViconCGStreamType::UInt32 m_SegmentID;

  /// Segment translation from parent origin
  ViconCGStreamType::Double m_Translation[ 3 ];

  /// Segment rotation matrix (row major format)
  ViconCGStreamType::Double m_Rotation[ 9 ];
  
  // Equality operator
  bool operator == ( const VLocalSegments_Segment & i_rOther ) const
  {
    return m_SegmentID == i_rOther.m_SegmentID &&
           IsEqual( m_Translation, i_rOther.m_Translation ) &&
           IsEqual( m_Rotation, i_rOther.m_Rotation );
  }
  
  /// Read function.
  bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_SegmentID ) &&
           i_rBuffer.Read( m_Translation ) &&
           i_rBuffer.Read( m_Rotation );
  }
      
  /// Write function.
  void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_SegmentID );
    i_rBuffer.Write( m_Translation );
    i_rBuffer.Write( m_Rotation );
  }  
};

//-------------------------------------------------------------------------------------------------
};

