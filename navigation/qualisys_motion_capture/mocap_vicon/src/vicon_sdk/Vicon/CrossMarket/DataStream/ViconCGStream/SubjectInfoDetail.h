
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
/// Contains the details declaration of the ViconCGStream::VSubjectInfo class.

#include "Enum.h"
#include <StreamCommon/Buffer.h>
#include "IsEqual.h"

namespace ViconCGStreamDetail
{
//-------------------------------------------------------------------------------------------------

/// Contains a segment name, segment and parent segment identifiers, and segment bounding box.
class VSubjectInfo_Segment
{
public:
  /// Parent segment identifier (or zero)
  ViconCGStreamType::UInt32 m_ParentID;

  /// Segment identifier
  ViconCGStreamType::UInt32 m_SegmentID;

  /// Segment axis aligned bounds (min and max)
  ViconCGStreamType::Float m_Bounds[ 6 ];

  /// Segment name
  std::string m_Name;

  /// Equality function
  bool IsEqual( const VSubjectInfo_Segment & i_rOther ) const
  {
    return m_ParentID == i_rOther.m_ParentID && 
           m_SegmentID == i_rOther.m_SegmentID && 
           m_Name == i_rOther.m_Name &&
           ViconCGStreamDetail::IsEqual( m_Bounds, i_rOther.m_Bounds );
  }
  
  /// Equality operator
  bool operator == ( const VSubjectInfo_Segment & i_rOther ) const
  {
    return IsEqual( i_rOther );
  }
  
  /// Read function.
  bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_ParentID ) &&
           i_rBuffer.Read( m_SegmentID ) &&
           i_rBuffer.Read( m_Bounds ) &&
           i_rBuffer.Read( m_Name );
  }
  
  /// Write function.
  void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_ParentID );
    i_rBuffer.Write( m_SegmentID );
    i_rBuffer.Write( m_Bounds );
    i_rBuffer.Write( m_Name );
  }    
};

/// Contains a marker name and identifier.
class VSubjectInfo_Marker
{
public:
  /// Marker identifier
  ViconCGStreamType::UInt32 m_MarkerID;

  /// Marker name
  std::string m_Name;

  /// Equality function
  bool IsEqual( const VSubjectInfo_Marker & i_rOther ) const
  {
    return m_MarkerID == i_rOther.m_MarkerID && 
           m_Name == i_rOther.m_Name;
  }
  
  /// Equality operator
  bool operator == ( const VSubjectInfo_Marker & i_rOther ) const
  {
    return IsEqual( i_rOther );
  }
  
  /// Read function.
  bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_MarkerID ) &&
           i_rBuffer.Read( m_Name );
  }
    
  /// Write function.
  void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_MarkerID );
    i_rBuffer.Write( m_Name );
  }    
  
};

/// (Member of VSubjectInfo) Contains a marker identifier, and parent segment identifier.
class VSubjectInfo_Attachment
{
public:

  /// Maker identifier
  ViconCGStreamType::UInt32 m_MarkerID;

  /// Parent segment identifier
  ViconCGStreamType::UInt32 m_SegmentID;

  /// Equality function
  bool IsEqual( const VSubjectInfo_Attachment & i_rOther ) const
  {
    return m_MarkerID == i_rOther.m_MarkerID &&
           m_SegmentID == i_rOther.m_SegmentID;
  }

  /// Equality operator
  bool operator == ( const VSubjectInfo_Attachment & i_rOther ) const
  {
    return IsEqual( i_rOther );
  }
  
  /// Read function.
  bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_MarkerID ) &&
           i_rBuffer.Read( m_SegmentID );
  }
  
  /// Write function.
  void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_MarkerID );
    i_rBuffer.Write( m_SegmentID );
  }    
  
};

//-------------------------------------------------------------------------------------------------

};

