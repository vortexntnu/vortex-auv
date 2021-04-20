
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
/// Contains the declaration of the ViconCGStream::VSubjectInfo class.

#include "Item.h"
#include "SubjectInfoDetail.h"
#include <vector>

namespace ViconCGStream
{
//-------------------------------------------------------------------------------------------------

/// Contains the static description of a single subject.
class VSubjectInfo : public VItem
{
public:

  /// Subject identifier
  ViconCGStreamType::UInt32 m_SubjectID;

  /// Subject name
  std::string m_Name;

  /// Segments
  std::vector< ViconCGStreamDetail::VSubjectInfo_Segment > m_Segments;

  /// Markers
  std::vector< ViconCGStreamDetail::VSubjectInfo_Marker > m_Markers;

  /// Marker to segment attachments
  std::vector< ViconCGStreamDetail::VSubjectInfo_Attachment > m_Attachments; 

  /// Equality operator
  bool operator == ( const VSubjectInfo& i_rOther ) const
  {
    return  ( m_SubjectID   == i_rOther.m_SubjectID ) &&
            ( m_Name        == i_rOther.m_Name )      &&
            ( m_Segments    == i_rOther.m_Segments )  &&
            ( m_Markers     == i_rOther.m_Markers )   &&
            ( m_Attachments == i_rOther.m_Attachments );
  }

  /// Object type enum.
  virtual ViconCGStreamType::Enum TypeID() const
  {
    return ViconCGStreamEnum::SubjectInfo;
  }
  
  /// Filter ID
  virtual ViconCGStreamType::UInt32 FilterID() const
  {
    return m_SubjectID;
  }

  /// Read function.
  virtual bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_SubjectID ) &&
           i_rBuffer.Read( m_Name ) &&
           i_rBuffer.Read( m_Segments ) &&
           i_rBuffer.Read( m_Markers ) &&
           i_rBuffer.Read( m_Attachments );
  }

  /// Write function.
  virtual void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_SubjectID );
    i_rBuffer.Write( m_Name );
    i_rBuffer.Write( m_Segments );
    i_rBuffer.Write( m_Markers );
    i_rBuffer.Write( m_Attachments );
  }

};

//-------------------------------------------------------------------------------------------------
};

