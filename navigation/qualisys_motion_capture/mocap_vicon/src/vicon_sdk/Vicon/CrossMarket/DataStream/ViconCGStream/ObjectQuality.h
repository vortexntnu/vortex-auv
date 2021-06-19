
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
/// Contains the declaration of the ViconCGStream::VObjectQuality class.

#include "Item.h"
#include <vector>

namespace ViconCGStream
{
//-------------------------------------------------------------------------------------------------

/// Contains a quality estimate for a single object.
class VObjectQuality : public VItem
{
public:

  /// Subject identifier
  ViconCGStreamType::UInt32 m_SubjectID;

  /// Quality estimate from 0.0 (bad) to 1.0 (excellent).
  ViconCGStreamType::Double m_ObjectQuality;
  
  /// Equality operator
  bool operator == ( const VObjectQuality& i_rOther ) const
  {
    return  ( m_SubjectID == i_rOther.m_SubjectID ) &&
            ( m_ObjectQuality == i_rOther.m_ObjectQuality );
  }

  /// Object type enum.
  virtual ViconCGStreamType::Enum TypeID() const
  {
    return ViconCGStreamEnum::ObjectQuality;
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
      i_rBuffer.Read( m_ObjectQuality );
  }

  /// Write function.
  virtual void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_SubjectID );
    i_rBuffer.Write( m_ObjectQuality );
  }

};

//-------------------------------------------------------------------------------------------------
};

