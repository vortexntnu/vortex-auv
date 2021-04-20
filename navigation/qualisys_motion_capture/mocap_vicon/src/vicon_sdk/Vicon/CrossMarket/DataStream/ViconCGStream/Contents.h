
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
/// Contains the declaration of the ViconCGStream::VContents class.

#include "Item.h"
#include <map>
#include <set>

namespace ViconCGStream
{
//-------------------------------------------------------------------------------------------------

/// Contents.
class VContents : public VItem
{
public:

  /// Number of objects of each requested type present:
  /// The map contains an entry for each object type requested, along 
  /// with a Size to indicate the number of objects present.
  std::map< ViconCGStreamType::Enum, ViconCGStreamType::UInt32 > m_EnumsTable;

  /// Set of requested types that do not contain changes:
  /// If all objects of a given type remain unchanged since the last object 
  /// packet, then the type value will appear in this container.
  std::set< ViconCGStreamType::Enum > m_EnumsUnchanged;

  /// Equality operator 
  bool operator == ( const VContents & i_rOther ) const
  {
    return m_EnumsTable == i_rOther.m_EnumsTable && m_EnumsUnchanged == i_rOther.m_EnumsUnchanged;
  }

  /// Object type enum.
  virtual ViconCGStreamType::Enum TypeID() const
  {
    return ViconCGStreamEnum::Contents;
  }

  /// Filter ID
  virtual ViconCGStreamType::UInt32 FilterID() const
  {
    return FILTER_NA;
  }

  /// Read function.
  virtual bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_EnumsTable ) &&
           i_rBuffer.Read( m_EnumsUnchanged );
  }

  /// Write function.
  virtual void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_EnumsTable );
    i_rBuffer.Write( m_EnumsUnchanged );
  }

};

//-------------------------------------------------------------------------------------------------
};

