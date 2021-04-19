
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

#include <StreamCommon/Buffer.h>
#include "Enum.h"
#include <string>
#include <vector>

namespace ViconCGStream
{
  class VItem;
}

namespace ViconCGStreamIO
{
//-------------------------------------------------------------------------------------------------

/** \class VScopedReader
This class provides functionality for reading nested blocks of data from a buffer.
Each block has a header which contains a ViconCGStreamEnum identifier, and a block length.
**/

class VScopedReader
{
public:
  
  /// Constructor taking the underlying buffer to read from.
  VScopedReader( VBuffer & i_rBuffer )
  : m_rBuffer( i_rBuffer )
  , m_Enum( 0 )
  , m_Start( 0 )
  , m_End( 0 )
  {
    ViconCGStreamType::UInt32 Length;

    if( !m_rBuffer.Read( m_Enum ) || !m_rBuffer.Read( Length ) )
    {
      return;
    }

    m_Start = m_rBuffer.Offset();
    m_End = m_Start + Length;
  }

  /// Destructor.
  ~VScopedReader()
  {
    if( m_Start < m_End )
    {
      m_rBuffer.SetOffset( m_End );
    }
  }

  /// Determine if the current buffer offset is valid.
  bool Ok()
  {
    ViconCGStreamType::UInt32 Offset = m_rBuffer.Offset();
    return Offset >= m_Start && Offset < m_End;
  }

  /// Get the Enum read from the block header.
  /// This function is used to determine the block type.
  /// If the block is valid, then the value returned will be defined in the ViconCGStreamEnum namespace.
  ViconCGStreamType::Enum Enum() const
  {
    return m_Enum;
  }

  /// Read a single item from the buffer.
  bool Read( ViconCGStream::VItem & o_rItem ) const
  {
    return m_rBuffer.Read( o_rItem );
  }

private:
  VBuffer&                  m_rBuffer;
  ViconCGStreamType::Enum   m_Enum;
  ViconCGStreamType::UInt32 m_Start;
  ViconCGStreamType::UInt32 m_End;
};

//-------------------------------------------------------------------------------------------------
};


