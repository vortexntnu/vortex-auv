
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
#include <vector>

namespace ViconCGStreamIO
{
//-------------------------------------------------------------------------------------------------

/** \class VScopedWriter
This class provides functionality for writing nested blocks of data to a buffer.
Each block has a header which contains a ViconCGStreamEnum identifier, and a block length.
**/

class VScopedWriter
{
public:
  
  /// Constructor taking the underlying buffer to write to and the enum for this block.
  VScopedWriter( VBuffer & i_rBuffer, ViconCGStreamType::Enum i_Enum = ViconCGStreamEnum::Objects )
  : m_rBuffer( i_rBuffer )
  { 
    m_rBuffer.Write( i_Enum );
    m_Start = m_rBuffer.Offset();
    m_rBuffer.SetOffset( m_Start + sizeof( ViconCGStreamType::UInt32 ) );
  }

  /// Destructor that fills out length of block in header.
  ~VScopedWriter()
  {
    ViconCGStreamType::UInt32 Offset = m_rBuffer.Offset();
    ViconCGStreamType::UInt32 Length = ( Offset - m_Start ) - sizeof( ViconCGStreamType::UInt32 );
    
    m_rBuffer.SetOffset( m_Start );
    m_rBuffer.Write( Length );
    m_rBuffer.SetOffset( Offset );
  }

  /// Write an item and its enum type to the buffer.
  void Write( const ViconCGStream::VItem & i_rItem )
  {
    VScopedWriter ScopedWriter( m_rBuffer, i_rItem.TypeID() );
    m_rBuffer.Write( i_rItem );
  }

private:
  VBuffer & m_rBuffer;
  ViconCGStreamType::UInt32 m_Start;
};

//-------------------------------------------------------------------------------------------------
};


