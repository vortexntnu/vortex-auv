
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
/// Contains the detail declaration of the ViconCGStream::VGreyscaleBlobs class.

#include "Enum.h"
#include <StreamCommon/Buffer.h>
#include <vector>

namespace ViconCGStreamDetail
{
//-------------------------------------------------------------------------------------------------

/**
  * Contains a greyscale line
  */
class VGreyscaleBlobs_GreyscaleLine
{
public:

  /// Greyscale values for the line pixels
  std::vector< ViconCGStreamType::UInt8 > m_Greyscale; 

  /// 2D position  
  ViconCGStreamType::Int16 m_Position[ 2 ];  
  
  /// Equality operator 
  bool operator == ( const VGreyscaleBlobs_GreyscaleLine & i_rOther ) const
  {
    return m_Position[ 0 ] == i_rOther.m_Position[ 0 ] &&
           m_Position[ 1 ] == i_rOther.m_Position[ 1 ] && 
           m_Greyscale == i_rOther.m_Greyscale;
  }
  
  /// Read function.
  bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_Greyscale ) &&
           i_rBuffer.Read( m_Position );
  }

  /// Write function.
  void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_Greyscale );
    i_rBuffer.Write( m_Position );
  }    
};

/**
  * Contains greyscale lines
  */
class VGreyscaleBlobs_GreyscaleBlob
{
public:

  /// Vector of lines comprising the blob
  std::vector< VGreyscaleBlobs_GreyscaleLine > m_GreyscaleLines;  
  
  /// Equality operator 
  bool operator == ( const VGreyscaleBlobs_GreyscaleBlob & i_rOther ) const
  {
    return m_GreyscaleLines == i_rOther.m_GreyscaleLines;
  }
  
  /// Read function.
  bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_GreyscaleLines );
  }
    
  /// Write function.
  void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_GreyscaleLines );
  }    
};  

//-------------------------------------------------------------------------------------------------
};

