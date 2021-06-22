
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
/// Contains the detail declaration of the ViconCGStream::VEdgePairs class.

#include "Enum.h"
#include <StreamCommon/Buffer.h>
#include "IsEqual.h"

namespace ViconCGStreamDetail
{
//-------------------------------------------------------------------------------------------------

/// Contains a frame of edge pairs, along with the camera identifier and frame number.
class VEdgePairs_EdgePair
{
public:

  /// 2D position
  ViconCGStreamType::Int16 m_Position[ 2 ];
  
  /// Number of pixels in the line
  ViconCGStreamType::UInt16 m_Length;
  
  /// Equality operator
  bool operator == ( const VEdgePairs_EdgePair & i_rOther ) const
  {
    return IsEqual( m_Position, i_rOther.m_Position ) &&
           m_Length == i_rOther.m_Length;
  }
  
  /// Read function.
  bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_Position ) &&
           i_rBuffer.Read( m_Length );
  }  
  
  /// Write function.
  void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_Position );
    i_rBuffer.Write( m_Length );
  }
};

//-------------------------------------------------------------------------------------------------
};

