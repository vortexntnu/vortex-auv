
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
/// Contains the declaration of the ViconCGStream::VVideoFrame class.

#include "Item.h"
#include <vector>

namespace ViconCGStream
{
//-------------------------------------------------------------------------------------------------

/// Contains a frame of video, along with the camera identifier and frame number.
class VVideoFrame : public VItem
{
public:

  /// Camera frame number
  ViconCGStreamType::UInt32 m_FrameID;

  /// Camera identifier
  ViconCGStreamType::UInt32 m_CameraID;

  /// Video frame window offset
  ViconCGStreamType::Int16 m_Position[ 2 ];
  
  /// Video frame window width
  ViconCGStreamType::UInt16 m_Width;

  /// Video frame window height
  ViconCGStreamType::UInt16 m_Height;
  
  // Video frame format
  enum EFormat { ENoVideo = 0, EMono8, EBayerRG8, EBayerGB8, EBayerGR8, EBayerBG8, ERGB888, EBGR888 };
  ViconCGStreamType::UInt32 m_Format;

  /// Video data. 
  /// The first pixel in this buffer should be displayed in the top-left position when rendered.
  std::vector< ViconCGStreamType::UInt8 > m_VideoData;

  /// Equality operator 
  bool operator == ( const VVideoFrame & i_rOther ) const
  {
    return m_FrameID == i_rOther.m_FrameID &&
           m_CameraID == i_rOther.m_CameraID &&
           m_Position[ 0 ] == i_rOther.m_Position[ 0 ] &&
           m_Position[ 1 ] == i_rOther.m_Position[ 1 ] &&
           m_Width == i_rOther.m_Width &&
           m_Height == i_rOther.m_Height &&
           m_Format == i_rOther.m_Format &&
           m_VideoData == i_rOther.m_VideoData;
  }
  
  /// Object type enum.
  virtual ViconCGStreamType::Enum TypeID() const
  {
    return ViconCGStreamEnum::VideoFrame;
  }
  
  /// Filter ID
  virtual ViconCGStreamType::UInt32 FilterID() const
  {
    return m_CameraID;
  }

  /// Read function.
  virtual bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_FrameID ) &&
           i_rBuffer.Read( m_CameraID ) &&
           i_rBuffer.Read( m_Position ) &&
           i_rBuffer.Read( m_Width ) &&
           i_rBuffer.Read( m_Height ) &&
           i_rBuffer.Read( m_Format ) &&
           i_rBuffer.Read( m_VideoData );
  }
  
  /// Write function.
  virtual void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_FrameID );
    i_rBuffer.Write( m_CameraID );
    i_rBuffer.Write( m_Position[ 0 ] );
    i_rBuffer.Write( m_Position[ 1 ] );
    i_rBuffer.Write( m_Width );
    i_rBuffer.Write( m_Height );
    i_rBuffer.Write( m_Format );
    i_rBuffer.Write( m_VideoData );
  }

};

//-------------------------------------------------------------------------------------------------
};

