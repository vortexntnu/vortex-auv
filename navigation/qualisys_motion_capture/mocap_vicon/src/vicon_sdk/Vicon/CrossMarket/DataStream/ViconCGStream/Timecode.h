
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
/// Contains the declaration of the ViconCGStream::VTimecode class.

#include "Item.h"

namespace ViconCGStream
{
//-------------------------------------------------------------------------------------------------

class VTimecode : public VItem
{
public:

  /// Time code standard
  enum ETimecodeStandard
  {
    ETimecodePAL = 0,           ///< PAL
    ETimecodeNTSC,              ///< NTSC
    ETimecodeNTSCDrop,          ///< NTSCDrop
    ETimecodeFilm,              ///< Film at 24fps
    ETimecodeNTSCFilm,  ///< NTSC Film (23.98fps)
    ETimecodeATSC,      ///< ATSC (30fps)
    ETimecodeStandardCount,     ///< Standard
    ETimecodeNone = -1          ///< Invalid
  };

  /// Timecode
  ViconCGStreamType::UInt8  m_Hours;
  ViconCGStreamType::UInt8  m_Minutes;
  ViconCGStreamType::UInt8  m_Seconds;
  ViconCGStreamType::UInt8  m_Frames;
  ViconCGStreamType::UInt16 m_Subframes;
  ViconCGStreamType::UInt16 m_SubframesPerFrame;
  ViconCGStreamType::UInt32 m_UserBits;
  ViconCGStreamType::UInt8  m_FieldFlag;
  ViconCGStreamType::UInt32 m_Standard;

  VTimecode()
  : m_Hours( 0 )
  , m_Minutes( 0 )
  , m_Seconds( 0 )
  , m_Frames( 0 )
  , m_Subframes( 0 )
  , m_SubframesPerFrame( 0 )
  , m_UserBits( 0 )
  , m_FieldFlag( 0 )
  , m_Standard( ETimecodeNone )
  {
  }

  /// Equality operator
  bool operator == ( const VTimecode & i_rOther ) const
  {
    return  m_Hours == i_rOther.m_Hours &&
            m_Minutes == i_rOther.m_Minutes &&
            m_Seconds == i_rOther.m_Seconds &&
            m_Frames == i_rOther.m_Frames &&
            m_Subframes == i_rOther.m_Subframes &&
            m_SubframesPerFrame == i_rOther.m_SubframesPerFrame &&
            m_UserBits == i_rOther.m_UserBits &&
            m_FieldFlag == i_rOther.m_FieldFlag &&
            m_Standard == i_rOther.m_Standard;
  }

  /// Object type enum.
  virtual ViconCGStreamType::Enum TypeID() const
  {
    return ViconCGStreamEnum::Timecode;
  }

  /// Filter ID
  virtual ViconCGStreamType::UInt32 FilterID() const
  {
    return FILTER_NA;
  }

  /// Read function.
  virtual bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_Hours ) &&
           i_rBuffer.Read( m_Minutes ) &&
           i_rBuffer.Read( m_Seconds ) &&
           i_rBuffer.Read( m_Frames ) &&
           i_rBuffer.Read( m_Subframes ) &&
           i_rBuffer.Read( m_SubframesPerFrame ) &&
           i_rBuffer.Read( m_Standard ) &&
           i_rBuffer.Read( m_UserBits ) &&
           i_rBuffer.Read( m_FieldFlag );
  }

  /// Write function.
  virtual void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_Hours );
    i_rBuffer.Write( m_Minutes );
    i_rBuffer.Write( m_Seconds );
    i_rBuffer.Write( m_Frames );
    i_rBuffer.Write( m_Subframes );
    i_rBuffer.Write( m_SubframesPerFrame );
    i_rBuffer.Write( m_Standard );
    i_rBuffer.Write( m_UserBits );
    i_rBuffer.Write( m_FieldFlag );
  }
};

//-------------------------------------------------------------------------------------------------
}
