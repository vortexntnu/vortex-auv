
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
#include <set>

namespace ViconCGStream
{
//-------------------------------------------------------------------------------------------------

/// Start client multicast session
class VStartMulticastSender : public VItem
{
public:

  /// Destination IP address, should be multicast or broadcast
  ViconCGStreamType::UInt32 m_MulticastIpAddress;
  
  // Source IP address
  ViconCGStreamType::UInt32 m_SourceIpAddress;
  
  // Port number
  ViconCGStreamType::UInt16 m_Port;

  /// Equality operator
  bool operator == ( const VStartMulticastSender & i_rOther ) const
  {
    return m_MulticastIpAddress == i_rOther.m_MulticastIpAddress
        && m_SourceIpAddress    == i_rOther.m_SourceIpAddress
        && m_Port               == i_rOther.m_Port;
  }

  /// Object type enum.
  virtual ViconCGStreamType::Enum TypeID() const
  {
    return ViconCGStreamEnum::StartMulticastSender;
  }

  /// Filter ID
  virtual ViconCGStreamType::UInt32 FilterID() const
  {
    return FILTER_NA;
  }
  
  /// Read function.
  virtual bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_MulticastIpAddress )
        && i_rBuffer.Read( m_SourceIpAddress )
        && i_rBuffer.Read( m_Port );
  }

  /// Write function.
  virtual void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_MulticastIpAddress );
    i_rBuffer.Write( m_SourceIpAddress );
    i_rBuffer.Write( m_Port );;
  }

};

//-------------------------------------------------------------------------------------------------
};

