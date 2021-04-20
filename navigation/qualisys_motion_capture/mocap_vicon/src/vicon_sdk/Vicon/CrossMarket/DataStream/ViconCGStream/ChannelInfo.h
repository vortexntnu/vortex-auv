
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
/// Contains the declaration of the ViconCGStream::VChannelInfo class.

#include "Item.h"
#include <string>

namespace ViconCGStream
{
//-------------------------------------------------------------------------------------------------

/// Contains the device and channel identifiers, the channel name, and the channel component names.
class VChannelInfo : public VItem
{
public:
  VChannelInfo() {}
  VChannelInfo( ViconCGStreamType::UInt32 i_DeviceID,
                ViconCGStreamType::UInt32 i_ChannelID,
                std::string&& i_rName,
                std::vector< std::string >&& i_rComponentNames )
  : m_DeviceID( i_DeviceID )
  , m_ChannelID( i_ChannelID )
  , m_Name( i_rName )
  , m_ComponentNames( i_rComponentNames )
  {

  }

  /// Device identifier
  ViconCGStreamType::UInt32 m_DeviceID;

  /// Channel identifier
  ViconCGStreamType::UInt32 m_ChannelID;

  /// Channel name
  std::string m_Name;

  /// Component names
  std::vector< std::string > m_ComponentNames;

  /// Equality function
  bool IsEqual( const VChannelInfo & i_rOther ) const
  {
    return m_DeviceID == i_rOther.m_DeviceID &&
           m_ChannelID == i_rOther.m_ChannelID &&
           m_Name == i_rOther.m_Name &&
           m_ComponentNames == i_rOther.m_ComponentNames;
  }

  /// Equality operator
  bool operator == ( const VChannelInfo & i_rOther ) const
  {
    return IsEqual( i_rOther );
  }

  /// Object type enum.
  virtual ViconCGStreamType::Enum TypeID() const
  {
    return ViconCGStreamEnum::ChannelInfo;
  }

  /// Filter ID
  virtual ViconCGStreamType::UInt32 FilterID() const
  {
    return m_DeviceID;
  }

  /// Read function.
  virtual bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_DeviceID ) &&
           i_rBuffer.Read( m_ChannelID ) &&
           i_rBuffer.Read( m_Name ) &&
           i_rBuffer.Read( m_ComponentNames );
  }
  
  /// Write function.
  virtual void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_DeviceID );
    i_rBuffer.Write( m_ChannelID );
    i_rBuffer.Write( m_Name );
    i_rBuffer.Write( m_ComponentNames );
  }
};

/// Extra information to supplement the VChannelInfo
/// Contains the device and channel identifiers, device unit and the type.
class VChannelInfoExtra : public VItem
{
public:

  /// Device identifier
  ViconCGStreamType::UInt32 m_DeviceID;

  /// Channel identifier
  ViconCGStreamType::UInt32 m_ChannelID;

  /// Channel unit
  std::string m_Unit;

  /// Channel type
  std::string m_Type;

  /// Equality function
  bool IsEqual( const VChannelInfoExtra & i_rOther ) const
  {
    return m_DeviceID == i_rOther.m_DeviceID &&
           m_ChannelID == i_rOther.m_ChannelID &&
           m_Unit == i_rOther.m_Unit &&
           m_Type == i_rOther.m_Type;
  }

  /// Equality operator
  bool operator == ( const VChannelInfoExtra & i_rOther ) const
  {
    return IsEqual( i_rOther );
  }

  /// Object type enum.
  virtual ViconCGStreamType::Enum TypeID() const
  {
    return ViconCGStreamEnum::ChannelInfoExtra;
  }

  /// Filter ID
  virtual ViconCGStreamType::UInt32 FilterID() const
  {
    return m_DeviceID;
  }

  /// Read function.
  virtual bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_DeviceID ) &&
           i_rBuffer.Read( m_ChannelID ) &&
           i_rBuffer.Read( m_Unit ) &&
           i_rBuffer.Read( m_Type );
  }
  
  /// Write function.
  virtual void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_DeviceID );
    i_rBuffer.Write( m_ChannelID );
    i_rBuffer.Write( m_Unit );
    i_rBuffer.Write( m_Type );
  }
};


//-------------------------------------------------------------------------------------------------
};

