
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

#include "BufferDetail.h"
#include "Type.h"

#include <string>
#include <vector>
#include <map>
#include <set>

namespace ViconCGStreamIO
{
//-------------------------------------------------------------------------------------------------

/// Disable: 'this' : used in base member initializer list
#pragma warning( push )
#pragma warning( disable : 4355 )

/** \class VBuffer
**/
class VBuffer
{
public:

  VBuffer( const std::vector< unsigned char > & i_rBuffer = std::vector< unsigned char >() )
  : m_BufferImpl( *this, i_rBuffer )
  {
  }

  /// Read value.
  template< typename T >
  bool Read( T & o_rValue ) const
  {
    return VBufferDetail< VIsPod< T >::Answer >::Read( m_BufferImpl, o_rValue );
  }  

  /// Read fixed sized array.
  template< typename T, unsigned int N >
  bool Read( T ( & o_pValues )[ N ] ) const
  {
    return VBufferDetail< VIsPod< T >::Answer >::Read( m_BufferImpl, o_pValues, N );
  }
  
  /// Read vector.
  template< typename T >
  bool Read( std::vector< T > & o_rValues ) const
  {
    o_rValues.clear();
    ViconCGStreamType::UInt32 Size = 0;
    if( !m_BufferImpl.ReadPod( Size ) )
    {
      return false;
    }
    o_rValues.resize( Size );
    return VBufferDetail< VIsPod< T >::Answer >::Read( m_BufferImpl, o_rValues.empty() ? 0 : &o_rValues[ 0 ], Size );
  }  
  
  /// Read map.
  template< typename K, typename V >
  bool Read( std::map< K, V > & o_rValues ) const
  {
    o_rValues.clear();
    ViconCGStreamType::UInt32 Size = 0;
    if( !m_BufferImpl.ReadPod( Size ) )
    {
      return false;
    }
    for( ViconCGStreamType::UInt32 Index = 0; Index != Size; ++Index )
    {
      K Key;
      if( !Read( Key ) )
      {
        return false;
      }
      V Value;
      if( !Read( Value ) )
      {
        return false;
      }
      o_rValues.insert( std::make_pair( Key, Value ) );
    }
    return true;
  }
  
  /// Read set.
  template< typename T >
  bool Read( std::set< T > & o_rValues ) const
  {
    o_rValues.clear();
    ViconCGStreamType::UInt32 Size = 0;
    if( !m_BufferImpl.ReadPod( Size ) )
    {
      return false;
    }
    for( ViconCGStreamType::UInt32 Index = 0; Index != Size; ++Index )
    {
      T Value;
      if( !Read( Value ) )
      {
        return false;
      }
      o_rValues.insert( Value );
    }
    return true;
  }
  
  /// Write value.
  template< typename T >
  void Write( const T & i_rValue )
  {
    VBufferDetail< VIsPod< T >::Answer >::Write( m_BufferImpl, i_rValue );
  }
  
  /// Write fixed sized array.
  template< typename T, unsigned int N >
  void Write( const T ( & i_pValues )[ N ] )
  {
    VBufferDetail< VIsPod< T >::Answer >::Write( m_BufferImpl, i_pValues, N );
  }
  
  /// Write vector.
  template< typename T >
  void Write( const std::vector< T > & i_rValues )
  {
    m_BufferImpl.WritePod< ViconCGStreamType::UInt32 >( static_cast< ViconCGStreamType::UInt32 >( i_rValues.size() ) );
    VBufferDetail< VIsPod< T >::Answer >::Write( m_BufferImpl, i_rValues.empty() ? 0 : &i_rValues[ 0 ], static_cast< unsigned int >( i_rValues.size() ) );
  }
  
  /// Write map.
  template< typename K, typename V >
  void Write( const std::map< K, V > & i_rValues )
  {
    m_BufferImpl.WritePod< ViconCGStreamType::UInt32 >( static_cast< ViconCGStreamType::UInt32 >( i_rValues.size() ) );
    
    typename std::map< K, V >::const_iterator It = i_rValues.begin();
    typename std::map< K, V >::const_iterator End = i_rValues.end();
    for( ; It != End; ++It )
    {
      Write( ( *It ).first );
      Write( ( *It ).second );
    }
  }  
  
  /// Write set.
  template< typename T >
  void Write( const std::set< T > & i_rValues )
  {
    m_BufferImpl.WritePod< ViconCGStreamType::UInt32 >( static_cast< ViconCGStreamType::UInt32 >( i_rValues.size() ) );
    
    typename std::set< T >::const_iterator It = i_rValues.begin();
    typename std::set< T >::const_iterator End = i_rValues.end();
    for( ; It != End; ++It )
    {
      Write( *It );
    }    
  }

  /// Get offset into internal buffer.  
  unsigned int Offset() const
  {
    return m_BufferImpl.Offset();
  }
  
  /// Set offset into internal buffer.
  void SetOffset( unsigned int i_Offset ) const
  {
    m_BufferImpl.SetOffset( i_Offset );
  }  
  
  /// Access to internal buffer.
  const std::vector< unsigned char > & Data() const
  {
    return m_BufferImpl.Data();
  }

  /// Access to internal buffer raw.
  unsigned char * Raw()
  {
    return m_BufferImpl.Raw();
  }
  
  /// Access to internal buffer raw.
  const unsigned char * Raw() const
  {
    return m_BufferImpl.Raw();
  }  

  /// Length
  unsigned int Length() const
  {
    return m_BufferImpl.Length();
  }
  
  /// Set length
  void SetLength( unsigned int i_Length )
  {
    m_BufferImpl.SetLength( i_Length );
  }
  
  /// Clear
  void Clear()
  {
    m_BufferImpl.Clear();
  }

  /// Access to buffer impl. Use only for low level work and double check correctness.
  const ViconCGStreamIO::VBufferImpl & BufferImpl() const
  {
    return m_BufferImpl;
  }

  /// Access to buffer impl. Use only for low level work and double check correctness.
  ViconCGStreamIO::VBufferImpl & BufferImpl()
  {
    return m_BufferImpl;
  }

private:
  ViconCGStreamIO::VBufferImpl m_BufferImpl;
};

#pragma warning( pop )
//-------------------------------------------------------------------------------------------------
};


