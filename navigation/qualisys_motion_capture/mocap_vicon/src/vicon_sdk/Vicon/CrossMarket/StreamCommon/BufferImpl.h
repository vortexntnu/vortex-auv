
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

#include <algorithm>
#include <string.h>
#include <vector>
#include <type_traits>

namespace ViconCGStreamIO
{
//-------------------------------------------------------------------------------------------------

class VBuffer;

/** \class VBufferImpl
**/
class VBufferImpl
{
public:

  /// Construct buffer impl
  VBufferImpl( VBuffer & i_rParent, const std::vector< unsigned char > & i_rBuffer = std::vector< unsigned char >() )
  : m_rParent( i_rParent )
  , m_Offset( 0 )
  , m_Buffer( i_rBuffer )
  {
  }

  /// Write singluar pod.
  template< typename T >
  void WritePod( const T & i_rValue )
  {
    const unsigned int End = m_Offset + sizeof( T );
    if( m_Buffer.size() < End )
    {
      m_Buffer.resize( End );
    }
    
    *reinterpret_cast< T * >( &m_Buffer[ 0 ] + m_Offset ) = i_rValue;
    m_Offset += sizeof( T );
  }
  
  /// Write array of pods.
  template< typename T >
  void WritePodArray( const T * i_pValue, unsigned int i_Size )
  {
    const unsigned int End = m_Offset + sizeof( T ) * i_Size;
    if( m_Buffer.size() < End )
    {
      m_Buffer.resize( End );
    }
    
    memcpy( reinterpret_cast< T * >( &m_Buffer[ 0 ] + m_Offset ), i_pValue, sizeof( T ) * i_Size );
    m_Offset += sizeof( T ) * i_Size;
  }
    
  /// Read singluar pod.
  template< typename T >
  bool ReadPod( T & o_rValue ) const
  {
   const size_t sizeOfT = sizeof( T );

   if( m_Offset + sizeOfT > m_Buffer.size() )
   {
     return false;
   }

   memcpy( &o_rValue, &m_Buffer[ 0 ] + m_Offset, sizeOfT );
   m_Offset += sizeOfT;
   return true;
  }

  /// Read array of pods.
  template< typename T >
  bool ReadPodArray( T * o_pValue, unsigned int i_Size ) const
  {
    if( m_Offset + sizeof( T ) * i_Size > m_Buffer.size() )
    {
      return false;
    }

    memcpy( o_pValue, &m_Buffer[ 0 ] + m_Offset, sizeof( T ) * i_Size );
    m_Offset += sizeof( T ) * i_Size;
    return true;
  }

  /// Get offset into internal buffer.  
  unsigned int Offset() const
  {
    return m_Offset;
  }
  
  /// Set offset into internal buffer.
  void SetOffset( unsigned int i_Offset ) const
  {
    m_Offset = i_Offset;
  }  
  
  /// Access to internal buffer.
  const std::vector< unsigned char > & Data() const
  {
    return m_Buffer;
  }

  /// Access to internal buffer raw.
  unsigned char * Raw()
  {
    return m_Buffer.empty() ? 0 : &m_Buffer[ 0 ];
  }

  /// Access to internal buffer raw.
  const unsigned char * Raw() const
  {
    return m_Buffer.empty() ? 0 : &m_Buffer[ 0 ];
  }

  /// Return buffer length.
  unsigned int Length() const
  {
    return static_cast< unsigned int >( m_Buffer.size() );
  }
  
  /// Set buffer length.
  void SetLength( unsigned int i_Length )
  {
    m_Buffer.resize( i_Length );
    m_Offset = ( std::min )( m_Offset, i_Length );
  }
  
  /// Clear buffer.
  void Clear()
  {
    m_Buffer.clear();
    m_Offset = 0; 
  }

  /// Get parent buffer object.
  const VBuffer & Parent() const
  {
    return m_rParent;
  }

  /// Get parent buffer object.
  VBuffer & Parent()
  {
    return m_rParent;
  }

private:
  VBuffer & m_rParent;
  mutable unsigned int m_Offset;

  std::vector< unsigned char > m_Buffer;
};

//-------------------------------------------------------------------------------------------------
};


