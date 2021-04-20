
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

#include "BufferImpl.h"
#include "Type.h"
#include <string>

namespace ViconCGStreamIO
{
//-------------------------------------------------------------------------------------------------

template< typename T >
struct VIsPod
{
  enum { Answer = 0 };
};

template< typename T >
struct VIsPod< const T >
{
  enum { Answer = VIsPod< T >::Answer };
};

template<>
struct VIsPod< bool >
{
  enum { Answer = 1 };
};

template<>
struct VIsPod< char >
{
  enum { Answer = 1 };
};

template<>
struct VIsPod< unsigned char >
{
  enum { Answer = 1 };
};

template<>
struct VIsPod< short >
{
  enum { Answer = 1 };
};

template<>
struct VIsPod< unsigned short >
{
  enum { Answer = 1 };
};

template<>
struct VIsPod< int >
{
  enum { Answer = 1 };
};

template<>
struct VIsPod< unsigned int >
{
  enum { Answer = 1 };
};

template<>
struct VIsPod< long >
{
  enum { Answer = 1 };
};

template<>
struct VIsPod< unsigned long >
{
  enum { Answer = 1 };
};

template<>
struct VIsPod< long long >
{
  enum { Answer = 1 };
};

template<>
struct VIsPod< unsigned long long >
{
  enum { Answer = 1 };
};

template<>
struct VIsPod< float >
{
  enum { Answer = 1 };
};

template<>
struct VIsPod< double >
{
  enum { Answer = 1 };
};

//-------------------------------------------------------------------------------------------------

template< int IsPod >
class VBufferDetail;

template<>
class VBufferDetail< 1 >
{
public:
  template< typename T >
  static bool Read( const VBufferImpl & i_rBufferImpl, T & o_rValue )
  {
    return i_rBufferImpl.ReadPod( o_rValue );
  }
  
  template< typename T >
  static bool Read( const VBufferImpl & i_rBufferImpl, T * o_pValues, unsigned int i_Size )
  {
    return i_rBufferImpl.ReadPodArray( o_pValues, i_Size );
  }

  template< typename T >
  static void Write( VBufferImpl & i_rBufferImpl, const T & i_rValue )
  {
    i_rBufferImpl.WritePod( i_rValue );
  }
  
  template< typename T >
  static void Write( VBufferImpl & i_rBufferImpl, const T * i_pValues, unsigned int i_Size )
  {
    i_rBufferImpl.WritePodArray( i_pValues, i_Size );
  }
};  

template<>
class VBufferDetail< 0 >
{
public:

  template< typename T >
  static bool Read( const VBufferImpl & i_rBufferImpl, T & o_rValue )
  {
    return o_rValue.Read( i_rBufferImpl.Parent() );
  }
  
  template< typename T >
  static bool Read( const VBufferImpl & i_rBufferImpl, T * o_pValues, unsigned int i_Size )
  {
    for( unsigned int Index = 0; Index != i_Size; ++Index )
    {
      if( !Read( i_rBufferImpl, o_pValues[ Index ] ) )
      {
        return false;
      }
    }
    return true;
  }
  
  static bool Read( const VBufferImpl & i_rBufferImpl, std::string & o_rValue )
  {
    ViconCGStreamType::UInt32 Size = 0;
    if( !i_rBufferImpl.ReadPod( Size ) )
    {
      return false;
    }
    const unsigned int Offset = i_rBufferImpl.Offset();
    if( Offset + Size > i_rBufferImpl.Length() )
    {
      return false;
    }
    o_rValue.assign( reinterpret_cast< const char * >( i_rBufferImpl.Raw() + Offset ), Size );
    i_rBufferImpl.SetOffset( Offset + Size );
    return true;
  }

  template< typename VBufferImpl, typename T >
  static void Write( VBufferImpl & i_rBufferImpl, const T & i_rValue )
  {
    i_rValue.Write( i_rBufferImpl.Parent() );
  }
  
  template< typename VBufferImpl, typename T >
  static void Write( VBufferImpl & i_rBufferImpl, const T * i_pValues, unsigned int i_Size )
  {
    for( unsigned int Index = 0; Index != i_Size; ++Index )
    {
      Write( i_rBufferImpl, i_pValues[ Index ] );
    }
  }

  static void Write( VBufferImpl & i_rBufferImpl, const std::string & i_rValue )
  {
    i_rBufferImpl.WritePod< ViconCGStreamType::UInt32 >( static_cast< ViconCGStreamType::UInt32 >( i_rValue.size() ) );
    i_rBufferImpl.WritePodArray( i_rValue.c_str(), static_cast< unsigned int >( i_rValue.size() ) );
  }

};

//-------------------------------------------------------------------------------------------------
};


