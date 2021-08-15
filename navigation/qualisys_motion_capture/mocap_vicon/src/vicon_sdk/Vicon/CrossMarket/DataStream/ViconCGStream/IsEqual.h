
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

namespace ViconCGStreamDetail
{
//-------------------------------------------------------------------------------------------------

template< unsigned int N >
struct VDetail
{
  template< typename T >
  static bool IsEqual( const T * i_pLeft, const T * i_pRight )
  {
    return *i_pLeft == *i_pRight && VDetail< N - 1 >::IsEqual( i_pLeft + 1, i_pRight + 1 );
  }
};

template<>
struct VDetail< 1 >
{
  template< typename T >
  static bool IsEqual( const T * i_pLeft, const T * i_pRight )
  {
    return *i_pLeft == *i_pRight;
  }
};

template< typename T, unsigned int N >
bool IsEqual( const T ( & i_rpLeft )[ N ], const T ( & i_rpRight )[ N ] )
{
  return VDetail< N >::IsEqual( i_rpLeft, i_rpRight );
}

//-------------------------------------------------------------------------------------------------
};
