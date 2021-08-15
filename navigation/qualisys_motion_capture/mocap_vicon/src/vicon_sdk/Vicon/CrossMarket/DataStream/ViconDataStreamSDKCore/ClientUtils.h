
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


#include "Constants.h"

#include <boost/array.hpp>
#include <vector>

namespace ClientUtils
{
  // These functions clear a value
  void Clear( bool & o_rValue );

  void Clear( unsigned int & o_rValue );

  void Clear( double & o_rValue );

  void Clear( std::string & o_rValue );

  void Clear( ViconDataStreamSDK::Core::TimecodeStandard::Enum & o_rValue );

  void Clear( ViconDataStreamSDK::Core::DeviceType::Enum & o_rValue );

  void Clear( ViconDataStreamSDK::Core::Unit::Enum & o_rValue );

  template< typename T, unsigned int N >
  void Clear( T (&o_rValue)[ N ] )
  {
    for( unsigned int i = 0 ; i < N ; ++i )
    {
      Clear( o_rValue[ i ] );
    }
  }

  template< typename T >
  void Clear( std::vector< T > & o_rValue )
  {
    o_rValue.clear();
  }

  // Determine whether an axis is positive or negative
  double ComponentSign( ViconDataStreamSDK::Core::Direction::Enum i_Direction );

  // Determine the index of an enum
  unsigned int ComponentIndex( ViconDataStreamSDK::Core::Direction::Enum i_Direction );

  // Test description of component enum
  std::string ComponentName( ViconDataStreamSDK::Core::Direction::Enum i_Direction );

  std::string AdaptCameraName( const std::string & i_rCameraName,
                               const std::string & i_rDisplayType,
                               const unsigned int  i_CameraID );

  std::string AdaptDeviceName( const std::string & i_rDeviceName, 
                               const unsigned int  i_DeviceID );

  std::string AdaptDeviceOutputName( const std::string & i_rDeviceOutputName, 
                                     const unsigned int  i_DeviceOutputIndex );

  bool IsValidMulticastIP( const std::string & i_MulticastIPAddress );

  void MatrixToQuaternion( const double i_M[9], double (&o_Q)[4]);
  
  void MatrixToHelical( const double i_rM[9], double (&o_rAA)[3]);

  void MatrixToEulerXYZ( const double i_M[9], double (&o_rE)[3]);

  // Insanely simplistic operators for basic vector operations on 
  // boost arrays. Arrays of length 9 are assumed to be 3x3 matrices.
  // Arrays of length 3 are assumed to be 3x1 vectors.
  // Only the handful of operations required by Client and AxisMapping have been provided.

  boost::array< double, 3 > operator*( const boost::array< double, 9 > & i_rM, const boost::array< double, 3 > & i_rX );

  boost::array< double, 3 > & operator/=( boost::array< double, 3 > & i_rX, double i_Val );

  boost::array< double, 3 > & operator*=( boost::array< double, 3 > & i_rX, double i_Val );

  boost::array< double, 3 > operator+( const boost::array< double, 3 > & i_rX, double i_Val );

  boost::array< double, 3 > operator+( const boost::array< double, 3 > & i_rX, const boost::array< double, 3 > & i_rY );

  boost::array< double, 3 > operator-( const boost::array< double, 3 > & i_rX, const boost::array< double, 3 > & i_rY );
    
  boost::array< double, 9 > operator*( const boost::array< double, 9 > & i_rM, const boost::array< double, 9 > & i_rN );

  boost::array< double, 9 > Transpose( const boost::array< double, 9 > & i_rM );

  boost::array< double, 16 > Transpose( const boost::array< double, 16 > & i_rM );

  boost::array< double, 3 > CrossProduct( boost::array< double, 3 > & i_rA, boost::array< double, 3 > & i_rB );

  double DotProduct( boost::array< double, 3 > & i_rA, boost::array< double, 3 > & i_rB );
}
