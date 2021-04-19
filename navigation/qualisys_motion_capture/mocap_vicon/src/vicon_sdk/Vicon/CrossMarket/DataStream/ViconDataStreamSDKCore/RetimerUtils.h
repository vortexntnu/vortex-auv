
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

#include <boost/array.hpp>

namespace ClientUtils
{
  typedef boost::array< double, 4 > Quaternion;
  typedef boost::array< double, 9 > RotationMatrix;
  typedef boost::array< double, 3 > Axis;
  typedef boost::array< double, 3 > Displacement;

  // Simple Quaternion and Vector operation, to remove vnl/tmv dependency
  Quaternion Conjugate( const Quaternion & i_rInput );
  Quaternion Inverse( const Quaternion & i_rInput );

  Axis Imaginary( const Quaternion & i_rInput );
  double Magnitude( const Axis & i_rInput );
  void Normalize( Axis & io_rInput );

  bool ToAxisAngle( const Quaternion & i_rInput, Axis & o_rAxis, double & o_rAngle );
  Quaternion FromAxisAngle( const Axis & i_rAxis, const double & i_rAngle );
  RotationMatrix ToRotationMatrix( const Quaternion & i_rInput );

  Axis operator*( const Axis & i_rLeft, double i_rVal );
  Axis operator+=( Axis & i_rLeft, const Axis & i_rRight );
  Quaternion operator*( const Quaternion & i_rLeft, const Quaternion & i_rRight );

  /// Returns a rotation prediction at time t3 from 2 rotation r1 at time t1 and r2 and time t2
  /// where t3 > t2 > t1
  Quaternion PredictRotation( Quaternion r1, double t1, Quaternion r2, double t2, double t3 );

  /// Returns a translation prediction at time t3 from 2 translations at time d1, t1; d2, t2 where t3 > t2 > t1 
  /// Can also be used for linear interpolation where t2 > t3 > t1
  Displacement PredictDisplacement( const Displacement & d1, double t1, const Displacement & d2, double t2, double t3 );
}


