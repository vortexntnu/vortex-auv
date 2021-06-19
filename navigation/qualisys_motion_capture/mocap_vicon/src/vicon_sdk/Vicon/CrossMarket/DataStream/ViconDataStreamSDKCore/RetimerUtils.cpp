
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
#include "RetimerUtils.h"
#include "ClientUtils.h"

#include <boost/math/constants/constants.hpp>

namespace bmc = boost::math::constants;

namespace ClientUtils
{
  typedef boost::array< double, 4 > Quaternion;
  typedef boost::array< double, 3 > Axis;
  typedef boost::array< double, 3 > Displacement;

  Quaternion Conjugate( const Quaternion & i_rInput )
  {
    Quaternion conj = { -i_rInput[0], -i_rInput[1], -i_rInput[2], i_rInput[3] };
    return conj;
  }

  Quaternion Inverse( const Quaternion & i_rInput )
  {
    Quaternion inv = Conjugate( i_rInput );
    double DotProd = 0;
    for( unsigned i = 0; i < 4; ++i ) DotProd += ( i_rInput[i] * i_rInput[i] );
    for( unsigned i = 0; i < 4; ++i ) inv[i] /= DotProd;
    return inv;
  }

  Axis Imaginary( const Quaternion & i_rInput )
  {
    Axis axis = { i_rInput[0], i_rInput[1], i_rInput[2] };
    return axis;
  }

  double Magnitude( const Axis & i_rInput )
  {
    double mag = 0;
    for( unsigned int i = 0; i < 3; ++i )
    {
      mag += i_rInput[i] * i_rInput[i];
    }
    return sqrt( mag );
  }


  void Normalize( Axis & io_rInput )
  {
    double Mag = Magnitude( io_rInput );
    io_rInput /= Mag;
  }

  bool ToAxisAngle( const Quaternion & i_rInput, Axis & o_rAxis, double & o_rAngle )
  {
    bool success = true;

    // angle is always positive
    Axis imag = Imaginary( i_rInput );
    double mag = Magnitude( imag );
    o_rAngle = 2 * std::atan2( mag, i_rInput[3] );
    
    // direc parallel to imag. part
    o_rAxis = imag;
    if( mag == 0.0 ) 
    {
      // or signal exception here.
      o_rAxis[2] = 1.0;                    
      success = false;
    }
    else
    {
      // normalize direction vector
      for( unsigned int i = 0; i < 3; ++i ) o_rAxis[i] /= mag;                       
    }
    return success;
  }

  Quaternion FromAxisAngle( const Axis & i_rAxis, const double & i_rAngle )
  {
    // This should take a normalized vector
    Axis AxisCopy = i_rAxis;
    Normalize( AxisCopy );

    Quaternion output;

    // half angle
    double a = i_rAngle * 0.5;  
    double s = std::sin( a );
    
    for( int i = 0; i < 3; i++ )            
    {
      // imaginary vector is sine of half angle multiplied with axis
      output[i] = s * AxisCopy[i];
    }

    // real part is cosine of half angle
    output[3] = std::cos( a );   

    return output;
  }

  RotationMatrix ToRotationMatrix( const Quaternion & i_rInput )
  {
    double x2 = i_rInput[0] * i_rInput[0];
    double xy = i_rInput[0] * i_rInput[1];
    double rx = i_rInput[3] * i_rInput[0];
    double y2 = i_rInput[1] * i_rInput[1];
    double yz = i_rInput[1] * i_rInput[2];
    double ry = i_rInput[3] * i_rInput[1];
    double z2 = i_rInput[2] * i_rInput[2];
    double zx = i_rInput[2] * i_rInput[0]; 
    double rz = i_rInput[3] * i_rInput[2];
    double r2 = i_rInput[3] * i_rInput[3];

    RotationMatrix Output;
    // fill diagonal terms 
    Output[ 0 ] = r2 + x2 - y2 - z2;    //(0,0)
    Output[ 4 ] = r2 - x2 + y2 - z2; // (1,1)
    Output[ 8 ] = r2 - x2 - y2 + z2; // (2,2)

    // fill off diagonal terms (output not transposed; differing from vnl_quaternion::rotation_matrix_transposed
    Output[ 3 ] = 2 * ( xy + rz ); // (1,0)            
    Output[ 6 ] = 2 * ( zx - ry ); // (2,0)
    Output[ 7 ] = 2 * ( yz + rx ); // (2,1)
    Output[ 1 ] = 2 * ( xy - rz ); // (0,1)
    Output[ 2 ] = 2 * ( zx + ry ); // (0,2)
    Output[ 5 ] = 2 * ( yz - rx ); // (1,2)

    return Output;
  }

  Axis operator*( const Axis & i_rLeft, double i_rVal )
  {
    Axis Result;
    std::transform( i_rLeft.begin(), i_rLeft.end(), Result.begin(), [&]( double x ){ return x * i_rVal; } );
    return Result;
  }

  Axis operator+=( Axis & i_rLeft, const Axis & i_rRight )
  {
    std::transform( i_rLeft.begin(), i_rLeft.end(), i_rRight.begin(), i_rLeft.begin(), []( double x, double y ){ return x + y; } );
    return i_rLeft;
  }

  Quaternion operator*( const Quaternion & i_rLeft, const Quaternion & i_rRight )
  {
    Quaternion output;
    double r1 = i_rLeft[3];                  // real and img parts of args
    double r2 = i_rRight[3];
    Axis i1 = Imaginary( i_rLeft );
    Axis i2 = Imaginary( i_rRight );
    double real_v = ( r1 * r2 ) - DotProduct( i1, i2 ); // real&img of product q1*q2
    Axis img = CrossProduct( i1, i2 );
    Axis i2r1 = i2*r1;
    Axis i1r2 = i1*r2;
    Axis i2r1i1r2 = i2r1 + i1r2;
    img += i2r1i1r2;
    output = { img[0], img[1], img[2], real_v };
    return output;
  }

  // Simple interpolation using linear interpolation and prediction
  /// Returns a rotation prediction at time t3 from 2 rotation r1 at time t1 and r2 and time t2
  /// where t3 > t2 > t1
  // From http://answers.unity3d.com/questions/168779/extrapolating-quaternion-rotation.html
  Quaternion PredictRotation( Quaternion r1, double t1, Quaternion r2, double t2, double t3 )
  {
    Quaternion rot = r2 * Inverse( r1 ); // rot is rotation from t1 to t2

    double dt = ( t3 - t1 ) / ( t2 - t1 ); // dt = extrapolation factor

    // Convert to Axis/Angle
    Axis axis;
    double angle;
    if( !ToAxisAngle( rot, axis, angle ) )
    {
      // Check the magnitude of the imaginary part is not zero, as then
      // the axis of the quaternion will not be well defined.
      return r1;
    }

    // Assume shortest path
    if( angle > bmc::pi< double >() ) angle -= ( 2 * bmc::pi< double >() );

    // Multiple angle by extrapolation factor
    angle = fmod( angle * dt, ( 2 * bmc::pi< double >() ) );

    // Combine with first rotation
    Quaternion r3 = FromAxisAngle( axis, angle ) * r1;

    return r3;
  }

  /// Returns a translation prediction at time t3 from 2 translations at time d1, t1; d2, t2 where t3 > t2 > t1 
  /// Can also be used for linear interpolation where t2 > t3 > t1
  Displacement PredictDisplacement( const Displacement & d1, double t1, const Displacement & d2, double t2, double t3 )
  {

    // disp is displacement from t1 to t2
    Displacement disp( d2 - d1 );

    // extrapolation factor
    double dt = ( t3 - t1 ) / ( t2 - t1 );

    // multiply displacement by extrapolation factor
    disp *= dt;

    // Combine with first displacement
    Displacement d3 = d1 + disp;

    return d3;
  }
}


