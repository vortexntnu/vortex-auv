
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
#include <memory>

namespace ViconDataStreamSDK
{
namespace Core
{

class VAxisMapping
{
public:
  virtual ~VAxisMapping();

  // Create a mapper and set the transformation matrix
  // Returns a null shared_ptr if there is an error in the axis system
  static std::shared_ptr< VAxisMapping > Create( AxisMappingResult::Enum & o_rError,
                                                   Direction::Enum           i_XAxis,
                                                   Direction::Enum           i_YAxis,
                                                   Direction::Enum           i_ZAxis );

  // Set the transform properties
  void GetAxisMapping( Direction::Enum & o_rXAxis, Direction::Enum & o_rYAxis, Direction::Enum & o_rZAxis ) const;

  // Transform world points and rotations
  void CopyAndTransformT( const double i_Translation[ 3 ], double ( & io_Translation )[ 3 ] ) const;
  void CopyAndTransformR( const double i_Rotation[ 9 ], double ( & io_Rotation )[ 9 ] ) const;

  void CopyAndTransformT( const double i_Translation[3], const double i_Orientation[9], double( &io_Translation )[3] ) const;
  void CopyAndTransformR( const double i_Rotation[9], const double i_Orientation[9], double( &io_Rotation )[9] ) const;

  // Get the underlying transformation matrix
  void GetTransformationMatrix( double( &o_Rotation )[9] );

private:
  VAxisMapping();
  
  // Set the transform matrix
  AxisMappingResult::Enum SetAxisMapping( const Direction::Enum i_XAxis,
                                          const Direction::Enum i_YAxis,
                                          const Direction::Enum i_ZAxis );

  // Raw axis mappings and equivalent transform matrix
  Direction::Enum   m_XAxis;
  Direction::Enum   m_YAxis;
  Direction::Enum   m_ZAxis;
  double m_Transform[ 9 ];
};

} // End of namespace Core
} // End of namespace ViconDataStreamSDK
