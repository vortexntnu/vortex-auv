
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
#include "AxisMapping.h"

#include "ClientUtils.h"

#include <vector>

using namespace ClientUtils;

namespace ViconDataStreamSDK
{
namespace Core
{

VAxisMapping::VAxisMapping()
{
}

VAxisMapping::~VAxisMapping()
{
}

std::shared_ptr< VAxisMapping > VAxisMapping::Create( AxisMappingResult::Enum & o_rError,
                                                        Direction::Enum           i_XAxis,
                                                        Direction::Enum           i_YAxis,
                                                        Direction::Enum           i_ZAxis )
{
  std::shared_ptr< VAxisMapping > pAxisMapping( new VAxisMapping() );

  o_rError = pAxisMapping->SetAxisMapping( i_XAxis, i_YAxis, i_ZAxis );
  if( o_rError != AxisMappingResult::Success )
  {
    pAxisMapping.reset();
  }

  return pAxisMapping;
}

AxisMappingResult::Enum VAxisMapping::SetAxisMapping( const Direction::Enum i_XAxis, 
                                                      const Direction::Enum i_YAxis, 
                                                      const Direction::Enum i_ZAxis )
{
  // Check for co-linear input
  {
    std::vector< unsigned int > Axes( 3, 0 );
    ++Axes[ ComponentIndex( i_XAxis ) ];
    ++Axes[ ComponentIndex( i_YAxis ) ];
    ++Axes[ ComponentIndex( i_ZAxis ) ];

    std::vector< unsigned int > ExpectedAxes( 3, 1 );

    if( Axes != ExpectedAxes )
    {
      return AxisMappingResult::CoLinearAxes;
    }
  }

  // Check that they do not specify a left handed co-ordinate system
  {
    boost::array< double, 3 > XAxis;
    std::fill( XAxis.begin(), XAxis.end(), 0.0 );
    XAxis[ ComponentIndex( i_XAxis ) ] = ComponentSign( i_XAxis );

    boost::array< double, 3 > YAxis;
    std::fill( YAxis.begin(), YAxis.end(), 0.0 );
    YAxis[ ComponentIndex( i_YAxis ) ] = ComponentSign( i_YAxis );

    boost::array< double, 3 > ZAxis;
    std::fill( ZAxis.begin(), ZAxis.end(), 0.0 );
    ZAxis[ ComponentIndex( i_ZAxis ) ] = ComponentSign( i_ZAxis );

    // In a right handed coordinate system, ZAxis == XAxis x YAxis
    if( ZAxis != ( CrossProduct( XAxis,  YAxis ) ) )
    {
      return AxisMappingResult::LeftHandedAxes;
    }
  }

  // Make a transform matrix
  std::fill( m_Transform, m_Transform + 9, 0.0 );
  m_Transform[ 0 + ComponentIndex( i_XAxis ) ] = ComponentSign( i_XAxis );
  m_Transform[ 3 + ComponentIndex( i_YAxis ) ] = ComponentSign( i_YAxis );
  m_Transform[ 6 + ComponentIndex( i_ZAxis ) ] = ComponentSign( i_ZAxis );

  // Store away the directions in case anyone asks us later
  m_XAxis = i_XAxis;
  m_YAxis = i_YAxis;
  m_ZAxis = i_ZAxis;

  return AxisMappingResult::Success;
}

void VAxisMapping::GetAxisMapping( Direction::Enum & o_rXAxis, Direction::Enum & o_rYAxis, Direction::Enum & o_rZAxis ) const
{
  o_rXAxis = m_XAxis;
  o_rYAxis = m_YAxis;
  o_rZAxis = m_ZAxis;
}

void VAxisMapping::CopyAndTransformT( const double i_Translation[ 3 ], double ( & io_Translation )[ 3 ] ) const
{
  boost::array< double, 9 > Tm;
  std::copy( m_Transform, m_Transform + 9, Tm.begin() );
  boost::array< double, 3 > T;
  std::copy( i_Translation, i_Translation + 3, T.begin() );
  boost::array< double, 3 > Result = Tm * T;
  std::copy( Result.begin(), Result.end(), &io_Translation[0] );
}

void VAxisMapping::CopyAndTransformR( const double i_Rotation[ 9 ], double ( & io_Rotation )[ 9 ] ) const
{
  boost::array< double, 9 > R;
  std::copy( i_Rotation, i_Rotation+9, R.begin() );
  boost::array< double, 9 > T;
  std::copy( m_Transform, m_Transform+9, T.begin() );
  boost::array< double, 9 > Result = T * R * Transpose( T );
  std::copy(Result.begin(), Result.end(), &io_Rotation[0] );
}

// From server to flow
// Axis(server).transpose * WorldPose(server) = Axis(server).transpose * Rotation(server) * LocalRotation(server) + Axis( server).transpose * Translation(server)
// Axis(server).transpose * WorldPose(server) = Axis(server).transpose * Rotation(server) * Axis(server) * Axis(server).transpose * LocalRotation(server) + Axis( server).transpose * Translation(server)
//                          WorldPose(server) = Axis(Server).transpose * Rotation(server) * Axis( server ) * LocaRotation(Flow) + Translation(flow)

// from flow to client
// Axis( client) * Axis(server).transpose * WorldPose(server) = Axis( Client) * Axis(server).transpose * Rotation(server) * Axis(server) * Axis(server).transpose * LocalRotation(server) + Axis( client ) * Axis( server).transpose * Translation(server)
// Axis( client) * Axis(server).transpose * WorldPose(server) = Axis( Client) * Axis(server).transpose * Rotation(server) * Axis(server) * Axis( client).transpose * Axis( client ) * Axis(server).transpose * LocalRotation(server) + Axis( client ) * Axis( server).transpose * Translation(server)
//                                          WorldPose( client) = Axis( Client) * Axis(server).transpose * Rotation(server) * Axis(server) * Axis( client).transpose * LocalRotation(server) + Translation(client)

void VAxisMapping::CopyAndTransformT( const double i_Translation[3], const double i_Orientation[9], double( &io_Translation )[3] ) const
{
  boost::array< double, 9 > Q;
  std::copy( i_Orientation, i_Orientation + 9, Q.begin() );
  boost::array< double, 9 > Tm;
  std::copy( m_Transform, m_Transform + 9, Tm.begin() );
  boost::array< double, 3 > T;
  std::copy( i_Translation, i_Translation + 3, T.begin() );
  boost::array< double, 3 > Result =  Tm * Transpose( Q ) * T;
  std::copy( Result.begin(), Result.end(), &io_Translation[0] );
}


void VAxisMapping::CopyAndTransformR( const double i_Rotation[9], const double i_Orientation[9], double( &io_Rotation )[9] ) const
{
  boost::array< double, 9 > Q;
  std::copy( i_Orientation, i_Orientation + 9, Q.begin() );
  boost::array< double, 9 > R;
  std::copy( i_Rotation, i_Rotation + 9, R.begin() );
  boost::array< double, 9 > T;
  std::copy( m_Transform, m_Transform + 9, T.begin() );
  boost::array< double, 9 > Result = T * Transpose( Q ) * R  * Q * Transpose( T ) ;
  std::copy( Result.begin(), Result.end(), &io_Rotation[0] );
}

void VAxisMapping::GetTransformationMatrix( double( &o_Rotation )[9] )
{
  std::copy( m_Transform, m_Transform + 9, o_Rotation );
}

} // End of namespace Core
} // End of namespace ViconDataStreamSDK
