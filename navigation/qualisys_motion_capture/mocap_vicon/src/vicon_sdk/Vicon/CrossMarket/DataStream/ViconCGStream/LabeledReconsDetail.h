
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
/// Contains the detail declaration of the ViconCGStream::VLabeledRecons class.

#include "Enum.h"
#include <StreamCommon/Buffer.h>
#include "IsEqual.h"

namespace ViconCGStreamDetail
{
//-------------------------------------------------------------------------------------------------

/**
 * Contains a single labeled recon.
 */
class VLabeledRecons_LabeledRecon
{
public:

  /// Subject identifier
  ViconCGStreamType::UInt32 m_SubjectID;

  /// Marker identifier (for subject)
  ViconCGStreamType::UInt32 m_MarkerID;

  /// Recon radius
  ViconCGStreamType::Double m_Radius;

  /// Recon position
  ViconCGStreamType::Double m_Position[ 3 ];

  /// Recon covariance matrix (row major format)
  ViconCGStreamType::Double m_Covariance[ 9 ];

  /// Trajectory id
  ViconCGStreamType::UInt32 m_TrajectoryId;
  
  /// Equality operator
  bool operator == ( const VLabeledRecons_LabeledRecon & i_rOther ) const
  {
    return m_SubjectID == i_rOther.m_SubjectID && 
           m_MarkerID == i_rOther.m_MarkerID && 
           m_Radius == i_rOther.m_Radius &&
           IsEqual( m_Position, i_rOther.m_Position ) &&
           IsEqual( m_Covariance, i_rOther.m_Covariance ) &&
           m_TrajectoryId == i_rOther.m_TrajectoryId;
  }
  
  /// Read function.
  bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_SubjectID ) &&
           i_rBuffer.Read( m_MarkerID ) &&
           i_rBuffer.Read( m_Radius ) &&
           i_rBuffer.Read( m_Position ) &&
           i_rBuffer.Read( m_Covariance ) &&
           i_rBuffer.Read( m_TrajectoryId );
  }
    
  /// Write function.
  void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_SubjectID );
    i_rBuffer.Write( m_MarkerID );
    i_rBuffer.Write( m_Radius );
    i_rBuffer.Write( m_Position );
    i_rBuffer.Write( m_Covariance );
    i_rBuffer.Write( m_TrajectoryId );
  }
  
};

//-------------------------------------------------------------------------------------------------
};

