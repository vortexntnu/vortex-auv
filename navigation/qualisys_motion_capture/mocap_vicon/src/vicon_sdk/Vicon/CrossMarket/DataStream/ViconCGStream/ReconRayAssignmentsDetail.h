
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
/// Contains the declaration of the ViconCGStreamDetail::VRayAssignments classes.

#include "Enum.h"
#include <StreamCommon/Buffer.h>
#include <vector>

namespace ViconCGStreamDetail
{
//-------------------------------------------------------------------------------------------------

/// (Used int VReconRayAssignments) Contains all the ray assignments for a single recon
class VReconRay
{
public:

  /// Camera frame number
  ViconCGStreamType::UInt32 m_FrameID;
  
  /// Camera ID of the Ray
  ViconCGStreamType::UInt32 m_CameraID;

  /// Index into centroid array for the camera frame
  ViconCGStreamType::UInt32 m_CentroidIndex;

  /// Equality operator
  bool operator == ( const VReconRay & i_rOther ) const
  {
    return m_FrameID == i_rOther.m_FrameID &&
           m_CameraID == i_rOther.m_CameraID &&
           m_CentroidIndex == i_rOther.m_CentroidIndex;
  }

  /// Read function.
  bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_FrameID ) &&
           i_rBuffer.Read( m_CameraID ) &&
           i_rBuffer.Read( m_CentroidIndex );
  }
  
  /// Write function.
  void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_FrameID );
    i_rBuffer.Write( m_CameraID );
    i_rBuffer.Write( m_CentroidIndex );
  }
};

/// (Used int VUnlabeledReconRayAssigments and VLabeledReconRayAssigments) Contains all the ray assignments for a single recon
class VReconRayAssignments
{
public:

  /// Subject identifier
  ViconCGStreamType::UInt32 m_SubjectID;

  /// Marker identifier (for subject)
  ViconCGStreamType::UInt32 m_MarkerID;

  /// Vector of ray identifiers
  std::vector< VReconRay > m_ReconRays;

  /// Equality operator
  bool operator == ( const VReconRayAssignments & i_rOther ) const
  {
    return m_SubjectID == i_rOther.m_SubjectID &&
           m_MarkerID == i_rOther.m_MarkerID && 
           m_ReconRays == i_rOther.m_ReconRays;
  }

  /// Read function.
  bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_SubjectID ) &&
           i_rBuffer.Read( m_MarkerID ) && 
           i_rBuffer.Read( m_ReconRays );
  }
  
  /// Write function.
  void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_SubjectID );
    i_rBuffer.Write( m_MarkerID );
    i_rBuffer.Write( m_ReconRays );
  }
};

//-------------------------------------------------------------------------------------------------
}


