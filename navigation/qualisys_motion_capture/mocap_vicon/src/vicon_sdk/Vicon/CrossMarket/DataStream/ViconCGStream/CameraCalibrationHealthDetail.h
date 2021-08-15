
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
/// Contains the Detail of the ViconCGStream::VCameraCalirationHealth_Camera class

#include "Enum.h"
#include <StreamCommon/Buffer.h>

#include <vector>

namespace ViconCGStreamDetail
{
//-------------------------------------------------------------------------------------------------

/// Contains the Camera ID and Calibration Health for a single camera
class VCameraCalibrationHealth_Camera
{
public:

  /// Device identifier
  ViconCGStreamType::UInt32 m_DeviceID;

  /// Health Indicators
  ///
  /// All health indicators are computed over a variable time window
  /// controlled by the underlying generator
  ///
  /// The underlying generator will currently use only LABELLED reconstructions
  /// if available. Potentially model markers could be used. 
  ///   
  /// m_ReconUsage is the proportion of reconstructions visible in the 
  /// frustrum of the camera, that have associated centroids
  /// varies between 0 and 1 (full usage)
  ViconCGStreamType::Double m_ReconUsage;
  
  /// m_AverageReprojectionError - average distance between 
  ///  centroid and projected reconstruction for this camera
  ViconCGStreamType::Double m_AverageReprojectionError;

  /// m_NReconsInFrustrum  - the number of reconstructions visible 
  /// in the frustrum of the camera
  ViconCGStreamType::Double m_NReconsInFrustrum;

  /// m_NReprojections - the number of 2d-3d correspondences used to 
  ///  to compute the m_AverageReprojectionError score
  ViconCGStreamType::Double m_NReprojections;

  /// experimental - the average number of unused centroids per frame
  ViconCGStreamType::Double m_UnusedCentroids;


  /// Equality operator
  bool operator == ( const VCameraCalibrationHealth_Camera & i_rOther ) const
  {
    return    m_DeviceID == i_rOther.m_DeviceID 
           && m_ReconUsage == i_rOther.m_ReconUsage
           && m_AverageReprojectionError == i_rOther.m_AverageReprojectionError
           && m_NReconsInFrustrum == i_rOther.m_NReconsInFrustrum
           && m_NReprojections == i_rOther.m_NReprojections
           && m_UnusedCentroids == i_rOther.m_UnusedCentroids;
  }
  
  /// Read function.
  bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_DeviceID ) && 
           i_rBuffer.Read( m_ReconUsage ) && 
           i_rBuffer.Read( m_AverageReprojectionError ) && 
           i_rBuffer.Read( m_NReconsInFrustrum ) && 
           i_rBuffer.Read( m_NReprojections ) && 
           i_rBuffer.Read( m_UnusedCentroids );
  }

  /// Write function.
  void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_DeviceID );
    i_rBuffer.Write( m_ReconUsage );
    i_rBuffer.Write( m_AverageReprojectionError );
    i_rBuffer.Write( m_NReconsInFrustrum );
    i_rBuffer.Write( m_NReprojections );
    i_rBuffer.Write( m_UnusedCentroids );
  }

};

//-------------------------------------------------------------------------------------------------
};

