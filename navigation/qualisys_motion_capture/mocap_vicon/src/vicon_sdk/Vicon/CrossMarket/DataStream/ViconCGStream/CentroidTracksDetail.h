
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
/// Contains the details declaration of the ViconCGStream::VCentroidTracks class.

#include "Enum.h"
#include <StreamCommon/Buffer.h>

namespace ViconCGStreamDetail
{
//-------------------------------------------------------------------------------------------------

/// Contains the centroid index, track id, track length and track quality.
class VCentroidTracks_CentroidTrack
{
public:

  /// Centroid index 
  ViconCGStreamType::UInt32 m_CentroidIndex;

  /// 64 bit track id.
  ViconCGStreamType::UInt64 m_TrackId;

  /// Length of track (saturates).
  ViconCGStreamType::UInt32 m_TrackLength;

  /// Track quality.
  ViconCGStreamType::Float m_TrackQuality;
    
  // Equality operator
  bool operator == ( const VCentroidTracks_CentroidTrack & i_rOther ) const
  {
    return m_CentroidIndex == i_rOther.m_CentroidIndex &&
           m_TrackId == i_rOther.m_TrackId &&
           m_TrackLength == i_rOther.m_TrackLength &&
           m_TrackQuality == i_rOther.m_TrackQuality;
  }
  
  /// Read function.
  bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
  {
    return i_rBuffer.Read( m_CentroidIndex ) &&
           i_rBuffer.Read( m_TrackId ) &&
           i_rBuffer.Read( m_TrackLength ) &&
           i_rBuffer.Read( m_TrackQuality );
  }
    
  /// Write function.
  void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
  {
    i_rBuffer.Write( m_CentroidIndex );
    i_rBuffer.Write( m_TrackId );
    i_rBuffer.Write( m_TrackLength );
    i_rBuffer.Write( m_TrackQuality );
  }
};

//-------------------------------------------------------------------------------------------------
};

