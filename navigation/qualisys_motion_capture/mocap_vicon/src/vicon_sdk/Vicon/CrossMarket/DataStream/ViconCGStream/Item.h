
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

#include "Enum.h"
#include <StreamCommon/Buffer.h>
#include "IsEqual.h"

namespace ViconCGStream
{
//-------------------------------------------------------------------------------------------------

const ViconCGStreamType::Int32 FILTER_NA = -1;

/// This is the base class for all top level items
class VItem
{
public:
  
  /// Type enum
  virtual ViconCGStreamType::Enum TypeID() const = 0;
  
  /// The value used to filter items belonging to a logical group
  /// Examples of return values are CameraID, DeviceID, SubjectID, etc.
  /// Top-level items that have no affinity with logical children should return FILTER_NA 
  virtual ViconCGStreamType::UInt32 FilterID() const = 0;

  /// Read function
  virtual bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer ) = 0;
  
  // Write function
  virtual void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const = 0;
  
  // Virtual destructor
  virtual ~VItem()
  {
  }
};

//-------------------------------------------------------------------------------------------------
};
