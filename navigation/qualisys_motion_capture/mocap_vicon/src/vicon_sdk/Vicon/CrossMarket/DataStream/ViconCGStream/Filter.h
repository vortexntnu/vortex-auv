
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
/// Adds finer grained control to which items are sent from the CGServer.  For example,
/// if Centroid data is requested, data for all cameras will be sent by default.  Use a 
/// filter to limit the data to a subset of cameras, and conserve network bandwidth and CPU load

#include "Item.h"
#include <set>
#include <map>

namespace ViconCGStream
{
  //-------------------------------------------------------------------------------------------------

  class VFilter : public VItem
  {
  public:

    // Maps Item type to a list of allowed instance IDs (i.e. CameraID, DeviceID, SubjectID, etc.)
    typedef std::map< ViconCGStreamType::Enum, std::set< ViconCGStreamType::UInt32 > > TFilterMap;

    // Add a filter id that will pass the filter
    void Add( ViconCGStreamType::Enum i_Type, ViconCGStreamType::UInt32 i_Filter )
    {
      m_FilterMap[ i_Type ].insert( i_Filter );
    }

    // Remove a previously added filter
    void Remove( ViconCGStreamType::Enum i_Type, ViconCGStreamType::UInt32 i_Filter )
    {
      TFilterMap::iterator itr = m_FilterMap.find( i_Type );
      if ( itr != m_FilterMap.end() )
      {
        itr->second.erase( i_Filter );
      }
    }

    // Clear all filters for an item type
    void Clear( ViconCGStreamType::Enum i_Type )
    {
      TFilterMap::iterator itr = m_FilterMap.find( i_Type );
      if ( itr != m_FilterMap.end() )
      {
        m_FilterMap.erase( itr );
      }
    }

    // Reset the filter to empty
    void ClearAll()
    {
      m_FilterMap.clear();
    }

    // Returns true if the item is allowed to be sent (meaning either the 
    // item filter ID is found, or no filter is set for the type id, allowing
    // it by default )
    bool Allow( const VItem & i_rItem )
    {
      TFilterMap::iterator iItems = m_FilterMap.find( i_rItem.TypeID() );
      if ( iItems != m_FilterMap.end() )
      {
        std::set< ViconCGStreamType::UInt32 > & Allowed = iItems->second;
        return std::find( Allowed.begin(), Allowed.end(), i_rItem.FilterID() ) != Allowed.end();
      }

      // not found, allow by default
      return true;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////
    // VItem

    /// Type enum
    virtual ViconCGStreamType::Enum TypeID() const
    {
      return ViconCGStreamEnum::Filter;
    }

    /// Filter ID 
    virtual ViconCGStreamType::UInt32 FilterID() const
    {
      return FILTER_NA;
    }

    /// Read function
    virtual bool Read( const ViconCGStreamIO::VBuffer & i_rBuffer )
    {
      return i_rBuffer.Read( m_FilterMap );
    }

    // Write function
    virtual void Write( ViconCGStreamIO::VBuffer & i_rBuffer ) const
    {
      return i_rBuffer.Write( m_FilterMap );
    }


  private:
    
    TFilterMap m_FilterMap;
  };

}
