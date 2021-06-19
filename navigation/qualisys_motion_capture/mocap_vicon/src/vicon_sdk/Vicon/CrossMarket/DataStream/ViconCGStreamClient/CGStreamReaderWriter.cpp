
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
#include "CGStreamReaderWriter.h"

VCGStreamReaderWriter::VCGStreamReaderWriter( std::shared_ptr< boost::asio::ip::tcp::socket > i_pSocket ) 
: m_pSocket( i_pSocket )
{
  // linger on shutdown a bit to ensure close packet arrives
  boost::system::error_code Error;
  m_pSocket->set_option( boost::asio::socket_base::linger( true, 10 ), Error );
}

VCGStreamReaderWriter::VCGStreamReaderWriter( std::shared_ptr< boost::asio::ip::udp::socket > i_pMulticastSocket ) 
: m_pMulticastSocket( i_pMulticastSocket  )
{
}

bool VCGStreamReaderWriter::DataReady( bool & o_rbDataReady ) const
{
  boost::asio::socket_base::bytes_readable Command( true );
  boost::system::error_code Error;
  
  if( m_pSocket->io_control( Command, Error ) )
  {
    return false;
  }
  
  size_t Bytes = Command.get();
  o_rbDataReady = ( Bytes != 0 );
  return true;
}

bool VCGStreamReaderWriter::IsOpen() const
{
  return m_pSocket->is_open();
}

void VCGStreamReaderWriter::Close()
{
  boost::system::error_code DontCareError;
  m_pSocket->shutdown( boost::asio::ip::tcp::socket::shutdown_both, DontCareError );
  m_pSocket->close();
}

bool VCGStreamReaderWriter::Fill()
{
  try
  {
    if( m_pMulticastSocket )
    {
      SetLength( 64 * 1024 );
      m_pMulticastSocket->receive( boost::asio::buffer( Raw(), Length() ) );
      SetOffset( 0 );
    }
    else
    {
      const unsigned int HeaderSize = sizeof( ViconCGStreamType::Enum ) + sizeof( ViconCGStreamType::UInt32 );
      SetLength( HeaderSize );
      SetOffset( 0 );
      boost::asio::read( *m_pSocket, boost::asio::buffer( Raw(), Length() ) );
      
      SetOffset( sizeof( ViconCGStreamType::Enum ) );
      ViconCGStreamType::UInt32 BlockLength;
      Read( BlockLength );
      
      SetLength( Length() + BlockLength );
      boost::asio::read( *m_pSocket, boost::asio::buffer( Raw() + HeaderSize, BlockLength ) );
      SetOffset( 0 );
    }

  } 
  catch( boost::system::system_error & rError )
  {
    std::string Error = rError.what();
    return false;
  }
  
  return true;
}

bool VCGStreamReaderWriter::Flush()
{
  // Will generate an error if called on when initialized with a multicast socket
  try
  {
    SetOffset( 0 );
    boost::asio::write( *m_pSocket, boost::asio::buffer( Raw(), Length() ) );
    Clear();
  } 
  catch( boost::system::system_error & rError )
  {
    std::string Error = rError.what();
    return false;
  }
  
  return true;
}

