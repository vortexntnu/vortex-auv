
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

#include "CGStreamAsyncReaderWriter.h"
#include <boost/bind.hpp>
#include <boost/ref.hpp>

#ifdef WIN32
#pragma warning(disable:4503) // warning C4503: decorated name length exceeded...
#endif

void VCGStreamAsyncReaderWriter::ReadBuffer(  boost::asio::ip::tcp::socket& i_rSocket,
                                              ViconCGStreamIO::VBuffer& i_rBuffer,
                                              boost::function< void( bool ) > i_Handler )
{
  i_rBuffer.SetLength( sizeof( ViconCGStreamType::Enum ) + sizeof( ViconCGStreamType::UInt32 ) );
  i_rBuffer.SetOffset( 0 );
  boost::asio::async_read(  i_rSocket,
                            boost::asio::buffer( i_rBuffer.Raw(), i_rBuffer.Length() ),
                            boost::bind( &VCGStreamAsyncReaderWriter::OnBufferHeaderRead, _1, _2,
                              boost::ref( i_rSocket ), boost::ref( i_rBuffer ), i_Handler ) );
}

void VCGStreamAsyncReaderWriter::WriteBuffer( boost::asio::ip::tcp::socket& i_rSocket,
                                              const ViconCGStreamIO::VBuffer& i_rBuffer,
                                              boost::function< void( bool ) > i_Handler )
{
  boost::asio::async_write( i_rSocket,
                            boost::asio::buffer( i_rBuffer.Raw(), i_rBuffer.Length() ),
                            boost::bind( &VCGStreamAsyncReaderWriter::OnBufferWritten, _1, _2, i_Handler ) );
}

void VCGStreamAsyncReaderWriter::SendBuffer( std::shared_ptr< boost::asio::ip::udp::socket > i_pSocket,
                                             const ViconCGStreamIO::VBuffer& i_rBuffer,
                                             boost::function< void( bool ) > i_Handler )
{
  i_pSocket->async_send( boost::asio::buffer( i_rBuffer.Raw(), i_rBuffer.Length() ),
                         boost::bind( &VCGStreamAsyncReaderWriter::OnBufferSent, i_pSocket, _1, _2, i_Handler ) );
}

void VCGStreamAsyncReaderWriter::SendBufferTo( std::shared_ptr< boost::asio::ip::udp::socket > i_pSocket,
                                             const ViconCGStreamIO::VBuffer& i_rBuffer,
                                             const boost::asio::ip::udp::endpoint& i_rEndpoint,
                                             boost::function< void( bool ) > i_Handler )
{
  i_pSocket->async_send_to( boost::asio::buffer( i_rBuffer.Raw(), i_rBuffer.Length() ), i_rEndpoint,
                         boost::bind( &VCGStreamAsyncReaderWriter::OnBufferSent, i_pSocket, _1, _2, i_Handler ) );
}

void VCGStreamAsyncReaderWriter::OnBufferHeaderRead(  const boost::system::error_code i_Error,
                                                      const std::size_t /*i_BytesRead*/,
                                                      boost::asio::ip::tcp::socket& i_rSocket,
                                                      ViconCGStreamIO::VBuffer& i_rBuffer,
                                                      boost::function< void( bool ) > i_Handler )
{
  if( i_Error )
  {
    i_Handler( false );
    return;
  }

  ViconCGStreamType::UInt32 BlockLength;
  i_rBuffer.SetOffset( sizeof( ViconCGStreamType::Enum ) );
  i_rBuffer.Read( BlockLength );
  i_rBuffer.SetLength( i_rBuffer.Length() + BlockLength );
  boost::asio::mutable_buffers_1 AsioBuffer(  boost::asio::buffer( i_rBuffer.Raw() + i_rBuffer.Offset(), BlockLength ) );
  i_rBuffer.SetOffset( 0 );
  boost::asio::async_read(  i_rSocket, AsioBuffer, boost::bind( &VCGStreamAsyncReaderWriter::OnBufferBodyRead, _1, _2, i_Handler ) );
}

void VCGStreamAsyncReaderWriter::OnBufferBodyRead(    const boost::system::error_code i_Error,
                                                      const std::size_t /*i_BytesRead*/,
                                                      boost::function< void( bool ) > i_Handler )
{
  i_Handler( !i_Error );
}

void VCGStreamAsyncReaderWriter::OnBufferWritten(     const boost::system::error_code i_Error,
                                                      const std::size_t /*i_BytesRead*/,
                                                      boost::function< void( bool ) > i_Handler )
{
  i_Handler( !i_Error );
}

void VCGStreamAsyncReaderWriter::OnBufferSent(        std::shared_ptr< boost::asio::ip::udp::socket > /*i_pSocket*/,
                                                      const boost::system::error_code i_Error,
                                                      const std::size_t /*i_BytesRead*/,
                                                      boost::function< void( bool ) > i_Handler )
{
  i_Handler( !i_Error );
}

