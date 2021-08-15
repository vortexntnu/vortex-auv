
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
#include "ViconCGStreamBayer.h"

#include <cassert>
#include <cstring>

// Bayer pattern:
// 
// R G R G R G R G 
// G B G B G B G B
// R G R G R G R G 
// G B G B G B G B
// R G R G R G R G 
// G B G B G B G B
// R G R G R G R G 
// G B G B G B G B

void VViconCGStreamBayer::BayerGBToBGR( unsigned int i_Width, unsigned int i_Height, const unsigned char * i_pBayerData, unsigned char * o_pBGRData )
{
  assert( ( i_Width % 2 ) == 0 );
  assert( ( i_Height % 2 ) == 0 );

  std::memset( o_pBGRData, 0, i_Width * 3 );

  const unsigned char * pLineAbove = i_pBayerData;
  const unsigned char * pLine = i_pBayerData + i_Width;
  unsigned char * pBGR = o_pBGRData + i_Width * 3;

  for( unsigned int Y = 1; Y < i_Height; Y++ )
  {
    if( ( Y & 1 ) == 0 )
    {
      for( unsigned int X = 0; ( X + 1 ) < i_Width; X += 2 )
      {
        // Second line (odd lines): red, green
        pBGR[ 0 ] = pLineAbove[ 1 ];
        pBGR[ 1 ] = ( pLineAbove[ 0 ] + pLine[ 1 ] ) / 2;
        pBGR[ 2 ] = pLine[ 0 ];

        pLine++;
        pLineAbove++;
        pBGR += 3;

        pBGR[ 0 ] = pLineAbove[ 0 ];
        pBGR[ 1 ] = ( pLineAbove[ 1 ] + pLine[ 0 ] ) / 2;
        pBGR[ 2 ] = pLine[ 1 ];

        pLine++;
        pLineAbove++;
        pBGR += 3;
      }
    }
    else
    {
      for( unsigned int X = 0; ( X + 1 ) < i_Width; X += 2 )
      {
        // First line (even lines): green, blue
        pBGR[ 0 ] = pLine[ 1 ];
        pBGR[ 1 ] = ( pLine[ 0 ] + pLineAbove[ 1 ] ) / 2;
        pBGR[ 2 ] = pLineAbove[ 0 ];

        pLine++;
        pLineAbove++;
        pBGR += 3;

        pBGR[ 0 ] = pLine[ 0 ];
        pBGR[ 1 ] = ( pLine[ 1 ] + pLineAbove[ 0 ] ) / 2;
        pBGR[ 2 ] = pLineAbove[ 1 ];

        pLine++;
        pLineAbove++;
        pBGR += 3;
      }
    }

    // Clear right-hand edge.
    *( pBGR - 1 ) = 0;
    *( pBGR - 2 ) = 0;
    *( pBGR - 3 ) = 0;
  }
}

// Bayer pattern:
// 
// G R G R G R G 
// B G B G B G B
// G R G R G R G 
// B G B G B G B
// G R G R G R G 
// B G B G B G B
// G R G R G R G 
// B G B G B G B

void VViconCGStreamBayer::BayerBGToBGR( unsigned int i_Width, unsigned int i_Height, const unsigned char * i_pBayerData, unsigned char * o_pBGRData )
{
  assert( ( i_Width % 2 ) == 0 );
  assert( ( i_Height % 2 ) == 0 );

  std::memset( o_pBGRData, 0, i_Width * 3 );

  const unsigned char * pLineAbove = i_pBayerData;
  const unsigned char * pLine = i_pBayerData + i_Width;
  unsigned char * pBGR = o_pBGRData + i_Width * 3;

  for( unsigned int Y = 1; Y < i_Height; Y++ )
  {
    if( ( Y & 1 ) == 0 )
    {
      for( unsigned int X = 0; ( X + 1 ) < i_Width; X += 2 )
      {
        // Second line (odd lines): green, red
        pBGR[ 0 ] = pLineAbove[ 0 ];
        pBGR[ 1 ] = ( pLineAbove[ 1 ] + pLine[ 0 ] ) / 2;
        pBGR[ 2 ] = pLine[ 1 ];

        pLine++;
        pLineAbove++;
        pBGR += 3;

        pBGR[ 0 ] = pLineAbove[ 1 ];
        pBGR[ 1 ] = ( pLineAbove[ 0 ] + pLine[ 1 ] ) / 2;
        pBGR[ 2 ] = pLine[ 0 ];

        pLine++;
        pLineAbove++;
        pBGR += 3;
      }
    }
    else
    {
      for( unsigned int X = 0; ( X + 1 ) < i_Width; X += 2 )
      {
        // First line (odd lines): blue, green
        pBGR[ 0 ] = pLine[ 0 ];
        pBGR[ 1 ] = ( pLine[ 1 ] + pLineAbove[ 0 ] ) / 2;
        pBGR[ 2 ] = pLineAbove[ 1 ];

        pLine++;
        pLineAbove++;
        pBGR += 3;

        pBGR[ 0 ] = pLine[ 1 ];
        pBGR[ 1 ] = ( pLine[ 0 ] + pLineAbove[ 1 ] ) / 2;
        pBGR[ 2 ] = pLineAbove[ 0 ];

        pLine++;
        pLineAbove++;
        pBGR += 3;
      }
    }

    // Clear right-hand edge.
    *( pBGR - 1 ) = 0;
    *( pBGR - 2 ) = 0;
    *( pBGR - 3 ) = 0;
  }
}

// Bayer pattern:
// 
// G B G B G B G  
// R G R G R G R 

void VViconCGStreamBayer::BayerRGToBGR( unsigned int i_Width, unsigned int i_Height, const unsigned char * i_pBayerData, unsigned char * o_pBGRData )
{
  assert( ( i_Width  % 2 ) == 0 );
  assert( ( i_Height % 2 ) == 0 );

  std::memset( o_pBGRData, 0, i_Width * 3 );

  const unsigned char * pLineAbove = i_pBayerData;
  const unsigned char * pLine = i_pBayerData + i_Width;
  unsigned char * pBGR = o_pBGRData + i_Width * 3;

  for( unsigned int Y = 1; Y < i_Height; Y++ )
  {
    if( ( Y & 1 ) == 0 )
    {
      for( unsigned int X = 0; ( X + 1 ) < i_Width; X += 2 )
      {
        // Second line (odd lines): green, blue
        pBGR[ 0 ] = pLine[ 1 ];
        pBGR[ 1 ] = ( pLineAbove[ 1 ] + pLine[ 0 ] ) / 2;
        pBGR[ 2 ] = pLineAbove[ 0 ];

        pLine++;
        pLineAbove++;
        pBGR += 3;

        pBGR[ 0 ] = pLine[ 0 ];
        pBGR[ 1 ] = ( pLineAbove[ 0 ] + pLine[ 1 ] ) / 2;
        pBGR[ 2 ] = pLineAbove[ 1 ];

        pLine++;
        pLineAbove++;
        pBGR += 3;
      }
    }
    else
    {
      for( unsigned int X = 0; ( X + 1 ) < i_Width; X += 2 )
      {
        // First line (odd lines): red, green
        pBGR[ 0 ] = pLineAbove[ 1 ];
        pBGR[ 1 ] = ( pLine[ 1 ] + pLineAbove[ 0 ] ) / 2;
        pBGR[ 2 ] = pLine[ 0 ];

        pLine++;
        pLineAbove++;
        pBGR += 3;

        pBGR[ 0 ] = pLineAbove[ 0 ];
        pBGR[ 1 ] = ( pLine[ 0 ] + pLineAbove[ 1 ] ) / 2;
        pBGR[ 2 ] = pLine[ 1 ];

        pLine++;
        pLineAbove++;
        pBGR += 3;
      }
    }

    // Clear right-hand edge.
    *( pBGR - 1 ) = 0;
    *( pBGR - 2 ) = 0;
    *( pBGR - 3 ) = 0;
  }
}
