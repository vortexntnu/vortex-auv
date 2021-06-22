
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
/// Contains the declaration of the ViconCGStreamType namespace.

/**
 * This namespace defines the set of primitive types used by the protocol Readers and Writers, and
 * also by the objects defined in the ViconCGStream namespace.
 * 
 * See ViconCGStreamIO::VReader, ViconCGStreamIO::VWriter.
 */
namespace ViconCGStreamType
{
//-------------------------------------------------------------------------------------------------

/// Single 8-bit character.
typedef char Char;
/// 8-bit signed integer.
typedef char Int8;
/// 16-bit signed integer.
typedef short Int16;
/// 32-bit signed integer.
typedef int Int32;
/// 64-bit signed integer.
typedef long long Int64;

/// 8-bit unsigned integer.
typedef unsigned char UInt8;
/// 16-bit unsigned integer.
typedef unsigned short UInt16;
/// 32-bit unsigned integer.
typedef unsigned int UInt32;
/// 64-bit unsigned integer.
typedef unsigned long long UInt64;

/// 32-bit IEEE-format floating-point number.
typedef float Float;
/// 64-bit IEEE-format floating-point number.
typedef double Double;

/** This is a special type to distinguish members of the ViconCGStreamEnum namespace from other
  * primitives in the stream.
  */
typedef UInt32 Enum;

//-------------------------------------------------------------------------------------------------
};

