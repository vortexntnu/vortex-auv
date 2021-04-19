
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

namespace ViconDataStreamSDK
{
namespace Core
{

namespace Direction
{
  enum Enum
  {
    Up,
    Down,
    Left,
    Right,
    Forward,
    Backward
  };
}

namespace ServerOrientation
{
  enum Enum
  {
    Unknown,
    YUp,
    ZUp,
  };
}

namespace AxisMappingResult
{
  enum Enum
  {
    Success = 0,
    CoLinearAxes,
    LeftHandedAxes
  };
}

namespace StreamMode
{
  enum Enum
  {
    ClientPull,
    ClientPullPreFetch,
    ServerPush
  };
}

namespace TimecodeStandard
{
  enum Enum
  {
    None,
    PAL,
    NTSC,
    NTSCDrop,
    Film,
    NTSCFilm,
    ATSC
  };
}

namespace DeviceType
{
  enum Enum
  {
    Unknown,
    ForcePlate,
    EyeTracker
  };
};

namespace Unit
{
  enum Enum
  {
    Unknown,
    Volt,
    Newton,
    NewtonMeter,
    Meter,
    Kilogram,
    Second,
    Ampere,
    Kelvin,
    Mole,
    Candela,
    Radian,
    Steradian,
    MeterSquared,
    MeterCubed,
    MeterPerSecond,
    MeterPerSecondSquared,
    RadianPerSecond,
    RadianPerSecondSquared,
    Hertz,
    Joule,
    Watt,
    Pascal,
    Lumen,
    Lux,
    Coulomb,
    Ohm,
    Farad,
    Weber,
    Tesla,
    Henry,
    Siemens,
    Becquerel,
    Gray,
    Sievert,
    Katal
  };
};

namespace Result
{
  enum Enum
  {
    Unknown,
    NotImplemented,
    Success,
    InvalidHostName,
    InvalidMulticastIP,
    NullClient,
    ClientAlreadyConnected,
    ClientConnectionFailed,
    ServerAlreadyTransmittingMulticast,
    ServerNotTransmittingMulticast,
    NotConnected,
    NoFrame,
    InvalidIndex,
    InvalidCameraName,
    InvalidSubjectName,
    InvalidSegmentName,
    InvalidMarkerName,
    InvalidDeviceName,
    InvalidDeviceOutputName,
    InvalidLatencySampleName,
    InvalidFrameRateName,
    CoLinearAxes,
    LeftHandedAxes,
    HapticAlreadySet,
    EarlyDataRequested,
    LateDataRequested,
    InvalidOperation
  };
}

}
}
