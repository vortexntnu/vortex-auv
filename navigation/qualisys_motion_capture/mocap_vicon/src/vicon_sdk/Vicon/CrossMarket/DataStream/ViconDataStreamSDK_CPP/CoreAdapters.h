
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

#include "IDataStreamClientBase.h"
#include <ViconDataStreamSDKCore/Constants.h>

// This function is provided to insulate us from changes to ViconDataStreamSDK::Core::Axis::Enum 
static ViconDataStreamSDK::Core::Direction::Enum Adapt(const ViconDataStreamSDK::CPP::Direction::Enum i_Direction)
{
  switch (i_Direction)
  {
  default:
  case ViconDataStreamSDK::CPP::Direction::Up: return ViconDataStreamSDK::Core::Direction::Up;
  case ViconDataStreamSDK::CPP::Direction::Down: return ViconDataStreamSDK::Core::Direction::Down;
  case ViconDataStreamSDK::CPP::Direction::Left: return ViconDataStreamSDK::Core::Direction::Left;
  case ViconDataStreamSDK::CPP::Direction::Right: return ViconDataStreamSDK::Core::Direction::Right;
  case ViconDataStreamSDK::CPP::Direction::Forward: return ViconDataStreamSDK::Core::Direction::Forward;
  case ViconDataStreamSDK::CPP::Direction::Backward: return ViconDataStreamSDK::Core::Direction::Backward;
  }
}

// This function is provided to insulate us from changes to ViconDataStreamSDK::Core::Axis::Enum 
static ViconDataStreamSDK::CPP::Direction::Enum Adapt(const ViconDataStreamSDK::Core::Direction::Enum i_Direction)
{
  switch (i_Direction)
  {
  default:
  case ViconDataStreamSDK::Core::Direction::Up: return ViconDataStreamSDK::CPP::Direction::Up;
  case ViconDataStreamSDK::Core::Direction::Down: return ViconDataStreamSDK::CPP::Direction::Down;
  case ViconDataStreamSDK::Core::Direction::Left: return ViconDataStreamSDK::CPP::Direction::Left;
  case ViconDataStreamSDK::Core::Direction::Right: return ViconDataStreamSDK::CPP::Direction::Right;
  case ViconDataStreamSDK::Core::Direction::Forward: return ViconDataStreamSDK::CPP::Direction::Forward;
  case ViconDataStreamSDK::Core::Direction::Backward: return ViconDataStreamSDK::CPP::Direction::Backward;
  }
}

static ViconDataStreamSDK::CPP::ServerOrientation::Enum Adapt(const ViconDataStreamSDK::Core::ServerOrientation::Enum i_Orientation)
{
  switch (i_Orientation)
  {
  default:
  case ViconDataStreamSDK::Core::ServerOrientation::Unknown: return ViconDataStreamSDK::CPP::ServerOrientation::Unknown;
  case ViconDataStreamSDK::Core::ServerOrientation::YUp: return ViconDataStreamSDK::CPP::ServerOrientation::YUp;
  case ViconDataStreamSDK::Core::ServerOrientation::ZUp: return ViconDataStreamSDK::CPP::ServerOrientation::ZUp;
  }
}

static ViconDataStreamSDK::Core::ServerOrientation::Enum Adapt(const ViconDataStreamSDK::CPP::ServerOrientation::Enum i_Orientation)
{
  switch (i_Orientation)
  {
  default:
  case ViconDataStreamSDK::CPP::ServerOrientation::Unknown: return ViconDataStreamSDK::Core::ServerOrientation::Unknown;
  case ViconDataStreamSDK::CPP::ServerOrientation::YUp: return ViconDataStreamSDK::Core::ServerOrientation::YUp;
  case ViconDataStreamSDK::CPP::ServerOrientation::ZUp: return ViconDataStreamSDK::Core::ServerOrientation::ZUp;
  }
}

// This function is provided to insulate us from changes to ViconDataStreamSDK::CPP::StreamMode::Enum 
static ViconDataStreamSDK::Core::StreamMode::Enum Adapt(ViconDataStreamSDK::CPP::StreamMode::Enum i_Mode)
{
  switch (i_Mode)
  {
  default:
  case ViconDataStreamSDK::CPP::StreamMode::ClientPull: return ViconDataStreamSDK::Core::StreamMode::ClientPull;
  case ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch: return ViconDataStreamSDK::Core::StreamMode::ClientPullPreFetch;
  case ViconDataStreamSDK::CPP::StreamMode::ServerPush: return ViconDataStreamSDK::Core::StreamMode::ServerPush;
  }
}
// This function is provided to insulate us from changes to ViconDataStreamSDK::Core::Result::Enum 
static ViconDataStreamSDK::CPP::Result::Enum Adapt(ViconDataStreamSDK::Core::Result::Enum i_Result)
{
  switch (i_Result)
  {
  case ViconDataStreamSDK::Core::Result::Unknown: return ViconDataStreamSDK::CPP::Result::Unknown;
  case ViconDataStreamSDK::Core::Result::NotImplemented: return ViconDataStreamSDK::CPP::Result::NotImplemented;
  case ViconDataStreamSDK::Core::Result::Success: return ViconDataStreamSDK::CPP::Result::Success;
  case ViconDataStreamSDK::Core::Result::InvalidHostName: return ViconDataStreamSDK::CPP::Result::InvalidHostName;
  case ViconDataStreamSDK::Core::Result::InvalidMulticastIP: return ViconDataStreamSDK::CPP::Result::InvalidMulticastIP;
  case ViconDataStreamSDK::Core::Result::ClientAlreadyConnected: return ViconDataStreamSDK::CPP::Result::ClientAlreadyConnected;
  case ViconDataStreamSDK::Core::Result::ClientConnectionFailed: return ViconDataStreamSDK::CPP::Result::ClientConnectionFailed;
  case ViconDataStreamSDK::Core::Result::ServerAlreadyTransmittingMulticast: return ViconDataStreamSDK::CPP::Result::ServerAlreadyTransmittingMulticast;
  case ViconDataStreamSDK::Core::Result::ServerNotTransmittingMulticast: return ViconDataStreamSDK::CPP::Result::ServerNotTransmittingMulticast;
  case ViconDataStreamSDK::Core::Result::NotConnected: return ViconDataStreamSDK::CPP::Result::NotConnected;
  case ViconDataStreamSDK::Core::Result::NoFrame: return ViconDataStreamSDK::CPP::Result::NoFrame;
  case ViconDataStreamSDK::Core::Result::InvalidIndex: return ViconDataStreamSDK::CPP::Result::InvalidIndex;
  case ViconDataStreamSDK::Core::Result::InvalidCameraName: return ViconDataStreamSDK::CPP::Result::InvalidCameraName;
  case ViconDataStreamSDK::Core::Result::InvalidSubjectName: return ViconDataStreamSDK::CPP::Result::InvalidSubjectName;
  case ViconDataStreamSDK::Core::Result::InvalidSegmentName: return ViconDataStreamSDK::CPP::Result::InvalidSegmentName;
  case ViconDataStreamSDK::Core::Result::InvalidMarkerName: return ViconDataStreamSDK::CPP::Result::InvalidMarkerName;
  case ViconDataStreamSDK::Core::Result::InvalidDeviceName: return ViconDataStreamSDK::CPP::Result::InvalidDeviceName;
  case ViconDataStreamSDK::Core::Result::InvalidDeviceOutputName: return ViconDataStreamSDK::CPP::Result::InvalidDeviceOutputName;
  case ViconDataStreamSDK::Core::Result::InvalidLatencySampleName: return ViconDataStreamSDK::CPP::Result::InvalidLatencySampleName;
  case ViconDataStreamSDK::Core::Result::CoLinearAxes: return ViconDataStreamSDK::CPP::Result::CoLinearAxes;
  case ViconDataStreamSDK::Core::Result::LeftHandedAxes: return ViconDataStreamSDK::CPP::Result::LeftHandedAxes;
  case ViconDataStreamSDK::Core::Result::HapticAlreadySet: return ViconDataStreamSDK::CPP::Result::HapticAlreadySet;
  case ViconDataStreamSDK::Core::Result::EarlyDataRequested: return ViconDataStreamSDK::CPP::Result::EarlyDataRequested;
  case ViconDataStreamSDK::Core::Result::LateDataRequested: return ViconDataStreamSDK::CPP::Result::LateDataRequested;
  case ViconDataStreamSDK::Core::Result::InvalidOperation: return ViconDataStreamSDK::CPP::Result::InvalidOperation;
  case ViconDataStreamSDK::Core::Result::NullClient:
  default: assert(!"Unexpected result code returned");
    return ViconDataStreamSDK::CPP::Result::Unknown;
  }
}

// This function is provided to insulate us from changes to ViconDataStreamSDK::Core::TimecodeStandard::Enum 
static ViconDataStreamSDK::CPP::TimecodeStandard::Enum Adapt(ViconDataStreamSDK::Core::TimecodeStandard::Enum i_TimecodeStandard)
{
  switch (i_TimecodeStandard)
  {
  case ViconDataStreamSDK::Core::TimecodeStandard::PAL: return ViconDataStreamSDK::CPP::TimecodeStandard::PAL;
  case ViconDataStreamSDK::Core::TimecodeStandard::NTSC: return ViconDataStreamSDK::CPP::TimecodeStandard::NTSC;
  case ViconDataStreamSDK::Core::TimecodeStandard::NTSCDrop: return ViconDataStreamSDK::CPP::TimecodeStandard::NTSCDrop;
  case ViconDataStreamSDK::Core::TimecodeStandard::Film: return ViconDataStreamSDK::CPP::TimecodeStandard::Film;
  case ViconDataStreamSDK::Core::TimecodeStandard::NTSCFilm: return ViconDataStreamSDK::CPP::TimecodeStandard::NTSCFilm;
  case ViconDataStreamSDK::Core::TimecodeStandard::ATSC: return ViconDataStreamSDK::CPP::TimecodeStandard::ATSC;
  default: assert(!"Unexpected result code returned");
  case ViconDataStreamSDK::Core::TimecodeStandard::None: return ViconDataStreamSDK::CPP::TimecodeStandard::None;
  }
}

// This function is provided to insulate us from changes to ViconDataStreamSDK::Core::DeviceType::Enum 
static ViconDataStreamSDK::CPP::DeviceType::Enum Adapt(ViconDataStreamSDK::Core::DeviceType::Enum i_DeviceType)
{
  switch (i_DeviceType)
  {
  case ViconDataStreamSDK::Core::DeviceType::ForcePlate: return ViconDataStreamSDK::CPP::DeviceType::ForcePlate;
  case ViconDataStreamSDK::Core::DeviceType::EyeTracker: return ViconDataStreamSDK::CPP::DeviceType::EyeTracker;
  default: assert(!"Unexpected result code returned");
  case ViconDataStreamSDK::Core::DeviceType::Unknown: return ViconDataStreamSDK::CPP::DeviceType::Unknown;
  }
}

// This function is provided to insulate us from changes to ViconDataStreamSDK::Core::Unit::Enum 
static ViconDataStreamSDK::CPP::Unit::Enum Adapt(ViconDataStreamSDK::Core::Unit::Enum i_Unit)
{
  switch (i_Unit)
  {
  case ViconDataStreamSDK::Core::Unit::Volt: return ViconDataStreamSDK::CPP::Unit::Volt;
  case ViconDataStreamSDK::Core::Unit::Newton: return ViconDataStreamSDK::CPP::Unit::Newton;
  case ViconDataStreamSDK::Core::Unit::NewtonMeter: return ViconDataStreamSDK::CPP::Unit::NewtonMeter;
  case ViconDataStreamSDK::Core::Unit::Meter: return ViconDataStreamSDK::CPP::Unit::Meter;
  case ViconDataStreamSDK::Core::Unit::Kilogram: return ViconDataStreamSDK::CPP::Unit::Kilogram;
  case ViconDataStreamSDK::Core::Unit::Second: return ViconDataStreamSDK::CPP::Unit::Second;
  case ViconDataStreamSDK::Core::Unit::Ampere: return ViconDataStreamSDK::CPP::Unit::Ampere;
  case ViconDataStreamSDK::Core::Unit::Kelvin: return ViconDataStreamSDK::CPP::Unit::Kelvin;
  case ViconDataStreamSDK::Core::Unit::Mole: return ViconDataStreamSDK::CPP::Unit::Mole;
  case ViconDataStreamSDK::Core::Unit::Candela: return ViconDataStreamSDK::CPP::Unit::Candela;
  case ViconDataStreamSDK::Core::Unit::Radian: return ViconDataStreamSDK::CPP::Unit::Radian;
  case ViconDataStreamSDK::Core::Unit::Steradian: return ViconDataStreamSDK::CPP::Unit::Steradian;
  case ViconDataStreamSDK::Core::Unit::MeterSquared: return ViconDataStreamSDK::CPP::Unit::MeterSquared;
  case ViconDataStreamSDK::Core::Unit::MeterCubed: return ViconDataStreamSDK::CPP::Unit::MeterCubed;
  case ViconDataStreamSDK::Core::Unit::MeterPerSecond: return ViconDataStreamSDK::CPP::Unit::MeterPerSecond;
  case ViconDataStreamSDK::Core::Unit::MeterPerSecondSquared: return ViconDataStreamSDK::CPP::Unit::MeterPerSecondSquared;
  case ViconDataStreamSDK::Core::Unit::RadianPerSecond: return ViconDataStreamSDK::CPP::Unit::RadianPerSecond;
  case ViconDataStreamSDK::Core::Unit::RadianPerSecondSquared: return ViconDataStreamSDK::CPP::Unit::RadianPerSecondSquared;
  case ViconDataStreamSDK::Core::Unit::Hertz: return ViconDataStreamSDK::CPP::Unit::Hertz;
  case ViconDataStreamSDK::Core::Unit::Joule: return ViconDataStreamSDK::CPP::Unit::Joule;
  case ViconDataStreamSDK::Core::Unit::Watt: return ViconDataStreamSDK::CPP::Unit::Watt;
  case ViconDataStreamSDK::Core::Unit::Pascal: return ViconDataStreamSDK::CPP::Unit::Pascal;
  case ViconDataStreamSDK::Core::Unit::Lumen: return ViconDataStreamSDK::CPP::Unit::Lumen;
  case ViconDataStreamSDK::Core::Unit::Lux: return ViconDataStreamSDK::CPP::Unit::Lux;
  case ViconDataStreamSDK::Core::Unit::Coulomb: return ViconDataStreamSDK::CPP::Unit::Coulomb;
  case ViconDataStreamSDK::Core::Unit::Ohm: return ViconDataStreamSDK::CPP::Unit::Ohm;
  case ViconDataStreamSDK::Core::Unit::Farad: return ViconDataStreamSDK::CPP::Unit::Farad;
  case ViconDataStreamSDK::Core::Unit::Weber: return ViconDataStreamSDK::CPP::Unit::Weber;
  case ViconDataStreamSDK::Core::Unit::Tesla: return ViconDataStreamSDK::CPP::Unit::Tesla;
  case ViconDataStreamSDK::Core::Unit::Henry: return ViconDataStreamSDK::CPP::Unit::Henry;
  case ViconDataStreamSDK::Core::Unit::Siemens: return ViconDataStreamSDK::CPP::Unit::Siemens;
  case ViconDataStreamSDK::Core::Unit::Becquerel: return ViconDataStreamSDK::CPP::Unit::Becquerel;
  case ViconDataStreamSDK::Core::Unit::Gray: return ViconDataStreamSDK::CPP::Unit::Gray;
  case ViconDataStreamSDK::Core::Unit::Sievert: return ViconDataStreamSDK::CPP::Unit::Sievert;
  case ViconDataStreamSDK::Core::Unit::Katal: return ViconDataStreamSDK::CPP::Unit::Katal;

  default: assert(!"Unexpected result code returned");
  case ViconDataStreamSDK::Core::Unit::Unknown: return ViconDataStreamSDK::CPP::Unit::Unknown;
  }
}

