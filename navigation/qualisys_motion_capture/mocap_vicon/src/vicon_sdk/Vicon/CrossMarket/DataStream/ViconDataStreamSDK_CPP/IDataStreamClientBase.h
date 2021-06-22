
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
/// The SDK client and supporting definitions.

#ifdef WIN32

#ifdef _EXPORTING
   #define CLASS_DECLSPEC    __declspec(dllexport)
#else
   #define CLASS_DECLSPEC    __declspec(dllimport)
#endif // _EXPORTING

#elif defined( __GNUC__ )

#if __GNUC__ < 4
    #error gcc 4 is required.
  #endif
  #define CLASS_DECLSPEC     __attribute__((visibility("default")))

#else

#define CLASS_DECLSPEC

#endif

#include <string>
#include <vector>

namespace ViconDataStreamSDK
{
namespace CPP
{

class IStringFactory
{
public:
  virtual char * AllocAndCopyString( const char * i_pSource ) = 0;
  virtual void FreeString( char * i_pString ) = 0;
protected:
  virtual ~IStringFactory() {}
};

// A string-managing class for DLL boundary safe text.
// It ensures that strings are de-allocated with the same 
// memory manager they are made with.  This allows you to use a debug 
// build of your client with a release SDK.
class String
{
public:
  // A string which we are not responsible for deallocating
  inline
  String( const char * i_pString = 0 )
  : m_pString( 0 )
  , m_pConstString( i_pString )
  , m_pStringFactory( 0 )
  {
  }

  // A string which we are not responsible for deallocating
  String( const std::string & i_rString )
  : m_pString( 0 )
  , m_pConstString( i_rString.c_str() )
  , m_pStringFactory( 0 )
  {
  }

  // Copy constructor
  inline
  String( const String & i_rString )
  {
    m_pConstString = i_rString.m_pConstString;
    m_pStringFactory = i_rString.m_pStringFactory;
    if( m_pStringFactory )
    {
      m_pString = m_pStringFactory->AllocAndCopyString( i_rString.m_pString );
    }
    else
    {
      m_pString = 0;
    }
  }

  inline
  String & operator=( const String & i_rString )
  {
    m_pConstString = i_rString.m_pConstString;
    m_pStringFactory = i_rString.m_pStringFactory;
    if ( m_pStringFactory )
    {
      m_pString = m_pStringFactory->AllocAndCopyString( i_rString.m_pString );
    }
    else
    {
      m_pString = 0;
    }
    return *this;
  }

  inline
  ~String()
  {
    if( m_pStringFactory )
    {
      m_pStringFactory->FreeString( m_pString );
    }
  }

  // A string which we are responsible for deallocating
  inline
  void Set( const char * i_pString, IStringFactory & i_rStringFactory )
  {
    m_pString = i_rStringFactory.AllocAndCopyString( i_pString );
    m_pStringFactory = &i_rStringFactory;
    m_pConstString = 0;
  }

  inline
  operator std::string() const
  {
    if( m_pStringFactory )
    {
      return std::string( m_pString );
    }
    else
    {
      return std::string( m_pConstString );
    }
  }

private:
        char     * m_pString;
  const char     * m_pConstString;
  IStringFactory * m_pStringFactory;
};

// Streaming operator for String
inline std::ostream & operator<<( std::ostream & io_rStream, const String & i_rString )
{
  io_rStream << std::string( i_rString );
  return io_rStream;
}

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
}

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

}

namespace Result
{
  /// The Result code indicates the success or failure of a function.
  enum Enum
  {
    Unknown, ///< The result is unknown. Treat it as a failure.
    NotImplemented, ///< The function called has not been implemented in this version of the SDK.
    Success, ///< The function call succeeded.
    InvalidHostName, ///< The "HostName" parameter passed to Connect() is invalid.
    InvalidMulticastIP, ///< The "MulticastIP" parameter was not in the range "224.0.0.0" - "239.255.255.255"
    ClientAlreadyConnected, ///< Connect() was called whilst already connected to a DataStream.
    ClientConnectionFailed, ///< Connect() could not establish a connection to the DataStream server.
    ServerAlreadyTransmittingMulticast, ///< StartTransmittingMulticast() was called when the current DataStream server was already transmitting multicast on behalf of this client.
    ServerNotTransmittingMulticast, ///< StopTransmittingMulticast() was called when the current DataStream server was not transmitting multicast on behalf of this client.
    NotConnected, ///< You have called a function which requires a connection to the DataStream server, but do not have a connection.
    NoFrame, ///< You have called a function which requires a frame to be fetched from the DataStream server, but do not have a frame.
    InvalidIndex, ///< An index you have passed to a function is out of range.
    InvalidCameraName, ///< The Camera Name you passed to a function is invalid in this frame.
    InvalidSubjectName, ///< The Subject Name you passed to a function is invalid in this frame.
    InvalidSegmentName, ///< The Segment Name you passed to a function is invalid in this frame.
    InvalidMarkerName, ///< The Marker Name you passed to a function is invalid in this frame.
    InvalidDeviceName, ///< The Device Name you passed to a function is invalid in this frame.
    InvalidDeviceOutputName, ///< The Device Output Name you passed to a function is invalid in this frame.
    InvalidLatencySampleName, ///< The Latency Sample Name you passed to a function is invalid in this frame.
    CoLinearAxes, ///< The directions passed to SetAxisMapping() contain input which would cause two or more axes to lie along the same line, e.g. "Up" and "Down" are on the same line.
    LeftHandedAxes, ///< The directions passed to SetAxisMapping() would result in a left-handed coordinate system. This is not supported in the SDK.
    HapticAlreadySet, ///< Haptic feedback is already set.
    EarlyDataRequested, ///< Re-timed data requested is from before the first time sample we still have
    LateDataRequested, ///< Re-timed data requested is too far into the future to be predicted
    InvalidOperation ///< The method called is not valid in the current mode of operation
  };
}

  /// The version information returned by Client::GetVersion.
  class Output_GetVersion
  {
  public:
    /// Major version number of the SDK.
    unsigned int Major;
    /// Minor version number of the SDK.
    unsigned int Minor;
    /// Point version number of the SDK.
    unsigned int Point;
  };

  class Output_SimpleResult
  {
  public:
    Result::Enum Result;
  };

  // Output objects that only return a result enum
  class Output_Connect                    : public Output_SimpleResult {};
  class Output_ConnectToMulticast         : public Output_SimpleResult {};
  class Output_Disconnect                 : public Output_SimpleResult {};
  class Output_StartTransmittingMulticast : public Output_SimpleResult {};
  class Output_StopTransmittingMulticast  : public Output_SimpleResult {};
  class Output_EnableSegmentData          : public Output_SimpleResult {};
  class Output_EnableMarkerData           : public Output_SimpleResult {};
  class Output_EnableUnlabeledMarkerData  : public Output_SimpleResult {};
  class Output_EnableMarkerRayData        : public Output_SimpleResult {};
  class Output_EnableDeviceData           : public Output_SimpleResult {};
  class Output_EnableCentroidData         : public Output_SimpleResult {};
  class Output_EnableGreyscaleData        : public Output_SimpleResult {};
  class Output_EnableDebugData            : public Output_SimpleResult {};
  class Output_DisableSegmentData         : public Output_SimpleResult {};
  class Output_DisableMarkerData          : public Output_SimpleResult {};
  class Output_DisableUnlabeledMarkerData : public Output_SimpleResult {};
  class Output_DisableMarkerRayData       : public Output_SimpleResult {};
  class Output_DisableDeviceData          : public Output_SimpleResult {};
  class Output_DisableCentroidData        : public Output_SimpleResult {};
  class Output_DisableGreyscaleData       : public Output_SimpleResult {};
  class Output_DisableDebugData           : public Output_SimpleResult {};
  class Output_SetStreamMode              : public Output_SimpleResult {};
  class Output_SetApexDeviceFeedback      : public Output_SimpleResult {};
  class Output_SetAxisMapping             : public Output_SimpleResult {};
  class Output_GetFrame                   : public Output_SimpleResult {};
  class Output_UpdateFrame                : public Output_SimpleResult {};
  class Output_WaitForFrame               : public Output_SimpleResult {};
  class Output_SetCameraFilter            : public Output_SimpleResult {};

  class Output_EnabledFlag
  {
  public:
    bool Enabled;
  };

  // Output objects that only return an enabled flag
  class Output_IsSegmentDataEnabled         : public Output_EnabledFlag {};
  class Output_IsMarkerDataEnabled          : public Output_EnabledFlag {};
  class Output_IsUnlabeledMarkerDataEnabled : public Output_EnabledFlag {};
  class Output_IsMarkerRayDataEnabled       : public Output_EnabledFlag {};
  class Output_IsDeviceDataEnabled          : public Output_EnabledFlag {};
  class Output_IsCentroidDataEnabled        : public Output_EnabledFlag {};
  class Output_IsGreyscaleDataEnabled       : public Output_EnabledFlag {};
  class Output_IsDebugDataEnabled           : public Output_EnabledFlag {};

  class Output_IsConnected
  {
  public:
    bool Connected;
  };

  class Output_GetAxisMapping
  {
  public:
    Direction::Enum XAxis;
    Direction::Enum YAxis;
    Direction::Enum ZAxis;
  };

  class Output_GetServerOrientation
  {
  public:
    Result::Enum Result;
    ServerOrientation::Enum Orientation;
  };

  class Output_GetFrameNumber
  {
  public:
    Result::Enum Result;
    unsigned int FrameNumber;
  };

  class Output_GetTimecode
  {
  public:
    Result::Enum           Result;
    unsigned int           Hours;
    unsigned int           Minutes;
    unsigned int           Seconds;
    unsigned int           Frames;
    unsigned int           SubFrame;
    bool                   FieldFlag;
    TimecodeStandard::Enum Standard;
    unsigned int           SubFramesPerFrame;
    unsigned int           UserBits;
  };

  class Output_GetFrameRate
  {
  public:
    Result::Enum Result;
    double FrameRateHz;
  };

  class Output_GetLatencySampleCount
  {
  public:
    Result::Enum Result;
    unsigned int Count;
  };

  class Output_GetLatencySampleName
  {
  public:
    Result::Enum Result;
    String       Name;
  };

  class Output_GetLatencySampleValue
  {
  public:
    Result::Enum Result;
    double       Value;
  };

  class Output_GetLatencyTotal
  {
  public:
    Result::Enum Result;
    double       Total;
  };

  class Output_GetFrameRateCount
  {
  public:
    Result::Enum Result;
    unsigned int Count;
  };

  class Output_GetFrameRateName
  {
  public:
    Result::Enum Result;
    String       Name;
  };

  class Output_GetFrameRateValue
  {
  public:
    Result::Enum Result;
    double       Value;
  };

  class Output_GetHardwareFrameNumber
  {
  public:
    Result::Enum Result;
    unsigned int HardwareFrameNumber;
  };


  class Output_GetSubjectCount
  {
  public:
    Result::Enum Result;
    unsigned int SubjectCount;
  };

  class Output_GetSubjectName
  {
  public:
    Result::Enum Result;
    String       SubjectName;
  };

  class Output_GetSubjectRootSegmentName
  {
  public:
    Result::Enum Result;
    String       SegmentName;
  };

  class Output_GetSegmentChildCount
  {
  public:
    Result::Enum Result;
    unsigned int SegmentCount;
  };

  class Output_GetSegmentChildName
  {
  public:
    Result::Enum Result;
    String       SegmentName;
  };

  class Output_GetSegmentParentName
  {
  public:
    Result::Enum Result;
    String       SegmentName;
  };

  class Output_GetSegmentCount
  {
  public:
    Result::Enum Result;
    unsigned int SegmentCount;
  };

  class Output_GetSegmentName
  {
  public:
    Result::Enum Result;
    String       SegmentName;
  };

  class Output_GetSegmentStaticTranslation
  {
  public:
    Result::Enum Result;
    double       Translation[ 3 ];
  };

  class Output_GetSegmentStaticRotationHelical
  {
  public:
    Result::Enum Result;
    double       Rotation[ 3 ];
  };

  class Output_GetSegmentStaticRotationMatrix
  {
  public:
    Result::Enum Result;
    double       Rotation[ 9 ];
  };

  class Output_GetSegmentStaticRotationQuaternion
  {
  public:
    Result::Enum Result;
    double       Rotation[ 4 ];
  };

  class Output_GetSegmentStaticRotationEulerXYZ
  {
  public:
    Result::Enum Result;
    double       Rotation[ 3 ];
  };

  class Output_GetSegmentGlobalTranslation
  {
  public:
    Result::Enum Result;
    double       Translation[ 3 ];
    bool         Occluded;
  };

  class Output_GetSegmentGlobalRotationHelical
  {
  public:
    Result::Enum Result;
    double       Rotation[ 3 ];
    bool         Occluded;
  };

  class Output_GetSegmentGlobalRotationMatrix
  {
  public:
    Result::Enum Result;
    double       Rotation[ 9 ];
    bool         Occluded;
  };

  class Output_GetSegmentGlobalRotationQuaternion
  {
  public:
    Result::Enum Result;
    double       Rotation[ 4 ];
    bool         Occluded;
  };

  class Output_GetSegmentGlobalRotationEulerXYZ
  {
  public:
    Result::Enum Result;
    double       Rotation[ 3 ];
    bool         Occluded;
  };

  class Output_GetSegmentLocalTranslation
  {
  public:
    Result::Enum Result;
    double       Translation[ 3 ];
    bool         Occluded;
  };

  class Output_GetSegmentLocalRotationHelical
  {
  public:
    Result::Enum Result;
    double       Rotation[ 3 ];
    bool         Occluded;
  };

  class Output_GetSegmentLocalRotationMatrix
  {
  public:
    Result::Enum Result;
    double       Rotation[ 9 ];
    bool         Occluded;
  };

  class Output_GetSegmentLocalRotationQuaternion
  {
  public:
    Result::Enum Result;
    double       Rotation[ 4 ];
    bool         Occluded;
  };

  class Output_GetSegmentLocalRotationEulerXYZ
  {
  public:
    Result::Enum Result;
    double       Rotation[ 3 ];
    bool         Occluded;
  };

  class Output_GetObjectQuality
  {
  public:
    Result::Enum Result;
    double Quality;
  };

  class Output_GetMarkerCount
  {
  public:
    Result::Enum Result;
    unsigned int MarkerCount;
  };

  class Output_GetMarkerName
  {
  public:
    Result::Enum Result;
    String       MarkerName;
  };

  class Output_GetMarkerParentName
  {
  public:
    Result::Enum Result;
    String       SegmentName;
  };

  class Output_GetMarkerGlobalTranslation
  {
  public:
    Result::Enum Result;
    double       Translation[ 3 ];
    bool         Occluded;
  };

  class Output_GetMarkerRayContributionCount
  {
  public:
    Result::Enum Result;
    unsigned int RayContributionsCount;
  };

  class Output_GetMarkerRayContribution
  {
  public:
    Result::Enum Result;
    unsigned int CameraID;
    unsigned int CentroidIndex;
  };

  class Output_GetUnlabeledMarkerCount
  {
  public:
    Result::Enum Result;
    unsigned int MarkerCount;
  };

  class Output_GetUnlabeledMarkerGlobalTranslation
  {
  public:
    Result::Enum Result;
    double       Translation[ 3 ];
  };

  class Output_GetLabeledMarkerCount
  {
  public:
    Result::Enum Result;
    unsigned int MarkerCount;
  };

  class Output_GetLabeledMarkerGlobalTranslation
  {
  public:
    Result::Enum Result;
    double       Translation[ 3 ];
  };

  class Output_GetDeviceCount
  {
  public:
    Result::Enum Result;
    unsigned int DeviceCount;
  };

  class Output_GetDeviceName
  {
  public:
    Result::Enum     Result;
    String           DeviceName;
    DeviceType::Enum DeviceType;
  };

  class Output_GetDeviceOutputCount
  {
  public:
    Result::Enum Result;
    unsigned int DeviceOutputCount;
  };

  class Output_GetDeviceOutputName
  {
  public:
    Result::Enum Result;
    String       DeviceOutputName;
    Unit::Enum   DeviceOutputUnit;
  };

  class Output_GetDeviceOutputValue
  {
  public:
    Result::Enum Result;
    double       Value;
    bool         Occluded;
  };

  class Output_GetDeviceOutputSubsamples
  {
  public:
    Result::Enum Result;
    unsigned int DeviceOutputSubsamples;
    bool         Occluded;
  };

  class Output_GetForcePlateCount
  {
  public:
    Result::Enum Result;
    unsigned int ForcePlateCount;
  };

  class Output_GetGlobalForceVector
  {
  public:
    Result::Enum Result;
    double       ForceVector[ 3 ];
  };

  class Output_GetGlobalMomentVector
  {
  public:
    Result::Enum Result;
    double       MomentVector[ 3 ];
  };

  class Output_GetGlobalCentreOfPressure
  {
  public:
    Result::Enum Result;
    double       CentreOfPressure[ 3 ];
  };

  class Output_GetForcePlateSubsamples
  {
  public:
    Result::Enum Result;
    unsigned int ForcePlateSubsamples;
  };

  class Output_GetEyeTrackerCount
  {
  public:
    Result::Enum Result;
    unsigned int EyeTrackerCount;
  };

  class Output_GetEyeTrackerGlobalPosition
  {
  public:
    Result::Enum Result;
    double       Position[ 3 ];
    bool         Occluded;
  };

  class Output_GetEyeTrackerGlobalGazeVector
  {
  public:
    Result::Enum Result;
    double       GazeVector[ 3 ];
    bool         Occluded;
  };

  class Output_GetCameraCount
  {
  public:
    Result::Enum Result;
    unsigned int CameraCount;
  };

  class Output_GetCameraName
  {
  public:
    Result::Enum     Result;
    String           CameraName;
  };

  class Output_GetCameraId
  {
  public:
    Result::Enum Result;
    unsigned int CameraId;
  };

  class Output_GetCameraUserId
  {
  public:
    Result::Enum Result;
    unsigned int CameraUserId;
  };

  class Output_GetCameraType
  {
  public:
    Result::Enum     Result;
    String           CameraType;
  };

  class Output_GetCameraDisplayName
  {
  public:
    Result::Enum     Result;
    String           CameraDisplayName;
  };

  class Output_GetCameraResolution
  {
  public:
    Result::Enum Result;
    unsigned int ResolutionX;
    unsigned int ResolutionY;
  };

  class Output_GetIsVideoCamera
  {
  public:
    Result::Enum Result;
    bool IsVideoCamera;
  };

  class Output_GetCentroidCount
  {
  public:
    Result::Enum Result;
    unsigned int CentroidCount;
  };

  class Output_GetCentroidPosition
  {
  public:
    Result::Enum Result;
    double CentroidPosition[ 2 ];
    double Radius;
//    double Accuracy;
  };

  class Output_GetCentroidWeight
  {
  public:
    Result::Enum Result;
    double Weight;
  };

  class Output_GetGreyscaleBlobCount
  {
  public:
    Result::Enum Result;
    unsigned int BlobCount;
  };

  class Output_GetGreyscaleBlob
  {
  public:
    Result::Enum Result;
    std::vector< unsigned int > BlobLinePositionsX;
    std::vector< unsigned int > BlobLinePositionsY;
    std::vector< std::vector< unsigned char > > BlobLinePixelValues;
  };

  ///  Vicon DataStream SDK client.
  ///  The core client class for C++.
  class CLASS_DECLSPEC IDataStreamClientBase
  {
  public:

    /// Get the version of the Vicon DataStream SDK.
    ///   + **Major**
    ///   When this number increases, we break backward compatibility with previous major versions.
    ///   + **Minor**
    ///   When this number increases, we have probably added new functionality to the SDK without breaking 
    ///   backward compatibility with previous versions.
    ///   + **Point**
    ///   When this number increases, we have introduced a bug fix or performance enhancement without breaking 
    ///   backward compatibility with previous versions.
    /// 
    /// The function can be called without the client being connected.
    /// 
    ///      ViconDataStreamSDK::CPP::Client MyClient;
    ///      Output_GetVersion Output = MyClient.GetVersion();
    /// 
    /// \return Output_GetVersion class containing the version information.
    virtual Output_GetVersion  GetVersion() const = 0;

    /// Disconnect from the Vicon DataStream Server.
    /// 
    /// See Also: Connect(), IsConnected()
    /// 
    ///       ViconDataStreamSDK::CPP::Client MyClient;
    ///       MyClient.Connect( "localhost" );
    ///       Output_Disconnect Output = MyClient.Disconnect();
    /// 
    /// \return An Output_Disconnect class containing the result of the operation.
    ///         - The Result will be: 
    ///           + Success
    ///           + NotConnected
    virtual Output_Disconnect  Disconnect() = 0;

    /// Discover whether client is connected to the Vicon DataStream Server.
    /// 
    /// See Also: Connect(), Disconnect()
    /// 
    ///       ViconDataStreamSDK::CPP::CPP::Client MyClient;
    ///       Output_IsConnected Output = MyClient.IsConnected()
    ///       // Output.Connected == false
    ///       MyClient.Connect( "localhost" );
    ///       Output_IsConnected Output = MyClient.IsConnected()
    ///       // Output.Connected == true
    ///       // (assuming localhost is serving)
    /// 
    /// \return An Output_IsConnected class containing a true value for Connected if you are 
    ///         connected to the stream, otherwise false.
    virtual Output_IsConnected IsConnected() const = 0;

   
    /// Remaps the 3D axis.
    /// Vicon Data uses a right-handed coordinate system, with +X forward, +Y left, and +Z up. 
    /// Other systems use different coordinate systems. The SDK can transform its data into any valid right-handed coordinate system by re-mapping each axis. 
    /// Valid directions are "Up", "Down", "Left", "Right", "Forward", and "Backward". Note that "Forward" means moving away from you, and "Backward" is moving towards you. 
    /// Common usages are
    /// Z-up: SetAxisMapping( Forward, Left, Up )
    /// Y-up: SetAxisMapping( Forward, Up, Right )
    ///
    /// See Also: GetAxisMapping()
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.SetAxisMapping( ViconDataStreamSDK::CPP::Direction::Forward,
    ///        ViconDataStreamSDK::CPP::Direction::Left,
    ///        ViconDataStreamSDK::CPP::Direction::Up );
    ///        
    ///        MyClient = Client();
    ///        MyClient.SetAxisMapping( Direction.Forward, Direction.Left, Direction.Up );
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.SetAxisMapping( ViconDataStreamSDK.DotNET.Direction.Forward,
    ///        ViconDataStreamSDK.DotNET.Direction.Left,
    ///        ViconDataStreamSDK.DotNET.Direction.Up );
    ///
    /// \param XAxis Specify the direction of your X axis relative to yourself as the observer.
    /// \param YAxis Specify the direction of your Y axis relative to yourself as the observer.
    /// \param ZAxis Specify the direction of your Z axis relative to yourself as the observer.
    ///
    /// \return An Output_SetAxisMapping class containing the result of the operation.
    ///         - The Result will be:
    ///           + Success
    ///           + CoLinearAxes
    ///           + LeftHandedAxes
    virtual Output_SetAxisMapping SetAxisMapping( const Direction::Enum XAxis, const Direction::Enum YAxis, const Direction::Enum ZAxis ) = 0;

    /// Get the current Axis mapping.
    ///
    /// See Also: SetAxisMapping()
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        Output_GetAxisMapping Output = MyClient.GetAxisMapping();
    ///          // Output.XAxis == ViconDataStreamSDK::CPP::Direction::Forward
    ///          // Output.YAxis == ViconDataStreamSDK::CPP::Direction::Left
    ///          // Output.ZAxis == ViconDataStreamSDK::CPP::Direction::Up
    ///        
    ///        MyClient = Client();
    ///        Output = MyClient.GetAxisMapping();
    ///           % Output.XAxis == Direction.Forward
    ///           % Output.YAxis == Direction.Left
    ///           % Output.ZAxis == Direction.Up
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///        Output_GetAxisMapping Output = MyClient.GetAxisMapping();
    ///         // Output.XAxis == ViconDataStreamSDK.DotNET.Direction.Forward
    ///         // Output.YAxis == ViconDataStreamSDK.DotNET.Direction.Left
    ///         // Output.ZAxis == ViconDataStreamSDK.DotNET.Direction.Up
    ///
    /// \return An Output_GetAxisMapping class containing the result of the operation.
    ///         - The Result will be:
    ///           + XAxis, YAxis, ZAxis
    virtual Output_GetAxisMapping GetAxisMapping() const = 0;

    /// Return the number of subjects in the DataStream. This information can be used in conjunction with GetSubjectName.
    ///
    /// See Also: GetSubjectName()
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.Connect( "localhost" );
    ///        Output_GetSubjectCount Output;
    ///        Output = MyClient.GetSubjectCount(); // Output.Result == NoFrame
    ///                                            // Ooutput.SubjectCount == 0
    ///        MyClient.GetFrame();
    ///        Output = MyClient.GetSubjectCount(); // Output.Result == Success
    ///                                            // Output.SubjectCount >= 0
    ///        
    ///        MyClient = Client();
    ///        MyClient.Connect( 'localhost' );
    ///        Output = MyClient.GetSubjectCount(); % Output.Result == NoFrame
    ///                                             % Ooutput.SubjectCount == 0
    ///        MyClient.GetFrame();
    ///        Output = MyClient.GetSubjectCount(); % Output.Result == Success
    ///                                             % Output.SubjectCount >= 0
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.Connect( "localhost" );
    ///        Output_GetSubjectCount Output;
    ///        Output = MyClient.GetSubjectCount(); // Output.Result == NoFrame
    ///                                             // Ooutput.SubjectCount == 0
    ///        MyClient.GetFrame();
    ///        Output = MyClient.GetSubjectCount(); // Output.Result == Success
    ///                                             // Output.SubjectCount >= 0
    ///
    /// \return An Output_GetSubjectCount class containing the result of the operation and the number of subjects.
    ///         - The Result will be:
    ///           + Success
    ///           + NotConnected
    ///           + NoFrame
    virtual Output_GetSubjectCount GetSubjectCount() const = 0;

    /// Return the name of a subject. This can be passed into segment and marker functions.
    ///
    /// See Also: GetSubjectCount()
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSubjectCount OutputGSC;
    ///        OutputGSC = MyClient.GetSubjectCount(); // OutputGSC.Result == Success
    ///                                                // OutputGSC.SubjectCount == 2
    ///        Output_GetSubjectName OutputGSN;
    ///        OutputGSN = MyClient.GetSubjectName(0);// OutputGSN.Result == Success
    ///                                               // OutputGSN.SubjectName == "Al"
    ///        OutputGSN = MyClient.GetSubjectName(1);// OutputGSN.Result == Success
    ///                                               // OutputGSN .SubjectName == "Bob"
    ///        OutputGSN = MyClient.GetSubjectName(2);// OutputGSN.Result == InvalidIndex
    ///                                               // OutputGSN.SubjectName == ""
    ///        
    ///        MyClient = Client;
    ///        MyClient.Connect( 'localhost' );
    ///        MyClient.GetFrame();
    ///        OutputGSC = MyClient.GetSubjectCount(); % OutputGSC.Result == Success
    ///                                                % OutputGSC.SubjectCount == 2
    ///        OutputGSN = MyClient.GetSubjectName(1); % OutputGSN.Result == Success
    ///                                                % OutputGSN.SubjectName == 'Al'
    ///        OutputGSN = MyClient.GetSubjectName(2); % OutputGSN.Result == Success
    ///                                                % OutputGSN .SubjectName == 'Bob'
    ///        OutputGSN = MyClient.GetSubjectName(3); % OutputGSN.Result == InvalidIndex
    ///                                                % OutputGSN.SubjectName == ''
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient =
    ///        new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSubjectCount OutputGSC;
    ///        OutputGSC = MyClient.GetSubjectCount(); // OutputGSC.Result == Success
    ///                                                // OutputGSC.SubjectCount == 2
    ///        Output_GetSubjectName OutputGSN;
    ///        OutputGSN = MyClient.GetSubjectName(0);// OutputGSN.Result == Success
    ///                                               // OutputGSN.SubjectName == "Al"
    ///        OutputGSN = MyClient.GetSubjectName(1);// OutputGSN.Result == Success
    ///                                               // OutputGSN .SubjectName == "Bob"
    ///        OutputGSN = MyClient.GetSubjectName(2);// OutputGSN.Result == InvalidIndex
    ///                                               // OutputGSN.SubjectName == ""
    ///
    ///
    /// \param Subject Index The index of the subject. A valid Subject Index is between 0 and GetSubjectCount()-1.
    //                                         Matlab: A valid Subject Index is between 1 and GetSubjectCount().
    /// \return An Output_GetSubjectName GetSubjectName class containing the result of the operation and the name of the subject.
    ///         - The Result will be:
    ///           + Success
    ///           + NotConnected
    ///           + NoFrame
    ///           + InvalidIndex
    virtual Output_GetSubjectName GetSubjectName( const unsigned int SubjectIndex ) const = 0;

    /// Return the name of the root segment for a specified subject. This can be passed into segment functions.The root segment is the ancestor of all other segments in the subject.
    ///
    /// See Also: GetSegmentCount(), GetSegmentParentName(), GetSegmentChildCount(), GetSegmentChildName()
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        Output_GetSubjectRootSegmentName Output;
    ///        Output = MyClient.GetSubjectRootSegmentName( "Bob" );
    ///        // Output.Result == Success
    ///        // Output.SegmentName == "Pelvis"
    ///        
    ///        MyClient = Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        Output = MyClient.GetSubjectRootSegmentName( "Bob" );
    ///        % Output.Result == Success
    ///        % Output.SegmentName == "Pelvis"
    ///        
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        Output_GetSubjectRootSegmentName Output;
    ///        Output = MyClient.GetSubjectRootSegmentName( "Bob" );
    ///        // Output.Result == Success
    ///        // Output.SegmentName == "Pelvis"
    ///
    /// \param Subject Name The name of the subject
    /// \return An Output_GetSubjectRootSegmentName class containing the result of the operation and the name of the root segment.
    ///         - The Result will be:
    ///           + Success
    ///           + NotConnected
    ///           + NoFrame
    ///           + InvalidIndex
    virtual Output_GetSubjectRootSegmentName GetSubjectRootSegmentName( const String & SubjectName ) const = 0;

    /// Return the number of segments for a specified subject in the DataStream. This information can be used in conjunction with GetSegmentName.
    ///
    /// See Also: GetSubjectName(), GetSegmentName()
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.EnableSegmentData();
    ///        MyClient.Connect( "localhost" );
    ///        Output_GetSegmentCount Output;
    ///        Output = MyClient.GetSegmentCount( "Bob" ); // Output.Result == NoFrame
    ///                                                    // Output.SegmentCount == 0
    ///        MyClient.GetFrame();
    ///        Output = MyClient.GetSegmentCount( "Al" ); // Output.Result ==
    ///                                                   // InvalidSubjectName
    ///        // Output.SegmentCount == 0
    ///        Output = MyClient.GetSegmentCount( "Bob" );// Output.Result == Success
    ///                                                   // Output.SegmentCount >= 0
    ///        
    ///        MyClient = Client();
    ///        MyClient.EnableSegmentData();
    ///        MyClient.Connect( "localhost" );
    ///        Output = MyClient.GetSegmentCount( "Bob" ); % Output.Result == NoFrame
    ///                                                   % Output.SegmentCount == 0
    ///        MyClient.GetFrame();
    ///        Output = MyClient.GetSegmentCount( "Al" ); % Output.Result ==
    ///                                                   % InvalidSubjectName
    ///                                                   % Output.SegmentCount == 0
    ///        Output = MyClient.GetSegmentCount( "Bob" ); % Output.Result == Success
    ///                                                    % Output.SegmentCount >= 0
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.EnableSegmentData();
    ///        MyClient.Connect( "localhost" );
    ///        Output_GetSegmentCount Output;
    ///        Output = MyClient.GetSegmentCount( "Bob" ); // Output.Result == NoFrame
    ///                                                    // Output.SegmentCount == 0
    ///        MyClient.GetFrame();
    ///        Output = MyClient.GetSegmentCount( "Al" ); // Output.Result ==
    ///                                                  // InvalidSubjectName
    ///                                                  // Output.SegmentCount == 0
    ///        Output = MyClient.GetSegmentCount( "Bob" ); // Output.Result == Success
    ///                                                    // Output.SegmentCount >= 0
    ///
    /// \param Subject Name The name of the subject.
    /// \return An Output_GetSegmentCount class containing the result of the operation and the number of segments.
    ///         - The Result will be:
    ///           + Success
    ///           + NotConnected
    ///           + NoFrame
    ///           + InvalidIndex
    virtual Output_GetSegmentCount GetSegmentCount( const String  & SubjectName ) const = 0;

    /// Return the name of a subject segment specified by index.
    ///
    /// See Also: GetSegmentCount(), GetSegmentChildCount(), GetSegmentChildName(), GetSubjectRootSegmentName()
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentName Output;
    ///        // SegmentIndex must be between 0 and GetSegmentCount() - 1
    ///        Output = MyClient.GetSegmentName( "Bob", 0 );
    ///        
    ///        MyClient = Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        % SegmentIndex must be between 1 and GetSegmentCount()
    ///        Output = MyClient.GetSegmentName( "Bob", 1 );
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentParentName Output;
    ///        // SegmentIndex must be between 0 and GetSegmentCount() - 1
    ///        Output = MyClient.GetSegmentName( "Bob", 0 );
    ///
    /// \param Subject Name The name of the subject
    /// \param Segment Index The index of the segment
    /// \return An Output_GetSegmentName  class containing the result of the operation and the name of the parent segment or an empty string if it is the root segment.
    ///         - The Result will be:
    ///           + Success
    ///           + NotConnected
    ///           + NoFrame
    ///           + InvalidIndex
    ///           + InvalidSubjectName
    virtual Output_GetSegmentName GetSegmentName( const String       & SubjectName, const unsigned int   SegmentIndex ) const = 0;
    /// Return the name of a child segment for a specified subject segment. This can be passed into segment functions.
    ///
    /// See Also: GetSegmentCount()
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentChildCount OutputGSCC;
    ///        OutputGSCC = MyClient.GetSegmentChildCount( "Bob", "Pelvis" );
    ///        // OutputGSCC.Result == Success
    ///        // OutputGSCC.SegmentCount == 2
    ///        Output_GetSegmentChildName OutputGSCN;
    ///        OutputGSCN = MyClient.GetSegmentName( "Alice", 0 );
    ///        // OutputGSN.Result == InvalidSubjectName
    ///        // OutputGSN.SegmentName == ""
    ///        OutputGSCN = MyClient.GetSegmentName( "Bob", "Pelvis", 0 );
    ///        // OutputGSCN.Result == Success
    ///        // OutputGSCN.SegmentName == "LFemur"
    ///        OutputGSCN = MyClient.GetSegmentName( "Bob", "Pelvis", 1 );
    ///        // OutputGSCN.Result == Success
    ///        // OutputGSCN.SegmentName == "RFemur"
    ///        OutputGSCN = MyClient.GetSegmentName( "Bob", "Pelvis", 2 );
    ///        // OutputGSCN.Result == InvalidIndex
    ///        // OutputGSCN.SegmentName == ""
    ///        // (no third segment)
    ///
    ///        A valid Segment Index is between 1 and GetSegmentChildCount()
    ///        MyClient = Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        OutputGSCC = MyClient.GetSegmentChildCount( "Bob", "Pelvis" );
    ///        % OutputGSCC.Result == Success
    ///        % OutputGSCC.SegmentCount == 2
    ///        OutputGSCN = MyClient.GetSegmentChildName( "Alice", "Pelvis", 1 );
    ///        % OutputGSCN.Result == InvalidSubjectName
    ///        % OutputGSCN.SegmentName == ""
    ///        OutputGSCN = MyClient.GetSegmentChildName( "Bob", "Pelvis", 1 );
    ///        % OutputGSCN.Result == Success
    ///        % OutputGSCN.SegmentName == "LFemur"
    ///        OutputGSCN = MyClient.GetSegmentChildName( "Bob", "Pelvis", 2 );
    ///        % OutputGSCN.Result == Success
    ///        % OutputGSCN.SegmentName == "RFemur"
    ///        OutputGSCN = MyClient.GetSegmentChildName( "Bob", "Pelvis", 3 );
    ///        % OutputGSCN.Result == InvalidIndex
    ///        % OutputGSCN.SegmentName == ""
    ///        % (no third segment)
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentChildCount OutputGSCC;
    ///        OutputGSCC = MyClient.GetSegmentChildCount( "Bob", "Pelvis" );
    ///        // OutputGSCC.Result == Success
    ///        // OutputGSCC.SegmentCount == 2
    ///        Output_GetSegmentChildName OutputGSCN;
    ///        OutputGSCN = MyClient.GetSegmentChildName( "Alice", "Pelvis", 0 );
    ///        // OutputGSCN.Result == InvalidSubjectName
    ///        // OutputGSCN.SegmentName == ""
    ///        OutputGSCN = MyClient.GetSegmentChildName( "Bob", "Pelvis", 0 );
    ///        // OutputGSCN.Result == Success
    ///        // OutputGSCN.SegmentName == "LFemur"
    ///        OutputGSCN = MyClient.GetSegmentChildName( "Bob", "Pelvis", 1 );
    ///        // OutputGSCN.Result == Success
    ///        // OutputGSCN.SegmentName == "RFemur"
    ///        OutputGSCN = MyClient.GetSegmentChildName( "Bob", "Pelvis", 2 );
    ///        // OutputGSCN.Result == InvalidIndex
    ///        // OutputGSCN.SegmentName == ""
    ///        // (no third segment)
    ///
    /// \param Subject Name The name of the subject
    /// \param Segment Name The name of the segment
    /// \param Segment Index The index of the segment. A valid Segment Index is between 0 and GetSegmentChildCount()-1.
    /// \return An Output_GetSegmentChildCount  class containing the result of the operation and the name of the child segment.
    ///         - The Result will be:
    ///           + Success
    ///           + NotConnected
    ///           + NoFrame
    ///           + InvalidIndex
    ///           + InvalidSubjectName
    ///           + InvalidSegmentName
    virtual Output_GetSegmentChildCount GetSegmentChildCount( const String & SubjectName, const String & SegmentName ) const = 0;

    /// Return the name of the child segment for a specified subject segment and index.
    ///
    /// See Also: GetSegmentCount(), GetSegmentChildCount(), GetSegmentChildName(), GetSubjectRootSegmentName()
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentChildName Output;
    ///        // Segment index must be between 0 and GetSegmentChildCount() - 1
    ///        Output = MyClient.GetSegmentChildName( "Bob", "Pelvis", 0 );
    ///        
    ///        MyClient = Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        // Segment index must be between 1 and GetSegmentChildCount()
    ///        Output = MyClient.GetSegmentChildName( "Bob", "Pelvis", 1 );
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentChildName Output;
    ///        // Segment index must be between 0 and GetSegmentChildCount() - 1
    ///        Output = MyClient.GetSegmentChildName( "Bob", "Pelvis", 0 );
    ///
    ///
    /// \param Subject Name The name of the subject
    /// \param Segment Name The name of the segment
    /// \param Segment Index The index of the child segment. A valid Segment Index is between 0 and GetSegmentChildCount()-1.
    /// \return An Output_GetSegmentChildName  class containing the result of the operation and the name of the child segment.
    ///         - The Result will be:
    ///           + Success
    ///           + NotConnected
    ///           + NoFrame
    ///           + InvalidIndex
    ///           + InvalidSubjectName
    ///           + InvalidSegmentName
    virtual Output_GetSegmentChildName GetSegmentChildName( const String       & SubjectName, const String       & SegmentName, const unsigned int   SegmentIndex ) const = 0;

    /// Return the name of the parent segment for a specified subject segment. If the specified segment is the root segment of the subject then it will return an empty string.
    ///
    /// See Also: GetSegmentCount(), GetSegmentChildCount(), GetSegmentChildName(), GetSubjectRootSegmentName()
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentParentName Output;
    ///        Output = MyClient.GetSegmentParentName( "Bob", "Pelvis" );
    ///        // Output.Result == Success
    ///        // Output.SegmentName == ""
    ///        // This is the root segment
    ///        Output = MyClient.GetSegmentParentName( "Bob", "LFemur" );
    ///        // Output.Result == Success
    ///        // Output.SegmentName == "Pelvis"
    ///        
    ///        MyClient = Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        Output = MyClient.GetSegmentParentName( "Bob", "Pelvis" );
    ///        % Output.Result == Success
    ///        % Output.SegmentCount == ""
    ///        % This is the root segment
    ///        Output = MyClient.GetSegmentParentName( "Bob", "LFemur" );
    ///        % Output.Result == Success
    ///        % Output.SegmentCount == "Pelvis"
    ///          
    ///        ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentParentName Output;
    ///        Output = MyClient.GetSegmentParentName( "Bob", "Pelvis" );
    ///        // Output.Result == Success
    ///        // Output.SegmentName == ""
    ///        // This is the root segment
    ///        Output = MyClient.GetSegmentParentName( "Bob", "LFemur" );
    ///        // Output.Result == Success
    ///        // Output.SegmentName == "Pelvis"
    ///
    /// \param Subject Name The name of the subject
    /// \param Segment Name The name of the segment
    /// \return An Output_GetSegmentParentName  class containing the result of the operation and the name of the parent segment or an empty string if it is the root segment.
    ///         - The Result will be:
    ///           + Success
    ///           + NotConnected
    ///           + NoFrame
    ///           + InvalidSubjectName
    ///           + InvalidSegmentName
    virtual Output_GetSegmentParentName GetSegmentParentName( const String & SubjectName, const String & SegmentName ) const = 0;
    /// Return the static pose translation of a subject segment.
    ///
    /// See Also: GetSegmentStaticRotationHelical(), GetSegmentStaticRotationMatrix(), GetSegmentStaticRotationQuaternion(), GetSegmentStaticRotationEulerXYZ(), GetSegmentLocalTranslation(), GetSegmentLocalRotationHelical(), GetSegmentLocalRotationMatrix(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ()
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentStaticTranslation Output =
    ///        MyClient.GetSegmentStaticTranslation( "Alice", "Pelvis" );
    ///        
    ///        MyClient = Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        Output = MyClient.GetSegmentStaticTranslation( "Alice", "Pelvis" );
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentStaticTranslation Output =
    ///        MyClient.GetSegmentStaticTranslations( "Alice", "Pelvis" );
    ///
    /// \param Subject Name The name of the subject
    /// \param Segment Name The name of the segment
    /// \return An Output_GetSegmentStaticTranslation class containing the result of the operation and the translation of the segment.
    ///         - The Result will be:
    ///           + Success
    ///           + NotConnected
    ///           + NoFrame
    ///           + InvalidSubjectName
    ///           + InvalidSegmentName
    virtual Output_GetSegmentStaticTranslation GetSegmentStaticTranslation( const String & SubjectName, const String & SegmentName ) const = 0;
    /// Return the static pose rotation of a subject segment in helical coordinates.
    /// The helical coordinates represent a vector whose length is the amount of rotation in radians, and the direction is the axis about which to rotate.
    ///
    /// See Also: GetSegmentStaticTranslation(), GetSegmentStaticRotationMatrix(), GetSegmentStaticRotationQuaternion(), GetSegmentStaticRotationEulerXYZ(), GetSegmentLocalTranslation, GetSegmentLocalRotationHelical, GetSegmentLocalRotationMatrix, GetSegmentLocalRotationQuaternion, GetSegmentLocalRotationEulerXYZ
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentStaticRotationHelical Output =
    ///        MyClient.GetSegmentStaticRotationHelical( "Alice", "Pelvis" );
    ///        
    ///        MyClient = Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output = MyClient.GetSegmentStaticRotationHelical( "Alice", "Pelvis" );
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentStaticRotationHelical Output =
    ///        MyClient.GetSegmentStaticRotationHelical( "Alice", "Pelvis" );
    ///
    /// \param Subject Name The name of the subject
    /// \param Segment Name The name of the segment
    /// \return An Output_GetSegmentStaticRotationHelical  class containing the result of the operation and the rotation of the segment.
    ///         - The Result will be:
    ///           + Success
    ///           + NotConnected
    ///           + NoFrame
    ///           + InvalidSubjectName
    ///           + InvalidSegmentName
    virtual Output_GetSegmentStaticRotationHelical GetSegmentStaticRotationHelical( const String & SubjectName, const String & SegmentName ) const = 0;
    /// Return the static pose rotation of a subject segment as a 3x3 row-major matrix.
    ///
    /// See Also: GetSegmentStaticTranslation(), GetSegmentStaticRotationHelical(), GetSegmentStaticRotationQuaternion(), GetSegmentStaticRotationEulerXYZ(), GetSegmentLocalTranslation(), GetSegmentLocalRotationHelical(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ()
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentStaticRotationMatrix Output =
    ///        MyClient.GetSegmentStaticRotationMatrix( "Alice", "Pelvis" );
    ///        
    ///        MyClient = Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output = MyClient.GetSegmentStaticRotationMatrix( "Alice", "Pelvis" );
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentStaticRotationMatrix Output =
    ///        MyClient.GetSegmentStaticRotationMatrix( "Alice", "Pelvis" );
    ///
    /// \param Subject Name The name of the subject
    /// \param Segment Name The name of the segment
    /// \return An Output_GetSegmentStaticRotationMatrix  class containing the result of the operation and the rotation of the segment.
    ///         - The Result will be:
    ///           + Success
    ///           + NotConnected
    ///           + NoFrame
    ///           + InvalidSubjectName
    ///           + InvalidSegmentName
    virtual Output_GetSegmentStaticRotationMatrix GetSegmentStaticRotationMatrix( const String & SubjectName, const String & SegmentName ) const = 0;
    /// Return the static pose rotation of a subject segment in quaternion coordinates.
    /// The quaternion is of the form (x, y, z, w) where w is the real component and x, y and z are the imaginary components. 
    /// N.B. This is different from that used in many other applications, which use (w, x, y, z).
    ///
    /// See Also: GetSegmentStaticTranslation(), GetSegmentStaticRotationHelical(), GetSegmentStaticRotationMatrix(), GetSegmentStaticRotationEulerXYZ(), GetSegmentLocalTranslation(), GetSegmentLocalRotationHelical(), GetSegmentLocalRotationMatrix(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ()
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentStaticRotationQuaternion Output =
    ///        MyClient.GetSegmentStaticRotationQuaternion( "Alice", "Pelvis" );
    ///        
    ///        MyClient = Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output = MyClient.GetSegmentStaticRotationQuaternion( "Alice", "Pelvis" );
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentStaticRotationQuaternion Output =
    ///        MyClient.GetSegmentStaticRotationQuaternion( "Alice", "Pelvis" );
    ///
    ///
    /// \param Subject Name The name of the subject
    /// \param Segment Name The name of the segment
    /// \return An Output_GetSegmentStaticRotationQuaternion class containing the result of the operation and the rotation of the segment.
    ///         - The Result will be:
    ///           + Success
    ///           + NotConnected
    ///           + NoFrame
    ///           + InvalidSubjectName
    ///           + InvalidSegmentName
    virtual Output_GetSegmentStaticRotationQuaternion GetSegmentStaticRotationQuaternion( const String & SubjectName, const String & SegmentName ) const = 0;

    /// Return the static pose rotation of a subject segment in Euler XYZ coordinates.
    ///
    /// See Also: GetSegmentStaticTranslation(), GetSegmentStaticRotationHelical(), GetSegmentStaticRotationMatrix(), 
    /// GetSegmentStaticRotationQuaternion(), GetSegmentLocalTranslation(), GetSegmentLocalRotationHelical(), 
    /// GetSegmentLocalRotationMatrix(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ().
    /// 
    ///      ViconDataStreamSDK::CPP::Client MyClient;
    ///      MyClient.Connect( "localhost" );
    ///      MyClient.GetFrame();
    ///      Output_GetSegmentStaticRotationEulerXYZ Output;
    ///      Output = MyClient.GetSegmentStaticRotationEulerXYZ( "Alice", "Pelvis" );
    /// 
    ///      MyClient = Client();
    ///      MyClient.Connect( "localhost" );
    ///      MyClient.GetFrame();
    ///      Output = MyClient.GetSegmentStaticRotationEulerXYZ( "Alice", "Pelvis" );
    ///      
    ///      ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///      MyClient.Connect( "localhost" );
    ///      MyClient.GetFrame();
    ///      Output_GetSegmentStaticRotationEulerXYZ Output =
    ///      MyClient.GetSegmentStaticRotationEulerXYZ( "Alice", "Pelvis" );
    ///
    /// \param  SubjectName The name of the subject.
    /// \param  SegmentName The name of the segment.
    /// \return An Output_GetSegmentStaticRotationEulerXYZ class containing the result of the request
    ///         and the rotation of the segment \f$(x,y,z)\f$. 
    ///         - The Result will be:
    ///           + Success
    ///           + NotConnected
    ///           + NoFrame
    ///           + InvalidSubjectName
    ///           + InvalidSegmentName
    virtual Output_GetSegmentStaticRotationEulerXYZ GetSegmentStaticRotationEulerXYZ( const String & SubjectName, const String & SegmentName ) const = 0;
    /// Return the translation of a subject segment in global coordinates.
    /// The translation is of the form ( x, y, z ) where x, y and z are in millimeters with respect to the global origin.
    ///
    /// See Also: GetSegmentGlobalRotationHelical(), GetSegmentGlobalRotationMatrix(), GetSegmentGlobalRotationQuaternion(), GetSegmentGlobalRotationEulerXYZ(), GetSegmentLocalTranslation(), GetSegmentLocalRotationHelical(), GetSegmentLocalRotationMatrix(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ()
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentGlobalTranslation Output =
    ///        MyClient.GetSegmentGlobalTranslation( "Alice", "Pelvis" );
    ///        
    ///        MyClient = Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        Output = MyClient.GetSegmentGlobalTranslation( "Alice", "Pelvis" );
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient =
    ///        new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentGlobalTranslation Output =
    ///        MyClient.GetSegmentGlobalTranslations( "Alice", "Pelvis" );
    ///
    /// \param  SubjectName The name of the subject.
    /// \param  SegmentName The name of the segment.
    /// \return An Output_GetSegmentGlobalTranslation class containing the result of the operation, the translation of the segment, and whether the segment is occluded.
    ///         - The Result will be:
    ///            + Success
    ///            + NotConnected
    ///            + NoFrame
    ///            + InvalidSubjectName
    ///            + InvalidSegmentName
    ///         - Occluded will be True if the segment was absent at this frame. In this case the translation will be [0,0,0].
    virtual Output_GetSegmentGlobalTranslation GetSegmentGlobalTranslation( const String & SubjectName, const String & SegmentName ) const = 0;
    /// Return the rotation of a subject segment in global helical coordinates.
    ///
    /// See Also: GetSegmentGlobalTranslation(), GetSegmentGlobalRotationMatrix(), GetSegmentGlobalRotationQuaternion(), GetSegmentGlobalRotationEulerXYZ(), GetSegmentLocalTranslation(), GetSegmentLocalRotationHelical(), GetSegmentLocalRotationMatrix(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ()
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentGlobalRotationHelical Output =
    ///        MyClient.GetSegmentGlobalRotationHelical( "Alice", "Pelvis" );
    ///        
    ///        MyClient = Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output = MyClient.GetSegmentGlobalRotationHelical( "Alice", "Pelvis" );
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentGlobalRotationHelical Output =
    ///        MyClient.GetSegmentGlobalRotationHelical( "Alice", "Pelvis" );
    ///
    /// \param  SubjectName The name of the subject.
    /// \param  SegmentName The name of the segment.
    /// \return An Output_GetSegmentGlobalRotationHelical class containing the result of the operation, the rotation of the segment, and whether the segment is occluded.
    ///         - The Result will be:
    ///           + Success
    ///           + NotConnected
    ///           + NoFrame
    ///           + InvalidSubjectName
    ///           + InvalidSegmentName
    ///         - Occluded will be True if the segment was absent at this frame. In this case, the rotation will be [0,0,0].
    virtual Output_GetSegmentGlobalRotationHelical GetSegmentGlobalRotationHelical( const String & SubjectName, const String & SegmentName ) const = 0;

    /// Return the rotation of a subject segment as a 3x3 row-major matrix in global coordinates.
    ///
    /// See Also: GetSegmentGlobalTranslation(), GetSegmentGlobalRotationHelical(), GetSegmentGlobalRotationQuaternion(), GetSegmentGlobalRotationEulerXYZ(), GetSegmentLocalTranslation(), GetSegmentLocalRotationHelical(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ()
    ///
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentGlobalRotationMatrix Output =
    ///        MyClient.GetSegmentGlobalRotationMatrix( "Alice", "Pelvis" );
    ///        
    ///        MyClient = Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output = MyClient.GetSegmentGlobalRotationMatrix( "Alice", "Pelvis" );
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentGlobalRotationMatrix Output =
    ///        MyClient.GetSegmentGlobalRotationMatrix( "Alice", "Pelvis" );
    ///
    /// \param  SubjectName The name of the subject.
    /// \param  SegmentName The name of the segment.
    /// \return An Output_GetSegmentGlobalRotationMatrix  Class containing the result of the operation, the rotation of the segment, and whether the segment is occluded.
    ///         - The Result will be:
    ///           + Success
    ///           + NotConnected
    ///           + NoFrame
    ///           + InvalidSubjectName
    ///           + InvalidSegmentName
    ///         - Occluded will be True if the segment was absent at this frame.
    virtual Output_GetSegmentGlobalRotationMatrix GetSegmentGlobalRotationMatrix( const String & SubjectName, const String & SegmentName ) const = 0;
    /// Return the rotation of a subject segment in global quaternion coordinates.
    /// The quaternion is of the form (x, y, z, w) where w is the real component and x, y and z are the imaginary components. 
    /// N.B. This is different from that used in many other applications, which use (w, x, y, z).
    ///
    /// See Also: GetSegmentGlobalTranslation(), GetSegmentGlobalRotationHelical(), GetSegmentGlobalRotationMatrix(), GetSegmentGlobalRotationEulerXYZ(), GetSegmentLocalTranslation(), GetSegmentLocalRotationHelical(), GetSegmentLocalRotationMatrix(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ()
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentGlobalRotationQuaternion Output =
    ///        MyClient.GetSegmentGlobalRotationQuaternion( "Alice", "Pelvis" );
    ///        
    ///        MyClient = Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output = MyClient.GetSegmentGlobalRotationQuaternion( "Alice", "Pelvis" );
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentGlobalRotationQuaternion Output =
    ///        MyClient.GetSegmentGlobalRotationQuaternion( "Alice", "Pelvis" );
    ///
    /// \param  SubjectName The name of the subject.
    /// \param  SegmentName The name of the segment.
    /// \return An Output_GetSegmentGlobalRotationQuaternion  class containing the result of the operation, the rotation of the segment, and whether the segment is occluded.
    ///         - The Result will be:
    ///           + Success
    ///           + NotConnected
    ///           + NoFrame
    ///           + InvalidSubjectName
    ///           + InvalidSegmentName
    ///         - Occluded will be True if the segment was absent at this frame. In this case the Rotation will be [1,0,0,0].
    virtual Output_GetSegmentGlobalRotationQuaternion GetSegmentGlobalRotationQuaternion( const String & SubjectName, const String & SegmentName ) const = 0;
    /// Return the rotation of a subject segment in global Euler XYZ coordinates.
    ///
    /// See Also: GetSegmentGlobalTranslation(), GetSegmentGlobalRotationHelical(), GetSegmentGlobalRotationMatrix(), GetSegmentGlobalRotationQuaternion(), GetSegmentLocalTranslation(), GetSegmentLocalRotationHelical(), GetSegmentLocalRotationMatrix(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ()
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentGlobalRotationEulerXYZ Output =
    ///        MyClient.GetSegmentGlobalRotationEulerXYZ( "Alice", "Pelvis" );
    ///        
    ///        MyClient = Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output = MyClient.GetSegmentGlobalRotationEulerXYZ( "Alice", "Pelvis" );
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentGlobalRotationEulerXYZ Output =
    ///        MyClient.GetSegmentGlobalRotationEulerXYZ( "Alice", "Pelvis" );
    ///
    /// \param  SubjectName The name of the subject.
    /// \param  SegmentName The name of the segment.
    /// \return An Output_GetSegmentGlobalRotationEulerXYZ class containing the result of the operation, the rotation of the segment, and whether the segment is occluded.
    ///         - The Result will be:
    ///           + Success
    ///           + NotConnected
    ///           + NoFrame
    ///           + InvalidSubjectName
    ///           + InvalidSegmentName
    ///         - Occluded will be True if the segment was absent at this frame. In this case the rotation will be [0,0,0].
    virtual Output_GetSegmentGlobalRotationEulerXYZ GetSegmentGlobalRotationEulerXYZ( const String & SubjectName, const String & SegmentName ) const = 0;

    /// Return the translation of a subject segment in local coordinates relative to its parent segment.
    ///
    /// See Also: GetSegmentLocalRotationHelical(), GetSegmentLocalRotationMatrix(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ(), GetSegmentGlobalTranslation(), GetSegmentGlobalRotationHelical(), GetSegmentGlobalRotationMatrix(), GetSegmentGlobalRotationQuaternion(), GetSegmentGlobalRotationEulerXYZ()
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentLocalTranslation Output =
    ///        MyClient.GetSegmentLocalTranslation( "Alice", "Pelvis" );
    ///        
    ///        MyClient = Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        Output = MyClient.GetSegmentLocalTranslation( "Alice", "Pelvis" );
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.EnableSegmentData();
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentLocalTranslation Output =
    ///        MyClient.GetSegmentLocalTranslations( "Alice", "Pelvis" );
    ///
    /// \param  SubjectName The name of the subject.
    /// \param  SegmentName The name of the segment.
    /// \return An Output_GetSegmentLocalTranslation class containing the result of the operation, the translation of the segment, and whether the segment is occluded.
    ///         - The Result will be:
    ///           + Success
    ///           + NotConnected
    ///           + NoFrame
    ///           + InvalidSubjectName
    ///           + InvalidSegmentName
    ///         - Occluded will be True if the segment was absent at this frame. In this case the translation will be [0,0,0].
    virtual Output_GetSegmentLocalTranslation GetSegmentLocalTranslation( const String & SubjectName, const String & SegmentName ) const = 0;

    /// Return the rotation of a subject segment in local helical coordinates relative to its parent segment.
    ///
    /// See Also: GetSegmentLocalTranslation(), GetSegmentLocalRotationMatrix(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ(), GetSegmentGlobalTranslation(), GetSegmentGlobalRotationHelical(), GetSegmentGlobalRotationMatrix(), GetSegmentGlobalRotationQuaternion(), GetSegmentGlobalRotationEulerXYZ()
    ///
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentLocalRotationHelical Output =
    ///        MyClient.GetSegmentLocalRotationHelical( "Alice", "Pelvis" );
    ///        
    ///        MyClient = Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output = MyClient.GetSegmentLocalRotationHelical( "Alice", "Pelvis" );
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentLocalRotationHelical Output =
    ///        MyClient.GetSegmentLocalRotationHelical( "Alice", "Pelvis" );
    ///
    /// \param  SubjectName The name of the subject.
    /// \param  SegmentName The name of the segment.
    /// \return An Output_GetSegmentLocalRotationHelical class containing the result of the operation, the rotation of the segment, and whether the segment is occluded.
    ///         - The Result will be:
    ///           + Success
    ///           + NotConnected
    ///           + NoFrame
    ///           + InvalidSubjectName
    ///           + InvalidSegmentName
    ///         - Occluded will be True if the segment was absent at this frame. In this case the Rotation will be [0,0,0].
    virtual Output_GetSegmentLocalRotationHelical GetSegmentLocalRotationHelical( const String & SubjectName, const String & SegmentName ) const = 0;

    /// Return the rotation row-major matrix of a subject segment in local coordinates relative to its parent segment.
    ///
    /// See Also: GetSegmentLocalTranslation(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ(), GetSegmentGlobalTranslation(), GetSegmentGlobalRotationHelical(), GetSegmentGlobalRotationMatrix() , GetSegmentGlobalRotationQuaternion(), GetSegmentGlobalRotationEulerXYZ()
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentLocalRotationMatrix Output =
    ///        MyClient.GetSegmentLocalRotationMatrix( "Alice", "Pelvis" );
    ///        
    ///        MyClient = Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output = MyClient.GetSegmentLocalRotationMatrix( "Alice", "Pelvis" );
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentLocalRotationMatrix Output =
    ///        MyClient.GetSegmentLocalRotationMatrix( "Alice", "Pelvis" );
    ///
    /// \param  SubjectName The name of the subject.
    /// \param  SegmentName The name of the segment.
    /// \return An Output_GetSegmentLocalRotationMatrix class containing the result of the operation, the rotation of the segment, and whether the segment is occluded.
    ///         - The Result will be:
    ///           + Success
    ///           + NotConnected
    ///           + NoFrame
    ///           + InvalidSubjectName
    ///           + InvalidSegmentName
    ///         - Occluded will be True if the segment was absent at this frame.
    //
    virtual Output_GetSegmentLocalRotationMatrix GetSegmentLocalRotationMatrix( const String & SubjectName, const String & SegmentName ) const = 0;
    /// Return the rotation of a subject segment in local quaternion coordinates relative to its parent segment.
    /// The quaternion is of the form (x, y, z, w) where w is the real component and x, y and z are the imaginary components. N.B. This is different from that used in many other applications, which use (w, x, y, z).
    ///
    /// See Also: GetSegmentLocalTranslation(), GetSegmentLocalRotationHelical(), GetSegmentLocalRotationMatrix(), GetSegmentLocalRotationEulerXYZ(), GetSegmentGlobalTranslation(), GetSegmentGlobalRotationHelical(), GetSegmentGlobalRotationMatrix(), GetSegmentGlobalRotationQuaternion(), GetSegmentGlobalRotationEulerXYZ()
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentLocalRotationQuaternion Output =
    ///        MyClient.GetSegmentLocalRotationQuaternion( "Alice", "Pelvis" );
    ///        
    ///        MyClient = Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output = MyClient.GetSegmentLocalRotationQuaternion( "Alice", "Pelvis" );
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentLocalRotationQuaternion Output =
    ///        MyClient.GetSegmentLocalRotationQuaternion( "Alice", "Pelvis" );
    ///
    /// \param  SubjectName The name of the subject.
    /// \param  SegmentName The name of the segment.
    /// \return An Output_GetSegmentLocalRotationQuaternion class containing the result of the operation, the rotation of the segment, and whether the segment is occluded.
    ///         - The Result will be:
    ///           + Success
    ///           + NotConnected
    ///           + NoFrame
    ///           + InvalidSubjectName
    ///           + InvalidSegmentName
    ///         - Occluded will be True if the segment was absent at this frame. In this case the rotation will be [1,0,0,0].
    virtual Output_GetSegmentLocalRotationQuaternion GetSegmentLocalRotationQuaternion( const String & SubjectName, const String & SegmentName ) const = 0;

    /// Return the rotation of a subject segment in local Euler XYZ coordinates relative to its parent segment.
    ///
    /// See Also: GetSegmentLocalTranslation(), GetSegmentLocalRotationHelical(), GetSegmentLocalRotationMatrix(), GetSegmentLocalRotationQuaternion(), GetSegmentGlobalTranslation(), GetSegmentGlobalRotationHelical(), GetSegmentGlobalRotationMatrix() , GetSegmentGlobalRotationQuaternion(), GetSegmentGlobalRotationEulerXYZ()
    ///
    ///
    ///        ViconDataStreamSDK::CPP::Client MyClient;
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentLocalRotationEulerXYZ Output =
    ///        MyClient.GetSegmentLocalRotationEulerXYZ( "Alice", "Pelvis" );
    ///        
    ///        MyClient = Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output = MyClient.GetSegmentLocalRotationEulerXYZ( "Alice", "Pelvis" );
    ///        
    ///        ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
    ///        MyClient.Connect( "localhost" );
    ///        MyClient.GetFrame();
    ///        Output_GetSegmentLocalRotationEulerXYZ Output =
    ///        MyClient.GetSegmentLocalRotationEulerXYZ( "Alice", "Pelvis" );
    ///
    /// \param  SubjectName The name of the subject.
    /// \param  SegmentName The name of the segment.
    /// \return An Output_GetSegmentLocalRotationEulerXYZ class containing the result of the operation, the rotation of the segment, and whether the segment is occluded.
    ///         - The Result will be:
    ///           + Success
    ///           + NotConnected
    ///           + NoFrame
    ///           + InvalidSubjectName
    ///           + InvalidSegmentName
    ///         - Occluded will be True if the segment was absent at this frame. In this case the rotation will be [0,0,0].
    virtual Output_GetSegmentLocalRotationEulerXYZ GetSegmentLocalRotationEulerXYZ( const String & SubjectName, const String & SegmentName ) const = 0;

  };
} // End of namespace CPP
} // End of namespace ViconDataStreamSDK
