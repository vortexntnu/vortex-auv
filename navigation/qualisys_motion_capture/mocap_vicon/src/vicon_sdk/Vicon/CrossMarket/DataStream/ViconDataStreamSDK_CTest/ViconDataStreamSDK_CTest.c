
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

#include <CClient.h>
#include <CTypeDefs.h>

#pragma warning( push )
#pragma warning( disable:4255 )

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <conio.h>
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <time.h>

#pragma warning( pop )


  CString Adapt(const CBool i_Value)
  {
    return i_Value ? "True" : "False";
  }

  CString AdaptStandard( CTimecodeStandard i_Standard )
  {
    switch( i_Standard )
    {
    default:
    case None:
      return "0";
    case PAL:
      return "1";
    case NTSC:
      return "2";
    case NTSCDrop:
      return "3";
    case Film:
      return "4";
    case NTSCFilm:
      return "5";
    case ATSC:
      return "6";
    }
  }

  CString AdaptDirection( CDirection i_Direction)
  {
    switch (i_Direction)
    {
    case Forward:
      return "Forward";
    case Backward:
      return "Backward";
    case Left:
      return "Left";
    case Right:
      return "Right";
    case Up:
      return "Up";
    case Down:
      return "Down";
    default:
      return "Unknown";
    }
  }

  CString AdaptDeviceType( CDeviceType i_DeviceType)
  {
    switch (i_DeviceType)
    {
    case ForcePlate:
      return "ForcePlate";
    default:
      return "Unknown";
    }
  }

  CString AdaptUnit( CUnit i_Unit)
  {
    switch (i_Unit)
    {
    case Meter:
      return "Meter";
    case Volt:
      return "Volt";
    case NewtonMeter:
      return "NewtonMeter";
    case Newton:
      return "Newton";
    case Kilogram:
      return "Kilogram";
    case Second:
      return "Second";
    case Ampere:
      return "Ampere";
    case Kelvin:
      return "Kelvin";
    case Mole:
      return "Mole";
    case Candela:
      return "Candela";
    case Radian:
      return "Radian";
    case Steradian:
      return "Steradian";
    case MeterSquared:
      return "MeterSquared";
    case MeterCubed:
      return "MeterCubed";
    case MeterPerSecond:
      return "MeterPerSecond";
    case MeterPerSecondSquared:
      return "MeterPerSecondSquared";
    case RadianPerSecond:
      return "RadianPerSecond";
    case RadianPerSecondSquared:
      return "RadianPerSecondSquared";
    case Hertz:
      return "Hertz";
    case Joule:
      return "Joule";
    case Watt:
      return "Watt";
    case Pascal:
      return "Pascal";
    case Lumen:
      return "Lumen";
    case Lux:
      return "Lux";
    case Coulomb:
      return "Coulomb";
    case Ohm:
      return "Ohm";
    case Farad:
      return "Farad";
    case Weber:
      return "Weber";
    case Tesla:
      return "Tesla";
    case Henry:
      return "Henry";
    case Siemens:
      return "Siemens";
    case Becquerel:
      return "Becquerel";
    case Gray:
      return "Gray";
    case Sievert:
      return "Sievert";
    case Katal:
      return "Katal";
    default:
      return "Unknown";
    }
  }

#ifdef WIN32
  CBool Hit(void)
  {
    CBool hit = 0;
    while (_kbhit())
    {
      getchar();
      hit = 1;
    }
    return hit;
  }
#endif

int main(int argc, char* argv[])
{
  // Program options

  CString HostName = "localhost:801";
  if (argc > 1)
  {
    HostName = argv[1];
  }

  // log contains:
  // version number
  // log of framerate over time
  // --multicast
  // kill off internal app
  CString LogFile = "";
  CString MulticastAddress = "244.0.0.0:44801";
  CBool ConnectToMultiCast = 0;
  CBool EnableMultiCast = 0;
  CBool bReadCentroids = 0;
  CBool bReadRayData = 0;

  unsigned int ClientBufferSize = 0;
  CString AxisMapping = "ZUp";
  
  int a;
  for (a = 2; a < argc; ++a)
  {
    CString arg = argv[a];
    if ( strcmp( arg, "--help") == 0 )
    {
      printf( argv[0], " <HostName>: allowed options include:\n  --log_file <LogFile> --enable_multicast <MulticastAddress:Port> --connect_to_multicast <MulticastAddress:Port> --help --centroids --client-buffer-size <size>\n");
      return 0;
    }
    else if ( strcmp( arg, "--log_file") == 0 )
    {
      if (a < argc)
      {
        LogFile = argv[a + 1];
        printf("Using log file < %s> ...\n", LogFile);
        ++a;
      }
    }
    else if ( strcmp( arg, "--enable_multicast") == 0 )
    {
      EnableMultiCast = 1;
      if (a < argc)
      {
        CString MulticastAddress = argv[a + 1];
        printf( "Enabling multicast address <%s> ...\n", MulticastAddress);
        a = a +1;
      }
    }
    else if ( strcmp( arg, "--connect_to_multicast") == 0 )
    {
      ConnectToMultiCast = 1;
      if (a < argc)
      {
        MulticastAddress = argv[a + 1];
        printf( "connecting to multicast address <%s> ...\n", MulticastAddress );
        ++a;
      }
    }
    else if ( strcmp( arg, "--centroids") == 0  )
    {
      bReadCentroids = 1;
    }
    else if( strcmp( arg, "--rays" ) == 0 )
    {
      bReadRayData = 1;
    }
    else if (strcmp(arg, "--greyscale") == 0)
    {
      printf("Greyscale data not supported by the C Client\n");
    }
    else if (strcmp(arg, "--video") == 0)
    {
      printf("Video data not supported by the C Client\n");
    }
    else if( strcmp( arg, "--client-buffer-size" ) == 0 )
    {
      ++a;
      if (a < argc)
      {
        ClientBufferSize = atoi(argv[a]);
      }
    }
    else if( strcmp( arg, "--set-axis-mapping" ) == 0 )
    {
      ++a;
      if (a < argc)
      {
        AxisMapping = argv[a];

        if ( strcmp( AxisMapping, "XUp" ) == 0 || strcmp( AxisMapping, "YUp" ) == 0 || strcmp( AxisMapping, "ZUp") == 0 )
        {
          printf( "Setting Axis to %s\n", AxisMapping);
        }
        else
        {
          printf( "Unknown axis setting: %s . Should be XUp, YUp, or ZUp\n", AxisMapping);
          return 1;
        }
      }
    }
    else
    {
      printf("Failed to understand argument <%s>...exiting\n", argv[a]);
      return 1;
    }
  }

  /*
  std::ofstream ofs;
  if (!LogFile.empty())
  {
    ofs.open(LogFile.c_str());
    if (!ofs.is_open())
    {
      printf( "Could not open log file <", LogFile, ">...exiting", "\n");
      return 1;
    }
  }
  */

  // Make a new client
  CClient* pClient = Client_Create();
  
  int i;
  for (i = 0; i != 3; ++i) // repeat to check disconnecting doesn't wreck next connect
  {
    // Connect to a server
    printf( "Connecting to %s...", HostName);
    while (!Client_IsConnected(pClient))
    {
      // Direct connection

      CBool ok = 0;
      if (ConnectToMultiCast)
      {
        // Multicast connection
        ok = (Client_ConnectToMulticast(pClient, HostName, MulticastAddress));

      }
      else
      {        
        ok = Client_Connect(pClient, HostName);
      }
      if (!ok)
      {
        printf("Warning - connect failed...\n");
      }


      printf(".");
#ifdef WIN32
      Sleep(1000);
#else
      sleep(1);
#endif
    }
    printf("\n");


    // Enable some different data types
    Client_EnableSegmentData(pClient);
    Client_EnableMarkerData(pClient);
    Client_EnableUnlabeledMarkerData(pClient);
    Client_EnableMarkerRayData(pClient);
    Client_EnableDeviceData(pClient);
    Client_EnableDebugData(pClient);
    if (bReadCentroids)
    {
      Client_EnableCentroidData(pClient);
    }
    if (bReadRayData)
    {
      Client_EnableMarkerRayData(pClient);
    }

    printf( "Segment Data Enabled: %s\n", Adapt(Client_IsSegmentDataEnabled(pClient)));
    printf( "Marker Data Enabled: %s\n",  Adapt(Client_IsMarkerDataEnabled(pClient)));
    printf( "Unlabeled Marker Data Enabled: %s\n", Adapt(Client_IsUnlabeledMarkerDataEnabled(pClient)));
    printf( "Device Data Enabled: %s\n", Adapt(Client_IsDeviceDataEnabled(pClient)));
    printf( "Centroid Data Enabled: %s\n", Adapt(Client_IsCentroidDataEnabled(pClient)));
    printf( "Marker Ray Data Enabled: %s\n", Adapt(Client_IsMarkerRayDataEnabled(pClient)));

    // Set the streaming mode
    //Client_SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPull );
    // Client_SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch );
    Client_SetStreamMode(pClient, ServerPush);

    // Set the global up axis
    Client_SetAxisMapping(pClient, Forward, Left, Up); // Z-up

    if (strcmp( AxisMapping, "YUp") == 0 )
    {
      Client_SetAxisMapping(pClient, Forward, Up, Right); // Y-up
    }
    else if( strcmp(AxisMapping, "XUp") == 0 )
    {
      Client_SetAxisMapping(pClient, Up, Forward, Left); // Y-up
    }

    COutput_GetAxisMapping _Output_GetAxisMapping;
    Client_GetAxisMapping(pClient, &_Output_GetAxisMapping);
    printf( "Axis Mapping: X-%s Y-%s Z-%s\n"
       , AdaptDirection(_Output_GetAxisMapping.XAxis)
       , AdaptDirection(_Output_GetAxisMapping.YAxis)
       , AdaptDirection(_Output_GetAxisMapping.ZAxis)
    );

    // Discover the version number
    COutput_GetVersion _Output_GetVersion;
    Client_GetVersion(pClient, &_Output_GetVersion);
    printf( "Version: %d.%d.%d\n", _Output_GetVersion.Major, _Output_GetVersion.Minor , _Output_GetVersion.Point);

    if (EnableMultiCast)
    {

      //assert(Adapt(Client_IsConnected(pClient)));
      Client_StartTransmittingMulticast(pClient, HostName, MulticastAddress);
    }

    unsigned int FrameRateWindow = 1000; // frames
    unsigned int Counter = 0;
    clock_t LastTime = clock();
    // Loop until a key is pressed
#ifdef WIN32
    while (!Hit())
#else
    while (1)
#endif
    {
      // Get a frame
      printf( "Waiting for new frame..." );
      while (Client_GetFrame(pClient) != Success)
      {
        // Sleep a little so that we don't lumber the CPU with a busy poll
#ifdef WIN32
        Sleep(200);
#else
        sleep(1);
#endif

        printf( "." );
      }
      printf( "\n" );
      if (++Counter == FrameRateWindow)
      {
        clock_t Now = clock();
        double FrameRate = (double)(FrameRateWindow * CLOCKS_PER_SEC) / (double)(Now - LastTime);
        if (!LogFile)
        {
          time_t rawtime;
          struct tm * timeinfo = 0;
          time(&rawtime);
#ifdef _WIN32
          localtime_s(timeinfo, &rawtime);
          size_t TimeBufSize = 128;
          char TimeBuf[ 128 ];
          asctime_s( TimeBuf, TimeBufSize, timeinfo );
          printf( "Frame rate = %f at %s\n", FrameRate, TimeBuf );
#else
          timeinfo = localtime(&rawtime);
          char * TimeBuf = asctime( timeinfo );
          printf( "Frame rate = %f at %s\n", FrameRate, TimeBuf );
#endif
        }

        LastTime = Now;
        Counter = 0;
      }

      // Get the frame number
      COutput_GetFrameNumber _Output_GetFrameNumber;
      Client_GetFrameNumber(pClient, &_Output_GetFrameNumber);
      printf(  "Frame Number: %d\n", _Output_GetFrameNumber.FrameNumber);

      COutput_GetFrameRate Rate;
      Client_GetFrameRate(pClient, &Rate);
      // Using %g rather than %f in an attempt to copy std::cout's default output formatting as used in the other clients.
      // From https://stackoverflow.com/questions/2411903/getting-printf-to-drop-the-trailing-0-of-values
      printf( "Frame rate: %.6g\n", Rate.FrameRateHz );

      // Show frame rates
      COutput_GetFrameRateCount RateCount;
      Client_GetFrameRateCount(pClient, &RateCount);
      
      unsigned int FramerateIndex;
      for (FramerateIndex = 0; FramerateIndex < RateCount.Count; ++FramerateIndex)
      {
        char FramerateName[128];
        Client_GetFrameRateName(pClient, FramerateIndex, 128, FramerateName);
        COutput_GetFrameRateValue FramerateValue;
        Client_GetFrameRateValue(pClient, FramerateName, &FramerateValue);

        printf( "%s: %.6gHz\n", FramerateName, FramerateValue.Value );
      }
      printf( "\n" );

      // Get the timecode
      COutput_GetTimecode _Output_GetTimecode;
      Client_GetTimecode(pClient, &_Output_GetTimecode);

      //TODO Standard enum may cause an issue but this may be a bug in the cpp code!
      printf( "Timecode: %dh %dm %ds %df %dsf %s %s %d %d\n\n"
        , _Output_GetTimecode.Hours
        , _Output_GetTimecode.Minutes
        , _Output_GetTimecode.Seconds
        , _Output_GetTimecode.Frames
        , _Output_GetTimecode.SubFrame
        , Adapt(_Output_GetTimecode.FieldFlag)
        , AdaptStandard(_Output_GetTimecode.Standard)
        , _Output_GetTimecode.SubFramesPerFrame
        , _Output_GetTimecode.UserBits
      );

      // Get the latency
      
      COutput_GetLatencyTotal LatencyTotal;
      Client_GetLatencyTotal(pClient, &LatencyTotal);
      printf( "Latency: %fs\n", LatencyTotal.Total);
      
      COutput_GetLatencySampleCount LatencySampleCount;
      Client_GetLatencySampleCount(pClient, &LatencySampleCount);
      unsigned int LatencySampleIndex;
      for (LatencySampleIndex = 0; LatencySampleIndex < LatencySampleCount.Count; ++LatencySampleIndex)
      {
        char SampleName[128];
        Client_GetLatencySampleName(pClient, LatencySampleIndex, 128, SampleName);
        COutput_GetLatencySampleValue SampleValue;
        Client_GetLatencySampleValue(pClient, SampleName, &SampleValue);

        printf( "  %s %fs\n", SampleName, SampleValue.Value );
      }
      printf( "\n" );

      COutput_GetHardwareFrameNumber _Output_GetHardwareFrameNumber;
      Client_GetHardwareFrameNumber(pClient, &_Output_GetHardwareFrameNumber);
      printf( "Hardware Frame Number: %d\n", _Output_GetHardwareFrameNumber.HardwareFrameNumber );



      // Count the number of subjects
      COutput_GetSubjectCount SubjectCount;
      Client_GetSubjectCount(pClient, &SubjectCount);
      printf( "Subjects (%d):\n", SubjectCount.SubjectCount );
      unsigned int SubjectIndex;
      for (SubjectIndex = 0; SubjectIndex < SubjectCount.SubjectCount; ++SubjectIndex)
      {
        printf( "  Subject #%d\n", SubjectIndex);

        // Get the subject name
        char SubjectName[128];
        Client_GetSubjectName(pClient, SubjectIndex, 128, SubjectName);
        printf( "    Name: %s\n", SubjectName);

        // Get the root segment
        char RootSegment[128];
        Client_GetSubjectRootSegmentName(pClient, SubjectName, 128, RootSegment);
        printf( "    Root Segment: %s\n", RootSegment);

        // Count the number of segments
        COutput_GetSegmentCount SegmentCount;
        Client_GetSegmentCount(pClient, SubjectName, &SegmentCount);

        printf( "    Segments (%d):\n", SegmentCount.SegmentCount);
        unsigned int SegmentIndex;
        for (SegmentIndex = 0; SegmentIndex < SegmentCount.SegmentCount; ++SegmentIndex)
        {
          printf( "      Segment #%d\n", SegmentIndex);

          // Get the segment name
          char SegmentName[128];
          Client_GetSegmentName(pClient, SubjectName, SegmentIndex, 128, SegmentName);
          printf( "        Name: %s\n", SegmentName);

          // Get the segment parent
          char SegmentParentName[128];
          Client_GetSegmentParentName(pClient, SubjectName, SegmentName, 128, SegmentParentName);
          printf( "        Parent: %s\n", SegmentParentName );

          // Get the segment's children
          COutput_GetSegmentChildCount ChildCount;
          Client_GetSegmentChildCount(pClient, SubjectName, SegmentName, &ChildCount);
          printf( "     Children (%d):\n", ChildCount.SegmentCount );
          unsigned int ChildIndex;
          for (ChildIndex = 0; ChildIndex < ChildCount.SegmentCount; ++ChildIndex)
          {
            char SegmentChildName[128];
            Client_GetSegmentChildName( pClient, SubjectName, SegmentName, ChildIndex, 128, SegmentChildName );
            printf( "       %s\n", SegmentChildName );
          }

          // Get the static segment translation
          COutput_GetSegmentStaticTranslation _Output_GetSegmentStaticTranslation;
          Client_GetSegmentStaticTranslation(pClient, SubjectName, SegmentName, &_Output_GetSegmentStaticTranslation);
          printf( "        Static Translation: (%f, %f, %f)\n"
            , _Output_GetSegmentStaticTranslation.Translation[0]
            , _Output_GetSegmentStaticTranslation.Translation[1]
            , _Output_GetSegmentStaticTranslation.Translation[2]
          );

          // Get the static segment rotation in helical co-ordinates
          COutput_GetSegmentStaticRotationHelical _Output_GetSegmentStaticRotationHelical;
          Client_GetSegmentStaticRotationHelical(pClient, SubjectName, SegmentName, &_Output_GetSegmentStaticRotationHelical);
          printf( "        Static Rotation Helical: (%f, %f, %f)\n"
            , _Output_GetSegmentStaticRotationHelical.Rotation[0]
            , _Output_GetSegmentStaticRotationHelical.Rotation[1]
            , _Output_GetSegmentStaticRotationHelical.Rotation[2]
          );

          // Get the static segment rotation as a matrix
          COutput_GetSegmentStaticRotationMatrix _Output_GetSegmentStaticRotationMatrix;
          Client_GetSegmentStaticRotationMatrix(pClient, SubjectName, SegmentName, &_Output_GetSegmentStaticRotationMatrix);
          printf( "        Static Rotation Matrix: (%f, %f, %f, %f, %f, %f, %f, %f, %f)\n"
            , _Output_GetSegmentStaticRotationMatrix.Rotation[0]
            , _Output_GetSegmentStaticRotationMatrix.Rotation[1]
            , _Output_GetSegmentStaticRotationMatrix.Rotation[2]
            , _Output_GetSegmentStaticRotationMatrix.Rotation[3]
            , _Output_GetSegmentStaticRotationMatrix.Rotation[4]
            , _Output_GetSegmentStaticRotationMatrix.Rotation[5]
            , _Output_GetSegmentStaticRotationMatrix.Rotation[6]
            , _Output_GetSegmentStaticRotationMatrix.Rotation[7]
            , _Output_GetSegmentStaticRotationMatrix.Rotation[8]
          );

          // Get the static segment rotation in quaternion co-ordinates
          COutput_GetSegmentStaticRotationQuaternion _Output_GetSegmentStaticRotationQuaternion;
          Client_GetSegmentStaticRotationQuaternion(pClient, SubjectName, SegmentName, &_Output_GetSegmentStaticRotationQuaternion);
          printf( "        Static Rotation Quaternion: (%f, %f, %f, %f)\n"
            , _Output_GetSegmentStaticRotationQuaternion.Rotation[0]
            , _Output_GetSegmentStaticRotationQuaternion.Rotation[1]
            , _Output_GetSegmentStaticRotationQuaternion.Rotation[2]
            , _Output_GetSegmentStaticRotationQuaternion.Rotation[3]
          );

          // Get the static segment rotation in EulerXYZ co-ordinates
          COutput_GetSegmentStaticRotationEulerXYZ _Output_GetSegmentStaticRotationEulerXYZ;
          Client_GetSegmentStaticRotationEulerXYZ(pClient, SubjectName, SegmentName, &_Output_GetSegmentStaticRotationEulerXYZ);
          printf( "        Static Rotation EulerXYZ: (%f, %f, %f)\n"
            , _Output_GetSegmentStaticRotationEulerXYZ.Rotation[0]
            , _Output_GetSegmentStaticRotationEulerXYZ.Rotation[1]
            , _Output_GetSegmentStaticRotationEulerXYZ.Rotation[2]
          );

          // Get the global segment translation
          COutput_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation;
          Client_GetSegmentGlobalTranslation(pClient, SubjectName, SegmentName, &_Output_GetSegmentGlobalTranslation);
          printf( "        Global Translation: (%f, %f, %f) %s\n"
            , _Output_GetSegmentGlobalTranslation.Translation[0]
            , _Output_GetSegmentGlobalTranslation.Translation[1]
            , _Output_GetSegmentGlobalTranslation.Translation[2]
            , Adapt(_Output_GetSegmentGlobalTranslation.Occluded)
          );

          // Get the global segment rotation in helical co-ordinates
          COutput_GetSegmentGlobalRotationHelical _Output_GetSegmentGlobalRotationHelical;
          Client_GetSegmentGlobalRotationHelical(pClient, SubjectName, SegmentName, &_Output_GetSegmentGlobalRotationHelical);
          printf( "        Global Rotation Helical: (%f, %f, %f) %s\n"
            , _Output_GetSegmentGlobalRotationHelical.Rotation[0]
            , _Output_GetSegmentGlobalRotationHelical.Rotation[1]
            , _Output_GetSegmentGlobalRotationHelical.Rotation[2]
            , Adapt(_Output_GetSegmentGlobalRotationHelical.Occluded)
          );

          // Get the global segment rotation as a matrix
          COutput_GetSegmentGlobalRotationMatrix _Output_GetSegmentGlobalRotationMatrix;
          Client_GetSegmentGlobalRotationMatrix(pClient, SubjectName, SegmentName, &_Output_GetSegmentGlobalRotationMatrix);
          printf( "        Global Rotation Matrix: (%f, %f, %f, %f, %f, %f, %f, %f, %f) %s\n"
            , _Output_GetSegmentGlobalRotationMatrix.Rotation[0]
            , _Output_GetSegmentGlobalRotationMatrix.Rotation[1]
            , _Output_GetSegmentGlobalRotationMatrix.Rotation[2]
            , _Output_GetSegmentGlobalRotationMatrix.Rotation[3]
            , _Output_GetSegmentGlobalRotationMatrix.Rotation[4]
            , _Output_GetSegmentGlobalRotationMatrix.Rotation[5]
            , _Output_GetSegmentGlobalRotationMatrix.Rotation[6]
            , _Output_GetSegmentGlobalRotationMatrix.Rotation[7]
            , _Output_GetSegmentGlobalRotationMatrix.Rotation[8]
            , Adapt(_Output_GetSegmentGlobalRotationMatrix.Occluded));

          // Get the global segment rotation in quaternion co-ordinates
          COutput_GetSegmentGlobalRotationQuaternion _Output_GetSegmentGlobalRotationQuaternion;
          Client_GetSegmentGlobalRotationQuaternion(pClient, SubjectName, SegmentName, &_Output_GetSegmentGlobalRotationQuaternion);
          printf( "        Global Rotation Quaternion: (%f, %f, %f, %f) %s\n"
            , _Output_GetSegmentGlobalRotationQuaternion.Rotation[0]
            , _Output_GetSegmentGlobalRotationQuaternion.Rotation[1]
            , _Output_GetSegmentGlobalRotationQuaternion.Rotation[2]
            , _Output_GetSegmentGlobalRotationQuaternion.Rotation[3]
            , Adapt(_Output_GetSegmentGlobalRotationQuaternion.Occluded));

          // Get the global segment rotation in EulerXYZ co-ordinates
          COutput_GetSegmentGlobalRotationEulerXYZ _Output_GetSegmentGlobalRotationEulerXYZ;
          Client_GetSegmentGlobalRotationEulerXYZ(pClient, SubjectName, SegmentName, &_Output_GetSegmentGlobalRotationEulerXYZ);
          printf( "        Global Rotation EulerXYZ: (%f, %f, %f) %s\n"
            , _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[0]
            , _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[1]
            , _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[2]
            , Adapt(_Output_GetSegmentGlobalRotationEulerXYZ.Occluded));

          // Get the local segment translation
          COutput_GetSegmentLocalTranslation _Output_GetSegmentLocalTranslation;
          Client_GetSegmentLocalTranslation(pClient, SubjectName, SegmentName, &_Output_GetSegmentLocalTranslation);
          printf( "        Local Translation: (%f, %f, %f) %s\n"
            , _Output_GetSegmentLocalTranslation.Translation[0]
            , _Output_GetSegmentLocalTranslation.Translation[1]
            , _Output_GetSegmentLocalTranslation.Translation[2]
            , Adapt(_Output_GetSegmentLocalTranslation.Occluded));

          // Get the local segment rotation in helical co-ordinates
          COutput_GetSegmentLocalRotationHelical _Output_GetSegmentLocalRotationHelical;
          Client_GetSegmentLocalRotationHelical(pClient, SubjectName, SegmentName, &_Output_GetSegmentLocalRotationHelical);
          printf( "        Local Rotation Helical: (%f, %f, %f) %s\n"
            , _Output_GetSegmentLocalRotationHelical.Rotation[0]
            , _Output_GetSegmentLocalRotationHelical.Rotation[1]
            , _Output_GetSegmentLocalRotationHelical.Rotation[2]
            , Adapt(_Output_GetSegmentLocalRotationHelical.Occluded));

          // Get the local segment rotation as a matrix
          COutput_GetSegmentLocalRotationMatrix _Output_GetSegmentLocalRotationMatrix;
          Client_GetSegmentLocalRotationMatrix(pClient, SubjectName, SegmentName, &_Output_GetSegmentLocalRotationMatrix);
          printf( "        Local Rotation Matrix: (%f, %f, %f, %f, %f, %f, %f, %f, %f) %s\n"
            , _Output_GetSegmentLocalRotationMatrix.Rotation[0]
            , _Output_GetSegmentLocalRotationMatrix.Rotation[1]
            , _Output_GetSegmentLocalRotationMatrix.Rotation[2]
            , _Output_GetSegmentLocalRotationMatrix.Rotation[3]
            , _Output_GetSegmentLocalRotationMatrix.Rotation[4]
            , _Output_GetSegmentLocalRotationMatrix.Rotation[5]
            , _Output_GetSegmentLocalRotationMatrix.Rotation[6]
            , _Output_GetSegmentLocalRotationMatrix.Rotation[7]
            , _Output_GetSegmentLocalRotationMatrix.Rotation[8]
            , Adapt(_Output_GetSegmentLocalRotationMatrix.Occluded));

          // Get the local segment rotation in quaternion co-ordinates
          COutput_GetSegmentLocalRotationQuaternion _Output_GetSegmentLocalRotationQuaternion;
          Client_GetSegmentLocalRotationQuaternion(pClient, SubjectName, SegmentName, &_Output_GetSegmentLocalRotationQuaternion);
          printf( "        Local Rotation Quaternion: (%f, %f, %f, %f) %s\n"
            , _Output_GetSegmentLocalRotationQuaternion.Rotation[0]
            , _Output_GetSegmentLocalRotationQuaternion.Rotation[1]
            , _Output_GetSegmentLocalRotationQuaternion.Rotation[2]
            , _Output_GetSegmentLocalRotationQuaternion.Rotation[3]
            , Adapt(_Output_GetSegmentLocalRotationQuaternion.Occluded));

          // Get the local segment rotation in EulerXYZ co-ordinates
          COutput_GetSegmentLocalRotationEulerXYZ _Output_GetSegmentLocalRotationEulerXYZ;
          Client_GetSegmentLocalRotationEulerXYZ(pClient, SubjectName, SegmentName, &_Output_GetSegmentLocalRotationEulerXYZ);
          printf( "        Local Rotation EulerXYZ: (%f, %f, %f) %s\n"
            , _Output_GetSegmentLocalRotationEulerXYZ.Rotation[0]
            , _Output_GetSegmentLocalRotationEulerXYZ.Rotation[1]
            , _Output_GetSegmentLocalRotationEulerXYZ.Rotation[2]
            , Adapt(_Output_GetSegmentLocalRotationEulerXYZ.Occluded));
        }

        // Get the quality of the subject (object) if supported
        COutput_GetObjectQuality _Output_GetObjectQuality;
        Client_GetObjectQuality(pClient, SubjectName, &_Output_GetObjectQuality);
        if (_Output_GetObjectQuality.Result == Success)
        {
          double Quality = _Output_GetObjectQuality.Quality;
          printf( "    Quality: %f\n", Quality);
        }

        // Count the number of markers
        COutput_GetMarkerCount MarkerCount;
        Client_GetMarkerCount(pClient, SubjectName, &MarkerCount);
        printf( "    Markers (%d):\n", MarkerCount.MarkerCount);
        unsigned int MarkerIndex;
        for (MarkerIndex = 0; MarkerIndex < MarkerCount.MarkerCount; ++MarkerIndex)
        {
          // Get the marker name
          char MarkerName[128];
          Client_GetMarkerName(pClient, SubjectName, MarkerIndex, 128, MarkerName);

          // Get the marker parent
          char MarkerParentName[128];
          Client_GetMarkerParentName(pClient, SubjectName, MarkerName, 128, MarkerParentName);

          // Get the global marker translation
          COutput_GetMarkerGlobalTranslation _Output_GetMarkerGlobalTranslation;
          Client_GetMarkerGlobalTranslation(pClient, SubjectName, MarkerName, &_Output_GetMarkerGlobalTranslation);

          printf( "      Marker #%d: %s (%f, %f, %f) %s\n"
            , MarkerIndex
            , MarkerName
            , _Output_GetMarkerGlobalTranslation.Translation[0]
            , _Output_GetMarkerGlobalTranslation.Translation[1]
            , _Output_GetMarkerGlobalTranslation.Translation[2]
            , Adapt(_Output_GetMarkerGlobalTranslation.Occluded));

          if (bReadRayData)
          {
            COutput_GetMarkerRayContributionCount _Output_GetMarkerRayContributionCount;
            Client_GetMarkerRayContributionCount(pClient, SubjectName, MarkerName, &_Output_GetMarkerRayContributionCount);

            if (_Output_GetMarkerRayContributionCount.Result == Success)
            {
              printf( "      Contributed to by: ");
              unsigned int ContributionIndex;
              for (ContributionIndex = 0; ContributionIndex < _Output_GetMarkerRayContributionCount.RayContributionsCount; ++ContributionIndex)
              {
                COutput_GetMarkerRayContribution _Output_GetMarkerRayContribution;
                Client_GetMarkerRayContribution(pClient, SubjectName, MarkerName, ContributionIndex, &_Output_GetMarkerRayContribution);
                printf( "ID:%d Index:%d "
                  , _Output_GetMarkerRayContribution.CameraID
                  , _Output_GetMarkerRayContribution.CentroidIndex);
              }
              printf( "\n" );
            }
          }
        }
      }

      // Get the unlabeled markers
      COutput_GetUnlabeledMarkerCount UnlabeledMarkerCount;
      Client_GetUnlabeledMarkerCount(pClient, &UnlabeledMarkerCount);
      printf( "    Unlabeled Markers (%d):\n", UnlabeledMarkerCount.MarkerCount);
      unsigned int UnlabeledMarkerIndex;
      for (UnlabeledMarkerIndex = 0; UnlabeledMarkerIndex < UnlabeledMarkerCount.MarkerCount; ++UnlabeledMarkerIndex)
      {
        // Get the global marker translation
        COutput_GetUnlabeledMarkerGlobalTranslation _Output_GetUnlabeledMarkerGlobalTranslation;
        Client_GetUnlabeledMarkerGlobalTranslation(pClient, UnlabeledMarkerIndex, &_Output_GetUnlabeledMarkerGlobalTranslation);

        printf( "      Marker #%d: (%f, %f, %f)\n"
          , UnlabeledMarkerIndex
          , _Output_GetUnlabeledMarkerGlobalTranslation.Translation[0]
          , _Output_GetUnlabeledMarkerGlobalTranslation.Translation[1]
          , _Output_GetUnlabeledMarkerGlobalTranslation.Translation[2]);
      }

      // Get the labeled markers
      COutput_GetLabeledMarkerCount LabeledMarkerCount;
      Client_GetLabeledMarkerCount(pClient, &LabeledMarkerCount);
      printf("    Labeled Markers (%d):\n", LabeledMarkerCount.MarkerCount);
      unsigned int LabeledMarkerIndex;
      for (LabeledMarkerIndex = 0; LabeledMarkerIndex < LabeledMarkerCount.MarkerCount; ++LabeledMarkerIndex)
      {
        // Get the global marker translation
        COutput_GetLabeledMarkerGlobalTranslation _Output_GetLabeledMarkerGlobalTranslation;
        Client_GetLabeledMarkerGlobalTranslation(pClient, LabeledMarkerIndex, &_Output_GetLabeledMarkerGlobalTranslation);

        printf("      Marker #%d: (%f, %f, %f)\n"
          , LabeledMarkerIndex
          , _Output_GetLabeledMarkerGlobalTranslation.Translation[0]
          , _Output_GetLabeledMarkerGlobalTranslation.Translation[1]
          , _Output_GetLabeledMarkerGlobalTranslation.Translation[2]);
      }

      // Count the number of devices
      COutput_GetDeviceCount DeviceCount;
      Client_GetDeviceCount(pClient, &DeviceCount);
      printf( "  Devices (%d):\n", DeviceCount.DeviceCount);
      unsigned int DeviceIndex;
      for (DeviceIndex = 0; DeviceIndex < DeviceCount.DeviceCount; ++DeviceIndex)
      {
        printf( "    Device #%d:\n", DeviceIndex);

        // Get the device name and type
        char DeviceName[128];
        CEnum DeviceType;  //TODO does this need fixing? 
        Client_GetDeviceName(pClient, DeviceIndex, 128, DeviceName, &DeviceType);
        printf( "      Name: %s\n", DeviceName);
        printf( "      Type: %s\n", AdaptDeviceType(DeviceType));
        //TODO Is the device type accessed from the COutput_GetDeviceName struct?

        // Count the number of device outputs
        COutput_GetDeviceOutputCount DeviceOutputCount;
        Client_GetDeviceOutputCount(pClient, DeviceName, &DeviceOutputCount);
        printf( "      Device Outputs (%d):\n", DeviceOutputCount.DeviceOutputCount);
        unsigned int DeviceOutputIndex;
        for (DeviceOutputIndex = 0; DeviceOutputIndex < DeviceOutputCount.DeviceOutputCount; ++DeviceOutputIndex)
        {
          // Get the device output name and unit
          char DeviceOutputName[128];
          CEnum DeviceOutputUnit; //TODO does this need fixing? 
          Client_GetDeviceOutputName(pClient, DeviceName, DeviceOutputIndex, 128, DeviceOutputName, &DeviceOutputUnit);

          COutput_GetDeviceOutputSubsamples DeviceOutputSubsamples;
          Client_GetDeviceOutputSubsamples(pClient, DeviceName,
              DeviceOutputName, &DeviceOutputSubsamples);

          printf( "      Device Output #%d:\n", DeviceOutputIndex);
          printf( "      Samples (%d):\n", DeviceOutputSubsamples.DeviceOutputSubsamples);

          unsigned int DeviceOutputSubsample;
          for (DeviceOutputSubsample = 0; DeviceOutputSubsample < DeviceOutputSubsamples.DeviceOutputSubsamples; ++DeviceOutputSubsample)
          {
            printf( "        Sample #%d:\n", DeviceOutputSubsample);

            // Get the device output value
            //TODO check Client_GetDeviceOutputValue is equivelant to cpp
            COutput_GetDeviceOutputValue _Output_GetDeviceOutputValue;
            Client_GetDeviceOutputValue(pClient, DeviceName, DeviceOutputName, &_Output_GetDeviceOutputValue);
            
            printf( "          '%s' %f %s %s\n"
              , DeviceOutputName
              , _Output_GetDeviceOutputValue.Value
              , AdaptUnit(DeviceOutputUnit)
              , (Adapt(_Output_GetDeviceOutputValue.Occluded)));
          }
        }
      }

      // Output the force plate information.
      COutput_GetForcePlateCount ForcePlateCount;
      Client_GetForcePlateCount(pClient, &ForcePlateCount);
      printf( "  Force Plates: (%d)\n", ForcePlateCount.ForcePlateCount);

      unsigned int ForcePlateIndex;
      for (ForcePlateIndex = 0; ForcePlateIndex < ForcePlateCount.ForcePlateCount; ++ForcePlateIndex)
      {
        printf( "    Force Plate #%d:\n", ForcePlateIndex);

        COutput_GetForcePlateSubsamples ForcePlateSubsamples;
        Client_GetForcePlateSubsamples(pClient, ForcePlateIndex, &ForcePlateSubsamples);

        printf( "    Samples (%d):\n", ForcePlateSubsamples.ForcePlateSubsamples);
        unsigned int ForcePlateSubsample;
        for (ForcePlateSubsample = 0; ForcePlateSubsample < ForcePlateSubsamples.ForcePlateSubsamples; ++ForcePlateSubsample)
        {
          printf( "      Sample #%d:\n", ForcePlateSubsample);
          COutput_GetGlobalForceVector _Output_GetForceVector;
          Client_GetGlobalForceVectorForSubsample(pClient, ForcePlateIndex, ForcePlateSubsample, &_Output_GetForceVector);
          printf( "        Force (%f, %f, %f)\n"
            , _Output_GetForceVector.ForceVector[0]
            , _Output_GetForceVector.ForceVector[1]
            , _Output_GetForceVector.ForceVector[2]
          );
          COutput_GetGlobalMomentVector _Output_GetMomentVector;
          Client_GetGlobalMomentVectorForSubsample(pClient, ForcePlateIndex, ForcePlateSubsample, &_Output_GetMomentVector);
          printf( "        Moment (%f, %f, %f)\n"
            , _Output_GetMomentVector.MomentVector[0]
            , _Output_GetMomentVector.MomentVector[1]
            , _Output_GetMomentVector.MomentVector[2]
          );
          COutput_GetGlobalCentreOfPressure _Output_GetCentreOfPressure;
          Client_GetGlobalCentreOfPressureForSubsample(pClient, ForcePlateIndex, ForcePlateSubsample, &_Output_GetCentreOfPressure);
          printf( "        CoP (%f, %f, %f)\n"
            , _Output_GetCentreOfPressure.CentreOfPressure[0]
            , _Output_GetCentreOfPressure.CentreOfPressure[1]
            , _Output_GetCentreOfPressure.CentreOfPressure[2]
          );
        }
      }

      // Output eye tracker information.
      COutput_GetEyeTrackerCount EyeTrackerCount;
      Client_GetEyeTrackerCount(pClient, &EyeTrackerCount);
      printf( "  Eye Trackers: (%d)\n", EyeTrackerCount.EyeTrackerCount);
      unsigned int EyeTrackerIndex;
      for (EyeTrackerIndex = 0; EyeTrackerIndex < EyeTrackerCount.EyeTrackerCount; ++EyeTrackerIndex)
      {
        printf( "    Eye Tracker #%d:\n", EyeTrackerIndex);

        COutput_GetEyeTrackerGlobalPosition _Output_GetEyeTrackerGlobalPosition;
        Client_GetEyeTrackerGlobalPosition(pClient, EyeTrackerIndex, &_Output_GetEyeTrackerGlobalPosition);

        printf( "      Position (%f, %f, %f) %s\n"
          , _Output_GetEyeTrackerGlobalPosition.Position[0]
          , _Output_GetEyeTrackerGlobalPosition.Position[1]
          , _Output_GetEyeTrackerGlobalPosition.Position[2]
          , Adapt(_Output_GetEyeTrackerGlobalPosition.Occluded));

        COutput_GetEyeTrackerGlobalGazeVector _Output_GetEyeTrackerGlobalGazeVector;
        Client_GetEyeTrackerGlobalGazeVector(pClient, EyeTrackerIndex, &_Output_GetEyeTrackerGlobalGazeVector);

        printf( "      Gaze (%f, %f, %f) %s\n"
          , _Output_GetEyeTrackerGlobalGazeVector.GazeVector[0]
          , _Output_GetEyeTrackerGlobalGazeVector.GazeVector[1]
          , _Output_GetEyeTrackerGlobalGazeVector.GazeVector[2]
          , Adapt(_Output_GetEyeTrackerGlobalGazeVector.Occluded)
        );
      }

      if (bReadCentroids)
      {
        COutput_GetCameraCount CameraCount;
        Client_GetCameraCount(pClient, &CameraCount);
        printf( "Cameras(%d):\n", CameraCount.CameraCount);
        unsigned int CameraIndex;
        for (CameraIndex = 0; CameraIndex < CameraCount.CameraCount; ++CameraIndex)
        {
          printf( "  Camera #%d:\n", CameraIndex);

          char CameraName[128];
          Client_GetCameraName(pClient, CameraIndex, 128, CameraName);
          printf( "    Name: %s\n", CameraName);

          COutput_GetCameraId _Output_GetCameraId;
          Client_GetCameraId(pClient, CameraName, &_Output_GetCameraId);
          printf( "    Id: %d\n" , _Output_GetCameraId.CameraId );
          COutput_GetCameraUserId _Output_GetCameraUserId;
          Client_GetCameraUserId(pClient, CameraName, &_Output_GetCameraUserId);
          printf( "    User Id: %d\n" , _Output_GetCameraUserId.CameraUserId );
          char CameraType[128];
          Client_GetCameraType(pClient, CameraName, 128, CameraType);
          printf("    Type: %s\n", CameraType);
          char CameraDisplayName[128];
          Client_GetCameraDisplayName(pClient, CameraName, 128, CameraDisplayName);
          printf("    Display Name: %s\n", CameraDisplayName);
          COutput_GetCameraResolution _Output_GetCameraResolution;
          Client_GetCameraResolution(pClient, CameraName, &_Output_GetCameraResolution );
          printf( "    Resolution: %d x %d\n" , _Output_GetCameraResolution.ResolutionX , _Output_GetCameraResolution.ResolutionY );
          COutput_GetIsVideoCamera _Output_GetIsVideoCamera;
          Client_GetIsVideoCamera(pClient, CameraName, &_Output_GetIsVideoCamera);
          printf( "    Is Video Camera: %s\n" , _Output_GetIsVideoCamera.IsVideoCamera ? "true" : "false");

          COutput_GetCentroidCount CentroidCount;
          Client_GetCentroidCount(pClient, CameraName, &CentroidCount);
          printf( "    Centroids(%d):\n", CentroidCount.CentroidCount);
          unsigned int CentroidIndex;
          for (CentroidIndex = 0; CentroidIndex < CentroidCount.CentroidCount; ++CentroidIndex)
          {
            printf( "      Centroid #%d:\n", CentroidIndex);

            COutput_GetCentroidPosition _Output_GetCentroidPosition;
            Client_GetCentroidPosition(pClient, CameraName, CentroidIndex, &_Output_GetCentroidPosition);
            printf( "        Position: (%f, %f)\n"
              , _Output_GetCentroidPosition.CentroidPosition[0]
              , _Output_GetCentroidPosition.CentroidPosition[1]
            );
            printf( "        Radius: (%f)\n",  _Output_GetCentroidPosition.Radius);

            COutput_GetCentroidWeight _Output_GetCentroidWeight;
            Client_GetCentroidWeight(pClient, CameraName, CentroidIndex, &_Output_GetCentroidWeight);
            if (_Output_GetCentroidWeight.Result == Success)
            {
              printf( "        Weighting: %f\n", _Output_GetCentroidWeight.Weight);
            }
          }
        }
      }
    }

    if (EnableMultiCast)
    {
      Client_StopTransmittingMulticast(pClient);
    }
    Client_DisableSegmentData(pClient);
    Client_DisableMarkerData(pClient);
    Client_DisableUnlabeledMarkerData(pClient);
    Client_DisableDeviceData(pClient);
    if (bReadCentroids)
    {
      Client_DisableCentroidData(pClient);
    }
    if (bReadRayData)
    {
      bReadRayData = 0;
    }

    // Disconnect and dispose
    int t = clock();
    printf( " Disconnecting...\n" );
    Client_Disconnect(pClient);
    int dt = clock() - t;
    double secs = (double)(dt) / (double)CLOCKS_PER_SEC;
    printf( " Disconnect time = %f secs\n", secs);

  }

  return 0;
}
