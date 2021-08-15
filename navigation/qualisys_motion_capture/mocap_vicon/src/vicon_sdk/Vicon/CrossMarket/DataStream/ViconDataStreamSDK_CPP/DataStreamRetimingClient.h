
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

// \file 
// The SDK retiming client and supporting definitions.

#include <memory>
#include <string>
#include <map>
#include <vector>

#include "IDataStreamClientBase.h"

#include <string>
#include <vector>

namespace ViconDataStreamSDK
{
  namespace CPP
  {
    class RetimingClientImpl;

    ///  \class RetimingClient 
    ///  \brief The re-timing client class for C++.
    ///  Vicon DataStream SDK Re-Timing Client
    ///  ===========================
    ///  Intended uses
    ///  -------------
    ///  The Vicon DataStream re-timing client provides calls to obtain subject data from the 
    ///  DataStream with minimal latency and temporal jitter. 
    ///  When UpdateFrame() is called, the client uses re-timed data that has been linearly interpolated 
    ///  from an internal buffer to predict the position of each segment to the current time. 
    ///
    ///  The system and network latencies are used when determining the amount of prediction required.
    ///  If additional prediction is required, for example for use in a VR system where an additional latency 
    ///  is present due to rendering and display latency; this may be requested in the call to UpdateFrame().
    ///
    ///  The user will either call UpdateFrame(), which will update the current frame state to 
    ///  the time of calling and return immediately. This is intended for use in systems where you require
    ///  subject data positions at times driven by an external clock.
    ///
    ///  If you do not have an external clock, and require behavior similar to that of the standard DataStream 
    ///  client running in ServerPush streaming mode, than the system may be configured to provide frame data
    ///  at a consistent frame rate by providing a frame rate to the Connect() call. The user will then call 
    ///  WaitForFrame(), which will block in a similar method to Client::GetFrame(), but using retimed data
    ///  in order to keep the frame period very consistent.
    /// 
    ///  Examples of use
    ///  --------------
    /// 
    ///  If you are using the client in a situation where you need to obtain the position of subjects in  
    /// 
    ///       ViconDataStreamSDK::CPP::RetimingClient _MyClient;
    ///       _MyClient.Connect( "localhost" );
    ///       
    ///       // example render method. Real code would probably cache the subject and segment names and bind 
    ///       // them to a model, so this code would iterate over the model and update the joint positions.
    ///       void render() 
    ///       {
    ///         _MyClient.UpdateFrame();
    ///
    ///          Output_GetSubjectCount SubjectCount = _MyClient.GetSubjectCount();
    ///          if (SubjectCount.Result == Result::Success)
    ///          {
    ///            for (unsigned int SubjectIndex = 0; SubjectIndex < SubjectCount.SubjectCount; ++SubjectIndex)
    ///            {
    ///              Output_GetSubjectName SubjectName = _MyClient.GetSubjectName(SubjectIndex);
    ///              if (SubjectName.Result == Result::Success)
    ///              {
    ///                Output_GetSegmentCount SegmentCount = _MyClient.GetSegmentCount(SubjectName.SubjectName);
    ///                if (SegmentCount.Result == Result::Success)
    ///                {
    ///                  for (unsigned int SegmentIndex = 0; SegmentIndex < SegmentCount.SegmentCount; ++SegmentIndex)
    ///                  {
    ///                    Output_GetSegmentName SegmentName = 
    ///                       _MyClient.GetSegmentName(SubjectName.SubjectName, SegmentIndex);
    ///                    if (SegmentName.Result == Result::Success)
    ///                    {
    ///                      Output_GetSegmentGlobalRotationQuaternion SegmentRotation =
    ///                         _MyClient.GetSegmentGlobalRotationQuaternion
    ///                             (SubjectName.SubjectName, SegmentName.SegmentName);
    ///                      if (SegmentRotation.Result == Result::Success && !SegmentRotation.Occluded)
    ///                      {
    ///                        // use the segment rotation
    ///                      }
    ///                    }
    ///                  }
    ///                }
    ///              }
    ///            }
    ///          }
    /// 
    ///
    /// If using the client where there is no render call and you require your own timing.
    /// 
    ///       ViconDataStreamSDK::CPP::RetimingClient _MyClient;
    ///       // Request a retimed update frame rate of 90Hz.
    ///       _MyClient.Connect( "localhost", 90 );
    ///       while( _MyClient.IsConnected() )
    ///       {
    ///         Output_WaitForFrame WaitOutput = _MyClient.WaitForFrame();
    ///         if( WaitOutput.Result == Result::Success )
    ///         {
    ///           // iterate over subjects and segments and obtain the joint positions and rotations as above.
    ///         }
    ///       }
    ///        
    ///  For a more detailed example, see the ViconDataStreamSDK_CPPRetimingTest example. The SimpleViewer application
    ///  also provides an example of re-timing client use in a practical context.
    class CLASS_DECLSPEC RetimingClient : public IDataStreamClientBase
    {
    public:

      /// Construction.
      /// Instances of the Vicon Data Stream RetimingClient create a DataStreamClient internally that manages the connection 
      /// to the data stream.
      /// 
      /// The RetimingClient will set the underlying client up to receive the required data from the stream and to set the 
      /// correct data delivery mode, so it is not necessary to set this up manually.
      ///
      ///
      /// C example
      ///      
      ///      // The C version uses explicit creation methods
      ///      
      ///      CClient * pClient = RetimingClient_Create();
      ///      // C Client functions take the client as a parameter
      ///      CBool ok = RetimingClient_SomeFunction( pClient, Args );
      ///      // The C client needs to be explicitly destroyed
      ///      RetimingClient_Destroy( pClient );
      ///      
      /// C++ example
      ///      
      ///      //  The C++ version of the SDK is object oriented, so use the class constructor.
      ///      
      ///      {
      ///       ViconDataStreamSDK::CPP::RetimingClient StackRetimingClient;
      ///       Output_SomeFunction Output = StackRetimingClient.SomeFunction();
      ///       // ...
      ///      }
      ///      // Client is implicitly destroyed as it goes out of scope.
      ///      
      ///      // Alternatively the Client can be made on the heap.
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient * pHeapRetimingClient
      ///      = new ViconDataStreamSDK::CPP::RetimingClient();
      ///      Output_SomeFunction Output = pHeapRetimingClient->SomeFunction( Input );
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      // .NET is object oriented, so use the class constructor.
      ///      
      ///      // Because objects are lazily garbage collected, your instance may outlive the 
      ///      // last reference to it for some time.
      ///      
      ///      // If the instance is pre-fetching frame data for you, then it can still use CPU 
      ///      // and network bandwidth.
      ///      
      ///      // Consider explicitly disconnecting prior to destruction.
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient pHeapClient
      ///        = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      Output_SomeFunction Output = pHeapClient.SomeFunction(InputParam);
      ///      // Signal to the garbage collector that it can clean up
      ///      pHeapClient.Disconnect();
      ///      pHeapClient = null;
      /// -----      
      RetimingClient();


      /// Destruction.
      /// Destruction will Disconnect if required.
      /// 
      /// See RetimingClient::RetimingClient for an example.
      virtual ~RetimingClient();

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
      ///
      /// C example
      ///      
      ///      CRetimingClient * pClient = RetimingClient_Create();
      ///      COutput_GetVersion Output = RetimingClient_GetVersion( pClient );
      ///      RetimingClient_Destroy( pClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      Output_GetVersion Output = MyClient.GetVersion();
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      Output_GetVersion Output = MyClient.GetVersion();
      /// -----      
      /// 
      /// \return Output_GetVersion class containing the version information.
      Output_GetVersion  GetVersion() const;

      /// Establish a dedicated connection to a Vicon DataStream Server.
      /// 
      /// See Also: Disconnect(), IsConnected().
      /// 
      /// The function defaults to connecting on port 801. 
      /// You can specify an alternate port number after a colon.
      /// This is for future compatibility: current products serve data on port 801 only.
      /// 
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      COutput_Connect Output = RetimingClient_Connect( pRetimingClient, "localhost");
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      Output_Connect Output = MyClient.Connect( "localhost" );
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      Output_Connect Output = MyClient.Connect( "localhost" );
      ///      // Connect with alternative FrameRate parameter
      ///      Output_Connect Output = MyClient.Connect( "localhost", 90.0 );
      /// -----      
      /// 
      /// \param  HostName The DNS-identifiable name, or IP address of the PC hosting the DataStream server. 
      ///                  For example:
      ///                    + "localhost"
      ///                    + "MyViconPC:801"
      ///                    + "10.0.0.2"
      /// 
      /// \param FrameRate An optional parameter - if specified, the re-timing client's internal frame output
      ///        clock will be active. This is implemented by a separate overloaded method on .NET
      /// 
      /// \return An Output_Connect class containing the result of the connect operation.
      ///         - The Result will be: 
      ///           + Success
      ///           + InvalidHostName
      ///           + ClientAlreadyConnected 
      ///           + ClientConnectionFailed
      Output_Connect Connect(const String & HostName, double FrameRate = 0.0 );

      /// Disconnect from the Vicon DataStream Server.
      /// 
      /// See Also: Connect(), IsConnected()
      /// 
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      COutput_Disconnect Output = RetimingClient_Disconnect( pRetimingClient );
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      Output_Disconnect Output = MyClient.Disconnect();
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      Output_Disconnect Output = MyClient.Disconnect();
      /// -----      
      /// 
      /// \return An Output_Disconnect class containing the result of the operation.
      ///         - The Result will be: 
      ///           + Success
      ///           + NotConnected
      Output_Disconnect  Disconnect();

      /// Discover whether client is connected to the Vicon DataStream Server.
      /// 
      /// See Also: Connect(), Disconnect()
      /// 
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      CBool Output = RetimingClient_IsConnected( pRetimingClient );
      ///      // Output == 0
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      Output = RetimingClient_IsConnected( pRetimingClient );
      ///      // Output == 1
      ///      COutput_Disconnect Output = RetimingClient_Disconnect( pRetimingClient );
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::CPP::Client MyClient;
      ///      Output_IsConnected Output = MyClient.IsConnected()
      ///      // Output.Connected == false
      ///      MyClient.Connect( "localhost" );
      ///      Output_IsConnected Output = MyClient.IsConnected()
      ///      // Output.Connected == true
      ///      // (assuming localhost is serving)
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      Output_IsConnected Output = MyClient.IsConnected()
      ///      // Output.Connected == false
      ///      MyClient.Connect( "localhost" );
      ///      Output_IsConnected Output = MyClient.IsConnected()
      ///      // Output.Connected == true
      ///      // (assuming localhost is serving)
      /// -----      
      /// 
      /// \return An Output_IsConnected class containing a true value for Connected if you are 
      ///         connected to the stream, otherwise false.
      Output_IsConnected IsConnected() const;

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
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_SetAxisMapping(pRetimingClient, Forward, Left, Up); // Z-up
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.SetAxisMapping( ViconDataStreamSDK::CPP::Direction::Forward,
      ///      ViconDataStreamSDK::CPP::Direction::Left,
      ///      ViconDataStreamSDK::CPP::Direction::Up );
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
      ///      MyClient.SetAxisMapping( ViconDataStreamSDK.DotNET.Direction.Forward,
      ///      ViconDataStreamSDK.DotNET.Direction.Left,
      ///      ViconDataStreamSDK.DotNET.Direction.Up );
      /// -----      
      /// \param XAxis Specify the direction of your X axis relative to yourself as the observer.
      /// \param YAxis Specify the direction of your Y axis relative to yourself as the observer.
      /// \param ZAxis Specify the direction of your Z axis relative to yourself as the observer.
      ///
      /// \return An Output_SetAxisMapping class containing the result of the operation.
      ///         - The Result will be:
      ///           + Success
      ///           + CoLinearAxes
      ///           + LeftHandedAxes
      Output_SetAxisMapping SetAxisMapping(const Direction::Enum XAxis, const Direction::Enum YAxis, const Direction::Enum ZAxis);

      /// Get the current Axis mapping.
      ///
      /// See Also: SetAxisMapping()
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_SetAxisMapping(pRetimingClient, Forward, Left, Up); // Z-up
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      Output_GetAxisMapping Output = MyClient.GetAxisMapping();
      ///      // Output.XAxis == ViconDataStreamSDK::CPP::Direction::Forward
      ///      // Output.YAxis == ViconDataStreamSDK::CPP::Direction::Left
      ///      // Output.ZAxis == ViconDataStreamSDK::CPP::Direction::Up
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.Client MyClient = new ViconDataStreamSDK.DotNET.Client();
      ///      Output_GetAxisMapping Output = MyClient.GetAxisMapping();
      ///      // Output.XAxis == ViconDataStreamSDK.DotNET.Direction.Forward
      ///      // Output.YAxis == ViconDataStreamSDK.DotNET.Direction.Left
      ///      // Output.ZAxis == ViconDataStreamSDK.DotNET.Direction.Up
      /// -----      
      /// \return An Output_GetAxisMapping class containing the result of the operation.
      ///         - The Result will be:
      ///           + XAxis, YAxis, ZAxis
      Output_GetAxisMapping GetAxisMapping() const;

      /// Update the current frame state to represent the position of all active subjects at the 
      /// current time.
      /// 
      /// The position of each segment is estimated by predicting forwards from the most recent 
      /// frames received from the DataStream, taking into account the latency reported by the 
      /// system to determine the amount of prediction required.
      ///
      /// The results of calls which return details about the current frame state such as GetSubjectCount() 
      /// and GetSegmentGlobalRotationQuaternion() will all return the stream contents and position at the
      /// time that this call was made. 
      /// 
      /// If no call to UpdateFrame() is made, calls querying the stream state will return NoFrame
      /// 
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      CEnum Output = RetimingClient_GetFrame(); // Output == NotConnected
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      Output = RetimingClient_UpdateFrame(); // Output == Success
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      Output_UpdateFrame Output;
      ///      Output = MyClient.UpdateFrame(); // Output.Result == NotConnected
      ///      MyClient.Connect( "localhost" );
      ///      Output = MyClient.UpdateFrame(); // Output.Result == Success
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      Output_UpdateFrame Output;
      ///      Output = MyClient.UpdateFrame(); // Output.Result == NotConnected
      ///      MyClient.Connect( "localhost" );
      ///      Output = MyClient.UpdateFrame(); // Output.Result == Success
      ///      Output = MyClient.UpdateFrame(20); // Output.Result == Success
      /// -----      
      ///  
      /// \param Offset An additional offset that will be applied to the time at which which the predicted
      ///               position is calculated. This may be used to compensate for additional delays that 
      ///               are in the users system, such as render delay. This is implemnted in a separate overloaded
      ///               method in .NET.
      ///
      /// \return An Output_UpdateFrame class containing the result of the operation.
      ///         - The Result will be:
      ///           + Success
      ///           + NotConnected
      Output_UpdateFrame UpdateFrame( double Offset = 0.0 );

      /// Used when running the retiming client with a specified frame rate. This call will
      /// block until the next frame is available, as driven by an internal clock running at
      /// the frame rate specified by Connect( Host, FrameRate). The frame data is re-timed to the
      /// correct time point
      /// 
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      CEnum Output = RetimingClient_GetFrame(); // Output == NotConnected
      ///      RetimingClient_ConnectAndStart( pRetimingClient, "localhost", 200 );
      ///      Output = RetimingClient_WaitForFrame(); // Output == Success
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost", 200 );
      ///      Output = MyClient.WaitForFrame(); // Output.Result == Success
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost", 200 );
      ///      Output = MyClient.WaitForFrame(); // Output.Result == Success
      /// -----      
      ///  
      /// \return An Output_WaitForFrame class containing the result of the operation.
      ///         - The Result will be:
      ///           + Success
      ///           + NotConnected
      Output_WaitForFrame WaitForFrame();

      /// Return the number of subjects in the DataStream. This information can be used in conjunction with GetSubjectName.
      ///
      /// See Also: GetSubjectName()
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      COutput_GetSubjectCount SubjectCount;
      ///      RetimingClient_GetSubjectCount(pRetimingClient, &SubjectCount); // SubjectCount.Result == NoFrame
      ///      // SubjectCount.SubjectCount == 0;
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      RetimingClient_GetSubjectCount(pRetimingClient, &SubjectCount); // SubjectCount.Result == Success;
      ///      // SubjectCount.SubjectCount == 0;
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      Output_GetSubjectCount Output;
      ///      Output = MyClient.GetSubjectCount(); // Output.Result == NoFrame
      ///      // Ooutput.SubjectCount == 0
      ///      MyClient.GetFrame();
      ///      Output = MyClient.GetSubjectCount(); // Output.Result == Success
      ///      // Output.SubjectCount >= 0
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      Output_GetSubjectCount Output;
      ///      Output = MyClient.GetSubjectCount(); // Output.Result == NoFrame
      ///      // Ooutput.SubjectCount == 0
      ///      MyClient.UpdateFrame();
      ///      Output = MyClient.GetSubjectCount(); // Output.Result == Success
      ///      // Output.SubjectCount >= 0
      /// -----      
      ///        
      ///
      /// \return An Output_GetSubjectCount class containing the result of the operation and the number of subjects.
      ///         - The Result will be:
      ///           + Success
      ///           + NotConnected
      ///           + NoFrame
      Output_GetSubjectCount GetSubjectCount() const;

      /// Return the name of a subject. This can be passed into segment and marker functions.
      ///
      /// See Also: GetSubjectCount()
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      char SubjectName[128];
      ///      CEnum Output = RetimingClient_GetSubjectName(pRetimingClient, 0, 128, SubjectName);
      ///      // Output == Success
      ///      // SubjectName =="AI"
      ///      Output = RetimingClient_GetSubjectName(pRetimingClient, 1, 128, SubjectName);
      ///      // Output == Success
      ///      // SubjectName =="Bob"
      ///      Output = RetimingClient_GetSubjectName(pRetimingClient, 2, 128, SubjectName);
      ///      // Output == InvalidIndex
      ///      // SubjectName ==""
      ///      
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.GetFrame();
      ///      Output_GetSubjectCount OutputGSC;
      ///      OutputGSC = MyClient.GetSubjectCount(); // OutputGSC.Result == Success
      ///      // OutputGSC.SubjectCount == 2
      ///      Output_GetSubjectName OutputGSN;
      ///      OutputGSN = MyClient.GetSubjectName(0);// OutputGSN.Result == Success
      ///      // OutputGSN.SubjectName == "Al"
      ///      OutputGSN = MyClient.GetSubjectName(1);// OutputGSN.Result == Success
      ///      // OutputGSN .SubjectName == "Bob"
      ///      OutputGSN = MyClient.GetSubjectName(2);// OutputGSN.Result == InvalidIndex
      ///      // OutputGSN.SubjectName == ""
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.UpdateFrame();
      ///      Output_GetSubjectCount OutputGSC;
      ///      OutputGSC = MyClient.GetSubjectCount(); // OutputGSC.Result == Success
      ///      // OutputGSC.SubjectCount == 2
      ///      Output_GetSubjectName OutputGSN;
      ///      OutputGSN = MyClient.GetSubjectName(0);// OutputGSN.Result == Success
      ///      // OutputGSN.SubjectName == "Al"
      ///      OutputGSN = MyClient.GetSubjectName(1);// OutputGSN.Result == Success
      ///      // OutputGSN .SubjectName == "Bob"
      ///      OutputGSN = MyClient.GetSubjectName(2);// OutputGSN.Result == InvalidIndex
      ///      // OutputGSN.SubjectName == ""
      /// -----      
      ///
      ///
      /// \param SubjectIndex The index of the subject. A valid Subject Index is between 0 and GetSubjectCount()-1.
      ///                                         Matlab: A valid Subject Index is between 1 and GetSubjectCount().
      /// \return An Output_GetSubjectName GetSubjectName class containing the result of the operation and the name of the subject.
      ///         - The Result will be:
      ///           + Success
      ///           + NotConnected
      ///           + NoFrame
      ///           + InvalidIndex
      Output_GetSubjectName GetSubjectName(const unsigned int SubjectIndex) const;

      /// Return the name of the root segment for a specified subject. This can be passed into segment functions.The root segment is the ancestor of all other segments in the subject.
      ///
      /// See Also: GetSegmentCount(), GetSegmentParentName(), GetSegmentChildCount(), GetSegmentChildName()
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_EnableSegmentData( pRetimingClient );
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      char RootSegment[128];
      ///      CEnum Result = RetimingClient_GetSubjectRootSegmentName(pRetimingClient, "Bob", 128, RootSegment);
      ///      // Result == Success
      ///      // RootSegment == "Pelvis"
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.EnableSegmentData();
      ///      MyClient.GetFrame();
      ///      Output_GetSubjectRootSegmentName Output;
      ///      Output = MyClient.GetSubjectRootSegmentName( "Bob" );
      ///      // Output.Result == Success
      ///      // Output.SegmentName == "Pelvis"
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.UpdateFrame();
      ///      Output_GetSubjectRootSegmentName Output;
      ///      Output = MyClient.GetSubjectRootSegmentName( "Bob" );
      ///      // Output.Result == Success
      ///      // Output.SegmentName == "Pelvis"
      /// -----      
      ///        
      ///
      /// \param SubjectName The name of the subject
      /// \return An Output_GetSubjectRootSegmentName class containing the result of the operation and the name of the root segment.
      ///         - The Result will be:
      ///           + Success
      ///           + NotConnected
      ///           + NoFrame
      ///           + InvalidIndex
      Output_GetSubjectRootSegmentName GetSubjectRootSegmentName(const String & SubjectName) const;

      /// Return the number of segments for a specified subject in the DataStream. This information can be used in conjunction with GetSegmentName.
      ///
      /// See Also: GetSubjectName(), GetSegmentName()
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      COutput_GetSegmentCount SegmentCount;
      ///      RetimingClient_GetSegmentCount( pRetimingClient, "Bob", &SegmentCount );
      ///      // SegmentCount.Result == NOFrame
      ///      // SegmentCount.Value == 0
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      RetimingClient_GetSegmentCount( pRetimingClient, "AI", &SegmentCount );
      ///      //  SegmentCount.Result == InvalidSubjectName
      ///      //  SegmentCount.Value == 0
      ///      RetimingClient_GetSegmentCount( pRetimingClient, "Bob", &SegmentCount );
      ///      //  SegmentCount.Result == Success
      ///      //  SegmentCount.Value >= 0
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.EnableSegmentData();
      ///      MyClient.Connect( "localhost" );
      ///      Output_GetSegmentCount Output;
      ///      Output = MyClient.GetSegmentCount( "Bob" ); // Output.Result == NoFrame
      ///      // Output.SegmentCount == 0
      ///      MyClient.GetFrame();
      ///      Output = MyClient.GetSegmentCount( "Al" ); // Output.Result ==
      ///      // InvalidSubjectName
      ///      // Output.SegmentCount == 0
      ///      Output = MyClient.GetSegmentCount( "Bob" );// Output.Result == Success
      ///      // Output.SegmentCount >= 0
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      Output_GetSegmentCount Output;
      ///      Output = MyClient.GetSegmentCount( "Bob" ); // Output.Result == NoFrame
      ///      // Output.SegmentCount == 0
      ///      MyClient.UpdateFrame();
      ///      Output = MyClient.GetSegmentCount( "Al" ); // Output.Result ==
      ///      // InvalidSubjectName
      ///      // Output.SegmentCount == 0
      ///      Output = MyClient.GetSegmentCount( "Bob" );// Output.Result == Success
      ///      // Output.SegmentCount >= 0
      /// -----      
      ///        
      ///
      /// \param SubjectName The name of the subject.
      /// \return An Output_GetSegmentCount class containing the result of the operation and the number of segments.
      ///         - The Result will be:
      ///           + Success
      ///           + NotConnected
      ///           + NoFrame
      ///           + InvalidIndex
      Output_GetSegmentCount GetSegmentCount(const String  & SubjectName) const;

      /// Return the name of a subject segment specified by index.
      ///
      /// See Also: GetSegmentCount(), GetSegmentChildCount(), GetSegmentChildName(), GetSubjectRootSegmentName()
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      COutput_GetSegmentCount SegmentCount;
      ///      RetimingClient_GetSegmentCount( pRetimingClient, "Bob", &SegmentCount );
      ///      // SegmentCount.Result == NOFrame
      ///      // SegmentCount.Value == 0
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      RetimingClient_GetSegmentCount( pRetimingClient, "AI", &SegmentCount );
      ///      //  SegmentCount.Result == InvalidSubjectName
      ///      //  SegmentCount.Value == 0
      ///      RetimingClient_GetSegmentCount( pRetimingClient, "Bob", &SegmentCount );
      ///      //  SegmentCount.Result == Success
      ///      //  SegmentCount.Value >= 0
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.EnableSegmentData();
      ///      MyClient.GetFrame();
      ///      Output_GetSegmentName Output;
      ///      // SegmentIndex must be between 0 and GetSegmentCount() - 1
      ///      Output = MyClient.GetSegmentName( "Bob", 0 );
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.UpdateFrame();
      ///      Output_GetSegmentName Output;
      ///      // SegmentIndex must be between 0 and GetSegmentCount() - 1
      ///      Output = MyClient.GetSegmentName( "Bob", 0 );
      /// -----      
      ///        
      ///
      /// \param SubjectName The name of the subject
      /// \param SegmentIndex The index of the segment
      /// \return An Output_GetSegmentName  class containing the result of the operation and the name of the parent segment or an empty string if it is the root segment.
      ///         - The Result will be:
      ///           + Success
      ///           + NotConnected
      ///           + NoFrame
      ///           + InvalidIndex
      ///           + InvalidSubjectName
      Output_GetSegmentName GetSegmentName(const String       & SubjectName,
        const unsigned int   SegmentIndex) const;

      /// Return the number of child segments for a specified subject segment. This can be passed into segment functions.
      ///
      /// See Also: GetSegmentCount()
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      COutput_GetSegmentChildCount ChildCount;
      ///      RetimingClient_GetSegmentChildCount(pRetimingClient, "Bob", "Pelvis", &ChildCount);
      ///      // ChildCount.Result == Success
      ///      // ChildCount.SegmentCount == 2
      ///      RetimingClient_GetSegmentChildCount(pRetimingClient, "Alice", "Pelvis", &ChildCount);
      ///      // ChildCount.Result == InvalidSubjectName
      ///      // ChildCount.SegmentCount == 0
      ///      char SegmentName[128];
      ///      RetimingClient_GetSegmentName(pRetimingClient, "Bob", , 128, SegmentName);
      ///      RetimingClient_GetSegmentName(pRetimingClient, "Bob",  &SegmentName);
      ///      // ChildCount.Result == Success
      ///      // ChildCount.SegmentCount == 2
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.EnableSegmentData();
      ///      MyClient.GetFrame();
      ///      Output_GetSegmentChildCount OutputGSCC;
      ///      OutputGSCC = MyClient.GetSegmentChildCount( "Bob", "Pelvis" );
      ///      // OutputGSCC.Result == Success
      ///      // OutputGSCC.SegmentCount == 2
      ///      Output_GetSegmentChildName OutputGSCN;
      ///      OutputGSCN = MyClient.GetSegmentName( "Alice", 0 );
      ///      // OutputGSN.Result == InvalidSubjectName
      ///      // OutputGSN.SegmentName == ""
      ///      OutputGSCN = MyClient.GetSegmentName( "Bob", "Pelvis", 0 );
      ///      // OutputGSCN.Result == Success
      ///      // OutputGSCN.SegmentName == "LFemur"
      ///      OutputGSCN = MyClient.GetSegmentName( "Bob", "Pelvis", 1 );
      ///      // OutputGSCN.Result == Success
      ///      // OutputGSCN.SegmentName == "RFemur"
      ///      OutputGSCN = MyClient.GetSegmentName( "Bob", "Pelvis", 2 );
      ///      // OutputGSCN.Result == InvalidIndex
      ///      // OutputGSCN.SegmentName == ""
      ///      // (no third segment)
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.UpdateFrame();
      ///      Output_GetSegmentChildCount OutputGSCC;
      ///      OutputGSCC = MyClient.GetSegmentChildCount( "Bob", "Pelvis" );
      ///      // OutputGSCC.Result == Success
      ///      // OutputGSCC.SegmentCount == 2
      ///      Output_GetSegmentChildName OutputGSCN;
      ///      OutputGSCN = MyClient.GetSegmentName( "Alice", 0 );
      ///      // OutputGSN.Result == InvalidSubjectName
      ///      // OutputGSN.SegmentName == ""
      ///      OutputGSCN = MyClient.GetSegmentName( "Bob", "Pelvis", 0 );
      ///      // OutputGSCN.Result == Success
      ///      // OutputGSCN.SegmentName == "LFemur"
      ///      OutputGSCN = MyClient.GetSegmentName( "Bob", "Pelvis", 1 );
      ///      // OutputGSCN.Result == Success
      ///      // OutputGSCN.SegmentName == "RFemur"
      ///      OutputGSCN = MyClient.GetSegmentName( "Bob", "Pelvis", 2 );
      ///      // OutputGSCN.Result == InvalidIndex
      ///      // OutputGSCN.SegmentName == ""
      ///      // (no third segment)
      /// -----      
      ///
      /// \param SubjectName The name of the subject
      /// \param SegmentName The name of the segment
      /// \return An Output_GetSegmentChildCount  class containing the result of the operation and the number of child segments.
      ///         - The Result will be:
      ///           + Success
      ///           + NotConnected
      ///           + NoFrame
      ///           + InvalidIndex
      ///           + InvalidSubjectName
      ///           + InvalidSegmentName
      Output_GetSegmentChildCount GetSegmentChildCount(const String & SubjectName,
        const String & SegmentName) const;

      /// Return the name of the child segment for a specified subject segment and index.
      ///
      /// See Also: GetSegmentCount(), GetSegmentChildCount(), GetSegmentChildName(), GetSubjectRootSegmentName()
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_EnableSegmentData( pRetimingClient );
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      char SegmentChildName[128];
      ///      // Segment index must be between 0 and RetimingClient_GetSegmentChildCount() - 1
      ///      RetimingClient_GetSegmentChildName( pRetimingClient, "Bob", "Pelvis", 0, 128, SegmentChildName );
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.EnableSegmentData();
      ///      MyClient.GetFrame();
      ///      Output_GetSegmentChildName Output;
      ///      // Segment index must be between 0 and GetSegmentChildCount() - 1
      ///      Output = MyClient.GetSegmentChildName( "Bob", "Pelvis", 0 );
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.UpdateFrame();
      ///      Output_GetSegmentChildName Output;
      ///      // Segment index must be between 0 and GetSegmentChildCount() - 1
      ///      Output = MyClient.GetSegmentChildName( "Bob", "Pelvis", 0 );
      /// -----      
      ///
      /// \param SubjectName The name of the subject
      /// \param SegmentName The name of the segment
      /// \param SegmentIndex The index of the child segment. A valid Segment Index is between 0 and GetSegmentChildCount()-1.
      /// \return An Output_GetSegmentChildName  class containing the result of the operation and the name of the child segment.
      ///         - The Result will be:
      ///           + Success
      ///           + NotConnected
      ///           + NoFrame
      ///           + InvalidIndex
      ///           + InvalidSubjectName
      ///           + InvalidSegmentName
      Output_GetSegmentChildName GetSegmentChildName(const String & SubjectName,
        const String       & SegmentName,
        const unsigned int   SegmentIndex) const;

      /// Return the name of the parent segment for a specified subject segment. If the specified segment is 
      /// the root segment of the subject then it will return an empty string.
      ///
      /// See Also: GetSegmentCount(), GetSegmentChildCount(), GetSegmentChildName(), GetSubjectRootSegmentName()
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      char SegmentParentName[128];
      ///      CEnum Result = RetimingClient_GetSegmentParentName(
      ///        pRetimingClient, "Bob", "Pelvis", 128, SegmentParentName);
      ///      // Result == Success
      ///      // SegmentParentName = ""
      ///      // This is the root segment
      ///      Result = RetimingClient_GetSegmentParentName(pRetimingClient, "Bob", "LFemur", 128, SegmentParentName);
      ///      // Result == Success
      ///      // SegmentParentName = "Pelvis"
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.EnableSegmentData();
      ///      MyClient.GetFrame();
      ///      Output_GetSegmentParentName Output;
      ///      Output = MyClient.GetSegmentParentName( "Bob", "Pelvis" );
      ///      // Output.Result == Success
      ///      // Output.SegmentName == ""
      ///      // This is the root segment
      ///      Output = MyClient.GetSegmentParentName( "Bob", "LFemur" );
      ///      // Output.Result == Success
      ///      // Output.SegmentName == "Pelvis"
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.UpdateFrame();
      ///      Output_GetSegmentParentName Output;
      ///      Output = MyClient.GetSegmentParentName( "Bob", "Pelvis" );
      ///      // Output.Result == Success
      ///      // Output.SegmentName == ""
      ///      // This is the root segment
      ///      Output = MyClient.GetSegmentParentName( "Bob", "LFemur" );
      ///      // Output.Result == Success
      ///      // Output.SegmentName == "Pelvis"
      /// -----      
      ///
      /// \param SubjectName The name of the subject
      /// \param SegmentName The name of the segment
      /// \return An Output_GetSegmentParentName  class containing the result of the operation and the name of the parent segment or an empty string if it is the root segment.
      ///         - The Result will be:
      ///           + Success
      ///           + NotConnected
      ///           + NoFrame
      ///           + InvalidSubjectName
      ///           + InvalidSegmentName
      Output_GetSegmentParentName GetSegmentParentName(const String & SubjectName,
        const String & SegmentName) const;

      /// Return the static pose translation of a subject segment.
      ///
      /// See Also: GetSegmentStaticRotationHelical(), GetSegmentStaticRotationMatrix(), GetSegmentStaticRotationQuaternion(), GetSegmentStaticRotationEulerXYZ(), GetSegmentLocalTranslation(), GetSegmentLocalRotationHelical(), GetSegmentLocalRotationMatrix(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ()
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      COutput_GetSegmentStaticTranslation _Output_GetSegmentStaticTranslation;
      ///      RetimingClient_GetSegmentStaticTranslation(
      ///        pRetimingClient, "Alice", "Pelvis", &_Output_GetSegmentStaticTranslation);
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.EnableSegmentData();
      ///      MyClient.GetFrame();
      ///      Output_GetSegmentStaticTranslation Output =
      ///      MyClient.GetSegmentStaticTranslation( "Alice", "Pelvis" );
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.UpdateFrame();
      ///      Output_GetSegmentStaticTranslation Output =
      ///      MyClient.GetSegmentStaticTranslation( "Alice", "Pelvis" );
      /// -----      
      ///
      /// \param SubjectName The name of the subject
      /// \param SegmentName The name of the segment
      /// \return An Output_GetSegmentStaticTranslation class containing the result of the operation and the translation of the segment.
      ///         - The Result will be:
      ///           + Success
      ///           + NotConnected
      ///           + NoFrame
      ///           + InvalidSubjectName
      ///           + InvalidSegmentName
      Output_GetSegmentStaticTranslation GetSegmentStaticTranslation(const String & SubjectName,
        const String & SegmentName) const;

      /// Return the static pose rotation of a subject segment in helical coordinates.
      /// The helical coordinates represent a vector whose length is the amount of rotation in radians, and the direction is the axis about which to rotate.
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      COutput_GetSegmentStaticRotationHelical _Output_GetSegmentStaticRotationHelical;
      ///      RetimingClient_GetSegmentStaticRotationHelical(
      ///        pRetimingClient, "Alice", "Pelvis", &_Output_GetSegmentStaticRotationHelical);
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.GetFrame();
      ///      Output_GetSegmentStaticRotationHelical Output =
      ///      MyClient.GetSegmentStaticRotationHelical( "Alice", "Pelvis" );
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.UpdateFrame();
      ///      Output_GetSegmentStaticRotationHelical Output =
      ///      MyClient.GetSegmentStaticRotationHelical( "Alice", "Pelvis" );
      /// -----      
      /// See Also: GetSegmentStaticTranslation(), GetSegmentStaticRotationMatrix(), GetSegmentStaticRotationQuaternion(), GetSegmentStaticRotationEulerXYZ(), GetSegmentLocalTranslation, GetSegmentLocalRotationHelical, GetSegmentLocalRotationMatrix, GetSegmentLocalRotationQuaternion, GetSegmentLocalRotationEulerXYZ
      ///
      ///        
      ///
      /// \param SubjectName The name of the subject
      /// \param SegmentName The name of the segment
      /// \return An Output_GetSegmentStaticRotationHelical  class containing the result of the operation and the rotation of the segment.
      ///         - The Result will be:
      ///           + Success
      ///           + NotConnected
      ///           + NoFrame
      ///           + InvalidSubjectName
      ///           + InvalidSegmentName
      Output_GetSegmentStaticRotationHelical GetSegmentStaticRotationHelical(const String & SubjectName,
        const String & SegmentName) const;

      /// Return the static pose rotation of a subject segment as a 3x3 row-major matrix.
      ///
      /// See Also: GetSegmentStaticTranslation(), GetSegmentStaticRotationHelical(), GetSegmentStaticRotationQuaternion(), GetSegmentStaticRotationEulerXYZ(), GetSegmentLocalTranslation(), GetSegmentLocalRotationHelical(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ()
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      COutput_GetSegmentStaticRotationMatrix _Output_GetSegmentStaticRotationMatrix;
      ///      RetimingClient_GetSegmentStaticRotationMatrix(
      ///        pRetimingClient, "Alice", "Pelvis", &_Output_GetSegmentStaticRotationMatrix);
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.GetFrame();
      ///      Output_GetSegmentStaticRotationMatrix Output =
      ///      MyClient.GetSegmentStaticRotationMatrix( "Alice", "Pelvis" );
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.UpdateFrame();
      ///      Output_GetSegmentStaticRotationMatrix Output =
      ///      MyClient.GetSegmentStaticRotationMatrix( "Alice", "Pelvis" );
      /// -----      
      ///        
      ///
      /// \param SubjectName The name of the subject
      /// \param SegmentName The name of the segment
      /// \return An Output_GetSegmentStaticRotationMatrix  class containing the result of the operation and the rotation of the segment.
      ///         - The Result will be:
      ///           + Success
      ///           + NotConnected
      ///           + NoFrame
      ///           + InvalidSubjectName
      ///           + InvalidSegmentName
      Output_GetSegmentStaticRotationMatrix GetSegmentStaticRotationMatrix(const String & SubjectName,
        const String & SegmentName) const;

      /// Return the static pose rotation of a subject segment in quaternion coordinates.
      /// The quaternion is of the form (x, y, z, w) where w is the real component and x, y and z are the imaginary components. 
      /// N.B. This is different from that used in many other applications, which use (w, x, y, z).
      ///
      /// See Also: GetSegmentStaticTranslation(), GetSegmentStaticRotationHelical(), GetSegmentStaticRotationMatrix(), GetSegmentStaticRotationEulerXYZ(), GetSegmentLocalTranslation(), GetSegmentLocalRotationHelical(), GetSegmentLocalRotationMatrix(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ()
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      COutput_GetSegmentStaticRotationQuaternion _Output_GetSegmentStaticRotationQuaternion;
      ///      RetimingClient_GetSegmentStaticRotationQuaternion(
      ///        pRetimingClient, "Alice", "Pelvis", &_Output_GetSegmentStaticRotationQuaternion);
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.GetFrame();
      ///      Output_GetSegmentStaticRotationQuaternion Output =
      ///      MyClient.GetSegmentStaticRotationQuaternion( "Alice", "Pelvis" );
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.UpdateFrame();
      ///      Output_GetSegmentStaticRotationQuaternion Output =
      ///      MyClient.GetSegmentStaticRotationQuaternion( "Alice", "Pelvis" );
      /// -----      
      ///        
      ///
      ///
      /// \param SubjectName The name of the subject
      /// \param SegmentName The name of the segment
      /// \return An Output_GetSegmentStaticRotationQuaternion class containing the result of the operation and the rotation of the segment.
      ///         - The Result will be:
      ///           + Success
      ///           + NotConnected
      ///           + NoFrame
      ///           + InvalidSubjectName
      ///           + InvalidSegmentName
      Output_GetSegmentStaticRotationQuaternion GetSegmentStaticRotationQuaternion(const String & SubjectName,
        const String & SegmentName) const;

      /// Return the static pose rotation of a subject segment in Euler XYZ coordinates.
      ///
      /// See Also: GetSegmentStaticTranslation(), GetSegmentStaticRotationHelical(), GetSegmentStaticRotationMatrix(), 
      /// GetSegmentStaticRotationQuaternion(), GetSegmentLocalTranslation(), GetSegmentLocalRotationHelical(), 
      /// GetSegmentLocalRotationMatrix(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ().
      /// 
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      COutput_GetSegmentStaticRotationEulerXYZ _Output_GetSegmentStaticRotationEulerXYZ;
      ///      RetimingClient_GetSegmentStaticRotationEulerXYZ(
      ///        pRetimingClient, "Alice", "Pelvis", &_Output_GetSegmentStaticRotationEulerXYZ);
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.GetFrame();
      ///      Output_GetSegmentStaticRotationEulerXYZ Output;
      ///      Output = MyClient.GetSegmentStaticRotationEulerXYZ( "Alice", "Pelvis" );
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.UpdateFrame();
      ///      Output_GetSegmentStaticRotationEulerXYZ Output;
      ///      Output = MyClient.GetSegmentStaticRotationEulerXYZ( "Alice", "Pelvis" );
      /// -----      
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
      Output_GetSegmentStaticRotationEulerXYZ GetSegmentStaticRotationEulerXYZ(const String & SubjectName,
        const String & SegmentName) const;

      /// Return the translation of a subject segment in global coordinates.
      /// The translation is of the form ( x, y, z ) where x, y and z are in millimeters with respect to the global origin.
      ///
      /// See Also: GetSegmentGlobalRotationHelical(), GetSegmentGlobalRotationMatrix(), GetSegmentGlobalRotationQuaternion(), GetSegmentGlobalRotationEulerXYZ(), GetSegmentLocalTranslation(), GetSegmentLocalRotationHelical(), GetSegmentLocalRotationMatrix(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ()
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      COutput_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation;
      ///      RetimingClient_GetSegmentGlobalTranslation(
      ///        pRetimingClient, "Alice", "Pelvis", &_Output_GetSegmentGlobalTranslation);
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.EnableSegmentData();
      ///      MyClient.GetFrame();
      ///      Output_GetSegmentGlobalTranslation Output =
      ///      MyClient.GetSegmentGlobalTranslation( "Alice", "Pelvis" );
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.UpdateFrame();
      ///      Output_GetSegmentGlobalTranslation Output =
      ///      MyClient.GetSegmentGlobalTranslation( "Alice", "Pelvis" );
      /// -----      
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
      Output_GetSegmentGlobalTranslation GetSegmentGlobalTranslation(const String & SubjectName,
        const String & SegmentName) const;

      /// Return the rotation of a subject segment in global helical coordinates.
      ///
      /// See Also: GetSegmentGlobalTranslation(), GetSegmentGlobalRotationMatrix(), GetSegmentGlobalRotationQuaternion(), GetSegmentGlobalRotationEulerXYZ(), GetSegmentLocalTranslation(), GetSegmentLocalRotationHelical(), GetSegmentLocalRotationMatrix(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ()
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      COutput_GetSegmentGlobalRotationHelical _Output_GetSegmentGlobalRotationHelical;
      ///      RetimingClient_GetSegmentGlobalRotationHelical(
      ///        pRetimingClient, "Alice", "Pelvis", &_Output_GetSegmentGlobalRotationHelical);
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.GetFrame();
      ///      Output_GetSegmentGlobalRotationHelical Output =
      ///      MyClient.GetSegmentGlobalRotationHelical( "Alice", "Pelvis" );
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.UpdateFrame();
      ///      Output_GetSegmentGlobalRotationHelical Output =
      ///      MyClient.GetSegmentGlobalRotationHelical( "Alice", "Pelvis" );
      /// -----      
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
      Output_GetSegmentGlobalRotationHelical GetSegmentGlobalRotationHelical(const String & SubjectName,
        const String & SegmentName) const;

      /// Return the rotation of a subject segment as a 3x3 row-major matrix in global coordinates.
      ///
      /// See Also: GetSegmentGlobalTranslation(), GetSegmentGlobalRotationHelical(), GetSegmentGlobalRotationQuaternion(), GetSegmentGlobalRotationEulerXYZ(), GetSegmentLocalTranslation(), GetSegmentLocalRotationHelical(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ()
      ///
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      COutput_GetSegmentGlobalRotationMatrix _Output_GetSegmentGlobalRotationMatrix;
      ///      RetimingClient_GetSegmentGlobalRotationMatrix(
      ///        pRetimingClient, "Alice", "Pelvis", &_Output_GetSegmentGlobalRotationMatrix);
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.GetFrame();
      ///      Output_GetSegmentGlobalRotationMatrix Output =
      ///      MyClient.GetSegmentGlobalRotationMatrix( "Alice", "Pelvis" );
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.UpdateFrame();
      ///      Output_GetSegmentGlobalRotationMatrix Output =
      ///      MyClient.GetSegmentGlobalRotationMatrix( "Alice", "Pelvis" );
      /// -----      
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
      Output_GetSegmentGlobalRotationMatrix GetSegmentGlobalRotationMatrix(const String & SubjectName,
        const String & SegmentName) const;

      /// Return the rotation of a subject segment in global quaternion coordinates.
      /// The quaternion is of the form (x, y, z, w) where w is the real component and x, y and z are the imaginary components. 
      /// N.B. This is different from that used in many other applications, which use (w, x, y, z).
      ///
      /// See Also: GetSegmentGlobalTranslation(), GetSegmentGlobalRotationHelical(), GetSegmentGlobalRotationMatrix(), GetSegmentGlobalRotationEulerXYZ(), GetSegmentLocalTranslation(), GetSegmentLocalRotationHelical(), GetSegmentLocalRotationMatrix(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ()
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      COutput_GetSegmentGlobalRotationQuaternion _Output_GetSegmentGlobalRotationQuaternion;
      ///      RetimingClient_GetSegmentGlobalRotationQuaternion(
      ///        pRetimingClient, "Alice", "Pelvis", &_Output_GetSegmentGlobalRotationQuaternion);
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.GetFrame();
      ///      Output_GetSegmentGlobalRotationQuaternion Output =
      ///      MyClient.GetSegmentGlobalRotationQuaternion( "Alice", "Pelvis" );
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.UpdateFrame();
      ///      Output_GetSegmentGlobalRotationQuaternion Output =
      ///      MyClient.GetSegmentGlobalRotationQuaternion( "Alice", "Pelvis" );
      /// -----      
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
      Output_GetSegmentGlobalRotationQuaternion GetSegmentGlobalRotationQuaternion(const String & SubjectName,
        const String & SegmentName) const;

      /// Return the rotation of a subject segment in global Euler XYZ coordinates.
      ///
      /// See Also: GetSegmentGlobalTranslation(), GetSegmentGlobalRotationHelical(), GetSegmentGlobalRotationMatrix(), GetSegmentGlobalRotationQuaternion(), GetSegmentLocalTranslation(), GetSegmentLocalRotationHelical(), GetSegmentLocalRotationMatrix(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ()
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      COutput_GetSegmentGlobalRotationEulerXYZ _Output_GetSegmentGlobalRotationEulerXYZ;
      ///      RetimingClient_GetSegmentGlobalRotationEulerXYZ(
      ///        pRetimingClient, "Alice", "Pelvis", &_Output_GetSegmentGlobalRotationEulerXYZ);
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.GetFrame();
      ///      Output_GetSegmentGlobalRotationEulerXYZ Output =
      ///      MyClient.GetSegmentGlobalRotationEulerXYZ( "Alice", "Pelvis" );
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.UpdateFrame();
      ///      Output_GetSegmentGlobalRotationEulerXYZ Output =
      ///      MyClient.GetSegmentGlobalRotationEulerXYZ( "Alice", "Pelvis" );
      /// -----      
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
      Output_GetSegmentGlobalRotationEulerXYZ GetSegmentGlobalRotationEulerXYZ(const String & SubjectName,
        const String & SegmentName) const;

      /// Return the translation of a subject segment in local coordinates relative to its parent segment.
      ///
      /// See Also: GetSegmentLocalRotationHelical(), GetSegmentLocalRotationMatrix(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ(), GetSegmentGlobalTranslation(), GetSegmentGlobalRotationHelical(), GetSegmentGlobalRotationMatrix(), GetSegmentGlobalRotationQuaternion(), GetSegmentGlobalRotationEulerXYZ()
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      COutput_GetSegmentLocalTranslation _Output_GetSegmentLocalTranslation;
      ///      RetimingClient_GetSegmentLocalTranslation(
      ///        pRetimingClient, "Alice", "Pelvis", &_Output_GetSegmentLocalTranslation);
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.EnableSegmentData();
      ///      MyClient.GetFrame();
      ///      Output_GetSegmentLocalTranslation Output =
      ///      MyClient.GetSegmentLocalTranslation( "Alice", "Pelvis" );
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.UpdateFrame();
      ///      Output_GetSegmentLocalTranslation Output =
      ///      MyClient.GetSegmentLocalTranslation( "Alice", "Pelvis" );
      /// -----      
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
      Output_GetSegmentLocalTranslation GetSegmentLocalTranslation(const String & SubjectName,
        const String & SegmentName) const;

      /// Return the rotation of a subject segment in local helical coordinates relative to its parent segment.
      ///
      /// See Also: GetSegmentLocalTranslation(), GetSegmentLocalRotationMatrix(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ(), GetSegmentGlobalTranslation(), GetSegmentGlobalRotationHelical(), GetSegmentGlobalRotationMatrix(), GetSegmentGlobalRotationQuaternion(), GetSegmentGlobalRotationEulerXYZ()
      ///
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      COutput_GetSegmentLocalRotationHelical _Output_GetSegmentLocalRotationHelical;
      ///      RetimingClient_GetSegmentLocalRotationHelical(
      ///        pRetimingClient, "Alice", "Pelvis", &_Output_GetSegmentLocalRotationHelical);
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.GetFrame();
      ///      Output_GetSegmentLocalRotationHelical Output =
      ///      MyClient.GetSegmentLocalRotationHelical( "Alice", "Pelvis" );
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.UpdateFrame();
      ///      Output_GetSegmentLocalRotationHelical Output =
      ///      MyClient.GetSegmentLocalRotationHelical( "Alice", "Pelvis" );
      /// -----      
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
      Output_GetSegmentLocalRotationHelical GetSegmentLocalRotationHelical(const String & SubjectName,
        const String & SegmentName) const;

      /// Return the rotation row-major matrix of a subject segment in local coordinates relative to its parent segment.
      ///
      /// See Also: GetSegmentLocalTranslation(), GetSegmentLocalRotationQuaternion(), GetSegmentLocalRotationEulerXYZ(), GetSegmentGlobalTranslation(), GetSegmentGlobalRotationHelical(), GetSegmentGlobalRotationMatrix() , GetSegmentGlobalRotationQuaternion(), GetSegmentGlobalRotationEulerXYZ()
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      COutput_GetSegmentLocalRotationMatrix _Output_GetSegmentLocalRotationMatrix;
      ///      RetimingClient_GetSegmentLocalRotationMatrix(
      ///        pRetimingClient, "Alice", "Pelvis", &_Output_GetSegmentLocalRotationMatrix);
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.GetFrame();
      ///      Output_GetSegmentLocalRotationMatrix Output =
      ///      MyClient.GetSegmentLocalRotationMatrix( "Alice", "Pelvis" );
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.UpdateFrame();
      ///      Output_GetSegmentLocalRotationMatrix Output =
      ///      MyClient.GetSegmentLocalRotationMatrix( "Alice", "Pelvis" );
      /// -----      
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
      Output_GetSegmentLocalRotationMatrix GetSegmentLocalRotationMatrix(const String & SubjectName,
        const String & SegmentName) const;

      /// Return the rotation of a subject segment in local quaternion coordinates relative to its parent segment.
      /// The quaternion is of the form (x, y, z, w) where w is the real component and x, y and z are the imaginary components. N.B. This is different from that used in many other applications, which use (w, x, y, z).
      ///
      /// See Also: GetSegmentLocalTranslation(), GetSegmentLocalRotationHelical(), GetSegmentLocalRotationMatrix(), GetSegmentLocalRotationEulerXYZ(), GetSegmentGlobalTranslation(), GetSegmentGlobalRotationHelical(), GetSegmentGlobalRotationMatrix(), GetSegmentGlobalRotationQuaternion(), GetSegmentGlobalRotationEulerXYZ()
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      COutput_GetSegmentLocalRotationQuaternion _Output_GetSegmentLocalRotationQuaternion;
      ///      RetimingClient_GetSegmentLocalRotationQuaternion(
      ///        pRetimingClient, "Alice", "Pelvis", &_Output_GetSegmentLocalRotationQuaternion);
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.GetFrame();
      ///      Output_GetSegmentLocalRotationQuaternion Output =
      ///      MyClient.GetSegmentLocalRotationQuaternion( "Alice", "Pelvis" );
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.UpdateFrame();
      ///      Output_GetSegmentLocalRotationQuaternion Output =
      ///      MyClient.GetSegmentLocalRotationQuaternion( "Alice", "Pelvis" );
      /// -----      
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
      Output_GetSegmentLocalRotationQuaternion GetSegmentLocalRotationQuaternion(const String & SubjectName,
        const String & SegmentName) const;

      /// Return the rotation of a subject segment in local Euler XYZ coordinates relative to its parent segment.
      ///
      /// See Also: GetSegmentLocalTranslation(), GetSegmentLocalRotationHelical(), GetSegmentLocalRotationMatrix(), GetSegmentLocalRotationQuaternion(), GetSegmentGlobalTranslation(), GetSegmentGlobalRotationHelical(), GetSegmentGlobalRotationMatrix() , GetSegmentGlobalRotationQuaternion(), GetSegmentGlobalRotationEulerXYZ()
      ///
      ///
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      COutput_GetSegmentLocalRotationEulerXYZ _Output_GetSegmentLocalRotationEulerXYZ;
      ///      RetimingClient_GetSegmentLocalRotationEulerXYZ(
      ///        pRetimingClient, "Alice", "Pelvis", &_Output_GetSegmentLocalRotationEulerXYZ);
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.GetFrame();
      ///      Output_GetSegmentLocalRotationEulerXYZ Output =
      ///      MyClient.GetSegmentLocalRotationEulerXYZ( "Alice", "Pelvis" );
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.UpdateFrame();
      ///      Output_GetSegmentLocalRotationEulerXYZ Output =
      ///      MyClient.GetSegmentLocalRotationEulerXYZ( "Alice", "Pelvis" );
      /// -----      
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
      Output_GetSegmentLocalRotationEulerXYZ GetSegmentLocalRotationEulerXYZ(const String & SubjectName,
        const String & SegmentName) const;


      ///@private
      void SetOutputLatency(double OutputLatency);

      ///@private
      double OutputLatency() const;

      /// Sets the maximum amount by which the interpolation engine will predict later than the latest received frame. If required to predict by more than this amount,
      /// the result LateDataRequested will be returned.
      /// 
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_SetMaximumPrediction( pRetimingClient, 30 );
      ///      RetimingClient_Connect( pRetimingClient, "localhost" );
      ///      RetimingClient_GetFrame( pRetimingClient );
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.SetMaximumPrediction( 30 );
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.GetFrame();
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.SetMaximumPrediction( 30 );
      ///      MyClient.Connect( "localhost" );
      ///      MyClient.UpdateFrame();
      /// -----      
      /// \param MaxPrediction The maximum amount of prediction required in milliseconds
      void SetMaximumPrediction(double MaxPrediction);

      /// Returns the maximum prediction value currently in use. The default value is 100 ms.
      /// 
      ///
      /// C example
      ///      
      ///      CRetimingClient * pRetimingClient = RetimingClient_Create();
      ///      RetimingClient_SetMaximumPrediction( pRetimingClient, 30 );
      ///      RetimingClient_MaximumPrediction( pRetimingClient ); // Returns 30
      ///      RetimingClient_Destroy( pRetimingClient );
      ///      
      /// C++ example
      ///      
      ///      ViconDataStreamSDK::CPP::RetimingClient MyClient;
      ///      MyClient.SetMaximumPrediction( 30 );
      ///      MyClient.MaximumPrediction(); // Returns 30
      ///      
      /// MATLAB example
      ///      
      /// Not implemented
      ///      
      /// .NET example
      ///      
      ///      ViconDataStreamSDK.DotNET.RetimingClient MyClient = new ViconDataStreamSDK.DotNET.RetimingClient();
      ///      MyClient.SetMaximumPrediction( 30 );
      ///      MyClient.MaximumPrediction(); // Returns 30
      /// -----      
      /// \return The maximum prediction allowed in milliseconds
      double MaximumPrediction() const;

    private:
      RetimingClientImpl * m_pClientImpl;
    };
  }
}
