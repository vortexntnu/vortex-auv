
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

#include <boost/thread.hpp>
#include <boost/array.hpp>
#include <boost/thread/condition.hpp>
#include <boost/timer/timer.hpp>
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <deque>

#include "Constants.h"
#include "ClientUtils.h"

namespace ViconCGStreamClientSDK
{
  class ICGClient;
}

namespace ViconDataStreamSDK
{

  namespace Core
  {
    class VSegmentPose
    {
    public:

      VSegmentPose() :
        bOccluded(false) {}

      std::string Name;
      std::string Parent;

      boost::array< double, 3 > T;
      boost::array< double, 4 > R;

      boost::array< double, 3 > T_Rel;
      boost::array< double, 4 > R_Rel;

      boost::array< double, 3 > T_Stat;
      boost::array< double, 4 > R_Stat;

      std::vector< std::string > m_Children;

      bool   bOccluded;
    };

    class VSubjectPose
    {
    public:

      enum EResult
      {
        ESuccess,
        ENoData,
        ENotConnected,
        EUnknownSubject,
        EEarly,
        ELate,
        EInvalid
      };

      VSubjectPose()
        : Result(EInvalid)
        , FrameTime(0)
        , ReceiptTime(0)
      {}

      EResult Result;

      std::string Name;
      std::string RootSegment;

      double Latency;
      double FrameTime;
      double ReceiptTime;

      std::map< std::string, std::shared_ptr< VSegmentPose > > m_Segments;
    };
    
    class VClient;

    class VRetimingClient
    {
    public:

      VRetimingClient( VClient & i_rClient );
      ~VRetimingClient();

      // Connect client to the Vicon Data Stream
      Result::Enum Connect(std::shared_ptr< ViconCGStreamClientSDK::ICGClient > i_pClient,
        const std::string & i_rHostName);

      // Disconnect from the Vicon Data Stream
      Result::Enum Disconnect();

      // Interface
      Result::Enum StartOutput( double i_FrameRate );
      Result::Enum StopOutput();
      bool IsRunning() const;

      // Store a predicted pose for all subjects currently in the stream at the current time.
      Result::Enum UpdateFrame(double i_Offset);

      Result::Enum GetSubjectCount(unsigned int & o_rSubjectCount) const;
      Result::Enum GetSubjectName(const unsigned int i_SubjectIndex, std::string& o_rSubjectName) const;
      Result::Enum GetSubjectRootSegmentName(const std::string & i_rSubjectName, std::string & o_rSegmentName) const;

      Result::Enum GetSegmentCount(const std::string& i_rSubjectName, unsigned int& o_rSegmentCount) const;
      Result::Enum GetSegmentName(const std::string& i_rSubjectName, const unsigned int i_SegmentIndex, std::string& o_rSegmentName) const;
      Result::Enum GetSegmentChildCount(const std::string& i_rSubjectName, const std::string& i_rSegmentName, unsigned int & o_rSegmentCount) const;
      Result::Enum GetSegmentChildName(const std::string& i_rSubjectName, const std::string& i_rSegmentName, unsigned int i_SegmentIndex, std::string& o_rSegmentName) const;
      Result::Enum GetSegmentParentName(const std::string& i_rSubjectName, const std::string& i_rSegmentName, std::string& o_rSegmentName) const;

      Result::Enum GetSegmentStaticTranslation(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rThreeVector)[3]) const;
      Result::Enum GetSegmentStaticRotationHelical(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rThreeVector)[3]) const;
      Result::Enum GetSegmentStaticRotationMatrix(const std::string & i_rSubjectName, const std::string & i_rSegmentName, double(&o_rRotation)[9]) const;
      Result::Enum GetSegmentStaticRotationQuaternion(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rFourVector)[4]) const;
      Result::Enum GetSegmentStaticRotationEulerXYZ(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rThreeVector)[3]) const;

      Result::Enum GetSegmentGlobalTranslation(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rThreeVector)[3], bool& o_rbOccluded) const;
      Result::Enum GetSegmentGlobalRotationHelical(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rThreeVector)[3], bool& o_rbOccluded) const;
      Result::Enum GetSegmentGlobalRotationMatrix(const std::string & i_rSubjectName, const std::string & i_rSegmentName, double(&o_rRotation)[9], bool & o_rbOccluded) const;
      Result::Enum GetSegmentGlobalRotationQuaternion(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rFourVector)[4], bool& o_rbOccluded) const;
      Result::Enum GetSegmentGlobalRotationEulerXYZ(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rThreeVector)[3], bool& o_rbOccluded) const;

      Result::Enum GetSegmentLocalTranslation(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_pThreeVector)[3], bool& o_rbOccluded) const;
      Result::Enum GetSegmentLocalRotationHelical(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rThreeVector)[3], bool& o_rbOccluded) const;
      Result::Enum GetSegmentLocalRotationMatrix(const std::string & i_rSubjectName, const std::string & i_rSegmentName, double(&o_rRotation)[9], bool & o_rbOccluded) const;
      Result::Enum GetSegmentLocalRotationQuaternion(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rFourVector)[4], bool& o_rbOccluded) const;
      Result::Enum GetSegmentLocalRotationEulerXYZ(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rThreeVector)[3], bool& o_rbOccluded) const;

      // Wait for a frame, populates pose of all objects in o_rSubjects mapped to name
      Result::Enum WaitForFrame() const;

      // Set the latency required in the output stream; may be used to avoid the need to predict data forwards, 
      // where a small amount of latency is acceptable.
      void SetOutputLatency( double i_OutputLatency );

      // Return the output latency specified
      double OutputLatency() const;

      // Set the maximum amount by which the interpolation engine will predict forward
      void SetMaximumPrediction( double i_MaxPrediction );

      // Return the maximum prediction used by the system
      double MaximumPrediction() const;

      // Set an estimate for (fixed) network latency
      void SetNetworkLatency(double i_Latency);

      // This needs to be public, or make the unit test a friend
      std::shared_ptr< const VSubjectPose > Predict(std::shared_ptr< const VSubjectPose > p1, std::shared_ptr< const VSubjectPose > p2, double t) const;

    private:

      void InputThread();
      void StopInput();
      void AddData(const std::string & i_rName, std::shared_ptr< VSubjectPose > i_pData);
      Result::Enum GetSubject(const std::string & i_rSubjectName, std::shared_ptr< const VSubjectPose > & o_rpSubject) const;


      bool InitGet(Result::Enum & o_rResult) const;
      template < typename T > bool InitGet(Result::Enum & o_rResult, T & o_rOutput) const;
      template < typename T1, typename T2 > bool InitGet(Result::Enum & o_rResult, T1 & o_rOutput1, T2 & o_rOutput2) const;
      template < typename T1, typename T2, typename T3 > bool InitGet(Result::Enum & o_rResult, T1 & o_rOutput1, T2 & o_rOutput2, T3 & o_rOutput3) const;

    private:


      VClient & m_rClient;

      void OutputThread();


      mutable boost::recursive_mutex m_DataMutex;
      std::map< std::string, std::deque< std::shared_ptr< const VSubjectPose > > > m_Data;

      boost::recursive_mutex m_FrameRateMutex;
      double m_FrameRate;
      unsigned int m_OutputFrameNumber;

      mutable boost::condition m_OutputWait;
      std::map< std::string, std::shared_ptr< const VSubjectPose > > m_LatestOutputPoses;

      std::unique_ptr< boost::thread > m_pInputThread;
      bool m_bInputStopped;

      std::unique_ptr< boost::thread > m_pOutputThread;
      bool m_bOutputStopped;

      // Required output latency (in milliseconds)
      double m_OutputLatency;

      // Maximum time we should predict forwards (in milliseconds)
      double m_MaxPredictionTime;

      // Estimated amount for network latency
      double m_NetworkLatency;

      boost::timer::cpu_timer m_Timer;
    };

    template < typename T >
    bool ViconDataStreamSDK::Core::VRetimingClient::InitGet(Result::Enum & o_rResult, T & o_rOutput) const
    {
      ClientUtils::Clear(o_rOutput);
      return InitGet(o_rResult);
    }

    template < typename T1, typename T2 >
    bool ViconDataStreamSDK::Core::VRetimingClient::InitGet(Result::Enum & o_rResult, T1 & o_rOutput1, T2 & o_rOutput2) const
    {
      ClientUtils::Clear(o_rOutput1);
      ClientUtils::Clear(o_rOutput2);
      return InitGet(o_rResult);
    }

    template < typename T1, typename T2, typename T3 >
    bool ViconDataStreamSDK::Core::VRetimingClient::InitGet(Result::Enum & o_rResult, T1 & o_rOutput1, T2 & o_rOutput2, T3 & o_rOutput3) const
    {
      ClientUtils::Clear(o_rOutput1);
      ClientUtils::Clear(o_rOutput2);
      ClientUtils::Clear(o_rOutput3);
      return InitGet(o_rResult);
    }
  }
}
