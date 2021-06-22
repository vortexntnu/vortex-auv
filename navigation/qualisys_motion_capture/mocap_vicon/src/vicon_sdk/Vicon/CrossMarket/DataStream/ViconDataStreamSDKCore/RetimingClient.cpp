
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
#include "RetimingClient.h"
#include "CoreClient.h"

#include "RetimerUtils.h"

#pragma warning ( push )
#pragma warning ( disable : 4265 )
#include <thread>
#pragma warning ( pop )
#include <chrono>

#include <algorithm>
#include <chrono>

using namespace ClientUtils;



namespace ViconDataStreamSDK
{

  namespace Core
  {
    Result::Enum Adapt(const VSubjectPose::EResult & i_rResult)
    {
      switch (i_rResult)
      {
      case VSubjectPose::ESuccess: return Result::Success;
      case VSubjectPose::ENoData: return Result::NoFrame;
      case VSubjectPose::ENotConnected: return Result::NotConnected;
      case VSubjectPose::EUnknownSubject: return Result::InvalidSubjectName;
      case VSubjectPose::EEarly: return Result::EarlyDataRequested;
      case VSubjectPose::ELate: return Result::LateDataRequested;
      case VSubjectPose::EInvalid: return Result::InvalidOperation;
      };
      return Result::Unknown;
    }

    static unsigned int s_BufSize = 3;

    VRetimingClient::VRetimingClient(VClient & i_rClient)
      : m_rClient(i_rClient)
      , m_bInputStopped(false)
      , m_bOutputStopped(false)
      , m_OutputLatency(0.0)
      , m_MaxPredictionTime(100)
      , m_NetworkLatency( 0.3 )
    {

    }

    VRetimingClient::~VRetimingClient()
    {
      Disconnect();
    }

    // Connect client to the Vicon Data Stream
    Result::Enum VRetimingClient::Connect(
      std::shared_ptr< ViconCGStreamClientSDK::ICGClient > i_pClient,
      const std::string & i_rHostName)
    {

      Result::Enum Result = m_rClient.Connect(i_pClient, i_rHostName);

      if (Result == Result::Success)
      {
        // Ensure that segments are enabled
        m_rClient.EnableSegmentData();

        // We need to work in server push mode
        m_rClient.SetStreamMode(StreamMode::ServerPush);

        // Start frame acquisition thread
        m_pInputThread.reset(new boost::thread(boost::bind(&VRetimingClient::InputThread, this)));
      }

      return Result;
    }

    // Disconnect from the Vicon Data Stream
    Result::Enum VRetimingClient::Disconnect()
    {
      // Stop output thread
      StopOutput();

      // Stop input thread
      StopInput();

      return m_rClient.Disconnect();
    }


    Result::Enum VRetimingClient::StartOutput(double i_FrameRate)
    {
      // Can't start if our input thread isn't running
      if (!m_pInputThread)
      {
        return Result::NotConnected;
      }

      StopOutput();

      // Run for a bit first
      unsigned int FrameCount = 0;
      unsigned int RunInPeriod = 20;
      while (FrameCount < RunInPeriod)
      {
        m_rClient.GetFrame();
        ++FrameCount;
      }


      double FrameRate = 0;

      // Set output rate to input rate if none specified
      if (i_FrameRate <= 0)
      {
        FrameRate = i_FrameRate;
      }
      else
      {
        m_rClient.GetFrameRate(FrameRate);
      }


      boost::recursive_mutex::scoped_lock Lock(m_FrameRateMutex);
      m_FrameRate = FrameRate;
      m_OutputFrameNumber = 0;

      // start thread
      m_pOutputThread.reset(new boost::thread(boost::bind(&VRetimingClient::OutputThread, this)));

      return Result::Success;

    }

    Result::Enum VRetimingClient::StopOutput()
    {
      // stop thread
      if (m_pOutputThread)
      {
        m_bOutputStopped = true;
        m_pOutputThread->join();
        m_pOutputThread.reset();
      }

      m_OutputWait.notify_all();

      return Result::Success;
    }

    void VRetimingClient::StopInput()
    {
      m_bInputStopped = true;

      // Wait for it to stop
      if (m_pInputThread)
      {
        m_pInputThread->join();
        m_pInputThread.reset();
      }
    }

    bool VRetimingClient::IsRunning() const
    {
      return m_pOutputThread != nullptr;
    }


    bool VRetimingClient::InitGet(Result::Enum & o_rResult) const
    {
      o_rResult = Result::Success;

      if (!m_rClient.IsConnected())
      {
        o_rResult = Result::NotConnected;
      }
      else if (m_Data.empty())
      {
        o_rResult = Result::NoFrame;
      }

      return (o_rResult == Result::Success);
    }

    Result::Enum VRetimingClient::GetSubjectCount(unsigned int & o_rSubjectCount) const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);
      
      Result::Enum GetResult = Result::Success;
      if ( InitGet( GetResult, o_rSubjectCount ) )
      {
        o_rSubjectCount = static_cast<unsigned int>(m_Data.size());
      }

      return GetResult;
    }

    Result::Enum VRetimingClient::GetSubjectName(const unsigned int i_SubjectIndex, std::string& o_rSubjectName) const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);
      
      Result::Enum GetResult = Result::Success;
      if (!InitGet(GetResult, o_rSubjectName))
      {
        return GetResult;
      }

      if (i_SubjectIndex >= m_Data.size())
      {
        return Result::InvalidIndex;
      }

      auto rPair = m_Data.begin();
      std::advance(rPair, i_SubjectIndex);
      o_rSubjectName = rPair->first;
      return Result::Success;
    }

    Result::Enum VRetimingClient::GetSubjectRootSegmentName(const std::string & i_rSubjectName, std::string & o_rSegmentName) const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);
      Clear(o_rSegmentName);

      std::shared_ptr< const VSubjectPose > pSubject;
      Result::Enum Result = GetSubject(i_rSubjectName, pSubject );

      if( Result == Result::Success )
      {
        o_rSegmentName = pSubject->RootSegment;
      }

      return Result;
    }

    Result::Enum VRetimingClient::GetSegmentCount(const std::string& i_rSubjectName, unsigned int& o_rSegmentCount) const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);
      Clear(o_rSegmentCount);

      std::shared_ptr< const VSubjectPose > pSubject;
      Result::Enum Result = GetSubject(i_rSubjectName, pSubject);

      if( Result == Result::Success )
      {
        o_rSegmentCount = static_cast< unsigned int >( pSubject->m_Segments.size() );
      }

      return Result;
    }

    Result::Enum VRetimingClient::GetSegmentName(const std::string& i_rSubjectName, const unsigned int i_SegmentIndex, std::string& o_rSegmentName) const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);
      Clear(o_rSegmentName);

      std::shared_ptr< const VSubjectPose > pSubject;
      Result::Enum Result = GetSubject(i_rSubjectName, pSubject);

      if( Result == Result::Success )
      {
        if( i_SegmentIndex < pSubject->m_Segments.size() )
        {
          auto rPair = pSubject->m_Segments.begin();
          std::advance(rPair, i_SegmentIndex);
          o_rSegmentName = rPair->first;
        }
        else
        {
          Result = Result::InvalidIndex;
        }
      }

      return Result;
    }

    Result::Enum VRetimingClient::GetSegmentChildCount(const std::string& i_rSubjectName, const std::string& i_rSegmentName, unsigned int & o_rSegmentCount) const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);
      Clear(o_rSegmentCount);

      std::shared_ptr< const VSubjectPose > pSubject;
      Result::Enum Result = GetSubject(i_rSubjectName, pSubject);

      if( Result == Result::Success )
      {
        const auto pSegment = pSubject->m_Segments.find(i_rSegmentName);
        if( pSegment != pSubject->m_Segments.end() && pSegment->second )
        {
          o_rSegmentCount = static_cast< unsigned int >( pSegment->second->m_Children.size() );
        }
        else
        {
          Result = Result::InvalidSegmentName;
        }
      }

      return Result;
    }

    Result::Enum VRetimingClient::GetSegmentChildName(const std::string& i_rSubjectName, const std::string& i_rSegmentName, unsigned int i_SegmentIndex, std::string& o_rSegmentName) const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);
      Clear(o_rSegmentName);

      std::shared_ptr< const VSubjectPose > pSubject;
      Result::Enum Result = GetSubject(i_rSubjectName, pSubject);

      if( Result == Result::Success )
      {
        const auto pSegment = pSubject->m_Segments.find(i_rSegmentName);
        if( pSegment != pSubject->m_Segments.end() && pSegment->second )
        {
          if( i_SegmentIndex < pSegment->second->m_Children.size() )
          {
            o_rSegmentName = pSegment->second->m_Children[i_SegmentIndex];
          }
          else
          {
            Result = Result::InvalidIndex;
          }
        }
        else
        {
          Result = Result::InvalidSegmentName;
        }
      }

      return Result;
    }

    Result::Enum VRetimingClient::GetSegmentParentName(const std::string& i_rSubjectName, const std::string& i_rSegmentName, std::string& o_rSegmentName) const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);
      Clear(o_rSegmentName);

      std::shared_ptr< const VSubjectPose > pSubject;
      Result::Enum Result = GetSubject(i_rSubjectName, pSubject);

      if( Result == Result::Success )
      {
        const auto pSegment = pSubject->m_Segments.find(i_rSegmentName);
        if( pSegment != pSubject->m_Segments.end() && pSegment->second )
        {
          o_rSegmentName = pSegment->second->Parent;
        }
        else
        {
          Result = Result::InvalidSegmentName;
        }
      }

      return Result;
    }

    Result::Enum VRetimingClient::GetSegmentStaticTranslation(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rThreeVector)[3]) const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);

      Result::Enum GetResult = Result::Success;
      if( !InitGet(GetResult, o_rThreeVector) )
      {
        return GetResult;
      }

      std::shared_ptr< const VSubjectPose > pSubject;
      GetResult = GetSubject(i_rSubjectName, pSubject);

      if( GetResult == Result::Success )
      {
        // Get the segment
        auto SegIt = pSubject->m_Segments.find(i_rSegmentName);
        if( SegIt != pSubject->m_Segments.end() )
        {
          std::copy(SegIt->second->T_Stat.begin(), SegIt->second->T_Stat.end(), o_rThreeVector);
        }
      }
      return GetResult;
    }

    Result::Enum VRetimingClient::GetSegmentStaticRotationHelical(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rThreeVector)[3]) const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);

      Clear(o_rThreeVector);
      // InitGet is called by GetSegmentStaticRotationMatrix.

      // Get the answer as a rotation matrix
      double RotationArray[9];
      const Result::Enum _Result = GetSegmentStaticRotationMatrix(i_rSubjectName, i_rSegmentName, RotationArray);

      if( Result::Success == _Result )
      {
        MatrixToHelical(RotationArray, o_rThreeVector);
      }

      return _Result;
    }

    Result::Enum VRetimingClient::GetSegmentStaticRotationMatrix(const std::string & i_rSubjectName, const std::string & i_rSegmentName, double(&o_rRotation)[9]) const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);

      Result::Enum GetResult = Result::Success;
      if( !InitGet(GetResult, o_rRotation) )
      {
        return GetResult;
      }

      std::shared_ptr< const VSubjectPose > pSubject;
      GetResult = GetSubject(i_rSubjectName, pSubject);

      if( GetResult == Result::Success )
      {
        // Get the segment
        auto SegIt = pSubject->m_Segments.find(i_rSegmentName);
        if( SegIt != pSubject->m_Segments.end() )
        {
          boost::array< double, 9 > StaticRotation = ClientUtils::ToRotationMatrix(SegIt->second->R_Stat);
          std::copy(StaticRotation.begin(), StaticRotation.end(), o_rRotation);
        }
      }
      return GetResult;
    }

    Result::Enum VRetimingClient::GetSegmentStaticRotationQuaternion(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rFourVector)[4]) const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);

      Result::Enum GetResult = Result::Success;
      if( !InitGet(GetResult, o_rFourVector) )
      {
        return GetResult;
      }

      std::shared_ptr< const VSubjectPose > pSubject;
      GetResult = GetSubject(i_rSubjectName, pSubject);

      if( GetResult == Result::Success )
      {
        // Get the segment
        auto SegIt = pSubject->m_Segments.find(i_rSegmentName);
        if( SegIt != pSubject->m_Segments.end() )
        {
          std::copy(SegIt->second->R_Stat.begin(), SegIt->second->R_Stat.end(), o_rFourVector);
        }
      }

      return GetResult;
    }

    Result::Enum VRetimingClient::GetSegmentStaticRotationEulerXYZ(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rThreeVector)[3]) const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);

      Clear(o_rThreeVector);
      // InitGet is called by GetSegmentStaticRotationMatrix.

      // Get the answer as a rotation matrix
      double RotationArray[9];
      const Result::Enum _Result = GetSegmentStaticRotationMatrix(i_rSubjectName, i_rSegmentName, RotationArray);

      if( Result::Success == _Result)
      {
        MatrixToEulerXYZ(RotationArray, o_rThreeVector);
      }

      return _Result;
    }


    Result::Enum VRetimingClient::GetSegmentGlobalTranslation(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rThreeVector)[3], bool& o_rbOccluded) const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);
      
      Result::Enum GetResult = Result::Success;
      if (!InitGet(GetResult, o_rThreeVector, o_rbOccluded))
      {
        return GetResult;
      }

      std::shared_ptr< const VSubjectPose > pSubject;
      GetResult = GetSubject(i_rSubjectName, pSubject );

      if (GetResult == Result::Success)
      {
        // Get the segment
        auto SegIt = pSubject->m_Segments.find(i_rSegmentName);
        if (SegIt != pSubject->m_Segments.end() )
        {
          std::copy(SegIt->second->T.begin(), SegIt->second->T.end(), o_rThreeVector);
          o_rbOccluded = SegIt->second->bOccluded;
        }
      }
      return GetResult;
    }

    Result::Enum VRetimingClient::GetSegmentGlobalRotationHelical(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rThreeVector)[3], bool& o_rbOccluded) const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);

      Clear(o_rThreeVector);
      Clear(o_rbOccluded);
      // InitGet is called by GetSegmentGlobalRotationMatrix.

      // Get the answer as a rotation matrix
      double RotationArray[9];
      const Result::Enum _Result = GetSegmentGlobalRotationMatrix(i_rSubjectName, i_rSegmentName, RotationArray, o_rbOccluded);

      if (Result::Success == _Result && !o_rbOccluded)
      {
        MatrixToHelical(RotationArray, o_rThreeVector);
      }

      return _Result;
    }

    Result::Enum VRetimingClient::GetSegmentGlobalRotationMatrix(const std::string & i_rSubjectName, const std::string & i_rSegmentName, double(&o_rRotation)[9], bool & o_rbOccluded) const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);
      
      Result::Enum GetResult = Result::Success;
      if (!InitGet(GetResult, o_rRotation, o_rbOccluded))
      {
        return GetResult;
      }

      std::shared_ptr< const VSubjectPose > pSubject;
      GetResult = GetSubject(i_rSubjectName, pSubject);

      if (GetResult == Result::Success)
      {
        // Get the segment
        auto SegIt = pSubject->m_Segments.find(i_rSegmentName);
        if (SegIt != pSubject->m_Segments.end())
        {
          boost::array< double, 9 > GlobalRotation = ClientUtils::ToRotationMatrix(SegIt->second->R);
          std::copy(GlobalRotation.begin(), GlobalRotation.end(), o_rRotation);
          o_rbOccluded = SegIt->second->bOccluded;
        }
      }
      return GetResult;
    }

    Result::Enum VRetimingClient::GetSegmentGlobalRotationQuaternion(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rFourVector)[4], bool& o_rbOccluded) const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);
      
      Result::Enum GetResult = Result::Success;
      if (!InitGet(GetResult, o_rFourVector, o_rbOccluded))
      {
        return GetResult;
      }

      std::shared_ptr< const VSubjectPose > pSubject;
      GetResult = GetSubject(i_rSubjectName, pSubject);

      if (GetResult == Result::Success)
      {
        // Get the segment
        auto SegIt = pSubject->m_Segments.find(i_rSegmentName);
        if (SegIt != pSubject->m_Segments.end())
        {
          std::copy(SegIt->second->R.begin(), SegIt->second->R.end(), o_rFourVector);
          o_rbOccluded = SegIt->second->bOccluded;
        }
      }
      return GetResult;
    }

    Result::Enum VRetimingClient::GetSegmentGlobalRotationEulerXYZ(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rThreeVector)[3], bool& o_rbOccluded) const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);

      Clear(o_rThreeVector);
      Clear(o_rbOccluded);
      // InitGet is called by GetSegmentGlobalRotationMatrix.

      // Get the answer as a rotation matrix
      double RotationArray[9];
      const Result::Enum _Result = GetSegmentGlobalRotationMatrix(i_rSubjectName, i_rSegmentName, RotationArray, o_rbOccluded);

      if (Result::Success == _Result && !o_rbOccluded)
      {
        MatrixToEulerXYZ(RotationArray, o_rThreeVector);
      }

      return _Result;
    }

    Result::Enum VRetimingClient::GetSegmentLocalTranslation(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rThreeVector)[3], bool& o_rbOccluded) const
    {
      Result::Enum GetResult = Result::Success;
      if (!InitGet(GetResult, o_rThreeVector, o_rbOccluded))
      {
        return GetResult;
      }

      std::shared_ptr< const VSubjectPose > pSubject;
      GetResult = GetSubject(i_rSubjectName, pSubject);

      if (GetResult == Result::Success)
      {
        // Get the segment
        auto SegIt = pSubject->m_Segments.find(i_rSegmentName);
        if (SegIt != pSubject->m_Segments.end())
        {
          std::copy(SegIt->second->T_Rel.begin(), SegIt->second->T_Rel.end(), o_rThreeVector);
          o_rbOccluded = SegIt->second->bOccluded;
        }
      }
      return GetResult;
    }

    Result::Enum VRetimingClient::GetSegmentLocalRotationHelical(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rThreeVector)[3], bool& o_rbOccluded) const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);

      Clear(o_rThreeVector);
      Clear(o_rbOccluded);

      // Get the answer as a rotation matrix
      double RotationArray[9];
      const Result::Enum _Result = GetSegmentLocalRotationMatrix(i_rSubjectName, i_rSegmentName, RotationArray, o_rbOccluded);

      if (Result::Success == _Result && !o_rbOccluded)
      {
        MatrixToHelical(RotationArray, o_rThreeVector);
      }

      return _Result;
    }

    Result::Enum VRetimingClient::GetSegmentLocalRotationMatrix(const std::string & i_rSubjectName, const std::string & i_rSegmentName, double(&o_rRotation)[9], bool & o_rbOccluded) const
    {
      Result::Enum GetResult = Result::Success;
      if (!InitGet(GetResult, o_rRotation, o_rbOccluded))
      {
        return GetResult;
      }

      std::shared_ptr< const VSubjectPose > pSubject;
      GetResult = GetSubject(i_rSubjectName, pSubject);

      if (GetResult == Result::Success)
      {
        // Get the segment
        auto SegIt = pSubject->m_Segments.find(i_rSegmentName);
        if (SegIt != pSubject->m_Segments.end())
        {
          boost::array< double, 9 > LocalRotation = ClientUtils::ToRotationMatrix(SegIt->second->R_Rel);
          std::copy(LocalRotation.begin(), LocalRotation.end(), o_rRotation);
          o_rbOccluded = SegIt->second->bOccluded;
        }
      }
      return GetResult;
    }

    Result::Enum VRetimingClient::GetSegmentLocalRotationQuaternion(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rFourVector)[4], bool& o_rbOccluded) const
    {
      Result::Enum GetResult = Result::Success;
      if (!InitGet(GetResult, o_rFourVector, o_rbOccluded))
      {
        return GetResult;
      }

      std::shared_ptr< const VSubjectPose > pSubject;
      GetResult = GetSubject(i_rSubjectName, pSubject);

      if (GetResult == Result::Success)
      {
        // Get the segment
        auto SegIt = pSubject->m_Segments.find(i_rSegmentName);
        if (SegIt != pSubject->m_Segments.end())
        {
          std::copy(SegIt->second->R_Rel.begin(), SegIt->second->R_Rel.end(), o_rFourVector);
          o_rbOccluded = SegIt->second->bOccluded;
        }
      }
      return GetResult;
    }

    Result::Enum VRetimingClient::GetSegmentLocalRotationEulerXYZ(const std::string& i_rSubjectName, const std::string& i_rSegmentName, double(&o_rThreeVector)[3], bool& o_rbOccluded) const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);

      Clear(o_rThreeVector);
      Clear(o_rbOccluded);

      // Get the answer as a rotation matrix
      double RotationArray[9];
      const Result::Enum _Result = GetSegmentLocalRotationMatrix(i_rSubjectName, i_rSegmentName, RotationArray, o_rbOccluded);

      if (Result::Success == _Result && !o_rbOccluded)
      {
        MatrixToEulerXYZ(RotationArray, o_rThreeVector);
      }

      return _Result;
    }

    Result::Enum VRetimingClient::GetSubject(const std::string & i_rSubjectName, std::shared_ptr< const VSubjectPose > & o_rpSubject ) const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);
      
      Result::Enum OutputResult = Result::InvalidSubjectName;

      auto DataIt = m_LatestOutputPoses.find(i_rSubjectName);
      if (DataIt != m_LatestOutputPoses.end())
      {
        o_rpSubject = DataIt->second;
        if (!DataIt->second)
        {
          OutputResult = Result::NoFrame;
        }
        else
        {
          switch (DataIt->second->Result)
          {
          case Core::VSubjectPose::ESuccess:
            OutputResult = Result::Success;
            break;
          default:
          case Core::VSubjectPose::ENoData:
            OutputResult = Result::NoFrame;
            break;
          case Core::VSubjectPose::EUnknownSubject:
            OutputResult = Result::InvalidSubjectName;
            break;
          case Core::VSubjectPose::EEarly:
            OutputResult = Result::EarlyDataRequested;
            break;
          case Core::VSubjectPose::ELate:
            OutputResult = Result::LateDataRequested;
            break;
          }
        }
      }

      return OutputResult;
    }


    Result::Enum VRetimingClient::WaitForFrame() const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);

      if (!IsRunning())
      {
        return Result::NoFrame;
      }

      m_OutputWait.wait(Lock);
      return Result::Success;
    }

    void VRetimingClient::SetOutputLatency(double i_OutputLatency)
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);
      m_OutputLatency = i_OutputLatency;
    }

    double VRetimingClient::OutputLatency() const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);
      return m_OutputLatency;
    }

    void VRetimingClient::SetMaximumPrediction(double i_MaxPrediction)
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);
      m_MaxPredictionTime = i_MaxPrediction;
    }

    double VRetimingClient::MaximumPrediction() const
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);
      return m_MaxPredictionTime;
    }

    void VRetimingClient::AddData(const std::string & i_rObjectName, std::shared_ptr< VSubjectPose > i_pData)
    {
      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);

      // DSSDK-210. Don't add occluded data, as we will be able to interpolate from good data
      // as long as gaps are not too long. If an object becomes occluded for a significant time,
      // Predict will return ELate when the data becomes too stale.
      bool bOccluded = false;
      // For multi-segment subjects, don't add if any segments are occluded.
      for (const auto & rSegment : i_pData->m_Segments)
      {
        if (rSegment.second->bOccluded)
        {
          bOccluded = true;
          //break;
        }
      }
      
      if (bOccluded)
      {
        return;
      }

      auto & rData = m_Data[i_rObjectName];

      // Stamp the receipt time as current time now less the system latency
      i_pData->ReceiptTime = static_cast< double >(m_Timer.elapsed().wall) / 1000000.0 - i_pData->Latency;
      rData.push_back(i_pData);

      while (rData.size() > s_BufSize)
      {
        rData.pop_front();
      }
    }


    Result::Enum VRetimingClient::UpdateFrame(double i_rOffset)
    {
      //const auto Before = m_Timer.elapsed().wall;
      if (IsRunning())
      {
        return Result::InvalidOperation;
      }

      boost::recursive_mutex::scoped_lock Lock(m_DataMutex);

      m_LatestOutputPoses.clear();

      Result::Enum Result = Result::Success;

      for (const auto & Pair : m_Data)
      {
        std::shared_ptr< const VSubjectPose > pPose;
        const auto & rData = Pair.second;
        if (rData.size() >= 2)
        {
          double Now = static_cast< double >(m_Timer.elapsed().wall) / 1000000.0;
          double PTime = Now + i_rOffset;
          pPose = Predict(rData.front(), rData.back(), PTime);
          if (pPose->Result != VSubjectPose::ESuccess)
          {
            Result = Adapt( pPose->Result );
          }
        }

        m_LatestOutputPoses[Pair.first] = pPose;
      }

      //const auto After = m_Timer.elapsed().wall;
      //std::cout << "Update Frame took " << (After - Before) << " ns" << std::endl;;
      return Result;
    }

    void VRetimingClient::InputThread()
    {

      while (!m_bInputStopped)
      {
        if (m_rClient.IsConnected())
        {
          // Get a frame
          while (m_rClient.GetFrame() != Result::Success && m_rClient.IsConnected())
          {
            // Sleep a little so that we don't lumber the CPU with a busy poll
#ifdef WIN32
            //Sleep(200);
#else
            sleep( 1 );
#endif
          }

          double Latency = 0;
          Latency += m_NetworkLatency; 

          double SystemLatency;
          if (m_rClient.GetLatencyTotal(SystemLatency) == Result::Success)
          {
            Latency += SystemLatency * 1000.0;
          }

          unsigned int FrameNumber;
          double FrameRateHz;
          Result::Enum FrameNumberResult = m_rClient.GetFrameNumber(FrameNumber);
          Result::Enum FrameRateResult = m_rClient.GetFrameRate(FrameRateHz);

          if (FrameNumberResult == Result::Success && FrameRateResult == Result::Success)
          {
            // Get a lock here so that we add all subjects from one frame together.
            boost::recursive_mutex::scoped_lock Lock(m_DataMutex);

            double FrameReceiptTime = (1.0 / FrameRateHz) * static_cast<double>(FrameNumber);

            // Count the number of subjects
            unsigned int SubjectCount;
            m_rClient.GetSubjectCount(SubjectCount);
            for (unsigned int SubjectIndex = 0; SubjectIndex < SubjectCount; ++SubjectIndex)
            {

              std::shared_ptr< VSubjectPose > pPoseData( new VSubjectPose() );

              pPoseData->FrameTime = FrameReceiptTime;
              pPoseData->Latency = Latency;

              // Get the subject name
              std::string SubjectName;
              m_rClient.GetSubjectName(SubjectIndex, SubjectName);

              // Get the root segment
              std::string RootSegment;
              m_rClient.GetSubjectRootSegmentName(SubjectName, RootSegment);

              pPoseData->RootSegment = RootSegment;

              // Count the number of segments
              unsigned int SegmentCount;
              m_rClient.GetSegmentCount(SubjectName, SegmentCount);
              for (unsigned int SegmentIndex = 0; SegmentIndex < SegmentCount; ++SegmentIndex)
              {
                std::string SegmentName;
                m_rClient.GetSegmentName(SubjectName, SegmentIndex, SegmentName);

                std::shared_ptr< VSegmentPose > pSegmentPoseData( new VSegmentPose() );
                pSegmentPoseData->Name = SegmentName;

                // Get our parent name (if we have one)
                std::string ParentName;
                if (m_rClient.GetSegmentParentName(SubjectName, SegmentName, ParentName) == Result::Success)
                {
                  pSegmentPoseData->Parent = ParentName;
                }

                // Add some information about the children of this segment
                unsigned int ChildSegmentCount;
                if( m_rClient.GetSegmentChildCount(SubjectName, SegmentName, ChildSegmentCount) == Result::Success )
                {
                  for( unsigned int ChildSegmentIndex = 0; ChildSegmentIndex < ChildSegmentCount; ++ChildSegmentIndex )
                  {
                    std::string ChildSegmentName;
                    if( m_rClient.GetSegmentChildName(SubjectName, SegmentName, ChildSegmentIndex, ChildSegmentName) == Result::Success )
                    {
                      pSegmentPoseData->m_Children.push_back(ChildSegmentName);
                    }
                  }
                }

                // Get the global segment translation
                bool bOccluded = false;
                double Translation[3];
                m_rClient.GetSegmentGlobalTranslation(SubjectName, SegmentName, Translation, bOccluded);
                std::copy(Translation, Translation + 3, pSegmentPoseData->T.begin());

                // Get the global segment rotation in quaternion co-ordinates
                double Rotation[4];
                m_rClient.GetSegmentGlobalRotationQuaternion(SubjectName, SegmentName, Rotation, bOccluded);
                std::copy(Rotation, Rotation + 4, pSegmentPoseData->R.begin());

                // Get local translation
                double LocalTranslation[3];
                m_rClient.GetSegmentLocalTranslation(SubjectName, SegmentName, LocalTranslation, bOccluded);
                std::copy(LocalTranslation, LocalTranslation + 3, pSegmentPoseData->T_Rel.begin());

                // And local rotation
                double LocalRotation[4];
                m_rClient.GetSegmentLocalRotationQuaternion(SubjectName, SegmentName, LocalRotation, bOccluded);
                std::copy(LocalRotation, LocalRotation + 4, pSegmentPoseData->R_Rel.begin());

                // Get static translation
                double StaticTranslation[3];
                m_rClient.GetSegmentStaticTranslation(SubjectName, SegmentName, StaticTranslation);
                std::copy(StaticTranslation, StaticTranslation + 3, pSegmentPoseData->T_Stat.begin());

                // And static rotation
                double StaticRotation[4];
                m_rClient.GetSegmentStaticRotationQuaternion(SubjectName, SegmentName, StaticRotation);
                std::copy(StaticRotation, StaticRotation + 4, pSegmentPoseData->R_Stat.begin());

                pSegmentPoseData->bOccluded = bOccluded;

                pPoseData->m_Segments[ SegmentName ] = pSegmentPoseData;
              }

              AddData(SubjectName, pPoseData);
            }
          }
        }
      }

      m_bInputStopped = false;

    }

    void VRetimingClient::OutputThread()
    {
      // Keep track of retimed output frame number
      uint64_t CurrentOutputFrame = 0;

      while (!m_bOutputStopped)
      {
        // Calculate time for this frame
        double ThisFrameTime;
        {
          boost::recursive_mutex::scoped_lock Lock(m_FrameRateMutex);
          ++m_OutputFrameNumber;
          ThisFrameTime = m_FrameRate * static_cast<double>(m_OutputFrameNumber);
        }

        {
          boost::recursive_mutex::scoped_lock Lock(m_DataMutex);

          // Interpolate output positions if required.
          double Now = static_cast< double >(m_Timer.elapsed().wall) / 1000000.0 - m_OutputLatency;

          for (auto Pair : m_Data)
          {
            if (Pair.second.size() >= 2)
            {
              m_LatestOutputPoses[Pair.first] = Predict(Pair.second.front(), Pair.second.back(), Now);
            }
          }

          // Output the frame 
          m_OutputWait.notify_all();
        }

        // Increment our output frame number
        ++CurrentOutputFrame;

        // Yield until next frame is required.
        //UtilsThreading::Sleep(static_cast<unsigned int>((1.0 / m_FrameRate) * 1000));
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<unsigned int>((1.0 / m_FrameRate) * 1000)));

      }

      m_bOutputStopped = false;
    }

    std::shared_ptr< const VSubjectPose > VRetimingClient::Predict(std::shared_ptr< const VSubjectPose > p1, std::shared_ptr< const VSubjectPose > p2, double t) const
    {
      std::shared_ptr< VSubjectPose > pOutput( new VSubjectPose() );

      if (p2->ReceiptTime < p1->ReceiptTime)
      {
        pOutput->Result = VSubjectPose::EInvalid;
        return pOutput;
      }

      if (t < p1->ReceiptTime)
      {
        pOutput->Result = VSubjectPose::EEarly;
        return pOutput;
      }

      if (t > p2->ReceiptTime)
      {
        if (t - p2->ReceiptTime > m_MaxPredictionTime)
        {
          pOutput->Result = VSubjectPose::ELate;
          return pOutput;
        }
      }

      pOutput->FrameTime = t;
      pOutput->ReceiptTime = t;
      pOutput->Result = VSubjectPose::ESuccess;
      pOutput->RootSegment = p1->RootSegment;
      pOutput->Name = p1->Name;

      for (const auto SegIt : p1->m_Segments)
      {
        // Get corresponding segment from p2->
        for (const auto SegIt2 : p2->m_Segments)
        {
          std::shared_ptr< VSegmentPose > pSegment = SegIt.second;
          std::shared_ptr< VSegmentPose > pSegment2 = SegIt2.second;

          if (pSegment2->Name == pSegment->Name)
          {

            std::shared_ptr< VSegmentPose > pOutputSegment( new VSegmentPose() );
            pOutputSegment->Name = pSegment->Name;
            pOutputSegment->Parent = pSegment->Parent;
            std::copy(pSegment->T_Stat.begin(), pSegment->T_Stat.end(), pOutputSegment->T_Stat.begin());
            std::copy(pSegment->R_Stat.begin(), pSegment->R_Stat.end(), pOutputSegment->R_Stat.begin());

            pOutputSegment->m_Children.reserve(pSegment->m_Children.size());
            std::copy(pSegment->m_Children.begin(), pSegment->m_Children.end(), std::back_inserter( pOutputSegment->m_Children ) );

            pOutputSegment->bOccluded = pSegment->bOccluded || pSegment2->bOccluded;

            pOutputSegment->T = ClientUtils::PredictDisplacement(pSegment->T, p1->ReceiptTime, pSegment2->T, p2->ReceiptTime, t);
            pOutputSegment->R = ClientUtils::PredictRotation(pSegment->R, p1->ReceiptTime, pSegment2->R, p2->ReceiptTime, t);
            pOutputSegment->T_Rel = ClientUtils::PredictDisplacement(pSegment->T_Rel, p1->ReceiptTime, pSegment2->T_Rel, p2->ReceiptTime, t);
            pOutputSegment->R_Rel = ClientUtils::PredictRotation(pSegment->R_Rel, p1->ReceiptTime, pSegment2->R_Rel, p2->ReceiptTime, t);

            pOutput->m_Segments[ pSegment->Name ] = pOutputSegment;

            break;
          }
        }
      }

      return pOutput;
    }
  }
}
