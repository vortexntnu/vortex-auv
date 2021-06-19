
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
#define _EXPORTING

#include "DataStreamRetimingClient.h"

#include <ViconDataStreamSDKCore/CoreClient.h>
#include <ViconDataStreamSDKCore/RetimingClient.h>

#include "StringFactory.h"
#include "CoreAdapters.h"

namespace ViconDataStreamSDK
{
  namespace CPP
  {

    // This class just wraps something whose identity we don't want to expose
    // in our header file
    class RetimingClientImpl
    {
    public:
      RetimingClientImpl()
        : m_pCoreClient(new ViconDataStreamSDK::Core::VClient())
        , m_pCoreRetimingClient(new ViconDataStreamSDK::Core::VRetimingClient(*m_pCoreClient))
        , m_pStringFactory(new VStringFactory())
      {
      }

      std::shared_ptr< ViconDataStreamSDK::Core::VClient > m_pCoreClient;
      std::shared_ptr< ViconDataStreamSDK::Core::VRetimingClient > m_pCoreRetimingClient;
      std::shared_ptr< VStringFactory                    > m_pStringFactory;
    };
    CLASS_DECLSPEC
      RetimingClient::RetimingClient()
      : m_pClientImpl(new RetimingClientImpl())
    {
    }

    CLASS_DECLSPEC
      RetimingClient::~RetimingClient()
    {
      delete m_pClientImpl;
      m_pClientImpl = 0;
    }

    CLASS_DECLSPEC
    Output_GetVersion RetimingClient::GetVersion() const
    {
      Output_GetVersion Output;
      m_pClientImpl->m_pCoreClient->GetVersion(Output.Major,
        Output.Minor,
        Output.Point);

      return Output;
    }

    CLASS_DECLSPEC
      Output_Connect RetimingClient::Connect(const String & HostName, double FrameRate)
    {

      std::shared_ptr< ViconCGStreamClientSDK::ICGClient > pClient(ViconCGStreamClientSDK::ICGClient::CreateCGClient(),
        boost::bind(&ViconCGStreamClientSDK::ICGClient::Destroy, _1));
      Output_Connect Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->Connect(pClient, HostName));

      if (Output.Result == Result::Success && FrameRate > 0)
      {
        Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->StartOutput(FrameRate));
      }

      return Output;
    }

    CLASS_DECLSPEC
      Output_Disconnect  RetimingClient::Disconnect()
    {
      Output_Disconnect Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->Disconnect());
      return Output;
    }

    CLASS_DECLSPEC
      Output_IsConnected RetimingClient::IsConnected() const
    {
      Output_IsConnected Output;
      Output.Connected = m_pClientImpl->m_pCoreClient->IsConnected();
      return Output;
    }


    CLASS_DECLSPEC
      Output_SetAxisMapping RetimingClient::SetAxisMapping(const Direction::Enum XAxis, const Direction::Enum YAxis, const Direction::Enum ZAxis)
    {
      Output_SetAxisMapping Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreClient->SetAxisMapping(Adapt(XAxis), Adapt(YAxis), Adapt(ZAxis)));
      return Output;
    }

    CLASS_DECLSPEC
      Output_GetAxisMapping RetimingClient::GetAxisMapping() const
    {
      ViconDataStreamSDK::Core::Direction::Enum _XAxis;
      ViconDataStreamSDK::Core::Direction::Enum _YAxis;
      ViconDataStreamSDK::Core::Direction::Enum _ZAxis;

      m_pClientImpl->m_pCoreClient->GetAxisMapping(_XAxis, _YAxis, _ZAxis);

      Output_GetAxisMapping Output;
      Output.XAxis = Adapt(_XAxis);
      Output.YAxis = Adapt(_YAxis);
      Output.ZAxis = Adapt(_ZAxis);

      return Output;
    }


    CLASS_DECLSPEC
      Output_UpdateFrame RetimingClient::UpdateFrame(double i_Offset)
    {
      Output_UpdateFrame Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->UpdateFrame(i_Offset));
      return Output;
    }

    CLASS_DECLSPEC
      Output_WaitForFrame RetimingClient::WaitForFrame()
    {
      Output_WaitForFrame Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->WaitForFrame());
      return Output;
    }

    CLASS_DECLSPEC
      Output_GetSubjectCount RetimingClient::GetSubjectCount() const
    {
      Output_GetSubjectCount Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSubjectCount(Output.SubjectCount));

      return Output;
    }

    CLASS_DECLSPEC
      Output_GetSubjectName RetimingClient::GetSubjectName(const unsigned int SubjectIndex) const
    {
      Output_GetSubjectName Output;

      std::string SubjectName;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSubjectName(SubjectIndex, SubjectName));
      Output.SubjectName.Set(SubjectName.c_str(), *m_pClientImpl->m_pStringFactory.get());

      return Output;
    }

    CLASS_DECLSPEC
      Output_GetSubjectRootSegmentName RetimingClient::GetSubjectRootSegmentName(const String & SubjectName) const
    {
      Output_GetSubjectRootSegmentName Output;
      std::string SegmentName;

      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSubjectRootSegmentName(SubjectName, SegmentName));

      if (Output.Result != Result::Success)
      {
        std::cout << "Get Root Segment Name failed " << Output.Result << std::endl;
      }

      Output.SegmentName.Set(SegmentName.c_str(), *m_pClientImpl->m_pStringFactory.get());
      return Output;
    }

    CLASS_DECLSPEC
      Output_GetSegmentCount RetimingClient::GetSegmentCount(const String  & SubjectName) const
    {
      Output_GetSegmentCount Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSegmentCount(SubjectName, Output.SegmentCount));

      return Output;
    }

    CLASS_DECLSPEC
      Output_GetSegmentName RetimingClient::GetSegmentName(const String       & SubjectName,
      const unsigned int   SegmentIndex) const
    {
      Output_GetSegmentName Output;
      std::string SegmentName;

      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSegmentName(SubjectName, SegmentIndex, SegmentName));

      Output.SegmentName.Set(SegmentName.c_str(), *m_pClientImpl->m_pStringFactory.get());
      return Output;
    }

    CLASS_DECLSPEC
      Output_GetSegmentChildCount RetimingClient::GetSegmentChildCount(const String & SubjectName,
      const String & SegmentName) const
    {
      Output_GetSegmentChildCount Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSegmentChildCount(SubjectName, SegmentName, Output.SegmentCount));

      return Output;
    }

    CLASS_DECLSPEC
      Output_GetSegmentChildName RetimingClient::GetSegmentChildName(const String       & SubjectName,
      const String       & SegmentName,
      const unsigned int   SegmentIndex) const
    {
      Output_GetSegmentChildName Output;
      std::string SegmentChildName;

      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSegmentChildName(SubjectName, SegmentName, SegmentIndex, SegmentChildName));

      Output.SegmentName.Set(SegmentChildName.c_str(), *m_pClientImpl->m_pStringFactory.get());
      return Output;
    }

    CLASS_DECLSPEC
      Output_GetSegmentParentName RetimingClient::GetSegmentParentName(const String & SubjectName,
      const String & SegmentName) const
    {
      Output_GetSegmentParentName Output;
      std::string SegmentParentName;

      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSegmentParentName(SubjectName, SegmentName, SegmentParentName));

      Output.SegmentName.Set(SegmentParentName.c_str(), *m_pClientImpl->m_pStringFactory.get());
      return Output;
    }

    CLASS_DECLSPEC
      Output_GetSegmentStaticTranslation RetimingClient::GetSegmentStaticTranslation(const String & SubjectName,
      const String & SegmentName) const
    {
      Output_GetSegmentStaticTranslation Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSegmentStaticTranslation(SubjectName,
        SegmentName,
        Output.Translation));

      return Output;
    }

    CLASS_DECLSPEC
      Output_GetSegmentStaticRotationHelical RetimingClient::GetSegmentStaticRotationHelical(const String & SubjectName,
      const String & SegmentName) const
    {
      Output_GetSegmentStaticRotationHelical Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSegmentStaticRotationHelical(SubjectName,
        SegmentName,
        Output.Rotation));

      return Output;
    }

    CLASS_DECLSPEC
      Output_GetSegmentStaticRotationMatrix RetimingClient::GetSegmentStaticRotationMatrix(const String & SubjectName,
      const String & SegmentName) const
    {
      Output_GetSegmentStaticRotationMatrix Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSegmentStaticRotationMatrix(SubjectName,
        SegmentName,
        Output.Rotation));

      return Output;
    }

    CLASS_DECLSPEC
      Output_GetSegmentStaticRotationQuaternion RetimingClient::GetSegmentStaticRotationQuaternion(const String & SubjectName,
      const String & SegmentName) const
    {
      Output_GetSegmentStaticRotationQuaternion Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSegmentStaticRotationQuaternion(SubjectName,
        SegmentName,
        Output.Rotation));

      return Output;
    }

    CLASS_DECLSPEC
    Output_GetSegmentStaticRotationEulerXYZ RetimingClient::GetSegmentStaticRotationEulerXYZ( const String & SubjectName,
                                                                                              const String & SegmentName) const
    {
      Output_GetSegmentStaticRotationEulerXYZ Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSegmentStaticRotationEulerXYZ(SubjectName,
        SegmentName,
        Output.Rotation));

      return Output;
    }

    CLASS_DECLSPEC
    Output_GetSegmentGlobalTranslation RetimingClient::GetSegmentGlobalTranslation( const String & SubjectName,
                                                                                    const String & SegmentName) const
    {
      Output_GetSegmentGlobalTranslation Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSegmentGlobalTranslation(SubjectName,
        SegmentName,
        Output.Translation,
        Output.Occluded)); 
      return Output;
    }

    CLASS_DECLSPEC
    Output_GetSegmentGlobalRotationHelical RetimingClient::GetSegmentGlobalRotationHelical( const String & SubjectName,
                                                                                            const String & SegmentName) const
    {
      Output_GetSegmentGlobalRotationHelical Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSegmentGlobalRotationHelical(SubjectName,
        SegmentName,
        Output.Rotation,
        Output.Occluded));
      return Output;
    }

    CLASS_DECLSPEC
      Output_GetSegmentGlobalRotationMatrix RetimingClient::GetSegmentGlobalRotationMatrix(const String & SubjectName,
      const String & SegmentName) const
    {
      Output_GetSegmentGlobalRotationMatrix Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSegmentGlobalRotationMatrix(SubjectName,
        SegmentName,
        Output.Rotation,
        Output.Occluded));
      return Output;
    }

    CLASS_DECLSPEC
    Output_GetSegmentGlobalRotationQuaternion RetimingClient::GetSegmentGlobalRotationQuaternion( const String & SubjectName,
                                                                                                  const String & SegmentName) const
    {
      Output_GetSegmentGlobalRotationQuaternion Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSegmentGlobalRotationQuaternion(SubjectName,
        SegmentName,
        Output.Rotation,
        Output.Occluded));
      return Output;
    }

    CLASS_DECLSPEC
    Output_GetSegmentGlobalRotationEulerXYZ RetimingClient::GetSegmentGlobalRotationEulerXYZ( const String & SubjectName,
                                                                                              const String & SegmentName) const
    {
      Output_GetSegmentGlobalRotationEulerXYZ Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSegmentGlobalRotationEulerXYZ(SubjectName,
        SegmentName,
        Output.Rotation,
        Output.Occluded));
      return Output;
    }

    CLASS_DECLSPEC
    Output_GetSegmentLocalTranslation RetimingClient::GetSegmentLocalTranslation( const String & SubjectName,
                                                                                  const String & SegmentName) const
    {
      Output_GetSegmentLocalTranslation Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSegmentLocalTranslation(SubjectName,
        SegmentName,
        Output.Translation,
        Output.Occluded));
      return Output;
    }

    CLASS_DECLSPEC
    Output_GetSegmentLocalRotationHelical RetimingClient::GetSegmentLocalRotationHelical( const String & SubjectName,
                                                                                          const String & SegmentName) const
    {
      Output_GetSegmentLocalRotationHelical Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSegmentLocalRotationHelical(SubjectName,
        SegmentName,
        Output.Rotation,
        Output.Occluded));
      return Output;
    }

    CLASS_DECLSPEC
    Output_GetSegmentLocalRotationMatrix RetimingClient::GetSegmentLocalRotationMatrix( const String & SubjectName,
                                                                                        const String & SegmentName) const
    {
      Output_GetSegmentLocalRotationMatrix Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSegmentLocalRotationMatrix(SubjectName,
        SegmentName,
        Output.Rotation,
        Output.Occluded));
      return Output;
    }

    CLASS_DECLSPEC
    Output_GetSegmentLocalRotationQuaternion RetimingClient::GetSegmentLocalRotationQuaternion( const String & SubjectName,
                                                                                                const String & SegmentName) const
    {
      Output_GetSegmentLocalRotationQuaternion Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSegmentLocalRotationQuaternion(SubjectName,
        SegmentName,
        Output.Rotation,
        Output.Occluded));
      return Output;
    }

    CLASS_DECLSPEC
    Output_GetSegmentLocalRotationEulerXYZ RetimingClient::GetSegmentLocalRotationEulerXYZ( const String & SubjectName,
                                                                                            const String & SegmentName) const
    {
      Output_GetSegmentLocalRotationEulerXYZ Output;
      Output.Result = Adapt(m_pClientImpl->m_pCoreRetimingClient->GetSegmentLocalRotationEulerXYZ(SubjectName,
        SegmentName,
        Output.Rotation,
        Output.Occluded));
      return Output;
    }

    
    CLASS_DECLSPEC
    void RetimingClient::SetOutputLatency( double i_OutputLatency )
    {
      m_pClientImpl->m_pCoreRetimingClient->SetOutputLatency(i_OutputLatency);
    }

    CLASS_DECLSPEC
    double RetimingClient::OutputLatency() const
    {
      return m_pClientImpl->m_pCoreRetimingClient->OutputLatency();
    }

    CLASS_DECLSPEC
    void RetimingClient::SetMaximumPrediction( double i_MaxPrediction )
    {
      m_pClientImpl->m_pCoreRetimingClient->SetMaximumPrediction(i_MaxPrediction);
    }

    CLASS_DECLSPEC
    double RetimingClient::MaximumPrediction() const
    {
      return m_pClientImpl->m_pCoreRetimingClient->MaximumPrediction();
    }
  }
  }
