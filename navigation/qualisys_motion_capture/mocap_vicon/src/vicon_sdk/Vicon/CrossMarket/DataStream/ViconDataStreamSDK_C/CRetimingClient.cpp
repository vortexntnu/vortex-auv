
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
#include "CRetimingClient.h"
#include <cstring>

#include "CRetimingClient.h"
#include <ViconDataStreamSDK_CPP/DataStreamClient.h>
#include <ViconDataStreamSDK_CPP/DataStreamRetimingClient.h>

using namespace ViconDataStreamSDK::CPP;


#ifdef _WIN32
#define snprintf(str,size,format,arg) _snprintf_s(str,size,_TRUNCATE,format,arg)
#endif

CRetimingClient* RetimingClient_Create(void)
{
  RetimingClient* retimer = new RetimingClient();
  return retimer;
}

void RetimingClient_Destroy( CRetimingClient* client )
{
  if( client )
  {
    delete ( (CRetimingClient*)client );
  }
}

void RetimingClient_GetVersion(CRetimingClient* client, COutput_GetVersion* outptr)
{
  const Output_GetVersion& outp = ((RetimingClient*)client)->GetVersion();
  outptr->Major = outp.Major; outptr->Minor = outp.Minor; outptr->Point = outp.Point;
}

CEnum RetimingClient_Connect(CRetimingClient* client, CString HostName)
{
  return ((RetimingClient*)client)->Connect(String(HostName)).Result;
}

CEnum RetimingClient_ConnectAndStart(CRetimingClient* client, CString HostName, double FrameRate )
{
  return ((RetimingClient*)client)->Connect(String(HostName), FrameRate ).Result;
}

CEnum RetimingClient_Disconnect(CRetimingClient* client)
{
  return ((RetimingClient*)client)->Disconnect().Result;
}

CBool RetimingClient_IsConnected(CRetimingClient* client)
{
  return ((RetimingClient*)client)->IsConnected().Connected;
}

CEnum RetimingClient_SetAxisMapping(CRetimingClient* client, CEnum XAxis, CEnum YAxis, CEnum ZAxis)
{
  return ((RetimingClient*)client)->SetAxisMapping((Direction::Enum) XAxis, (Direction::Enum) YAxis,
    (Direction::Enum) ZAxis).Result;
}

void RetimingClient_GetAxisMapping(CRetimingClient* client, COutput_GetAxisMapping* outptr)
{
  const Output_GetAxisMapping& outp = ((RetimingClient*)client)->GetAxisMapping();

  outptr->XAxis = outp.XAxis;
  outptr->YAxis = outp.YAxis;
  outptr->ZAxis = outp.ZAxis;
}

CEnum RetimingClient_UpdateFrame(CRetimingClient* client )
{
  return ((RetimingClient*)client)->UpdateFrame( 0.0 ).Result;
}

CEnum RetimingClient_UpdateFrameOffset(CRetimingClient* client, double i_Offset)
{
  return ((RetimingClient*)client)->UpdateFrame(i_Offset).Result;
}

CEnum RetimingClient_WaitForFrame(CRetimingClient* client)
{
  return ((RetimingClient*)client)->WaitForFrame().Result;
}

void RetimingClient_GetSubjectCount(CRetimingClient* client, COutput_GetSubjectCount* outptr)
{
  const Output_GetSubjectCount& outp = ((RetimingClient*)client)->GetSubjectCount();

  outptr->Result = outp.Result;
  outptr->SubjectCount = outp.SubjectCount;
}

CEnum RetimingClient_GetSubjectName(CRetimingClient* client, unsigned int SubjectIndex, int sizeOfBuffer, char* outstr)
{
  const Output_GetSubjectName& outp = ((RetimingClient*)client)->GetSubjectName(SubjectIndex);

  snprintf(outstr, sizeOfBuffer, "%s", std::string(outp.SubjectName).c_str());
  return outp.Result;
}

CEnum RetimingClient_GetSubjectRootSegmentName(CRetimingClient* client, CString SubjectName, int sizeOfBuffer, char* outstr)
{
  const Output_GetSubjectRootSegmentName& outp = ((RetimingClient*)client)->GetSubjectRootSegmentName(String(SubjectName));

  snprintf(outstr, sizeOfBuffer, "%s", std::string(outp.SegmentName).c_str());
  return outp.Result;
}

void RetimingClient_GetSegmentCount(CRetimingClient* client, CString SubjectName, COutput_GetSegmentCount* outptr)
{
  const Output_GetSegmentCount& outp = ((RetimingClient*)client)->GetSegmentCount(String(SubjectName));

  outptr->Result = outp.Result;
  outptr->SegmentCount = outp.SegmentCount;
}

CEnum RetimingClient_GetSegmentName(CRetimingClient* client, CString SubjectName,
  unsigned int   SegmentIndex, int sizeOfBuffer, char* outstr)
{
  const Output_GetSegmentName& outp = ((RetimingClient*)client)->GetSegmentName(String(SubjectName), SegmentIndex);

  snprintf(outstr, sizeOfBuffer, "%s", std::string(outp.SegmentName).c_str());
  return outp.Result;
}

void RetimingClient_GetSegmentChildCount(CRetimingClient* client, CString SubjectName,
  CString SegmentName, COutput_GetSegmentChildCount* outptr)
{
  const Output_GetSegmentChildCount& outp = ((RetimingClient*)client)->GetSegmentChildCount(String(SubjectName),
    String(SegmentName));

  outptr->Result = outp.Result;
  outptr->SegmentCount = outp.SegmentCount;
}

CEnum RetimingClient_GetSegmentChildName(CRetimingClient* client, CString SubjectName,
  CString SegmentName,
  unsigned int   SegmentIndex,
  int sizeOfBuffer, char* outstr)
{
  const Output_GetSegmentChildName& outp = ((RetimingClient*)client)->GetSegmentChildName(String(SubjectName),
    String(SegmentName), SegmentIndex);

  snprintf(outstr, sizeOfBuffer, "%s", std::string(outp.SegmentName).c_str());
  return outp.Result;
}

CEnum RetimingClient_GetSegmentParentName(CRetimingClient* client, CString SubjectName,
  CString SegmentName, int sizeOfBuffer, char* outstr)
{
  const Output_GetSegmentParentName& outp = ((RetimingClient*)client)->GetSegmentParentName(String(SubjectName),
    String(SegmentName));

  snprintf(outstr, sizeOfBuffer, "%s", std::string(outp.SegmentName).c_str());
  return outp.Result;
}

void RetimingClient_GetSegmentStaticTranslation(CRetimingClient* client, CString SubjectName,
  CString SegmentName, COutput_GetSegmentStaticTranslation* outptr)
{
  const Output_GetSegmentStaticTranslation& outp = ((RetimingClient*)client)->GetSegmentStaticTranslation(String(SubjectName),
    String(SegmentName));

  outptr->Result = outp.Result;
  std::memcpy(outptr->Translation, outp.Translation, sizeof(outptr->Translation));
}

void RetimingClient_GetSegmentStaticRotationHelical(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentStaticRotationHelical* outptr)
{
  const Output_GetSegmentStaticRotationHelical& outp = ((RetimingClient*)client)->GetSegmentStaticRotationHelical(
    String(SubjectName), String(SegmentName));

  outptr->Result = outp.Result;
  std::memcpy(outptr->Rotation, outp.Rotation, sizeof(outptr->Rotation));
}

void RetimingClient_GetSegmentStaticRotationMatrix(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentStaticRotationMatrix* outptr)
{
  const Output_GetSegmentStaticRotationMatrix& outp = ((RetimingClient*)client)->GetSegmentStaticRotationMatrix(
    String(SubjectName), String(SegmentName));

  outptr->Result = outp.Result;
  std::memcpy(outptr->Rotation, outp.Rotation, sizeof(outptr->Rotation));
}

void RetimingClient_GetSegmentStaticRotationQuaternion(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentStaticRotationQuaternion* outptr)
{
  const Output_GetSegmentStaticRotationQuaternion& outp = ((RetimingClient*)client)->GetSegmentStaticRotationQuaternion(
    String(SubjectName), String(SegmentName));

  outptr->Result = outp.Result;
  std::memcpy(outptr->Rotation, outp.Rotation, sizeof(outptr->Rotation));
}

void RetimingClient_GetSegmentStaticRotationEulerXYZ(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentStaticRotationEulerXYZ* outptr)
{
  const Output_GetSegmentStaticRotationEulerXYZ& outp = ((RetimingClient*)client)->GetSegmentStaticRotationEulerXYZ(
    String(SubjectName), String(SegmentName));

  outptr->Result = outp.Result;
  std::memcpy(outptr->Rotation, outp.Rotation, sizeof(outptr->Rotation));
}

void RetimingClient_GetSegmentGlobalTranslation(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentGlobalTranslation* outptr)
{
  const Output_GetSegmentGlobalTranslation& outp = ((RetimingClient*)client)->GetSegmentGlobalTranslation(
    String(SubjectName), String(SegmentName));

  outptr->Result = outp.Result;
  std::memcpy(outptr->Translation, outp.Translation, sizeof(outptr->Translation));
  outptr->Occluded = outp.Occluded;
}

void RetimingClient_GetSegmentGlobalRotationHelical(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentGlobalRotationHelical* outptr)
{
  const Output_GetSegmentGlobalRotationHelical& outp = ((RetimingClient*)client)->GetSegmentGlobalRotationHelical(
    String(SubjectName), String(SegmentName));

  outptr->Result = outp.Result;
  std::memcpy(outptr->Rotation, outp.Rotation, sizeof(outptr->Rotation));
  outptr->Occluded = outp.Occluded;
}

void RetimingClient_GetSegmentGlobalRotationMatrix(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentGlobalRotationMatrix* outptr)
{
  const Output_GetSegmentGlobalRotationMatrix& outp = ((RetimingClient*)client)->GetSegmentGlobalRotationMatrix(
    String(SubjectName), String(SegmentName));

  outptr->Result = outp.Result;
  std::memcpy(outptr->Rotation, outp.Rotation, sizeof(outptr->Rotation));
  outptr->Occluded = outp.Occluded;
}

void RetimingClient_GetSegmentGlobalRotationQuaternion(CRetimingClient* client, CString  SubjectName, CString  SegmentName,
  COutput_GetSegmentGlobalRotationQuaternion* outptr)
{
  const Output_GetSegmentGlobalRotationQuaternion& outp = ((RetimingClient*)client)->GetSegmentGlobalRotationQuaternion(
    String(SubjectName), String(SegmentName));

  outptr->Result = outp.Result;

  std::memcpy(outptr->Rotation, outp.Rotation, sizeof(outptr->Rotation));

  outptr->Occluded = outp.Occluded;
}


void RetimingClient_GetSegmentGlobalRotationEulerXYZ(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentGlobalRotationEulerXYZ* outptr)
{
  const Output_GetSegmentGlobalRotationEulerXYZ& outp = ((RetimingClient*)client)->GetSegmentGlobalRotationEulerXYZ(
    String(SubjectName), String(SegmentName));

  outptr->Result = outp.Result;
  std::memcpy(outptr->Rotation, outp.Rotation, sizeof(outptr->Rotation));
  outptr->Occluded = outp.Occluded;
}

void RetimingClient_GetSegmentLocalTranslation(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentLocalTranslation* outptr)
{
  const Output_GetSegmentLocalTranslation& outp = ((RetimingClient*)client)->GetSegmentLocalTranslation(
    String(SubjectName), String(SegmentName));

  outptr->Result = outp.Result;
  std::memcpy(outptr->Translation, outp.Translation, sizeof(outptr->Translation));
  outptr->Occluded = outp.Occluded;
}

void RetimingClient_GetSegmentLocalRotationHelical(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentLocalRotationHelical* outptr)
{
  const Output_GetSegmentLocalRotationHelical& outp = ((RetimingClient*)client)->GetSegmentLocalRotationHelical(
    String(SubjectName), String(SegmentName));

  outptr->Result = outp.Result;
  std::memcpy(outptr->Rotation, outp.Rotation, sizeof(outptr->Rotation));
  outptr->Occluded = outp.Occluded;
}

void RetimingClient_GetSegmentLocalRotationMatrix(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentLocalRotationMatrix* outptr)
{
  const Output_GetSegmentLocalRotationMatrix& outp = ((RetimingClient*)client)->GetSegmentLocalRotationMatrix(
    String(SubjectName), String(SegmentName));

  outptr->Result = outp.Result;
  std::memcpy(outptr->Rotation, outp.Rotation, sizeof(outptr->Rotation));
  outptr->Occluded = outp.Occluded;
}

void RetimingClient_GetSegmentLocalRotationQuaternion(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentLocalRotationQuaternion* outptr)
{
  const Output_GetSegmentLocalRotationQuaternion& outp = ((RetimingClient*)client)->GetSegmentLocalRotationQuaternion(
    String(SubjectName), String(SegmentName));

  outptr->Result = outp.Result;
  std::memcpy(outptr->Rotation, outp.Rotation, sizeof(outptr->Rotation));
  outptr->Occluded = outp.Occluded;
}

void RetimingClient_GetSegmentLocalRotationEulerXYZ(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentLocalRotationEulerXYZ* outptr)
{
  const Output_GetSegmentLocalRotationEulerXYZ& outp = ((RetimingClient*)client)->GetSegmentLocalRotationEulerXYZ(
    String(SubjectName), String(SegmentName));

  outptr->Result = outp.Result;
  std::memcpy(outptr->Rotation, outp.Rotation, sizeof(outptr->Rotation));
  outptr->Occluded = outp.Occluded;
}


void RetimingClient_SetOutputLatency( CRetimingClient* client, CReal i_OutputLatency )
{
  ((RetimingClient*)client)->SetOutputLatency( i_OutputLatency );
}

CReal RetimingClient_OutputLatency( CRetimingClient* client )
{
  return ((RetimingClient*)client)->OutputLatency();
}

void RetimingClient_SetMaximumPrediction( CRetimingClient* client, CReal i_MaxPrediction )
{
  ((RetimingClient*)client)->SetMaximumPrediction( i_MaxPrediction );
}

CReal RetimingClient_MaximumPrediction( CRetimingClient* client )
{
  return ((RetimingClient*)client)->MaximumPrediction();
}
