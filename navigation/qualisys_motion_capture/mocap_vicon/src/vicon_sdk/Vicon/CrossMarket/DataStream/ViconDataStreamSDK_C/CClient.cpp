
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
#include "CClient.h"
#include <ViconDataStreamSDK_CPP/DataStreamClient.h>
#include <cstring>

using namespace ViconDataStreamSDK::CPP;


#ifdef _WIN32
#define snprintf(str,size,format,arg) _snprintf_s(str,size,_TRUNCATE,format,arg)
#endif

CClient* Client_Create()
{
  Client* client = new Client();
  return client;
}

void Client_Destroy(CClient* client)
{
  if(client)
  {
    delete ((Client*) client);
  }
}

void Client_GetVersion(CClient* client, COutput_GetVersion* outptr)
{
  const Output_GetVersion& outp = ((Client*) client)->GetVersion();
  /*COutput_GetVersion c_outp = {outp.Major,outp.Minor,outp.Point};
  return c_outp;*/
  outptr->Major=outp.Major; outptr->Minor=outp.Minor; outptr->Point=outp.Point; 
}

CEnum Client_Connect(CClient* client, CString HostName )
{
  //COutput_Connect outp = {((Client*) client)->Connect(String(HostName)).Result};
  //return outp;
  return ((Client*) client)->Connect(String(HostName)).Result;
}

CEnum Client_ConnectToMulticast(CClient* client, CString LocalIP, CString MulticastIP )
{
  return ((Client*) client)->ConnectToMulticast(String(LocalIP), String(MulticastIP)).Result;
}

CEnum Client_Disconnect(CClient* client)
{
  return ((Client*) client)->Disconnect().Result;
}

CBool Client_IsConnected(CClient* client)
{
  //COutput_IsConnected outp = { ((Client*) client)->IsConnected().Connected};
  //return outp;
  //*((CBool*)outptr) = ((Client*) client)->IsConnected().Connected;
  return ((Client*) client)->IsConnected().Connected;
}

CEnum Client_StartTransmittingMulticast(CClient* client, CString ServerIP,
                                                      CString MulticastIP )
{
  return ((Client*) client)->StartTransmittingMulticast(String(ServerIP), String(MulticastIP)).Result;
}

CEnum Client_StopTransmittingMulticast(CClient* client)
{
  return ((Client*) client)->StopTransmittingMulticast().Result;
}

CEnum Client_EnableSegmentData(CClient* client)
{
  return ((Client*) client)->EnableSegmentData().Result;
}

CEnum Client_EnableMarkerData(CClient* client)
{
  return ((Client*) client)->EnableMarkerData().Result;
}

CEnum Client_EnableUnlabeledMarkerData(CClient* client)
{
  return ((Client*) client)->EnableUnlabeledMarkerData().Result;
}
CEnum Client_EnableDeviceData(CClient* client)
{
  return ((Client*) client)->EnableDeviceData().Result;
}

CEnum Client_DisableSegmentData(CClient* client)
{
  return ((Client*) client)->DisableSegmentData().Result;
}

CEnum Client_DisableMarkerData(CClient* client)
{
  return ((Client*) client)->DisableMarkerData().Result;
}

CEnum Client_DisableUnlabeledMarkerData(CClient* client)
{
  return ((Client*) client)->DisableUnlabeledMarkerData().Result;
}

CEnum Client_DisableDeviceData(CClient* client)
{
  return ((Client*) client)->DisableDeviceData().Result;
}

CBool Client_IsSegmentDataEnabled(CClient* client)
{
  return ((Client*) client)->IsSegmentDataEnabled().Enabled;
}

CBool Client_IsMarkerDataEnabled(CClient* client)
{
  return ((Client*) client)->IsMarkerDataEnabled().Enabled;
}

CBool Client_IsUnlabeledMarkerDataEnabled(CClient* client)
{
  return ((Client*) client)->IsUnlabeledMarkerDataEnabled().Enabled;
}

CBool Client_IsDeviceDataEnabled(CClient* client)
{
  return ((Client*) client)->IsDeviceDataEnabled().Enabled;
}

CEnum Client_SetStreamMode(CClient* client, CEnum Mode )
{
  return ((Client*) client)->SetStreamMode((StreamMode::Enum) Mode).Result;
}

CEnum Client_SetApexDeviceFeedback(CClient* client, CString i_rDeviceName, CBool i_bOn )
{
  return ((Client*) client)->SetApexDeviceFeedback(String(i_rDeviceName), i_bOn!=0).Result;
}

CEnum Client_SetAxisMapping(CClient* client, CEnum XAxis, CEnum YAxis, CEnum ZAxis )
{
  return ((Client*) client)->SetAxisMapping((Direction::Enum) XAxis, (Direction::Enum) YAxis,
                        (Direction::Enum) ZAxis).Result;
}
void Client_GetAxisMapping(CClient* client, COutput_GetAxisMapping* outptr)
{
  const Output_GetAxisMapping& outp = ((Client*) client)->GetAxisMapping();
  /*COutput_GetAxisMapping c_outp = {outp.XAxis,outp.YAxis,outp.ZAxis};
  return c_outp;*/
  outptr->XAxis = outp.XAxis;
  outptr->YAxis = outp.YAxis;
  outptr->ZAxis = outp.ZAxis;
}

CEnum Client_GetFrame(CClient* client)
{
  return ((Client*) client)->GetFrame().Result;
}

void Client_GetFrameNumber(CClient* client, COutput_GetFrameNumber* outptr)
{
  const Output_GetFrameNumber& outp = ((Client*) client)->GetFrameNumber();
  /*COutput_GetFrameNumber c_outp = {outp.Result, outp.FrameNumber};
  return c_outp;*/
  outptr->Result = outp.Result;
  outptr->FrameNumber = outp.FrameNumber;
}

void Client_GetTimecode(CClient* client, COutput_GetTimecode* outptr)
{
  const Output_GetTimecode& outp = ((Client*) client)->GetTimecode();
  /*COutput_GetTimecode c_outp = {outp.Result,outp.Hours,outp.Minutes,outp.Seconds,
                 outp.Frames, outp.SubFrame, outp.FieldFlag, outp.Standard,
                 outp.SubFramesPerFrame, outp.UserBits};
  return c_outp;*/
  outptr->Result=outp.Result; outptr->Hours=outp.Hours;
  outptr->Minutes=outp.Minutes; outptr->Seconds=outp.Seconds;
  outptr->Frames=outp.Frames; outptr->SubFrame=outp.SubFrame;
  outptr->FieldFlag=outp.FieldFlag; outptr->Standard=outp.Standard;
  outptr->SubFramesPerFrame=outp.SubFramesPerFrame; outptr->UserBits=outp.UserBits; 
}

void Client_GetFrameRate(CClient* client, COutput_GetFrameRate* outptr)
{
  const Output_GetFrameRate& outp = ((Client*) client)->GetFrameRate(); 
  /*COutput_GetFrameRate c_outp = {outp.Result,outp.FrameRateHz};
  return c_outp;*/
  outptr->Result = outp.Result;
  outptr->FrameRateHz = outp.FrameRateHz;
}

void Client_GetLatencySampleCount(CClient* client, COutput_GetLatencySampleCount* outptr)
{
  const Output_GetLatencySampleCount& outp = ((Client*) client)->GetLatencySampleCount();
  /*COutput_GetLatencySampleCount c_outp = {outp.Result,outp.Count};
  return c_outp;*/
  outptr->Result = outp.Result;
  outptr->Count = outp.Count;
}
CEnum  Client_GetLatencySampleName(CClient* client, unsigned int LatencySampleIndex, int sizeOfBuffer, char* outstr )
{
  const Output_GetLatencySampleName& outp = ((Client*) client)->GetLatencySampleName(LatencySampleIndex);
  /*COutput_GetLatencySampleName c_outp = {outp.Result,std::string(outp.Name).c_str()};
  return c_outp;*/
  /*outptr->Result = outp.Result;
  outptr->Name = std::string(outp.Name).c_str();*/
  snprintf(outstr, sizeOfBuffer, "%s",  std::string(outp.Name).c_str());
  return outp.Result;
}
void Client_GetLatencySampleValue(CClient* client, CString LatencySampleName, COutput_GetLatencySampleValue* outptr )
{
  const Output_GetLatencySampleValue& outp = ((Client*) client)->GetLatencySampleValue(String(LatencySampleName));
  /*COutput_GetLatencySampleValue c_outp = {outp.Result,outp.Value};
  return c_outp;*/
  outptr->Result = outp.Result;
  outptr->Value = outp.Value;
}
void  Client_GetLatencyTotal(CClient* client, COutput_GetLatencyTotal* outptr)
{
  const Output_GetLatencyTotal& outp = ((Client*) client)->GetLatencyTotal();
  /*COutput_GetLatencyTotal c_outp = {outp.Result,outp.Total};
  return c_outp;*/
  outptr->Result = outp.Result;
  outptr->Total = outp.Total;
}

void Client_GetSubjectCount(CClient* client, COutput_GetSubjectCount* outptr)
{
  const Output_GetSubjectCount& outp = ((Client*) client)->GetSubjectCount();
  /*COutput_GetSubjectCount c_outp = {outp.Result,outp.SubjectCount};
  return c_outp;*/
  outptr->Result = outp.Result;
  outptr->SubjectCount = outp.SubjectCount;
}

CEnum Client_GetSubjectName(CClient* client, unsigned int SubjectIndex, int sizeOfBuffer, char* outstr )
{
  const Output_GetSubjectName& outp = ((Client*) client)->GetSubjectName(SubjectIndex);
  /*COutput_GetSubjectName c_outp = {outp.Result, std::string(outp.SubjectName).c_str()};
  return c_outp;*/
  /*outptr->Result = outp.Result; 
  outptr->SubjectName = std::string(outp.SubjectName).c_str();*/

  /*static char dummy[] = "dummy";
  outptr->SubjectName = dummy;*/
  snprintf(outstr, sizeOfBuffer, "%s", std::string(outp.SubjectName).c_str());
  return outp.Result;
}

CEnum Client_GetSubjectRootSegmentName(CClient* client, CString SubjectName, int sizeOfBuffer, char* outstr )
{
  const Output_GetSubjectRootSegmentName& outp = ((Client*) client)->GetSubjectRootSegmentName(String(SubjectName));
  /*COutput_GetSubjectRootSegmentName c_outp = {outp.Result, std::string(outp.SegmentName).c_str()};
  return c_outp;*/
  /*outptr->Result = outp.Result;
  outptr->SegmentName = std::string(outp.SegmentName).c_str();*/
  snprintf(outstr, sizeOfBuffer, "%s", std::string(outp.SegmentName).c_str()); 
  return outp.Result;
}

void Client_GetSegmentCount( CClient* client, CString SubjectName, COutput_GetSegmentCount* outptr )
{
  const Output_GetSegmentCount& outp = ((Client*) client)->GetSegmentCount(String(SubjectName));
  /*COutput_GetSegmentCount c_outp = {outp.Result,outp.SegmentCount};
  return c_outp;*/
  outptr->Result = outp.Result;
  outptr->SegmentCount = outp.SegmentCount;
}

CEnum Client_GetSegmentName(CClient* client, CString SubjectName,
                unsigned int   SegmentIndex, int sizeOfBuffer, char* outstr )
{
  const Output_GetSegmentName& outp = ((Client*) client)->GetSegmentName(String(SubjectName),SegmentIndex);
  /*COutput_GetSegmentName c_outp = {outp.Result, std::string(outp.SegmentName).c_str()};
  return c_outp;  */
  /*outptr->Result = outp.Result;
  outptr->SegmentName = std::string(outp.SegmentName).c_str();*/
  snprintf(outstr, sizeOfBuffer, "%s", std::string(outp.SegmentName).c_str());
  return outp.Result;
}

void Client_GetSegmentChildCount(CClient* client, CString SubjectName,
                                          CString SegmentName, COutput_GetSegmentChildCount* outptr )
{
  const Output_GetSegmentChildCount& outp = ((Client*) client)->GetSegmentChildCount(String(SubjectName),
                              String(SegmentName));
  /*COutput_GetSegmentChildCount c_outp = {outp.Result,outp.SegmentCount};
  return c_outp;*/
  outptr->Result = outp.Result;
  outptr->SegmentCount = outp.SegmentCount;
}

CEnum Client_GetSegmentChildName(CClient* client, CString SubjectName,
                                        CString SegmentName,
                                        unsigned int   SegmentIndex,
                    int sizeOfBuffer, char* outstr)
{
  const Output_GetSegmentChildName& outp = ((Client*) client)->GetSegmentChildName(String(SubjectName),
                          String(SegmentName), SegmentIndex);
  /*COutput_GetSegmentChildName c_outp = {outp.Result, std::string(outp.SegmentName).c_str()};
  return c_outp;*/
  /*outptr->Result = outp.Result;
  outptr->SegmentName = std::string(outp.SegmentName).c_str();*/
  snprintf(outstr, sizeOfBuffer, "%s", std::string(outp.SegmentName).c_str());
  return outp.Result;
}

CEnum Client_GetSegmentParentName(CClient* client, CString SubjectName,
                                 CString SegmentName, int sizeOfBuffer, char* outstr )
{
  const Output_GetSegmentParentName& outp = ((Client*) client)->GetSegmentParentName(String(SubjectName),
                              String(SegmentName));
  /*COutput_GetSegmentParentName c_outp = {outp.Result,std::string(outp.SegmentName).c_str()};
  return c_outp;*/
  /*outptr->Result = outp.Result;
  outptr->SegmentName = std::string(outp.SegmentName).c_str();*/
  snprintf(outstr, sizeOfBuffer, "%s", std::string(outp.SegmentName).c_str());
  return outp.Result;
}

void Client_GetSegmentStaticTranslation(CClient* client, CString SubjectName,
                                        CString SegmentName, COutput_GetSegmentStaticTranslation* outptr )
{
  const Output_GetSegmentStaticTranslation& outp = ((Client*) client)->GetSegmentStaticTranslation(String(SubjectName),
                                  String(SegmentName));
  /*double* ptr = outp.Translation;
  COutput_GetSegmentStaticTranslation c_outp = {outp.Result, *ptr++,*ptr++,*ptr};
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->Translation,outp.Translation,sizeof(outptr->Translation));
}

void Client_GetSegmentStaticRotationHelical(CClient* client, CString  SubjectName,
                                            CString  SegmentName, COutput_GetSegmentStaticRotationHelical* outptr )
{
  const Output_GetSegmentStaticRotationHelical& outp = ((Client*) client)->GetSegmentStaticRotationHelical(
                          String(SubjectName), String(SegmentName));
  /*double* ptr = outp.Rotation;
  COutput_GetSegmentStaticRotationHelical c_outp = {outp.Result, *ptr++,*ptr++,*ptr};
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->Rotation,outp.Rotation,sizeof(outptr->Rotation));
}

void Client_GetSegmentStaticRotationMatrix(CClient* client, CString  SubjectName,
                                           CString  SegmentName, COutput_GetSegmentStaticRotationMatrix* outptr )
{
  const Output_GetSegmentStaticRotationMatrix& outp = ((Client*) client)->GetSegmentStaticRotationMatrix(
                          String(SubjectName), String(SegmentName));
  /*double* ptr = outp.Rotation;
  COutput_GetSegmentStaticRotationMatrix c_outp = {outp.Result,*ptr++,*ptr++,*ptr++,
                                  *ptr++,*ptr++,*ptr++
                                  *ptr++,*ptr++,*ptr};*/
  /*COutput_GetSegmentStaticRotationMatrix c_outp;
  c_outp.Result = outp.Result;
  std::memcpy(c_outp.Rotation,outp.Rotation,sizeof(c_outp.Rotation));
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->Rotation,outp.Rotation,sizeof(outptr->Rotation));
}

void Client_GetSegmentStaticRotationQuaternion(CClient* client, CString  SubjectName,
                                               CString  SegmentName, COutput_GetSegmentStaticRotationQuaternion* outptr )
{
  const Output_GetSegmentStaticRotationQuaternion& outp = ((Client*) client)->GetSegmentStaticRotationQuaternion(
                             String(SubjectName), String(SegmentName));
  /*double* ptr = outp.Rotation;
  COutput_GetSegmentStaticRotationQuaternion c_outp = {outp.Result,*ptr++,*ptr++,*ptr++,*ptr};
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->Rotation,outp.Rotation,sizeof(outptr->Rotation));
}

void Client_GetSegmentStaticRotationEulerXYZ(CClient* client, CString  SubjectName,
                                             CString  SegmentName, COutput_GetSegmentStaticRotationEulerXYZ* outptr )
{
  const Output_GetSegmentStaticRotationEulerXYZ& outp = ((Client*) client)->GetSegmentStaticRotationEulerXYZ(
                            String(SubjectName), String(SegmentName));
  /*double* ptr = outp.Rotation;
  COutput_GetSegmentStaticRotationEulerXYZ c_outp = {outp.Result,*ptr++,*ptr++,*ptr};
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->Rotation,outp.Rotation,sizeof(outptr->Rotation));
}

void Client_GetSegmentGlobalTranslation(CClient* client, CString  SubjectName,
                                        CString  SegmentName, COutput_GetSegmentGlobalTranslation* outptr )
{
  const Output_GetSegmentGlobalTranslation& outp = ((Client*) client)->GetSegmentGlobalTranslation(
                          String(SubjectName), String(SegmentName));
  /*double* ptr = outp.Translation;
  COutput_GetSegmentGlobalTranslation c_outp = {outp.Result,*ptr++,*ptr++,*ptr,outp.Occluded};
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->Translation,outp.Translation,sizeof(outptr->Translation));
  outptr->Occluded = outp.Occluded;
}

void Client_GetSegmentGlobalRotationHelical(CClient* client, CString  SubjectName,
                                            CString  SegmentName, COutput_GetSegmentGlobalRotationHelical* outptr )
{
  const Output_GetSegmentGlobalRotationHelical& outp = ((Client*) client)->GetSegmentGlobalRotationHelical(
                           String(SubjectName), String(SegmentName));
  /*double* ptr = outp.Rotation;
  COutput_GetSegmentGlobalRotationHelical c_outp = {outp.Result,*ptr++,*ptr++,*ptr,outp.Occluded};
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->Rotation,outp.Rotation,sizeof(outptr->Rotation));
  outptr->Occluded = outp.Occluded;
}

void Client_GetSegmentGlobalRotationMatrix(CClient* client, CString  SubjectName,
                                           CString  SegmentName, COutput_GetSegmentGlobalRotationMatrix* outptr )
{
  const Output_GetSegmentGlobalRotationMatrix& outp = ((Client*) client)->GetSegmentGlobalRotationMatrix(
                          String(SubjectName), String(SegmentName));
  /*COutput_GetSegmentGlobalRotationMatrix c_outp;
  c_outp.Result = outp.Result;
  std::memcpy(c_outp.Rotation, outp.Rotation, sizeof(c_outp.Rotation));
  c_outp.Occluded = outp.Occluded;
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->Rotation,outp.Rotation,sizeof(outptr->Rotation));
  outptr->Occluded = outp.Occluded;
}

void Client_GetSegmentGlobalRotationQuaternion(CClient* client, CString  SubjectName, CString  SegmentName, 
                         COutput_GetSegmentGlobalRotationQuaternion* outptr)
{
  const Output_GetSegmentGlobalRotationQuaternion& outp = ((Client*) client)->GetSegmentGlobalRotationQuaternion(
                             String(SubjectName), String(SegmentName));
  /*double* ptr = outp.Rotation;
  COutput_GetSegmentGlobalRotationQuaternion c_outp = {outp.Result,*ptr++,*ptr++,*ptr++,*ptr,outp.Occluded};
  return c_outp;*/
  outptr->Result = outp.Result;
  //outp.Rotation[0]=1.0; outp.Rotation[1]=2.0;outp.Rotation[2]=3.0;outp.Rotation[3]=4.0;
  std::memcpy(outptr->Rotation,outp.Rotation,sizeof(outptr->Rotation));
  //double* from = outp.Rotation, *to = outptr->Rotation;
  //*to++=*from++;*to++=*from++;*to++=*from++;*to=*from;
  outptr->Occluded = outp.Occluded;
}


void Client_GetSegmentGlobalRotationEulerXYZ(CClient* client, CString  SubjectName,
                                             CString  SegmentName, COutput_GetSegmentGlobalRotationEulerXYZ* outptr )
{
  const Output_GetSegmentGlobalRotationEulerXYZ& outp = ((Client*) client)->GetSegmentGlobalRotationEulerXYZ(
                            String(SubjectName), String(SegmentName));
  /*double* ptr = outp.Rotation;
  COutput_GetSegmentGlobalRotationEulerXYZ c_outp = {outp.Result,*ptr++,*ptr++,*ptr,outp.Occluded};
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->Rotation,outp.Rotation,sizeof(outptr->Rotation));
  outptr->Occluded = outp.Occluded;
}

void Client_GetSegmentLocalTranslation(CClient* client, CString  SubjectName,
                                       CString  SegmentName, COutput_GetSegmentLocalTranslation* outptr )
{
  const Output_GetSegmentLocalTranslation& outp = ((Client*) client)->GetSegmentLocalTranslation(
                         String(SubjectName), String(SegmentName));
  /*double* ptr = outp.Translation;
  COutput_GetSegmentLocalTranslation c_outp = {outp.Result,*ptr++,*ptr++,*ptr,outp.Occluded};
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->Translation,outp.Translation,sizeof(outptr->Translation));
  outptr->Occluded = outp.Occluded;
}

void Client_GetSegmentLocalRotationHelical(CClient* client, CString  SubjectName,
                                           CString  SegmentName, COutput_GetSegmentLocalRotationHelical* outptr )
{
  const Output_GetSegmentLocalRotationHelical& outp = ((Client*) client)->GetSegmentLocalRotationHelical(
                           String(SubjectName), String(SegmentName));
  /*double* ptr = outp.Rotation;
  COutput_GetSegmentLocalRotationHelical c_outp = {outp.Result,*ptr++,*ptr++,*ptr,outp.Occluded};
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->Rotation,outp.Rotation,sizeof(outptr->Rotation));
  outptr->Occluded = outp.Occluded;
}

void Client_GetSegmentLocalRotationMatrix(CClient* client, CString  SubjectName,
                                          CString  SegmentName, COutput_GetSegmentLocalRotationMatrix* outptr )
{
  const Output_GetSegmentLocalRotationMatrix& outp = ((Client*) client)->GetSegmentLocalRotationMatrix(
                          String(SubjectName), String(SegmentName));
  /*COutput_GetSegmentLocalRotationMatrix c_outp;
  c_outp.Result = outp.Result;
  std::memcpy(c_outp.Rotation,outp.Rotation,sizeof(c_outp.Rotation));
  c_outp.Occluded = outp.Occluded;
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->Rotation,outp.Rotation,sizeof(outptr->Rotation));
  outptr->Occluded = outp.Occluded;
}

void Client_GetSegmentLocalRotationQuaternion(CClient* client, CString  SubjectName,
                                              CString  SegmentName, COutput_GetSegmentLocalRotationQuaternion* outptr )
{
  const Output_GetSegmentLocalRotationQuaternion& outp = ((Client*) client)->GetSegmentLocalRotationQuaternion(
                             String(SubjectName), String(SegmentName));
  /*double* ptr = outp.Rotation;
  COutput_GetSegmentLocalRotationQuaternion c_outp = {outp.Result,*ptr++,*ptr++,*ptr++,*ptr,outp.Occluded};
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->Rotation,outp.Rotation,sizeof(outptr->Rotation));
  outptr->Occluded = outp.Occluded;
}

void Client_GetSegmentLocalRotationEulerXYZ(CClient* client, CString  SubjectName,
                                            CString  SegmentName, COutput_GetSegmentLocalRotationEulerXYZ* outptr )
{
  const Output_GetSegmentLocalRotationEulerXYZ& outp = ((Client*) client)->GetSegmentLocalRotationEulerXYZ(
                           String(SubjectName), String(SegmentName));
  /*double* ptr = outp.Rotation;
  COutput_GetSegmentLocalRotationEulerXYZ c_outp = {outp.Result,*ptr++,*ptr++,*ptr,outp.Occluded};
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->Rotation,outp.Rotation,sizeof(outptr->Rotation));
  outptr->Occluded = outp.Occluded;
}

void Client_GetMarkerCount(CClient* client, CString SubjectName, COutput_GetMarkerCount* outptr )
{
  const Output_GetMarkerCount& outp = ((Client*) client)->GetMarkerCount(String(SubjectName));
  /*COutput_GetMarkerCount c_outp = {outp.Result,outp.MarkerCount};
  return c_outp;*/
  outptr->Result = outp.Result;
  outptr->MarkerCount = outp.MarkerCount;
}

CEnum Client_GetMarkerName(CClient* client, CString  SubjectName,
                            unsigned int  MarkerIndex, int sizeOfBuffer, char* outstr )
{
  const Output_GetMarkerName& outp = ((Client*) client)->GetMarkerName(String(SubjectName),MarkerIndex);
  /*COutput_GetMarkerName c_outp = {outp.Result, std::string(outp.MarkerName).c_str()};
  return c_outp;*/
  /*outptr->Result = outp.Result;
  outptr->MarkerName = std::string(outp.MarkerName).c_str();*/
  snprintf(outstr, sizeOfBuffer, "%s", std::string(outp.MarkerName).c_str());
  return outp.Result;
}

CEnum Client_GetMarkerParentName(CClient* client, CString  SubjectName,
                                CString  MarkerName, int sizeOfBuffer, char* outstr )
{
  const Output_GetMarkerParentName& outp = ((Client*) client)->GetMarkerParentName(
                            String(SubjectName), String(MarkerName));
  /*COutput_GetMarkerParentName c_outp = {outp.Result,std::string(outp.SegmentName).c_str()};
  return c_outp;*/
  /*outptr->Result = outp.Result;
  outptr->SegmentName = std::string(outp.SegmentName).c_str();*/
  snprintf(outstr, sizeOfBuffer, "%s", std::string(outp.SegmentName).c_str());
  return outp.Result;
}

void Client_GetMarkerGlobalTranslation(CClient* client, CString  SubjectName,
                                       CString  MarkerName, COutput_GetMarkerGlobalTranslation* outptr )
{
  const Output_GetMarkerGlobalTranslation& outp = ((Client*) client)->GetMarkerGlobalTranslation(
                         String(SubjectName), String(MarkerName));
  /*double* ptr = outp.Translation;
  COutput_GetMarkerGlobalTranslation c_outp = {outp.Result,*ptr++,*ptr++,*ptr,outp.Occluded};
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->Translation,outp.Translation,sizeof(outptr->Translation));
  outptr->Occluded = outp.Occluded;
}

void Client_GetUnlabeledMarkerCount(CClient* client, COutput_GetUnlabeledMarkerCount* outptr)
{
  const Output_GetUnlabeledMarkerCount& outp = ((Client*) client)->GetUnlabeledMarkerCount();
  /*COutput_GetUnlabeledMarkerCount c_outp = {outp.Result, outp.MarkerCount};
  return c_outp;*/
  outptr->Result = outp.Result;
  outptr->MarkerCount = outp.MarkerCount;
}

void Client_GetUnlabeledMarkerGlobalTranslation(CClient* client, unsigned int MarkerIndex,
                         COutput_GetUnlabeledMarkerGlobalTranslation* outptr)
{
  const Output_GetUnlabeledMarkerGlobalTranslation& outp = ((Client*) client)->GetUnlabeledMarkerGlobalTranslation(MarkerIndex);
  /*double* ptr = outp.Translation;
  COutput_GetUnlabeledMarkerGlobalTranslation c_outp = {outp.Result,*ptr++,*ptr++,*ptr};
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->Translation,outp.Translation,sizeof(outptr->Translation));
}

void Client_GetDeviceCount(CClient* client, COutput_GetDeviceCount* outptr)
{
  const Output_GetDeviceCount& outp = ((Client*) client)->GetDeviceCount();
  /*COutput_GetDeviceCount c_outp = {outp.Result, outp.DeviceCount};
  return c_outp;*/
  outptr->Result = outp.Result;
  outptr->DeviceCount = outp.DeviceCount;
}

CEnum  Client_GetDeviceName(CClient* client, unsigned int DeviceIndex, 
              int sizeOfBuffer, char* outstr, CEnum* DeviceType )
{
  const Output_GetDeviceName& outp = ((Client*) client)->GetDeviceName(DeviceIndex);
  /*COutput_GetDeviceName c_outp = {outp.Result,std::string(outp.DeviceName).c_str(),outp.DeviceType};
  return c_outp;*/
  /*outptr->Result = outp.Result;
  outptr->DeviceName = std::string(outp.DeviceName).c_str();
  outptr->DeviceType = outp.DeviceType;*/
  snprintf(outstr, sizeOfBuffer, "%s", std::string(outp.DeviceName).c_str());
  *DeviceType = outp.DeviceType;
  return outp.Result;
}

void Client_GetDeviceOutputCount(CClient* client, CString DeviceName, COutput_GetDeviceOutputCount* outptr )
{
  const Output_GetDeviceOutputCount& outp = ((Client*) client)->GetDeviceOutputCount(String(DeviceName));
  /*COutput_GetDeviceOutputCount c_outp = {outp.Result,outp.DeviceOutputCount};
  return c_outp;*/
  outptr->Result = outp.Result;
  outptr->DeviceOutputCount = outp.DeviceOutputCount;
}

CEnum Client_GetDeviceOutputName(CClient* client, CString   DeviceName,
                         unsigned int   DeviceOutputIndex, int sizeOfBuffer, char* outstr, CEnum* DeviceOutputUnit )
{
  const Output_GetDeviceOutputName& outp = ((Client*) client)->GetDeviceOutputName(String(DeviceName),
                               DeviceOutputIndex);
  /*COutput_GetDeviceOutputName c_outp = {outp.Result,std::string(outp.DeviceOutputName).c_str(),outp.DeviceOutputUnit};
  return c_outp;*/
  /*outptr->Result = outp.Result;
  outptr->DeviceOutputName = std::string(outp.DeviceOutputName).c_str();
  outptr->DeviceOutputUnit = outp.DeviceOutputUnit;*/
  snprintf(outstr, sizeOfBuffer, "%s",  std::string(outp.DeviceOutputName).c_str());
  *DeviceOutputUnit = outp.DeviceOutputUnit;
  return outp.Result;
}

void Client_GetDeviceOutputValue(CClient* client, CString  DeviceName,
                                 CString  DeviceOutputName, COutput_GetDeviceOutputValue* outptr )
{
  const Output_GetDeviceOutputValue& outp = ((Client*) client)->GetDeviceOutputValue(String(DeviceName),
                              String(DeviceOutputName));
  /*COutput_GetDeviceOutputValue c_outp = {outp.Result,outp.Value,outp.Occluded};
  return c_outp;*/
  outptr->Result = outp.Result;
  outptr->Value = outp.Value;
  outptr->Occluded = outp.Occluded;
}

void Client_GetDeviceOutputSubsamples(CClient* client, CString  DeviceName,
                                      CString  DeviceOutputName, COutput_GetDeviceOutputSubsamples* outptr )
{
  const Output_GetDeviceOutputSubsamples& outp = ((Client*) client)->GetDeviceOutputSubsamples(
                        String(DeviceName), String(DeviceOutputName));
  /*COutput_GetDeviceOutputSubsamples c_outp = {outp.Result,outp.DeviceOutputSubsamples,outp.Occluded};
  return c_outp;*/
  outptr->Result = outp.Result;
  outptr->DeviceOutputSubsamples = outp.DeviceOutputSubsamples;
  outptr->Occluded = outp.Occluded;
}

void Client_GetDeviceOutputValueForSubsample(CClient* client, CString  DeviceName,
                                         CString  DeviceOutputName,
                                          unsigned int Subsample,
                      COutput_GetDeviceOutputValue* outptr)
{
  const Output_GetDeviceOutputValue& outp = ((Client*) client)->GetDeviceOutputValue(
                      String(DeviceName), String(DeviceOutputName),Subsample);
  /*COutput_GetDeviceOutputValue c_outp = {outp.Result,outp.Value,outp.Occluded};
  return c_outp;*/
  outptr->Result = outp.Result;
  outptr->Value = outp.Value;
  outptr->Occluded = outp.Occluded;

}

void Client_GetForcePlateCount(CClient* client, COutput_GetForcePlateCount* outptr)
{
  const Output_GetForcePlateCount& outp = ((Client*) client)->GetForcePlateCount();
  /*COutput_GetForcePlateCount c_outp = {outp.Result,outp.ForcePlateCount};
  return c_outp;*/
  outptr->Result = outp.Result;
  outptr->ForcePlateCount = outp.ForcePlateCount;
}

void Client_GetGlobalForceVector(CClient* client,  unsigned int ForcePlateIndex, COutput_GetGlobalForceVector* outptr )
{
  const Output_GetGlobalForceVector& outp = ((Client*) client)->GetGlobalForceVector(ForcePlateIndex);
  /*double* ptr = outp.ForceVector;
  COutput_GetGlobalForceVector c_outp = {outp.Result,*ptr++,*ptr++,*ptr};
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->ForceVector,outp.ForceVector,sizeof(outptr->ForceVector));
}

void Client_GetGlobalMomentVector(CClient* client,  unsigned int ForcePlateIndex, COutput_GetGlobalMomentVector* outptr )
{
  const Output_GetGlobalMomentVector& outp = ((Client*) client)->GetGlobalMomentVector(ForcePlateIndex);
  /*double* ptr = outp.MomentVector;
  COutput_GetGlobalMomentVector c_outp = {outp.Result,*ptr++,*ptr++,*ptr};
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->MomentVector,outp.MomentVector,sizeof(outptr->MomentVector));
}

void Client_GetGlobalCentreOfPressure(CClient* client,  unsigned int ForcePlateIndex, COutput_GetGlobalCentreOfPressure* outptr )
{
  const Output_GetGlobalCentreOfPressure& outp = ((Client*) client)->GetGlobalCentreOfPressure(ForcePlateIndex);
  /*double* ptr = outp.CentreOfPressure;
  COutput_GetGlobalCentreOfPressure c_outp = {outp.Result,*ptr++,*ptr++,*ptr};
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->CentreOfPressure,outp.CentreOfPressure,sizeof(outptr->CentreOfPressure));
}

void Client_GetForcePlateSubsamples(CClient* client,  unsigned int ForcePlateIndex, COutput_GetForcePlateSubsamples* outptr )
{
  const Output_GetForcePlateSubsamples& outp = ((Client*) client)->GetForcePlateSubsamples(ForcePlateIndex);
  /*COutput_GetForcePlateSubsamples c_outp = {outp.Result, outp.ForcePlateSubsamples};
  return c_outp;*/
  outptr->Result = outp.Result;
  outptr->ForcePlateSubsamples = outp.ForcePlateSubsamples;
}

void Client_GetGlobalForceVectorForSubsample(CClient* client, unsigned int ForcePlateIndex, unsigned int Subsample,
                      COutput_GetGlobalForceVector* outptr)
{
  const Output_GetGlobalForceVector& outp = ((Client*) client)->GetGlobalForceVector(ForcePlateIndex,Subsample);
  /*double* ptr = outp.ForceVector;
  COutput_GetGlobalForceVector c_outp = {outp.Result,*ptr++,*ptr++,*ptr};
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->ForceVector,outp.ForceVector,sizeof(outptr->ForceVector));
}

void Client_GetGlobalMomentVectorForSubsample(CClient* client, unsigned int ForcePlateIndex, unsigned int Subsample,
                        COutput_GetGlobalMomentVector* outptr)
{
  const Output_GetGlobalMomentVector& outp = ((Client*) client)->GetGlobalMomentVector(ForcePlateIndex,Subsample);
  /*double* ptr = outp.MomentVector;
  COutput_GetGlobalMomentVector c_outp = {outp.Result,*ptr++,*ptr++,*ptr};
  return c_outp;*/ 
  outptr->Result = outp.Result;
  std::memcpy(outptr->MomentVector,outp.MomentVector,sizeof(outptr->MomentVector));
}

void Client_GetGlobalCentreOfPressureForSubsample(CClient* client,  unsigned int ForcePlateIndex, unsigned int Subsample,
                          COutput_GetGlobalCentreOfPressure* outptr)
{
  const Output_GetGlobalCentreOfPressure& outp = ((Client*) client)->GetGlobalCentreOfPressure(ForcePlateIndex,Subsample);
  /*double* ptr = outp.CentreOfPressure;
  COutput_GetGlobalCentreOfPressure c_outp = {outp.Result,*ptr++,*ptr++,*ptr};
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->CentreOfPressure,outp.CentreOfPressure,sizeof(outptr->CentreOfPressure));
}

void Client_GetEyeTrackerCount(CClient* client, COutput_GetEyeTrackerCount* outptr)
{
  const Output_GetEyeTrackerCount& outp = ((Client*) client)->GetEyeTrackerCount();
  /*COutput_GetEyeTrackerCount c_outp = {outp.Result,outp.EyeTrackerCount};
  return c_outp;*/
  outptr->Result = outp.Result;
  outptr->EyeTrackerCount = outp.EyeTrackerCount;
}

void Client_GetEyeTrackerGlobalPosition(CClient* client,  unsigned int EyeTrackerIndex,
                    COutput_GetEyeTrackerGlobalPosition* outptr)
{
  const Output_GetEyeTrackerGlobalPosition& outp = ((Client*) client)->GetEyeTrackerGlobalPosition(EyeTrackerIndex);
  /*double* ptr = outp.Position;
  COutput_GetEyeTrackerGlobalPosition c_outp = {outp.Result,*ptr++,*ptr++,*ptr,outp.Occluded};
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->Position,outp.Position,sizeof(outptr->Position));
  outptr->Occluded = outp.Occluded;
}

void Client_GetEyeTrackerGlobalGazeVector(CClient* client,  unsigned int EyeTrackerIndex,
                      COutput_GetEyeTrackerGlobalGazeVector* outptr)
{
  const Output_GetEyeTrackerGlobalGazeVector& outp = ((Client*) client)->GetEyeTrackerGlobalGazeVector(EyeTrackerIndex);
  /*double* ptr = outp.GazeVector;
  COutput_GetEyeTrackerGlobalGazeVector c_outp = {outp.Result,*ptr++,*ptr++,*ptr,outp.Occluded};
  return c_outp;*/
  outptr->Result = outp.Result;
  std::memcpy(outptr->GazeVector,outp.GazeVector,sizeof(outptr->GazeVector));
  outptr->Occluded = outp.Occluded;
}

CEnum Client_EnableMarkerRayData( CClient* client )
{
  return ((Client*) client)->EnableMarkerRayData().Result;
}

CEnum Client_EnableCentroidData( CClient* client )
{
  return ( (Client*)client )->EnableCentroidData().Result;
}

CEnum Client_EnableGreyscaleData( CClient* client )
{
  return NotImplemented;
}

CEnum Client_EnableDebugData( CClient* client )
{
  return ( (Client*)client )->EnableDebugData().Result;
}

CEnum Client_DisableMarkerRayData( CClient* client )
{
  return ( (Client*)client )->DisableMarkerRayData().Result;
}

CEnum Client_DisableCentroidData( CClient* client )
{
  return ( (Client*)client )->DisableCentroidData().Result;
}

CEnum Client_DisableGreyscaleData( CClient* client )
{
  return NotImplemented;
}

CEnum Client_DisableDebugData( CClient* client )
{
  return ( (Client*)client )->DisableDebugData().Result;
}

CBool Client_IsMarkerRayDataEnabled( CClient* client )
{
  return ((Client*) client)->IsMarkerRayDataEnabled().Enabled;
}

CBool Client_IsCentroidDataEnabled( CClient* client )
{
  return ( (Client*)client )->IsCentroidDataEnabled().Enabled;
}

CBool Client_IsGreyscaleDataEnabled( CClient* client )
{
  return false;
}

CBool Client_IsDebugDataEnabled( CClient* client )
{
  return ( (Client*)client )->IsDebugDataEnabled().Enabled;
}

void Client_GetServerOrientation( CClient* client, COutput_GetServerOrientation* outptr )
{
  Output_GetServerOrientation outpt = ((Client*) client)->GetServerOrientation();
  outptr->Orientation = outpt.Orientation;
  outptr->Result = outpt.Result;
}

void Client_GetHardwareFrameNumber( CClient* client, COutput_GetHardwareFrameNumber* outptr )
{
  Output_GetHardwareFrameNumber outpt = ((Client*) client)->GetHardwareFrameNumber();
  outptr->HardwareFrameNumber = outpt.HardwareFrameNumber;
  outptr->Result = outpt.Result;
}

void Client_GetFrameRateCount( CClient* client, COutput_GetFrameRateCount* outptr )
{
  Output_GetFrameRateCount outpt = ((Client*) client)->GetFrameRateCount();
  outptr->Count = outpt.Count;
  outptr->Result = outpt.Result;
}

CEnum Client_GetFrameRateName( CClient* client, unsigned int FrameRateIndex, int sizeOfBuffer, char* outstr )
{
  Output_GetFrameRateName outpt = ((Client*) client)->GetFrameRateName( FrameRateIndex );
  snprintf( outstr, sizeOfBuffer, "%s", std::string( outpt.Name ).c_str() );
  return outpt.Result;
}

void Client_GetFrameRateValue( CClient* client, CString FrameRateName, COutput_GetFrameRateValue* outptr )
{
  Output_GetFrameRateValue outpt = ( (Client*)client )->GetFrameRateValue( (String)FrameRateName ); 
  outptr->Result = outpt.Result;
  outptr->Value = outpt.Value;
}

void Client_GetObjectQuality( CClient* client, CString ObjectName, COutput_GetObjectQuality* outptr )
{
  Output_GetObjectQuality outpt = ((Client*) client)->GetObjectQuality( (String)ObjectName) ;
  outptr->Result = outpt.Result;
  outptr->Quality = outpt.Quality;
}

void Client_GetMarkerRayContributionCount( CClient* client, CString SubjectName, CString MarkerName, COutput_GetMarkerRayContributionCount* outptr )
{
  Output_GetMarkerRayContributionCount outpt = ( (Client*)client )->GetMarkerRayContributionCount( (String)SubjectName, (String)MarkerName );
  outptr->Result = outpt.Result;
  outptr->RayContributionsCount = outpt.RayContributionsCount;
}

void Client_GetMarkerRayContribution( CClient* client, CString SubjectName, CString MarkerName, unsigned int MarkerRayContributionIndex, COutput_GetMarkerRayContribution* outptr )
{
  Output_GetMarkerRayContribution outpt = ( (Client*)client )->GetMarkerRayContribution( (String)SubjectName, (String)MarkerName, MarkerRayContributionIndex );
  outptr->Result = outpt.Result;
  outptr->CentroidIndex = outpt.CentroidIndex;
  outptr->CameraID = outpt.CameraID;
}

void Client_GetLabeledMarkerCount( CClient* client, COutput_GetLabeledMarkerCount* outptr )
{
  Output_GetLabeledMarkerCount outpt = ((Client*) client)->GetLabeledMarkerCount();
  outptr->Result = outpt.Result;
  outptr->MarkerCount = outpt.MarkerCount;
}

void Client_GetLabeledMarkerGlobalTranslation( CClient* client, unsigned int MarkerIndex, COutput_GetLabeledMarkerGlobalTranslation* outptr )
{
  Output_GetLabeledMarkerGlobalTranslation outpt = ((Client*) client)->GetLabeledMarkerGlobalTranslation( MarkerIndex );
  outptr->Result = outpt.Result;
  std::memcpy( outptr->Translation, outpt.Translation, sizeof( outptr->Translation ) );
}

void Client_GetCameraCount( CClient* client, COutput_GetCameraCount* outptr )
{
  Output_GetCameraCount outpt = ((Client*) client)->GetCameraCount();
  outptr->Result = outpt.Result;
  outptr->CameraCount = outpt.CameraCount;
}

CEnum Client_GetCameraName( CClient* client, unsigned int i_CameraIndex, int sizeOfBuffer, char* outstr )
{
  Output_GetCameraName outpt = ((Client*) client)->GetCameraName( i_CameraIndex );
  snprintf( outstr, sizeOfBuffer, "%s", std::string( outpt.CameraName ).c_str() );
  return outpt.Result;
}

void Client_GetCameraId( CClient* client, CString i_rCameraName, COutput_GetCameraId* outptr )
{
  Output_GetCameraId outpt = ((Client*) client)->GetCameraId( i_rCameraName );
  outptr->Result = outpt.Result;
  outptr->CameraId = outpt.CameraId;
}

void Client_GetCameraUserId( CClient* client, CString i_rCameraName, COutput_GetCameraUserId* outptr )
{
  Output_GetCameraUserId outpt = ((Client*) client)->GetCameraUserId( (String)i_rCameraName );
  outptr->Result = outpt.Result;
  outptr->CameraUserId = outpt.CameraUserId;
}

CEnum Client_GetCameraType( CClient* client, CString i_rCameraName, int sizeOfBuffer, char* outstr )
{
  Output_GetCameraType outpt = ((Client*) client)->GetCameraType( (String)i_rCameraName );
  snprintf( outstr, sizeOfBuffer, "%s", std::string( outpt.CameraType ).c_str() );
  return outpt.Result;
}

CEnum Client_GetCameraDisplayName( CClient* client, CString i_rCameraName, int sizeOfBuffer, char* outstr )
{
  Output_GetCameraDisplayName outpt = ((Client*) client)->GetCameraDisplayName( i_rCameraName );
  snprintf( outstr, sizeOfBuffer, "%s", std::string( outpt.CameraDisplayName ).c_str() );
  return outpt.Result;
}

void Client_GetCameraResolution( CClient* client, CString i_rCameraName, COutput_GetCameraResolution* outptr )
{
  Output_GetCameraResolution outpt = ((Client*) client)->GetCameraResolution( i_rCameraName );
  outptr->Result = outpt.Result;
  outptr->ResolutionX = outpt.ResolutionX;
  outptr->ResolutionY = outpt.ResolutionY;
}

void Client_GetIsVideoCamera( CClient* client, CString i_rCameraName, COutput_GetIsVideoCamera* outptr )
{
  Output_GetIsVideoCamera outpt = ((Client*) client)->GetIsVideoCamera( (String)i_rCameraName );
  outptr->Result = outpt.Result;
  outptr->IsVideoCamera = outpt.IsVideoCamera;
}

void Client_GetCentroidCount( CClient* client, CString i_rCameraName, COutput_GetCentroidCount* outptr )
{
  Output_GetCentroidCount outpt = ((Client*) client)->GetCentroidCount( (String)i_rCameraName );
  outptr->Result = outpt.Result;
  outptr->CentroidCount = outpt.CentroidCount;
}

void Client_GetCentroidPosition( CClient* client, CString i_rCameraName, unsigned int i_CentroidIndex, COutput_GetCentroidPosition* outptr )
{
  Output_GetCentroidPosition outpt = ((Client*) client)->GetCentroidPosition( (String)i_rCameraName, i_CentroidIndex );
  outptr->Result = outpt.Result;
  outptr->Radius = outpt.Radius;
  std::memcpy( outptr->CentroidPosition, outpt.CentroidPosition, sizeof( outptr->CentroidPosition ) );
}

void Client_GetCentroidWeight( CClient* client, CString i_rCameraName, unsigned int i_CentroidIndex, COutput_GetCentroidWeight* outptr )
{
  Output_GetCentroidWeight outpt = ((Client*) client)->GetCentroidWeight( (String)i_rCameraName, i_CentroidIndex );
  outptr->Result = outpt.Result;
  outptr->Weight = outpt.Weight;
}

/*
void Client_GetGreyscaleBlobCount( CClient* client, CString i_rCameraName, COutput_GetGreyscaleBlobCount* outptr )
{
  Output_GetGreyscaleBlobCount outpt = ((Client*) client)->GetGreyscaleBlobCount( (String) i_rCameraName );
  outptr->Result = outpt.Result;
  outptr->BlobCount = outpt.BlobCount;
}

void Client_GetGreyscaleBlob( CClient* client, CString i_rCameraName, unsigned int i_BlobIndex, COutput_GetGreyscaleBlob* outptr )
{
  Output_GetGreyscaleBlob outpt = ((Client*) client)->GetGreyscaleBlob( (String)i_rCameraName, i_BlobIndex );
  outptr->Result = outpt.Result;
}

CEnum Client_SetCameraFilter( CClient* client, std::vector< unsigned int > i_rCameraIdsForCentroids, std::vector< unsigned int > i_rCameraIdsForBlobs )
{
  Output_SetCameraFilter outpt;

  return ((Client*) client)->SetCameraFilter( i_rCameraIdsForCentroids, i_rCameraIdsForBlobs ) );
  return outpt;
}
*/

