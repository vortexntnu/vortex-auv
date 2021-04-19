
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

#ifdef WIN32

#ifdef _EXPORTING
#define CDLL_EXPORT    __declspec(dllexport)
#else
#define CDLL_EXPORT    __declspec(dllimport)
#endif // _EXPORTING

#elif defined( __GNUC__ )

#if __GNUC__ < 4
#error gcc 4 is required.
#endif
#define CDLL_EXPORT     __attribute__((visibility("default")))

#else

#define CDLL_EXPORT

#endif



#ifdef __cplusplus
extern "C" { 
#endif

#include "CTypeDefs.h"

CDLL_EXPORT CRetimingClient* RetimingClient_Create(void);
CDLL_EXPORT void RetimingClient_Destroy( CRetimingClient* client );

CDLL_EXPORT void  RetimingClient_GetVersion(CRetimingClient* client, COutput_GetVersion* outptr);

CDLL_EXPORT CBool RetimingClient_Connect(CRetimingClient* client, CString HostName);
CDLL_EXPORT CBool RetimingClient_ConnectAndStart(CRetimingClient* client, CString HostName, double FrameRate );
CDLL_EXPORT CEnum RetimingClient_Disconnect(CRetimingClient* client);
CDLL_EXPORT CEnum RetimingClient_IsConnected(CRetimingClient* client);

CDLL_EXPORT CEnum RetimingClient_SetAxisMapping(CRetimingClient* client, CEnum XAxis, CEnum YAxis, CEnum ZAxis);
CDLL_EXPORT void RetimingClient_GetAxisMapping(CRetimingClient* client, COutput_GetAxisMapping* outptr);

CDLL_EXPORT CEnum RetimingClient_UpdateFrame(CRetimingClient* client );
CDLL_EXPORT CEnum RetimingClient_UpdateFrameOffset(CRetimingClient* client, double i_Offset);
CDLL_EXPORT CEnum RetimingClient_WaitForFrame(CRetimingClient* client);

CDLL_EXPORT void RetimingClient_GetSubjectCount(CRetimingClient* client, COutput_GetSubjectCount* outptr);
CDLL_EXPORT CEnum RetimingClient_GetSubjectName(CRetimingClient* client, unsigned int SubjectIndex, int sizeOfBuffer, char* outstr);

CDLL_EXPORT CEnum RetimingClient_GetSubjectRootSegmentName(CRetimingClient* client, CString SubjectName, int sizeOfBuffer, char* outstr);

CDLL_EXPORT void RetimingClient_GetSegmentCount(CRetimingClient* client, CString SubjectName, COutput_GetSegmentCount* outptr);

CDLL_EXPORT CEnum RetimingClient_GetSegmentName(CRetimingClient* client, CString SubjectName,
  unsigned int   SegmentIndex, int sizeOfBuffer, char* outstr);

CDLL_EXPORT void RetimingClient_GetSegmentChildCount(CRetimingClient* client, CString SubjectName,
  CString SegmentName, COutput_GetSegmentChildCount* outptr);

CDLL_EXPORT CEnum RetimingClient_GetSegmentChildName(CRetimingClient* client, CString SubjectName,
  CString SegmentName,
  unsigned int   SegmentIndex, int sizeOfBuffer, char* outstr);

CDLL_EXPORT CEnum RetimingClient_GetSegmentParentName(CRetimingClient* client, CString SubjectName,
  CString SegmentName, int sizeOfBuffer, char* outstr);

CDLL_EXPORT void RetimingClient_GetSegmentStaticTranslation(CRetimingClient* client, CString SubjectName,
  CString SegmentName, COutput_GetSegmentStaticTranslation* outptr);

CDLL_EXPORT void RetimingClient_GetSegmentStaticRotationHelical(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentStaticRotationHelical* outptr);

CDLL_EXPORT void RetimingClient_GetSegmentStaticRotationMatrix(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentStaticRotationMatrix* outptr);

CDLL_EXPORT void RetimingClient_GetSegmentStaticRotationQuaternion(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentStaticRotationQuaternion* outptr);

CDLL_EXPORT void RetimingClient_GetSegmentStaticRotationEulerXYZ(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentStaticRotationEulerXYZ* outptr);

CDLL_EXPORT void RetimingClient_GetSegmentGlobalTranslation(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentGlobalTranslation* outptr);

CDLL_EXPORT void RetimingClient_GetSegmentGlobalRotationHelical(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentGlobalRotationHelical* outptr);

CDLL_EXPORT void RetimingClient_GetSegmentGlobalRotationMatrix(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentGlobalRotationMatrix* outptr);

CDLL_EXPORT void RetimingClient_GetSegmentGlobalRotationQuaternion(CRetimingClient* client, CString  SubjectName, CString  SegmentName,
  COutput_GetSegmentGlobalRotationQuaternion* outptr);

CDLL_EXPORT void RetimingClient_GetSegmentGlobalRotationEulerXYZ(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentGlobalRotationEulerXYZ* outptr);

CDLL_EXPORT void RetimingClient_GetSegmentLocalTranslation(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentLocalTranslation* outptr);

CDLL_EXPORT void RetimingClient_GetSegmentLocalRotationHelical(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentLocalRotationHelical* outptr);

CDLL_EXPORT void RetimingClient_GetSegmentLocalRotationMatrix(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentLocalRotationMatrix* outptr);

CDLL_EXPORT void RetimingClient_GetSegmentLocalRotationQuaternion(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentLocalRotationQuaternion* outptr);

CDLL_EXPORT void RetimingClient_GetSegmentLocalRotationEulerXYZ(CRetimingClient* client, CString  SubjectName,
  CString  SegmentName, COutput_GetSegmentLocalRotationEulerXYZ* outptr);


CDLL_EXPORT void RetimingClient_SetOutputLatency( CRetimingClient* client, CReal i_OutputLatency );
CDLL_EXPORT CReal RetimingClient_OutputLatency( CRetimingClient* client );

CDLL_EXPORT void RetimingClient_SetMaximumPrediction( CRetimingClient* client, CReal i_MaxPrediction );
CDLL_EXPORT CReal RetimingClient_MaximumPrediction( CRetimingClient* client );

#ifdef __cplusplus
}
#endif
