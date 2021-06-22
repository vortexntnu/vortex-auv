#define _CRT_SECURE_NO_WARNINGS
#define NOMINMAX

#include <memory.h>
#include <float.h>
#include <stdint.h>
#include <math.h>

#ifdef _WIN32
#include <Winsock2.h>
#else
#include <arpa/inet.h>
#endif

#include "RTPacket.h"

CRTPacket::CRTPacket(int nMajorVersion, int nMinorVersion, bool bBigEndian)
{
    mnMajorVersion = nMajorVersion;
    mnMinorVersion = nMinorVersion;
    mbBigEndian    = bBigEndian;

    ClearData();
}

void CRTPacket::GetVersion(unsigned int &nMajorVersion, unsigned int &nMinorVersion)
{
    nMajorVersion = mnMajorVersion;
    nMinorVersion = mnMinorVersion;
}

void CRTPacket::SetVersion(unsigned int nMajorVersion, unsigned int nMinorVersion)
{
    mnMajorVersion = nMajorVersion;
    mnMinorVersion = nMinorVersion;
}

bool CRTPacket::GetEndianness()
{
    return mbBigEndian;
}

void CRTPacket::SetEndianness(bool bBigEndian)
{
    mbBigEndian = bBigEndian;
}

void CRTPacket::ClearData()
{
    mpData                    = nullptr;
    mnComponentCount          = 0;
    mn2DCameraCount           = 0;
    mn2DLinCameraCount        = 0;
    mnImageCameraCount        = 0;
    mnAnalogDeviceCount       = 0;
    mnAnalogSingleDeviceCount = 0;
    mnForcePlateCount         = 0;
    mnForceSinglePlateCount   = 0;
    mnGazeVectorCount         = 0;
    mnTimecodeCount           = 0;
    mSkeletonCount           = 0;
    memset(mpComponentData, 0, ComponentNone * 4);
    memset(mp2DData, 0, MAX_CAMERA_COUNT * 4);
    memset(mp2DLinData, 0, MAX_CAMERA_COUNT * 4);
    memset(mpImageData, 0, MAX_CAMERA_COUNT * 4);
    memset(mpAnalogData, 0, MAX_ANALOG_DEVICE_COUNT * 4);
    memset(mpAnalogSingleData, 0, MAX_ANALOG_DEVICE_COUNT * 4);
    memset(mpForceData, 0, MAX_FORCE_PLATE_COUNT * 4);
    memset(mpForceSingleData, 0, MAX_FORCE_PLATE_COUNT * 4);
    memset(mpGazeVectorData, 0, MAX_GAZE_VECTOR_COUNT * 4);
    memset(mpSkeletonData, 0, MAX_SKELETON_COUNT * 4);
}

void CRTPacket::SetData(char* ptr)
{
    unsigned int nComponent;
    unsigned int nCamera, nDevice;

    mpData = ptr;

    mnComponentCount          = 0;
    mn2DCameraCount           = 0;
    mn2DLinCameraCount        = 0;
    mnImageCameraCount        = 0;
    mnAnalogDeviceCount       = 0;
    mnAnalogSingleDeviceCount = 0;
    mnForcePlateCount         = 0;
    mnForceSinglePlateCount   = 0;
    mnGazeVectorCount         = 0;
    mnTimecodeCount           = 0;
    mSkeletonCount           = 0;

    // Check if it's a data packet
    if (GetType() == PacketData)
    {
        // Reset all component data pointers
        for (nComponent = 1; nComponent < ComponentNone; nComponent++)
        {
            mpComponentData[nComponent - 1] = nullptr;
        }

        char*        pCurrentComponent = mpData + 24;
        unsigned int nComponentType    = SetByteOrder((unsigned int*)(pCurrentComponent + 4));

        mnComponentCount = SetByteOrder((unsigned int*)(mpData + 20));

        for (nComponent = 1; nComponent <= mnComponentCount && nComponentType > 0 && nComponentType < ComponentNone; nComponent++)
        {
            mpComponentData[nComponentType - 1] = pCurrentComponent;

            if (nComponentType == Component2d)
            {
                mn2DCameraCount = SetByteOrder((unsigned int*)(pCurrentComponent + 8));

                mp2DData[0] = pCurrentComponent + 16;
                for (nCamera = 1; nCamera < mn2DCameraCount; nCamera++)
                {
                    if (mnMajorVersion > 1 || mnMinorVersion > 7)
                    {
                        mp2DData[nCamera] = mp2DData[nCamera - 1] + 5 + Get2DMarkerCount(nCamera - 1) * 12;
                    }
                    else
                    {
                        mp2DData[nCamera] = mp2DData[nCamera - 1] + 4 + Get2DMarkerCount(nCamera - 1) * 12;
                    }
                }
            }
            if (nComponentType == Component2dLin)
            {
                mn2DLinCameraCount = SetByteOrder((unsigned int*)(pCurrentComponent + 8));

                mp2DLinData[0] = pCurrentComponent + 16;
                for (nCamera = 1; nCamera < mn2DLinCameraCount; nCamera++)
                {
                    if (mnMajorVersion > 1 || mnMinorVersion > 7)
                    {
                        mp2DLinData[nCamera] = mp2DLinData[nCamera - 1] + 5 + Get2DLinMarkerCount(nCamera - 1) * 12;
                    }
                    else
                    {
                        mp2DLinData[nCamera] = mp2DLinData[nCamera - 1] + 4 + Get2DLinMarkerCount(nCamera - 1) * 12;
                    }
                }
            }
            if (nComponentType == ComponentImage)
            {
                mnImageCameraCount = SetByteOrder((unsigned int*)(pCurrentComponent + 8));

                mpImageData[0] = pCurrentComponent + 12;
                for (nCamera = 1; nCamera < mnImageCameraCount; nCamera++)
                {
                    mpImageData[nCamera] = mpImageData[nCamera - 1] + 36 + SetByteOrder((unsigned int*)(mpImageData[nCamera - 1] + 32));
                }
            }
            if (nComponentType == ComponentAnalog)
            {
                if ((mnMajorVersion == 1) && (mnMinorVersion == 0))
                {
                    mnAnalogDeviceCount = 1;
                }
                else
                {
                    mnAnalogDeviceCount = SetByteOrder((unsigned int*)(pCurrentComponent + 8));
                }

                if ((mnMajorVersion > 1) || (mnMinorVersion > 7))
                {
                    mpAnalogData[0] = pCurrentComponent + 12;
                }
                else
                {
                    mpAnalogData[0] = pCurrentComponent + 16;
                }
                for (nDevice = 1; nDevice < mnAnalogDeviceCount; nDevice++)
                {
                    mpAnalogData[nDevice] = mpAnalogData[nDevice - 1] + 16 + 
                        (SetByteOrder((unsigned int*)(mpAnalogData[nDevice - 1] + 4)) *
                        SetByteOrder((unsigned int*)(mpAnalogData[nDevice - 1] + 8)) * 4);
                }
            }
            if (nComponentType == ComponentAnalogSingle)
            {
                mnAnalogSingleDeviceCount = SetByteOrder((unsigned int*)(pCurrentComponent + 8));

                if (mnMajorVersion > 1 || mnMinorVersion > 7)
                {
                    mpAnalogSingleData[0] = pCurrentComponent + 12;
                }
                else
                {
                    mpAnalogSingleData[0] = pCurrentComponent + 16;
                }

                for (nDevice = 1; nDevice < mnAnalogSingleDeviceCount; nDevice++)
                {
                    mpAnalogSingleData[nDevice] = mpAnalogSingleData[nDevice - 1] + 8 + 
                        SetByteOrder((unsigned int*)(mpAnalogSingleData[nDevice - 1] + 4)) * 4;
                }
            }
            if (nComponentType == ComponentForce)
            {
                mnForcePlateCount = SetByteOrder((unsigned int*)(pCurrentComponent + 8));

                if (mnMajorVersion > 1 || mnMinorVersion > 7)
                {
                    mpForceData[0] = pCurrentComponent + 12;
                }
                else
                {
                    mpForceData[0] = pCurrentComponent + 16;
                }
                for (nDevice = 1; nDevice < mnForcePlateCount; nDevice++)
                {
                    if ((mnMajorVersion == 1) && (mnMinorVersion == 0))
                    {
                        mpForceData[nDevice] = mpForceData[nDevice - 1] + 72;
                    }
                    else
                    {
                        mpForceData[nDevice] = mpForceData[nDevice - 1] + 12 + 
                            SetByteOrder((unsigned int*)(mpForceData[nDevice - 1] + 4)) * 36;
                    }
                }
            }
            if (nComponentType == ComponentForceSingle)
            {
                mnForceSinglePlateCount = SetByteOrder((unsigned int*)(pCurrentComponent + 8));

                mpForceSingleData[0] = pCurrentComponent + 12;

                for (nDevice = 1; nDevice < mnForceSinglePlateCount; nDevice++)
                {
                    mpForceSingleData[nDevice] = mpForceSingleData[nDevice - 1] + 4 + 36;
                }
            }
            if (nComponentType == ComponentGazeVector)
            {
                mnGazeVectorCount = SetByteOrder((unsigned int*)(pCurrentComponent + 8));

                mpGazeVectorData[0] = pCurrentComponent + 12;

                for (nDevice = 1; nDevice < mnGazeVectorCount; nDevice++)
                {
                    unsigned int nPrevSampleCount = SetByteOrder((unsigned int*)(mpGazeVectorData[nDevice - 1]));
                    mpGazeVectorData[nDevice] = mpGazeVectorData[nDevice - 1] + 4 + ((nPrevSampleCount == 0) ? 0 : 4) +
                                                nPrevSampleCount * 24;
                }
            }
            if (nComponentType == ComponentTimecode)
            {
                mnTimecodeCount = SetByteOrder((unsigned int*)(pCurrentComponent + 8));

                mpTimecodeData[0] = pCurrentComponent + 12;

                for (nDevice = 1; nDevice < mnTimecodeCount; nDevice++)
                {
                    mpTimecodeData[nDevice] = mpTimecodeData[nDevice - 1] + 12;
                }
            }
            if (nComponentType == ComponentSkeleton)
            {
                mSkeletonCount = SetByteOrder((unsigned int*)(pCurrentComponent + 8));

                mpSkeletonData[0] = pCurrentComponent + 12;

                for (nDevice = 1; nDevice < mSkeletonCount; nDevice++)
                {
                    unsigned int prevSegmentCount = SetByteOrder((unsigned int*)(mpSkeletonData[nDevice - 1]));
                    mpSkeletonData[nDevice] = mpSkeletonData[nDevice - 1] + 4 + prevSegmentCount * 32;
                }
            }
            pCurrentComponent += SetByteOrder((int*)pCurrentComponent);
            nComponentType     = SetByteOrder((unsigned int*)(pCurrentComponent + 4));
        }
    }
} // SetData


void CRTPacket::GetData(char* &ptr, unsigned int &nSize)
{
    if (mpData == nullptr)
    {
        nSize = 0;
    }
    else
    {
        ptr   = mpData;
        nSize = *((int*)mpData);
    }
}

unsigned int CRTPacket::GetSize()
{
    if (mpData == nullptr)
    {
        return 0;
    }
    if (mbBigEndian || ((mnMajorVersion == 1) && (mnMinorVersion == 0)))
    {
        return ntohl(*((unsigned int*)mpData));
    }
    return *((unsigned int*)mpData);
}

CRTPacket::EPacketType CRTPacket::GetType()
{
    if (GetSize() < 8)
    {
        return PacketNone;
    }
    if (mbBigEndian || ((mnMajorVersion == 1) && (mnMinorVersion == 0)))
    {
        return (EPacketType)ntohl(*(unsigned int*)(mpData + 4));
    }
    return (EPacketType)*((unsigned int*)(mpData + 4));
}

unsigned long long CRTPacket::GetTimeStamp()
{
    if (GetType() == PacketData)
    {
        return SetByteOrder((long long*)(mpData + 8));
    }
    return 0;
}

unsigned int CRTPacket::GetFrameNumber()
{
    if (GetType() == PacketData)
    {
        return SetByteOrder((unsigned int*)(mpData + 16));
    }
    return 0;
}

unsigned int CRTPacket::GetSize(char* pData, bool bBigEndian)
{
    if (bBigEndian)
    {
        return ntohl(*((unsigned int*)pData));
    }
    return *((unsigned int*)pData);
}

CRTPacket::EPacketType CRTPacket::GetType(char* pData, bool bBigEndian)
{
    if (GetSize(pData, bBigEndian) < 8)
    {
        return PacketNone;
    }
    if (bBigEndian)
    {
        return (EPacketType)ntohl(*(unsigned int*)(pData + 4));
    }
    return (EPacketType)*((unsigned int*)(pData + 4));
}

unsigned long long CRTPacket::GetTimeStamp(char* pData, bool bBigEndian)
{
    if (GetType(pData, bBigEndian) == PacketData)
    {
        if (bBigEndian)
        {
            return ((unsigned long long)(ntohl((long)*((long long*)(pData + 8)))) <<  32) + ntohl(*((long long*)(pData + 8)) >> 32);
        }
        return *((long long*)(pData + 8));
    }
    return 0;
}

unsigned int CRTPacket::GetFrameNumber(char* pData, bool bBigEndian)
{
    if (GetType(pData, bBigEndian) == PacketData)
    {
        if (bBigEndian)
        {
            return ntohl(*((unsigned int*)(pData + 16)));
        }
        return *((unsigned int*)(pData + 16));
    }
    return 0;
}

unsigned int CRTPacket::GetComponentCount()
{
    return mnComponentCount;
}

unsigned int CRTPacket::GetComponentSize(EComponentType eComponent)
{
    if (eComponent < Component3d || eComponent >= ComponentNone)
    {
        return 0;
    }
    if (mpComponentData[eComponent - 1] == nullptr)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(mpComponentData[eComponent - 1]));
}

char* CRTPacket::GetErrorString()
{
    if (GetType() == PacketError)
    {
        return mpData + 8;
    }
    return nullptr;
}


char* CRTPacket::GetCommandString()
{
    if (GetType() == PacketCommand)
    {
        return mpData + 8;
    }
    return nullptr;
}


char* CRTPacket::GetCommandString(char* pData, bool bBigEndian)
{
    if (GetType(pData, bBigEndian) == PacketCommand)
    {
        return pData + 8;
    }
    return nullptr;
}


char* CRTPacket::GetXMLString()
{
    if (GetType() == PacketXML)
    {
        return mpData + 8;
    }
    return nullptr;
}


short CRTPacket::GetDiscoverResponseBasePort()
{
    if (GetType() == PacketCommand)
    {
        if (GetSize() == (8 + strlen(mpData + 8) + 1 + 2))
        {
            return ntohs(*((short*)(mpData + 8 + strlen(mpData + 8) + 1)));
        }
    }
    return 0;
}

short CRTPacket::GetDiscoverResponseBasePort(char* pData, bool bBigEndian)
{
    if (GetType(pData, bBigEndian) == PacketCommand)
    {
        if (GetSize(pData, bBigEndian) == (8 + strlen(pData + 8) + 1 + 2))
        {
            return ntohs(*((short*)(pData + 8 + strlen(pData + 8) + 1)));
        }
    }
    return 0;
}

bool CRTPacket::GetEvent(EEvent &eEvent)
{
    if (GetType() == PacketEvent)
    {
        eEvent = (EEvent)*(mpData + 8);
        return true;
    }
    return false;
}

bool CRTPacket::GetEvent(EEvent &eEvent, char* pData, bool bBigEndian)
{
    if (GetType(pData, bBigEndian) == PacketEvent)
    {
        eEvent = (EEvent)*(pData + 8);
        return true;
    }
    return false;
}

unsigned short CRTPacket::GetDropRate()
{
    for (int i = 0; i <= 1; i++)
    {
        if (mpComponentData[i] != nullptr)
        {
            return SetByteOrder((unsigned short*)(mpComponentData[i] + 12));
        }
    }
    for (int i = 4; i <= 11; i++)
    {
        if (mpComponentData[i] != nullptr)
        {
            return SetByteOrder((unsigned short*)(mpComponentData[i] + 12));
        }
    }
    return 0;
}

unsigned short CRTPacket::GetOutOfSyncRate()
{
    for (int i = 0; i <= 1; i++)
    {
        if (mpComponentData[i] != nullptr)
        {
            return SetByteOrder((unsigned short*)(mpComponentData[i] + 14));
        }
    }
    for (int i = 4; i <= 11; i++)
    {
        if (mpComponentData[i] != nullptr)
        {
            return SetByteOrder((unsigned short*)(mpComponentData[i] + 14));
        }
    }
    return 0;
}



//-----------------------------------------------------------
//                         2D
//-----------------------------------------------------------
unsigned int CRTPacket::Get2DCameraCount()
{
    return mn2DCameraCount;
}

unsigned int CRTPacket::Get2DMarkerCount(unsigned int nCameraIndex)
{
    if (mn2DCameraCount <= nCameraIndex)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(mp2DData[nCameraIndex]));
}

unsigned char CRTPacket::Get2DStatusFlags(unsigned int nCameraIndex)
{
    if (mn2DCameraCount > nCameraIndex && ((mnMajorVersion > 1) || (mnMinorVersion > 7)))
    {
        return *((unsigned char*)(mp2DData[nCameraIndex] + 4));
    }
    return 0;
}

bool CRTPacket::Get2DMarker(unsigned int nCameraIndex, unsigned int nMarkerIndex, unsigned int &nX, unsigned int &nY,
                            unsigned short &nXDiameter, unsigned short &nYDiameter)
{
    int nOffset;

    if (mn2DCameraCount <= nCameraIndex || Get2DMarkerCount(nCameraIndex) <= nMarkerIndex)
    {
        return false;
    }

    if (mnMajorVersion > 1 || mnMinorVersion > 7)
    {
        nOffset = 5;
    }
    else
    {
        nOffset = 4;
    }
    nX         = SetByteOrder((unsigned int*)(mp2DData[nCameraIndex]   + nOffset +      nMarkerIndex * 12));
    nY         = SetByteOrder((unsigned int*)(mp2DData[nCameraIndex]   + nOffset + 4  + nMarkerIndex * 12));
    nXDiameter = SetByteOrder((unsigned short*)(mp2DData[nCameraIndex] + nOffset + 8  + nMarkerIndex * 12));
    nYDiameter = SetByteOrder((unsigned short*)(mp2DData[nCameraIndex] + nOffset + 10 + nMarkerIndex * 12));

    return true;
}


//-----------------------------------------------------------
//                      2D Linearized
//-----------------------------------------------------------
unsigned int CRTPacket::Get2DLinCameraCount()
{
    return mn2DLinCameraCount;
}

unsigned int CRTPacket::Get2DLinMarkerCount(unsigned int nCameraIndex)
{
    if (mn2DLinCameraCount <= nCameraIndex)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(mp2DLinData[nCameraIndex]));
}

unsigned char CRTPacket::Get2DLinStatusFlags(unsigned int nCameraIndex)
{
    if (mn2DLinCameraCount > nCameraIndex && ((mnMajorVersion > 1) || (mnMinorVersion > 7)))
    {
        return *((unsigned char*)(mp2DLinData[nCameraIndex] + 4));
    }
    return 0;
}

bool CRTPacket::Get2DLinMarker(unsigned int nCameraIndex, unsigned int nMarkerIndex, unsigned int &nX, unsigned int &nY,
                               unsigned short &nXDiameter, unsigned short &nYDiameter)
{
    int nOffset;

    if (mn2DLinCameraCount <= nCameraIndex || Get2DLinMarkerCount(nCameraIndex) <= nMarkerIndex)
    {
        return false;
    }

    if (mnMajorVersion > 1 || mnMinorVersion > 7)
    {
        nOffset = 5;
    }
    else
    {
        nOffset = 4;
    }
    nX         = SetByteOrder((unsigned int*)(mp2DLinData[nCameraIndex]   + nOffset +      nMarkerIndex * 12));
    nY         = SetByteOrder((unsigned int*)(mp2DLinData[nCameraIndex]   + nOffset + 4  + nMarkerIndex * 12));
    nXDiameter = SetByteOrder((unsigned short*)(mp2DLinData[nCameraIndex] + nOffset + 8  + nMarkerIndex * 12));
    nYDiameter = SetByteOrder((unsigned short*)(mp2DLinData[nCameraIndex] + nOffset + 10 + nMarkerIndex * 12));

    return true;
}


//-----------------------------------------------------------
//                            3D
//-----------------------------------------------------------
unsigned int CRTPacket::Get3DMarkerCount()
{
    if (GetComponentSize(CRTPacket::Component3d) == 0)
    {
        return 0;
    }
    
    char* pData = mpComponentData[Component3d - 1];

    if (pData == nullptr)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(pData + 8));
}

bool CRTPacket::Get3DMarker(unsigned int nMarkerIndex, float &fX, float &fY, float &fZ)
{
    char* pData = mpComponentData[Component3d - 1];

    if (Get3DMarkerCount() <= nMarkerIndex)
    {
        return false;
    }

    if (mnMajorVersion > 1 || mnMinorVersion > 7)
    {
        fX = SetByteOrder((float*)(pData + 16 + nMarkerIndex * 12));
        fY = SetByteOrder((float*)(pData + 20 + nMarkerIndex * 12));
        fZ = SetByteOrder((float*)(pData + 24 + nMarkerIndex * 12));
    }
    else
    {
        fX = (float)SetByteOrder((double*)(pData + 16 + nMarkerIndex * 24));
        fY = (float)SetByteOrder((double*)(pData + 24 + nMarkerIndex * 24));
        fZ = (float)SetByteOrder((double*)(pData + 32 + nMarkerIndex * 24));
    }
    return (isnan(fX) == 0);
}


//-----------------------------------------------------------
//                        3D Residual
//-----------------------------------------------------------
unsigned int CRTPacket::Get3DResidualMarkerCount()
{
    char* pData = mpComponentData[Component3dRes - 1];

    if (pData == nullptr)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(pData + 8));
}

bool CRTPacket::Get3DResidualMarker(unsigned int nMarkerIndex, float &fX, float &fY, float &fZ, float &fResidual)
{
    char* pData = mpComponentData[Component3dRes - 1];

    if (Get3DResidualMarkerCount() <= nMarkerIndex)
    {
        return false;
    }

    if (mnMajorVersion > 1 || mnMinorVersion > 7)
    {
        fX        = SetByteOrder((float*)(pData + 16 + nMarkerIndex * 16));
        fY        = SetByteOrder((float*)(pData + 20 + nMarkerIndex * 16));
        fZ        = SetByteOrder((float*)(pData + 24 + nMarkerIndex * 16));
        fResidual = SetByteOrder((float*)(pData + 28 + nMarkerIndex * 16));
    }
    else
    {
        fX        = (float)SetByteOrder((double*)(pData + 16 + nMarkerIndex * 32));
        fY        = (float)SetByteOrder((double*)(pData + 24 + nMarkerIndex * 32));
        fZ        = (float)SetByteOrder((double*)(pData + 32 + nMarkerIndex * 32));
        fResidual =        SetByteOrder((float*) (pData + 40 + nMarkerIndex * 32));
    }
    return (isnan(fX) == 0);
}


//-----------------------------------------------------------
//                        3D No Labels
//-----------------------------------------------------------
unsigned int CRTPacket::Get3DNoLabelsMarkerCount()
{
    char* pData = mpComponentData[Component3dNoLabels - 1];

    if (pData == nullptr)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(pData + 8));
}

bool CRTPacket::Get3DNoLabelsMarker(unsigned int nMarkerIndex, float &fX, float &fY, float &fZ, unsigned int &nId)
{
    char* pData = mpComponentData[Component3dNoLabels - 1];

    if (Get3DNoLabelsMarkerCount() <= nMarkerIndex)
    {
        return false;
    }

    if (mnMajorVersion > 1 || mnMinorVersion > 7)
    {
        fX  = SetByteOrder((float*)(pData + 16 + nMarkerIndex * 16));
        fY  = SetByteOrder((float*)(pData + 20 + nMarkerIndex * 16));
        fZ  = SetByteOrder((float*)(pData + 24 + nMarkerIndex * 16));
        nId = SetByteOrder((unsigned int*)(pData + 28 + nMarkerIndex * 16));
    }
    else
    {
        fX  = (float)SetByteOrder((double*)(pData + 16 + nMarkerIndex * 32));
        fY  = (float)SetByteOrder((double*)(pData + 24 + nMarkerIndex * 32));
        fZ  = (float)SetByteOrder((double*)(pData + 32 + nMarkerIndex * 32));
        nId =  SetByteOrder((unsigned int*)(pData + 40 + nMarkerIndex * 32));
    }
    return true;
}


//-----------------------------------------------------------
//                   3D No Labels Residual
//-----------------------------------------------------------
unsigned int CRTPacket::Get3DNoLabelsResidualMarkerCount()
{
    char* pData = mpComponentData[Component3dNoLabelsRes - 1];

    if (pData == nullptr)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(pData + 8));
}

bool CRTPacket::Get3DNoLabelsResidualMarker(unsigned int nMarkerIndex, float &fX, float &fY, float &fZ,
                                            unsigned int &nId, float &fResidual)
{
    char* pData = mpComponentData[Component3dNoLabelsRes - 1];

    if (Get3DNoLabelsResidualMarkerCount() <= nMarkerIndex)
    {
        return false;
    }

    if (mnMajorVersion > 1 || mnMinorVersion > 7)
    {
        fX        = SetByteOrder((float*)(pData + 16 + nMarkerIndex * 20));
        fY        = SetByteOrder((float*)(pData + 20 + nMarkerIndex * 20));
        fZ        = SetByteOrder((float*)(pData + 24 + nMarkerIndex * 20));
        nId       = SetByteOrder((unsigned int*)(pData + 28 + nMarkerIndex * 20));
        fResidual = SetByteOrder((float*)(pData + 32 + nMarkerIndex * 20));
    }
    else
    {
        fX        = (float)SetByteOrder((double*)(pData + 16 + nMarkerIndex * 32));
        fY        = (float)SetByteOrder((double*)(pData + 24 + nMarkerIndex * 32));
        fZ        = (float)SetByteOrder((double*)(pData + 32 + nMarkerIndex * 32));
        nId       =  SetByteOrder((unsigned int*)(pData + 40 + nMarkerIndex * 32));
        fResidual =        SetByteOrder((float*) (pData + 44 + nMarkerIndex * 32));
    }
    return true;
}


//-----------------------------------------------------------
//                           6DOF
//-----------------------------------------------------------
unsigned int CRTPacket::Get6DOFBodyCount()
{
    char* pData = mpComponentData[Component6d - 1];

    if (pData == nullptr)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(pData + 8));
}

bool CRTPacket::Get6DOFBody(unsigned int nBodyIndex, float &fX, float &fY, float &fZ, float afRotMatrix[9])
{
    char* pData = mpComponentData[Component6d - 1];

    if (Get6DOFBodyCount() <= nBodyIndex)
    {
        return false;
    }

    if (mnMajorVersion > 1 || mnMinorVersion > 7)
    {
        fX = SetByteOrder((float*)(pData + 16 + nBodyIndex * 48));
        fY = SetByteOrder((float*)(pData + 20 + nBodyIndex * 48));
        fZ = SetByteOrder((float*)(pData + 24 + nBodyIndex * 48));
        for (int i = 0; i < 9; i++)
        {
            afRotMatrix[i] = SetByteOrder((float*)(pData + 28 + (i * 4) + nBodyIndex * 48));
        }
    }
    else
    {
        fX = (float)SetByteOrder((double*)(pData + 16 + nBodyIndex * 96));
        fY = (float)SetByteOrder((double*)(pData + 24 + nBodyIndex * 96));
        fZ = (float)SetByteOrder((double*)(pData + 32 + nBodyIndex * 96));
        for (int i = 0; i < 9; i++)
        {
            afRotMatrix[i] = (float)SetByteOrder((float*)(pData + 40 + (i * 4) + nBodyIndex * 96));
        }
    }
    return true;
}


//-----------------------------------------------------------
//                      6DOF Residual
//-----------------------------------------------------------
unsigned int CRTPacket::Get6DOFResidualBodyCount()
{
    char* pData = mpComponentData[Component6dRes - 1];

    if (pData == nullptr)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(pData + 8));
}

bool CRTPacket::Get6DOFResidualBody(unsigned int nBodyIndex, float &fX, float &fY, float &fZ,
                                    float afRotMatrix[9], float &fResidual)
{
    char* pData = mpComponentData[Component6dRes - 1];

    if (Get6DOFResidualBodyCount() <= nBodyIndex)
    {
        return false;
    }

    if (mnMajorVersion > 1 || mnMinorVersion > 7)
    {
        fX = SetByteOrder((float*)(pData + 16 + nBodyIndex * 52));
        fY = SetByteOrder((float*)(pData + 20 + nBodyIndex * 52));
        fZ = SetByteOrder((float*)(pData + 24 + nBodyIndex * 52));
        for (int i = 0; i < 9; i++)
        {
            afRotMatrix[i] = SetByteOrder((float*)(pData + 28 + (i * 4) + nBodyIndex * 52));
        }
        fResidual = SetByteOrder((float*)(pData + 64 + nBodyIndex * 52));
    }
    else
    {
        fX = (float)SetByteOrder((double*)(pData + 16 + nBodyIndex * 104));
        fY = (float)SetByteOrder((double*)(pData + 24 + nBodyIndex * 104));
        fZ = (float)SetByteOrder((double*)(pData + 32 + nBodyIndex * 104));
        for (int i = 0; i < 9; i++)
        {
            afRotMatrix[i] = (float)SetByteOrder((double*)(pData + 40 + (i * 8) + nBodyIndex * 104));
        }
        fResidual = SetByteOrder((float*)(pData + 112 + nBodyIndex * 104));
    }
    return true;
}


//-----------------------------------------------------------
//                       6DOF Euler
//-----------------------------------------------------------
unsigned int CRTPacket::Get6DOFEulerBodyCount()
{
    char* pData = mpComponentData[Component6dEuler - 1];

    if (pData == nullptr)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(pData + 8));
}

bool CRTPacket::Get6DOFEulerBody(unsigned int nBodyIndex, float &fX, float &fY, float &fZ,
                                 float &fAng1, float &fAng2, float &fAng3)
{
    char* pData = mpComponentData[Component6dEuler - 1];

    if (Get6DOFEulerBodyCount() <= nBodyIndex)
    {
        return false;
    }

    if (mnMajorVersion > 1 || mnMinorVersion > 7)
    {
        fX    = SetByteOrder((float*)(pData + 16 + nBodyIndex * 24));
        fY    = SetByteOrder((float*)(pData + 20 + nBodyIndex * 24));
        fZ    = SetByteOrder((float*)(pData + 24 + nBodyIndex * 24));
        fAng1 = SetByteOrder((float*)(pData + 28 + nBodyIndex * 24));
        fAng2 = SetByteOrder((float*)(pData + 32 + nBodyIndex * 24));
        fAng3 = SetByteOrder((float*)(pData + 36 + nBodyIndex * 24));
    }
    else
    {
        fX    = (float)SetByteOrder((double*)(pData + 16 + nBodyIndex * 48));
        fY    = (float)SetByteOrder((double*)(pData + 24 + nBodyIndex * 48));
        fZ    = (float)SetByteOrder((double*)(pData + 32 + nBodyIndex * 48));
        fAng1 = (float)SetByteOrder((double*)(pData + 40 + nBodyIndex * 48));
        fAng2 = (float)SetByteOrder((double*)(pData + 48 + nBodyIndex * 48));
        fAng3 = (float)SetByteOrder((double*)(pData + 56 + nBodyIndex * 48));
    }
    return true;
}


//-----------------------------------------------------------
//                    6DOF Euler Residual
//-----------------------------------------------------------
unsigned int CRTPacket::Get6DOFEulerResidualBodyCount()
{
    char* pData = mpComponentData[Component6dEulerRes - 1];

    if (pData == nullptr)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(pData + 8));
}

bool CRTPacket::Get6DOFEulerResidualBody(unsigned int nBodyIndex, float &fX, float &fY, float &fZ,
                                         float &fAng1, float &fAng2, float &fAng3, float &fResidual)
{
    char* pData = mpComponentData[Component6dEulerRes - 1];

    if (Get6DOFEulerResidualBodyCount() <= nBodyIndex)
    {
        return false;
    }

    if (mnMajorVersion > 1 || mnMinorVersion > 7)
    {
        fX        = SetByteOrder((float*)(pData + 16 + nBodyIndex * 28));
        fY        = SetByteOrder((float*)(pData + 20 + nBodyIndex * 28));
        fZ        = SetByteOrder((float*)(pData + 24 + nBodyIndex * 28));
        fAng1     = SetByteOrder((float*)(pData + 28 + nBodyIndex * 28));
        fAng2     = SetByteOrder((float*)(pData + 32 + nBodyIndex * 28));
        fAng3     = SetByteOrder((float*)(pData + 36 + nBodyIndex * 28));
        fResidual = SetByteOrder((float*)(pData + 40 + nBodyIndex * 28));
    }
    else
    {
        fX        = (float)SetByteOrder((double*)(pData + 16 + nBodyIndex * 56));
        fY        = (float)SetByteOrder((double*)(pData + 24 + nBodyIndex * 56));
        fZ        = (float)SetByteOrder((double*)(pData + 32 + nBodyIndex * 56));
        fAng1     = (float)SetByteOrder((double*)(pData + 40 + nBodyIndex * 56));
        fAng2     = (float)SetByteOrder((double*)(pData + 48 + nBodyIndex * 56));
        fAng3     = (float)SetByteOrder((double*)(pData + 56 + nBodyIndex * 56));
        fResidual =        SetByteOrder( (float*)(pData + 64 + nBodyIndex * 56));
    }
    return true;
}


//-----------------------------------------------------------
//                       Gaze Vector
//-----------------------------------------------------------
unsigned int CRTPacket::GetGazeVectorCount()
{
    return mnGazeVectorCount;
}

unsigned int CRTPacket::GetGazeVectorSampleCount(unsigned int nVectorIndex)
{
    if (mnGazeVectorCount <= nVectorIndex)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(mpGazeVectorData[nVectorIndex]));
}

unsigned int CRTPacket::GetGazeVectorSampleNumber(unsigned int nVectorIndex)
{
    unsigned int nSampleCount = GetGazeVectorSampleCount(nVectorIndex);

    if (nSampleCount == 0)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(mpGazeVectorData[nVectorIndex] + 4));
}

bool CRTPacket::GetGazeVector(unsigned int nVectorIndex, unsigned int nSampleIndex, SGazeVector &sGazeVector)
{
    unsigned int nSampleCount = GetGazeVectorSampleCount(nVectorIndex);

    if (nSampleCount == 0 || nSampleIndex >= nSampleCount)
    {
        return false;
    }

    for (unsigned int k = 0; k < 6; k++)
    {
        *(((float*)&sGazeVector) + k) =
            (float)SetByteOrder((float*)(mpGazeVectorData[nVectorIndex] + 8 + k * sizeof(float) + nSampleIndex * 24));
    }

    return (isnan(sGazeVector.fPosX) == 0);
}

bool CRTPacket::GetGazeVector(unsigned int nVectorIndex, SGazeVector* pGazeVectorBuf, unsigned int nBufSize)
{
    unsigned int nSampleCount = GetGazeVectorSampleCount(nVectorIndex);

    if (nSampleCount == 0 || (nBufSize < nSampleCount * sizeof(SGazeVector)))
    {
        return false;
    }

    for (unsigned int nSample = 0; nSample < nSampleCount; nSample++)
    {
        for (unsigned int k = 0; k < 6; k++)
        {
            *(((float*)pGazeVectorBuf) + k + (nSample * sizeof(SGazeVector))) =
                (float)SetByteOrder((float*)(mpGazeVectorData[nVectorIndex] + 8 + k * sizeof(float) + nSample * 24));
        }
    }

    return true;
}


//-----------------------------------------------------------
//                       Timecode
//-----------------------------------------------------------
unsigned int CRTPacket::GetTimecodeCount()
{
    return mnTimecodeCount;
}

bool CRTPacket::GetTimecodeType(unsigned int nTimecodeIndex, CRTPacket::ETimecodeType &timecodeType)
{
    if (mnTimecodeCount <= nTimecodeIndex)
    {
        return false;
    }
    timecodeType = (CRTPacket::ETimecodeType)SetByteOrder((unsigned int*)(mpTimecodeData[nTimecodeIndex]));
    return true;
}

bool CRTPacket::GetTimecodeSMPTE(unsigned int nTimecodeIndex, int &hours, int &minutes, int &seconds, int &frame)
{
    if (mnTimecodeCount <= nTimecodeIndex)
    {
        return false;
    }
    CRTPacket::ETimecodeType timecodeType;
    if (GetTimecodeType(nTimecodeIndex, timecodeType))
    {
        if (timecodeType == TimecodeSMPTE)
        {
            hours   = 0x1f & SetByteOrder((unsigned int*)(mpTimecodeData[nTimecodeIndex] + 8));
            minutes = 0x3f & (SetByteOrder((unsigned int*)(mpTimecodeData[nTimecodeIndex] + 8)) >> 5);
            seconds = 0x3f & (SetByteOrder((unsigned int*)(mpTimecodeData[nTimecodeIndex] + 8)) >> 11);
            frame   = 0x1f & (SetByteOrder((unsigned int*)(mpTimecodeData[nTimecodeIndex] + 8)) >> 17);
            return true;
        }
    }
    return false;
}

bool CRTPacket::GetTimecodeIRIG(unsigned int nTimecodeIndex, int &year, int &day, int &hours, int &minutes, int &seconds, int &tenths)
{
    if (mnTimecodeCount <= nTimecodeIndex)
    {
        return false;
    }
    CRTPacket::ETimecodeType timecodeType;
    if (GetTimecodeType(nTimecodeIndex, timecodeType))
    {
        if (timecodeType == TimecodeIRIG)
        {
            year        = 0x007f & SetByteOrder((unsigned int*)(mpTimecodeData[nTimecodeIndex] + 4));
            day         = 0x01ff & (SetByteOrder((unsigned int*)(mpTimecodeData[nTimecodeIndex] + 4)) >> 7);
            hours       = 0x001f & SetByteOrder((unsigned int*)(mpTimecodeData[nTimecodeIndex] + 8));
            minutes     = 0x003f & (SetByteOrder((unsigned int*)(mpTimecodeData[nTimecodeIndex] + 8)) >> 5);
            seconds     = 0x003f & (SetByteOrder((unsigned int*)(mpTimecodeData[nTimecodeIndex] + 8)) >> 11);
            tenths      = 0x000f & (SetByteOrder((unsigned int*)(mpTimecodeData[nTimecodeIndex] + 8)) >> 17);
            return true;
        }
    }
    return false;
}

bool CRTPacket::GetTimecodeCameraTime(unsigned int nTimecodeIndex, unsigned long long &cameraTime)
{
    if (mnTimecodeCount <= nTimecodeIndex)
    {
        return false;
    }
    CRTPacket::ETimecodeType timecodeType;
    if (GetTimecodeType(nTimecodeIndex, timecodeType))
    {
        if (timecodeType == TimecodeCamerTime)
        {
            cameraTime = ((long long)SetByteOrder((unsigned int*)(mpTimecodeData[nTimecodeIndex] + 4))) << 32 |
                          (long long)SetByteOrder((unsigned int*)(mpTimecodeData[nTimecodeIndex] + 8));
            return true;
        }
    }
    return false;
}


//-----------------------------------------------------------
//                          Image
//-----------------------------------------------------------
unsigned int CRTPacket::GetImageCameraCount()
{
    return mnImageCameraCount;
}

unsigned int CRTPacket::GetImageCameraId(unsigned int nCameraIndex)
{
    if (mnImageCameraCount <= nCameraIndex)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(mpImageData[nCameraIndex]));
}

bool CRTPacket::GetImageFormat(unsigned int nCameraIndex, EImageFormat &eImageFormat)
{
    if (mnImageCameraCount <= nCameraIndex)
    {
        return false;
    }
    eImageFormat = (EImageFormat)SetByteOrder((unsigned int*)(mpImageData[nCameraIndex] + 4));

    return true;
}

bool CRTPacket::GetImageSize(unsigned int nCameraIndex, unsigned int &nWidth, unsigned int &nHeight)
{
    if (mnImageCameraCount <= nCameraIndex)
    {
        return false;
    }
    nWidth  = SetByteOrder((unsigned int*)(mpImageData[nCameraIndex] + 8));
    nHeight = SetByteOrder((unsigned int*)(mpImageData[nCameraIndex] + 12));

    return true;
}

bool CRTPacket::GetImageCrop(unsigned int nCameraIndex, float &fCropLeft, float &fCropTop,
                             float &fCropRight, float &fCropBottom)
{
    if (mnImageCameraCount <= nCameraIndex)
    {
        return false;
    }
    fCropLeft   = SetByteOrder((float*)(mpImageData[nCameraIndex] + 16));
    fCropTop    = SetByteOrder((float*)(mpImageData[nCameraIndex] + 20));
    fCropRight  = SetByteOrder((float*)(mpImageData[nCameraIndex] + 24));
    fCropBottom = SetByteOrder((float*)(mpImageData[nCameraIndex] + 28));

    return true;
}

unsigned int CRTPacket::GetImageSize(unsigned int nCameraIndex)
{
    if (((mnMajorVersion == 1) && (mnMinorVersion < 8)) || mnImageCameraCount <= nCameraIndex)
    {
        return 0;
    }

    return SetByteOrder((unsigned int*)(mpImageData[nCameraIndex] + 32));
}

unsigned int CRTPacket::GetImage(unsigned int nCameraIndex, char* pDataBuf, unsigned int nBufSize)
{
    if (((mnMajorVersion == 1) && (mnMinorVersion < 8)) || mnImageCameraCount <= nCameraIndex)
    {
        return 0;
    }

    unsigned int nSize = SetByteOrder((unsigned int*)(mpImageData[nCameraIndex] + 32));

    if (nBufSize < nSize)
    {
        return 0;
    }
    memcpy(pDataBuf, mpImageData[nCameraIndex] + 36, nSize);

    return nSize;
}


//-----------------------------------------------------------
//                          Analog
//-----------------------------------------------------------
unsigned int CRTPacket::GetAnalogDeviceCount()
{
    return mnAnalogDeviceCount;
}

unsigned int CRTPacket::GetAnalogDeviceId(unsigned int nDeviceIndex)
{
    if ((mnMajorVersion == 1) && (mnMinorVersion == 0))
    {
        return 1;
    }
    if (mnAnalogDeviceCount <= nDeviceIndex)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(mpAnalogData[nDeviceIndex]));
}

unsigned int CRTPacket::GetAnalogChannelCount(unsigned int nDeviceIndex)
{
    char* pData = mpComponentData[ComponentAnalog - 1];

    if ((mnMajorVersion == 1) && (mnMinorVersion == 0))
    {
        return SetByteOrder((unsigned int*)(pData + 8));
    }
    if (mnAnalogDeviceCount <= nDeviceIndex)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(mpAnalogData[nDeviceIndex] + 4));
}

unsigned int CRTPacket::GetAnalogSampleCount(unsigned int nDeviceIndex)
{
    if ((mnMajorVersion == 1) && (mnMinorVersion == 0))
    {
        return 1;
    }
    if (mnAnalogDeviceCount <= nDeviceIndex)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(mpAnalogData[nDeviceIndex] + 8));
}

unsigned int CRTPacket::GetAnalogSampleNumber(unsigned int nDeviceIndex)
{
    if ((mnMajorVersion == 1) && (mnMinorVersion == 0))
    {
        return GetFrameNumber();
    }

    if (mnAnalogDeviceCount <= nDeviceIndex)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(mpAnalogData[nDeviceIndex] + 12));
}

unsigned int CRTPacket::GetAnalogData(unsigned int nDeviceIndex, float* pDataBuf, unsigned int nBufSize)
{
    unsigned int nSize = 0;

    if (nDeviceIndex < mnAnalogDeviceCount)
    {
        unsigned int nChannelCount = GetAnalogChannelCount(nDeviceIndex);

        if ((mnMajorVersion == 1) && (mnMinorVersion == 0))
        {
            nSize = nChannelCount;
            if (nBufSize < nSize || pDataBuf == nullptr)
            {
                nSize = 0;
            }
            for (unsigned int i = 0; i < nSize; i++)
            {
                pDataBuf[i] = (float)SetByteOrder((double*)(mpAnalogData[nDeviceIndex] + i * sizeof(double)));
            }
        }
        else
        {
            nSize = nChannelCount * GetAnalogSampleCount(nDeviceIndex);
            if (nBufSize < nSize || pDataBuf == nullptr)
            {
                nSize = 0;
            }
            for (unsigned int i = 0; i < nSize; i++)
            {
                pDataBuf[i] = (float)SetByteOrder((float*)(mpAnalogData[nDeviceIndex] + 16 + i * sizeof(float)));
            }
        }
    }

    return nSize;
}

unsigned int CRTPacket::GetAnalogData(unsigned int nDeviceIndex, unsigned int nChannelIndex, float* pDataBuf, unsigned int nBufSize)
{
    unsigned int nSampleCount = 0;
    unsigned int nChannelCount = GetAnalogChannelCount(nDeviceIndex);

    if (nDeviceIndex < mnAnalogDeviceCount && nChannelIndex < nChannelCount)
    {
        if ((mnMajorVersion == 1) && (mnMinorVersion == 0))
        {
            if (nBufSize == 0 || pDataBuf == nullptr)
            {
                nSampleCount = 0;
            }
            else
            {
                nSampleCount = 1;
                pDataBuf[0] = (float)SetByteOrder((double*)(mpAnalogData[nDeviceIndex] + nChannelIndex * sizeof(double)));
            }
        }
        else
        {
            nSampleCount = GetAnalogSampleCount(nDeviceIndex);
            if (nBufSize < nSampleCount || pDataBuf == nullptr)
            {
                nSampleCount = 0;
            }
            for (unsigned int i = 0; i < nSampleCount; i++)
            {
                pDataBuf[i] = (float)SetByteOrder((float*)(mpAnalogData[nDeviceIndex] + 16 +
                               nChannelIndex * nSampleCount * sizeof(float) + i * sizeof(float)));
            }
        }
    }

    return nSampleCount;
}

bool CRTPacket::GetAnalogData(unsigned int nDeviceIndex, unsigned int nChannelIndex, unsigned int nSampleIndex,
                              float &fAnalogValue)
{
    if (nDeviceIndex < mnAnalogDeviceCount)
    {
        unsigned int nSampleCount = GetAnalogSampleCount(nDeviceIndex);

        if (GetAnalogChannelCount(nDeviceIndex) > nChannelIndex &&
            nSampleCount  > nSampleIndex)
        {
            if ((mnMajorVersion == 1) && (mnMinorVersion == 0))
            {
                fAnalogValue = (float)SetByteOrder((double*)(mpAnalogData[nDeviceIndex] + nChannelIndex * sizeof(double)));
            }
            else
            {
                fAnalogValue = SetByteOrder((float*)(mpAnalogData[nDeviceIndex] + 16 +
                               (nChannelIndex * nSampleCount + nSampleIndex) * sizeof(float)));
            }
            if (isnan(fAnalogValue) == 0)
            {
                return true;
            }
        }
    }
    return false;
}


//-----------------------------------------------------------
//                       Analog Single
//-----------------------------------------------------------
unsigned int CRTPacket::GetAnalogSingleDeviceCount()
{
    return mnAnalogSingleDeviceCount;
}

unsigned int CRTPacket::GetAnalogSingleDeviceId(unsigned int nDeviceIndex)
{
    if (mnAnalogSingleDeviceCount <= nDeviceIndex)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(mpAnalogSingleData[nDeviceIndex]));
}

unsigned int CRTPacket::GetAnalogSingleChannelCount(unsigned int nDeviceIndex)
{
    if (mnAnalogSingleDeviceCount <= nDeviceIndex)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(mpAnalogSingleData[nDeviceIndex] + 4));
}

unsigned int CRTPacket::GetAnalogSingleData(unsigned int nDeviceIndex, float* pDataBuf, unsigned int nBufSize)
{
    unsigned int nSize = 0;

    if (nDeviceIndex < mnAnalogSingleDeviceCount)
    {
        nSize = GetAnalogSingleChannelCount(nDeviceIndex);
        if (nBufSize < nSize || pDataBuf == nullptr)
        {
            nSize = 0;
        }
        for (unsigned int i = 0; i < nSize; i++)
        {
            pDataBuf[i] = SetByteOrder((float*)(mpAnalogSingleData[nDeviceIndex] + 8 + i * sizeof(float)));
        }
    }

    return nSize;
}

bool CRTPacket::GetAnalogSingleData(unsigned int nDeviceIndex, unsigned int nChannelIndex, float &fValue)
{
    if (nDeviceIndex < mnAnalogSingleDeviceCount)
    {
        if (nChannelIndex < GetAnalogSingleChannelCount(nDeviceIndex))
        {
            fValue = SetByteOrder(((float*)(mpAnalogSingleData[nDeviceIndex] + 8 + nChannelIndex * sizeof(float))));
            return (isnan(fValue) == 0);
        }
    }
    return false;
}


//-----------------------------------------------------------
//                          Force
//-----------------------------------------------------------
unsigned int CRTPacket::GetForcePlateCount()
{
    return mnForcePlateCount;
}

unsigned int CRTPacket::GetForcePlateId(unsigned int nPlateIndex)
{
    if ((mnMajorVersion == 1 && mnMinorVersion == 0) || mnForcePlateCount <= nPlateIndex)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(mpForceData[nPlateIndex]));
}

unsigned int CRTPacket::GetForceCount(unsigned int nPlateIndex)
{
    if (mnForcePlateCount <= nPlateIndex)
    {
        return 0;
    }
    if ((mnMajorVersion == 1) && (mnMinorVersion == 0))
    {
        return 1;
    }
    return SetByteOrder((unsigned int*)(mpForceData[nPlateIndex] + 4));
}

unsigned int CRTPacket::GetForceNumber(unsigned int nPlateIndex)
{
    if (mnForcePlateCount <= nPlateIndex)
    {
        return 0;
    }
    if ((mnMajorVersion == 1) && (mnMinorVersion == 0))
    {
        return GetFrameNumber();
    }
    return SetByteOrder((unsigned int*)(mpForceData[nPlateIndex] + 8));
}

unsigned int CRTPacket::GetForceData(unsigned int nPlateIndex, SForce* pForceBuf, unsigned int nBufSize)
{
    unsigned int nSize = 0;

    if (nPlateIndex < mnForcePlateCount)
    {
        if ((mnMajorVersion == 1) && (mnMinorVersion == 0))
        {
            if (nPlateIndex == 0)
            {
                for (unsigned int k = 0; k < 9; k++)
                {
                    *(((float*)pForceBuf) + k) =
                        (float)SetByteOrder((double*)(mpForceData[nPlateIndex] + k * sizeof(double)));
                }
                nSize = 1;
            }
        }
        else
        {
            nSize = GetForceCount(nPlateIndex);
            if (nBufSize < nSize || pForceBuf == nullptr)
            {
                nSize = 0;
            }
            for (unsigned int i = 0; i < nSize; i++)
            {
                for (unsigned int k = 0; k < 9; k++)
                {
                    *(((float*)&pForceBuf[i]) + k) =
                        SetByteOrder((float*)(mpForceData[nPlateIndex] + 12 + (k * 4) + i * sizeof(SForce)));
                }
            }
        }
    }
    return nSize;
}

bool CRTPacket::GetForceData(unsigned int nPlateIndex, unsigned int nForceIndex, SForce &sForce)
{
    if (nPlateIndex < mnForcePlateCount)
    {
        if ((mnMajorVersion == 1) && (mnMinorVersion == 0))
        {
            if (nPlateIndex == 0 && nForceIndex == 0)
            {
                for (unsigned int k = 0; k < 9; k++)
                {
                    *(((float*)&sForce) + k) =
                        (float)SetByteOrder((double*)(mpForceData[nPlateIndex] + k * sizeof(double)));

                    // Not a valid force if one of the values is not a valid float.
                    if (isnan(*(((float*)&sForce) + k)) != 0)
                    {
                        return false; 
                    }
                }
                return true;
            }
        }
        else
        {
            if (nForceIndex < GetForceCount(nPlateIndex))
            {
                for (unsigned int k = 0; k < 9; k++)
                {
                    *(((float*)&sForce) + k) =
                        SetByteOrder((float*)(mpForceData[nPlateIndex] + 12 + k * sizeof(float) + nForceIndex * sizeof(SForce)));

                    // Not a valid force if one of the values is not a valid float.
                    if (isnan(*(((float*)&sForce) + k)) != 0)
                    {
                        return false; 
                    }
                }
                return true;
            }
        }
    }
    return false;
}


//-----------------------------------------------------------
//                         Skeleton
//-----------------------------------------------------------
unsigned int CRTPacket::GetSkeletonCount()
{
    return mSkeletonCount;
}

unsigned int CRTPacket::GetSkeletonSegmentCount(unsigned int nSkeletonIndex)
{
    if (mSkeletonCount <= nSkeletonIndex)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(mpSkeletonData[nSkeletonIndex]));
}

bool CRTPacket::GetSkeletonSegments(unsigned int nSkeletonIndex, SSkeletonSegment* segmentBuffer, unsigned int nBufSize)
{
    if (mSkeletonCount <= nSkeletonIndex)
    {
        return false;
    }

    unsigned int segmentCount = GetSkeletonSegmentCount(nSkeletonIndex);
    if (segmentCount == 0)
    {
        return false;
    }

    if (nBufSize < segmentCount * 32 || segmentBuffer == nullptr)
    {
        segmentCount = 0;
        return false;
    }

    if (mbBigEndian)
    {
        for (unsigned int i = 0; i < segmentCount; i++)
        {
            segmentBuffer[i].id = SetByteOrder((unsigned int*)(mpSkeletonData[nSkeletonIndex] + 4 + i * 32));
            segmentBuffer[i].positionX = SetByteOrder((float*)(mpSkeletonData[nSkeletonIndex] + 8 + i * 32));
            segmentBuffer[i].positionY = SetByteOrder((float*)(mpSkeletonData[nSkeletonIndex] + 12 + i * 32));
            segmentBuffer[i].positionZ = SetByteOrder((float*)(mpSkeletonData[nSkeletonIndex] + 16 + i * 32));
            segmentBuffer[i].rotationX = SetByteOrder((float*)(mpSkeletonData[nSkeletonIndex] + 20 + i * 32));
            segmentBuffer[i].rotationY = SetByteOrder((float*)(mpSkeletonData[nSkeletonIndex] + 24 + i * 32));
            segmentBuffer[i].rotationZ = SetByteOrder((float*)(mpSkeletonData[nSkeletonIndex] + 28 + i * 32));
            segmentBuffer[i].rotationW = SetByteOrder((float*)(mpSkeletonData[nSkeletonIndex] + 32 + i * 32));
        }
    }
    else
    {
        memcpy(segmentBuffer, mpSkeletonData[nSkeletonIndex] + 4, sizeof(SSkeletonSegment) * segmentCount);
    }
    return true;
}

bool CRTPacket::GetSkeletonSegment(unsigned int nSkeletonIndex, unsigned segmentIndex, SSkeletonSegment &segment)
{
    if (mSkeletonCount <= nSkeletonIndex)
    {
        return false;
    }

    unsigned int segmentCount = GetSkeletonSegmentCount(nSkeletonIndex);
    if (segmentCount == 0)
    {
        return false;
    }

    if (segmentIndex >= segmentCount)
    {
        return false;
    }

    if (mbBigEndian)
    {
        segment.id = SetByteOrder((unsigned int*)(mpSkeletonData[nSkeletonIndex] + 4 + 32 * segmentIndex));
        segment.positionX = SetByteOrder((float*)(mpSkeletonData[nSkeletonIndex] + 8 + 32 * segmentIndex));
        segment.positionY = SetByteOrder((float*)(mpSkeletonData[nSkeletonIndex] + 12 + 32 * segmentIndex));
        segment.positionZ = SetByteOrder((float*)(mpSkeletonData[nSkeletonIndex] + 16 + 32 * segmentIndex));
        segment.rotationX = SetByteOrder((float*)(mpSkeletonData[nSkeletonIndex] + 20 + 32 * segmentIndex));
        segment.rotationY = SetByteOrder((float*)(mpSkeletonData[nSkeletonIndex] + 24 + 32 * segmentIndex));
        segment.rotationZ = SetByteOrder((float*)(mpSkeletonData[nSkeletonIndex] + 28 + 32 * segmentIndex));
        segment.rotationW = SetByteOrder((float*)(mpSkeletonData[nSkeletonIndex] + 32 + 32 * segmentIndex));
    }
    else
    {
        memcpy(&segment, mpSkeletonData[nSkeletonIndex] + 4 + 32 * segmentIndex, sizeof(SSkeletonSegment));
    }
    
    return true;
}


//-----------------------------------------------------------
//                       Force Single
//-----------------------------------------------------------
unsigned int CRTPacket::GetForceSinglePlateCount()
{
    return mnForceSinglePlateCount;
}

unsigned int CRTPacket::GetForceSinglePlateId(unsigned int nPlateIndex)
{
    if ((mnMajorVersion == 1 && mnMinorVersion == 0) || mnForceSinglePlateCount <= nPlateIndex)
    {
        return 0;
    }
    return SetByteOrder((unsigned int*)(mpForceSingleData[nPlateIndex]));
}

bool CRTPacket::GetForceSingleData(unsigned int nPlateIndex, SForce &sForce)
{
    if (nPlateIndex < mnForceSinglePlateCount)
    {
        for (unsigned int k = 0; k < 9; k++)
        {
            *(((float*)&sForce) + k) =
                SetByteOrder((float*)(mpForceSingleData[nPlateIndex] + 4 + k * sizeof(float)));

            // Not a valid force if one of the values is not a valid float.
            if (isnan(*(((float*)&sForce) + k)) != 0)
            {
                return false; 
            }
        }
        return true;
    }

    return false;
}

float CRTPacket::SetByteOrder(float* pfData)
{
    unsigned int nTmp;

    if (mbBigEndian)
    {
        nTmp = ntohl(*((unsigned int*)pfData));
        return *((float*)&nTmp);
    }
    return *pfData;
} // SetByteOrder

double CRTPacket::SetByteOrder(double* pfData)
{
    unsigned long long nTmp;

    if (mbBigEndian)
    {
        nTmp = (((unsigned long long)(ntohl((long)*((unsigned long long*)pfData))) << 32) + ntohl(*((unsigned long long*)pfData) >> 32));
        return *((double*)&nTmp);
    }
    return *pfData;
} // SetByteOrder

short CRTPacket::SetByteOrder(short* pnData)
{
    if (mbBigEndian)
    {
        return ntohs(*pnData);
    }
    return *pnData;
} // SetByteOrder

unsigned short CRTPacket::SetByteOrder(unsigned short* pnData)
{
    if (mbBigEndian)
    {
        return ntohs(*pnData);
    }
    return *pnData;
} // SetByteOrder

long CRTPacket::SetByteOrder(long* pnData)
{
    if (mbBigEndian)
    {
        return ntohl(*pnData);
    }
    return *pnData;
} // SetByteOrder

int CRTPacket::SetByteOrder(int* pnData)
{
    if (mbBigEndian)
    {
        return ntohl(*pnData);
    }
    return *pnData;
} // SetByteOrder

unsigned int CRTPacket::SetByteOrder(unsigned int* pnData)
{
    if (mbBigEndian)
    {
        return ntohl(*pnData);
    }
    return *pnData;
} // SetByteOrder

long long CRTPacket::SetByteOrder(long long* pnData)
{
    if (mbBigEndian)
    {
        return ((unsigned long long)(ntohl((long)*pnData)) << 32) + ntohl(*pnData >> 32);
    }
    return *pnData;
} // SetByteOrder

unsigned long long CRTPacket::SetByteOrder(unsigned long long* pnData)
{
    if (mbBigEndian)
    {
        return ((unsigned long long)(ntohl((long)*pnData)) << 32) + ntohl(*pnData >> 32);
    }
    return *pnData;
} // SetByteOrder