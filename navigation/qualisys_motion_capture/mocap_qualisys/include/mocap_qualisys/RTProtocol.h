#ifndef RTPROTOCOL_H
#define RTPROTOCOL_H


#include "RTPacket.h"
#include "Network.h"
#include <vector>
#include <string>
#include <map>

#ifdef _WIN32
#pragma warning (disable : 4251)
#endif


#ifdef EXPORT_DLL
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT
#endif


//#define DEFAULT_AUTO_DESCOVER_PORT 22226
//#define WAIT_FOR_DATA_TIMEOUT      5000000 // 5 s

class CMarkup;

class DLL_EXPORT CRTProtocol
{
public:
    static const unsigned int cDefaultBasePort          = 22222;
    static const unsigned int cDefaultTelnetPort        = cDefaultBasePort - 1;
    static const unsigned int cDefaultLegacyPort        = cDefaultBasePort;     // Only supports version 1.0 of the protocol.
    static const unsigned int cDefaultLittleEndianPort  = cDefaultBasePort + 1;
    static const unsigned int cDefaultBigEndianPort     = cDefaultBasePort + 2;
    static const unsigned int cDefaultOscPort           = cDefaultBasePort + 3;
    static const unsigned int cDefaultAutoDescoverPort  = cDefaultBasePort + 4;

    static const unsigned int cWaitForDataTimeout        = 5000000;   // 5 s
    static const unsigned int cWaitForCalibrationTimeout = 600000000; // 10 min

    static const unsigned int cComponent3d            = 0x000001;
    static const unsigned int cComponent3dNoLabels    = 0x000002;
    static const unsigned int cComponentAnalog        = 0x000004;
    static const unsigned int cComponentForce         = 0x000008;
    static const unsigned int cComponent6d            = 0x000010;
    static const unsigned int cComponent6dEuler       = 0x000020;
    static const unsigned int cComponent2d            = 0x000040;
    static const unsigned int cComponent2dLin         = 0x000080;
    static const unsigned int cComponent3dRes         = 0x000100;
    static const unsigned int cComponent3dNoLabelsRes = 0x000200;
    static const unsigned int cComponent6dRes         = 0x000400;
    static const unsigned int cComponent6dEulerRes    = 0x000800;
    static const unsigned int cComponentAnalogSingle  = 0x001000;
    static const unsigned int cComponentImage         = 0x002000;
    static const unsigned int cComponentForceSingle   = 0x004000;
    static const unsigned int cComponentGazeVector    = 0x008000;
    static const unsigned int cComponentTimecode      = 0x010000;
    static const unsigned int cComponentSkeleton      = 0x020000;

    struct SComponentOptions
    {
		SComponentOptions() :
			mAnalogChannels(nullptr),
			mSkeletonGlobalData(false)
		{
		}
        char* mAnalogChannels;
        bool mSkeletonGlobalData;
    };

    enum EStreamRate
    {
        RateNone             = 0,
        RateAllFrames        = 1,
        RateFrequency        = 2,
        RateFrequencyDivisor = 3
    };

    enum ECameraModel
    {
        ModelMacReflex       = 0,
        ModelProReflex120    = 1,
        ModelProReflex240    = 2,
        ModelProReflex500    = 3,
        ModelProReflex1000   = 4,
        ModelOqus100         = 5,
        ModelOqus300         = 6,
        ModelOqus300Plus     = 7,
        ModelOqus400         = 8,
        ModelOqus500         = 9,
        ModelOqus200C        = 10,
        ModelOqus500Plus     = 11,
        ModelOqus700         = 12,
        ModelOqus700Plus     = 13,
        ModelOqus600Plus     = 14,
        ModelMiqusM1         = 15,
        ModelMiqusM3         = 16,
        ModelMiqusM5         = 17,
        ModelMiqusSyncUnit   = 18,
        ModelMiqusVideo      = 19,
        ModelMiqusVideoColor = 20
    };

    enum ECameraMode
    {
        ModeMarker          = 0,
        ModeMarkerIntensity = 1,
        ModeVideo           = 2
    };

    enum EVideoResolution
    {
        VideoResolution1080p = 0,
        VideoResolution720p = 1,
        VideoResolution540p = 2,
        VideoResolution480p = 3,
        VideoResolutionNone = 4
    };

    enum EVideoAspectRatio
    {
        VideoAspectRatio16x9 = 0,
        VideoAspectRatio4x3 = 1,
        VideoAspectRatio1x1 = 2,
        VideoAspectRatioNone = 3
    };

    enum ESyncOutFreqMode
    {
        ModeShutterOut = 1, // A pulse per frame is sent
        ModeMultiplier,
        ModeDivisor,
        ModeIndependentFreq,
        ModeMeasurementTime,
        ModeFixed100Hz
    };

    enum ESignalSource
    {
        SourceControlPort = 0,
        SourceIRReceiver  = 1,
        SourceSMPTE       = 2,
        SourceVideoSync   = 3,
        SourceIRIG        = 4
    };

    enum EAxis
    {
        XPos = 0,
        XNeg = 1,
        YPos = 2,
        YNeg = 3,
        ZPos = 4,
        ZNeg = 5
    };

    enum EProcessingActions
    {
        ProcessingNone               = 0x0000,
        ProcessingTracking2D         = 0x0001,
        ProcessingTracking3D         = 0x0002,
        ProcessingTwinSystemMerge    = 0x0004,
        ProcessingSplineFill         = 0x0008,
        ProcessingAIM                = 0x0010,
        Processing6DOFTracking       = 0x0020,
        ProcessingForceData          = 0x0040,
        ProcessingGazeVector         = 0x0080,
        ProcessingExportTSV          = 0x0100,
        ProcessingExportC3D          = 0x0200,
        ProcessingExportMatlabFile   = 0x0800,
        ProcessingExportAviFile      = 0x1000,
        ProcessingPreProcess2D       = 0x2000
    };

    struct SPoint
    {
        float fX;
        float fY;
        float fZ;
    };

    struct SDiscoverResponse
    {
        char           message[128];
        unsigned int   nAddr;
        unsigned short nBasePort;
    };


    unsigned int GetSystemFrequency() const;
public:
    struct SSettingsGeneralCamera
    {
        unsigned int nID;
        ECameraModel eModel;
        bool         bUnderwater;
        bool         bSupportsHwSync;
        unsigned int nSerial;
        ECameraMode  eMode;
        EVideoResolution eVideoResolution;
        EVideoAspectRatio eVideoAspectRatio;
        unsigned int nVideoFrequency;
        unsigned int nVideoExposure;      // Micro seconds
        unsigned int nVideoExposureMin;   // Micro seconds
        unsigned int nVideoExposureMax;   // Micro seconds
        unsigned int nVideoFlashTime;     // Micro seconds
        unsigned int nVideoFlashTimeMin;  // Micro seconds
        unsigned int nVideoFlashTimeMax;  // Micro seconds
        unsigned int nMarkerExposure;     // Micro seconds
        unsigned int nMarkerExposureMin;  // Micro seconds
        unsigned int nMarkerExposureMax;  // Micro seconds
        unsigned int nMarkerThreshold;   
        unsigned int nMarkerThresholdMin;
        unsigned int nMarkerThresholdMax;
        float        fPositionX;
        float        fPositionY;
        float        fPositionZ;
        float        fPositionRotMatrix[3][3];
        unsigned int nOrientation;             // Degrees
        unsigned int nMarkerResolutionWidth;   // Sub pixels
        unsigned int nMarkerResolutionHeight;  // Sub pixels
        unsigned int nVideoResolutionWidth;    // Pixels
        unsigned int nVideoResolutionHeight;   // Pixels
        unsigned int nMarkerFOVLeft;           // Pixels
        unsigned int nMarkerFOVTop;            // Pixels
        unsigned int nMarkerFOVRight;          // Pixels
        unsigned int nMarkerFOVBottom;         // Pixels
        unsigned int nVideoFOVLeft;            // Pixels
        unsigned int nVideoFOVTop;             // Pixels
        unsigned int nVideoFOVRight;           // Pixels
        unsigned int nVideoFOVBottom;          // Pixels
        ESyncOutFreqMode eSyncOutMode[2];
        unsigned int nSyncOutValue[2];
        float        fSyncOutDutyCycle[2];     // Percent
        bool         bSyncOutNegativePolarity[3];
        float        fFocus;
        float        fAperture;
		bool         autoExposureEnabled;
		float        autoExposureCompensation;
        int          autoWhiteBalance;
    };

    struct SSettingsGeneralExternalTimebase
    {
        SSettingsGeneralExternalTimebase():
            bEnabled(false),
            eSignalSource(SourceControlPort),
            bSignalModePeriodic(false),
            nFreqMultiplier(0),
            nFreqDivisor(0),
            nFreqTolerance(0),
            fNominalFrequency(0.0f),
            bNegativeEdge(false),
            nSignalShutterDelay(0),
            fNonPeriodicTimeout(0.0f)
        {
        }

        bool          bEnabled;
        ESignalSource eSignalSource;
        bool          bSignalModePeriodic;
        unsigned int  nFreqMultiplier;
        unsigned int  nFreqDivisor;
        unsigned int  nFreqTolerance;
        float         fNominalFrequency;
        bool          bNegativeEdge;
        unsigned int  nSignalShutterDelay;
        float         fNonPeriodicTimeout;
    };

    struct SSettingsGeneral
    {
        SSettingsGeneral():
            nCaptureFrequency(0),
            fCaptureTime(0.0f),
            bStartOnExternalTrigger(false),
            bStartOnTrigNO(false),
            bStartOnTrigNC(false),
            bStartOnTrigSoftware(false),
            eProcessingActions(EProcessingActions::ProcessingNone),
            eRtProcessingActions(EProcessingActions::ProcessingNone),
            eReprocessingActions(EProcessingActions::ProcessingNone)
        {
            sExternalTimebase = { };
        }

        unsigned int nCaptureFrequency;
        float fCaptureTime;
        bool bStartOnExternalTrigger;
        bool bStartOnTrigNO;
        bool bStartOnTrigNC;
        bool bStartOnTrigSoftware;
        SSettingsGeneralExternalTimebase sExternalTimebase;
        EProcessingActions eProcessingActions;   // Binary flags.
        EProcessingActions eRtProcessingActions; // Binary flags.
        EProcessingActions eReprocessingActions; // Binary flags.
        std::vector< SSettingsGeneralCamera > vsCameras;
    };

    struct SSettings3DLabel
    {
        std::string  oName;
        unsigned int nRGBColor;
    };

    struct SSettingsBone
    {
        std::string fromName;
        std::string toName;
        unsigned int color;
    };

    struct SSettings3D
    {
        EAxis                           eAxisUpwards;
        char                            pCalibrationTime[32];
        std::vector< SSettings3DLabel > s3DLabels;
        std::vector< SSettingsBone >    sBones;
    };

    struct SSettings6DOFBody
    {
        std::string           oName;
        unsigned int          nRGBColor;
        std::vector< SPoint > vsPoints;
    };

    struct SSettings6DOF
    {
        std::vector<SSettings6DOFBody> bodySettings;
        std::string                    eulerFirst;
        std::string                    eulerSecond;
        std::string                    eulerThird;
    };

    struct SGazeVector
    {
        std::string    name;
        float          frequency;
    };

    struct SAnalogDevice 
    {
        unsigned int               nDeviceID;
        unsigned int               nChannels;
        std::string                oName;
        std::vector< std::string > voLabels;
        unsigned int               nFrequency;
        std::string                oUnit;
        float                      fMinRange;
        float                      fMaxRange;        
        std::vector< std::string > voUnits;
   };

    struct SForceChannel
    {
        unsigned int nChannelNumber;
        float        fConversionFactor;
    };

    struct SForcePlate 
    {
        unsigned int                 nID;
        unsigned int                 nAnalogDeviceID;
        std::string                  oType;
        std::string                  oName;
        unsigned int                 nFrequency;
        float                        fLength;
        float                        fWidth;
        SPoint                       asCorner[4];
        SPoint                       sOrigin;
        std::vector< SForceChannel > vChannels;
        bool                         bValidCalibrationMatrix;
        float                        afCalibrationMatrix[12][12];
        unsigned int                 nCalibrationMatrixRows;
        unsigned int                 nCalibrationMatrixColumns;
    };

    struct SSettingsForce
    {
        std::string                 oUnitLength;
        std::string                 oUnitForce;
        std::vector< SForcePlate >  vsForcePlates;
    };

    struct SImageCamera
    {
        unsigned int            nID;
        bool                    bEnabled;
        CRTPacket::EImageFormat eFormat;
        unsigned int            nWidth;
        unsigned int            nHeight;
        float                   fCropLeft;
        float                   fCropTop;
        float                   fCropRight;
        float                   fCropBottom;
    };

    struct SCalibrationFov
    {
        uint32_t left;
        uint32_t top;
        uint32_t right;
        uint32_t bottom;
    };

    struct SCalibrationTransform
    {
        double x;
        double y;
        double z;
        double r11;
        double r12;
        double r13;
        double r21;
        double r22;
        double r23;
        double r31;
        double r32;
        double r33;
    };

    struct SCalibrationIntrinsic
    {
        double focal_length;
        double sensor_min_u;
        double sensor_max_u;
        double sensor_min_v;
        double sensor_max_v;
        double focal_length_u;
        double focal_length_v;
        double center_point_u;
        double center_point_v;
        double skew;
        double radial_distortion_1;
        double radial_distortion_2;
        double radial_distortion_3;
        double tangental_distortion_1;
        double tangental_distortion_2;
    };

    struct SCalibrationCamera
    {
        bool active;
        bool calibrated;
        std::string message;
        uint32_t point_count;
        double avg_residual;
        uint32_t serial;
        std::string model;
        uint32_t view_rotation;
        SCalibrationFov fov_marker;
        SCalibrationFov fov_marker_max;
        SCalibrationFov fov_video;
        SCalibrationFov fov_video_max;
        SCalibrationTransform transform;
        SCalibrationIntrinsic intrinsic;
    };

    struct SCalibration
    {
        bool calibrated = false;
        std::string source = "";
        std::string created = "";
        std::string qtm_version = "";
        std::string type = "";
        double wand_length = 0.0;
        uint32_t max_frames = 0;
        double short_arm_end = 0.0;
        double long_arm_end = 0.0;
        double long_arm_middle = 0.0;
        double result_std_dev = 0.0;
        double result_min_max_diff = 0.0;
        std::vector<SCalibrationCamera> cameras;
    };

public:
    struct SSettingsSkeletonSegment : CRTPacket::SSkeletonSegment
    {
        std::string name;
        int parentId;
        int parentIndex;
    };

    struct SSettingsSkeleton
    {
        std::string name;
        std::vector<SSettingsSkeletonSegment> segments;
    };


public:
    CRTProtocol();
    ~CRTProtocol();

    bool       Connect(const char* pServerAddr, unsigned short nPort, unsigned short* pnUDPServerPort = nullptr,
                       int nMajorVersion = MAJOR_VERSION, int nMinorVersion = MINOR_VERSION, bool bBigEndian = false);
    unsigned short GetUdpServerPort();
    void       Disconnect();
    bool       Connected() const;
    bool       SetVersion(int nMajorVersion, int nMinorVersion);
    bool       GetVersion(unsigned int &nMajorVersion, unsigned int &nMinorVersion);
    bool       GetQTMVersion(char* pVersion, unsigned int nVersionLen);
    bool       GetByteOrder(bool &bBigEndian);
    bool       CheckLicense(const char* pLicenseCode);
    bool       DiscoverRTServer(unsigned short nServerPort, bool bNoLocalResponses, unsigned short nDiscoverPort = cDefaultAutoDescoverPort);
    int        GetNumberOfDiscoverResponses();
    bool       GetDiscoverResponse(unsigned int nIndex, unsigned int &nAddr, unsigned short &nBasePort, std::string& message);

    bool       GetCurrentFrame(const char* components);
    bool       GetCurrentFrame(unsigned int nComponentType, const SComponentOptions& componentOptions = { });
    bool       StreamFrames(EStreamRate eRate, unsigned int nRateArg, unsigned short nUDPPort, const char* pUDPAddr, const char* components);
    bool       StreamFrames(EStreamRate eRate, unsigned int nRateArg, unsigned short nUDPPort, const char* pUDPAddr,
                            unsigned int nComponentType, const SComponentOptions& componentOptions = { });
    bool       StreamFramesStop();
    bool       GetState(CRTPacket::EEvent &eEvent, bool bUpdate = true, int nTimeout = cWaitForDataTimeout);
    bool       GetCapture(const char* pFileName, bool bC3D);
    bool       SendTrig();
    bool       SetQTMEvent(const char* pLabel);
    bool       TakeControl(const char* pPassword = nullptr);
    bool       ReleaseControl();
    bool       IsControlling();
    bool       NewMeasurement();
    bool       CloseMeasurement();
    bool       StartCapture();
    bool       StartRTOnFile();
    bool       StopCapture();
    bool       Calibrate(const bool refine, SCalibration &calibrationResult, int timeout = cWaitForCalibrationTimeout);
    bool       LoadCapture(const char* pFileName);
    bool       SaveCapture(const char* pFileName, bool bOverwrite, char* pNewFileName = nullptr, int nSizeOfNewFileName = 0);
    bool       LoadProject(const char* pFileName);
    bool       Reprocess();

    static bool GetEventString(CRTPacket::EEvent eEvent, char* pStr);
    static bool ConvertRateString(const char* pRate, EStreamRate &eRate, unsigned int &nRateArg);
    static unsigned int ConvertComponentString(const char* pComponentType);
    static bool GetComponentString(char* pComponentStr, unsigned int nComponentType, const SComponentOptions& options = SComponentOptions());
    static std::vector<std::pair<unsigned int, std::string>> GetComponents(const std::string componentsString);
    static bool GetComponent(std::string componentStr, unsigned int &component, std::string &option);

    int        ReceiveRTPacket(CRTPacket::EPacketType &eType, bool bSkipEvents = true, int nTimeout = cWaitForDataTimeout); // nTimeout < 0 : Blocking receive
    

    CRTPacket* GetRTPacket();

    bool ReadXmlBool(CMarkup* xml, const std::string& element, bool& value) const;
    bool ReadCameraSystemSettings();
    bool ReadCalibrationSettings();
    bool Read3DSettings(bool &bDataAvailable);
    bool Read6DOFSettings(bool &bDataAvailable);
    bool ReadGazeVectorSettings(bool &bDataAvailable);
    bool ReadAnalogSettings(bool &bDataAvailable);
    bool ReadForceSettings(bool &bDataAvailable);
    bool ReadImageSettings(bool &bDataAvailable);
    bool ReadSkeletonSettings(bool &bDataAvailable, bool skeletonGlobalData = false);

    void GetSystemSettings(
        unsigned int &nCaptureFrequency, float &fCaptureTime,
        bool& bStartOnExtTrig, bool& trigNO, bool& trigNC, bool& trigSoftware,
        EProcessingActions &eProcessingActions, EProcessingActions &eRtProcessingActions, EProcessingActions &eReprocessingActions) const;

    void GetCalibrationSettings(SCalibration &calibrationSettings) const;

    void GetExtTimeBaseSettings(
        bool         &bEnabled,            ESignalSource &eSignalSource,
        bool         &bSignalModePeriodic, unsigned int  &nFreqMultiplier,
        unsigned int &nFreqDivisor,        unsigned int  &nFreqTolerance,
        float        &fNominalFrequency,   bool          &bNegativeEdge,
        unsigned int &nSignalShutterDelay, float         &fNonPeriodicTimeout) const;

    unsigned int GetCameraCount() const;
    std::vector<SSettingsGeneralCamera> GetDevices() const;

    bool GetCameraSettings(
        unsigned int nCameraIndex, unsigned int &nID,     ECameraModel &eModel, 
        bool         &bUnderwater, bool &bSupportsHwSync, unsigned int &nSerial, ECameraMode  &eMode) const;

    bool GetCameraMarkerSettings(
        unsigned int nCameraIndex,       unsigned int &nCurrentExposure,
        unsigned int &nMinExposure,      unsigned int &nMaxExposure,
        unsigned int &nCurrentThreshold, unsigned int &nMinThreshold,
        unsigned int &nMaxThreshold) const;

    bool GetCameraVideoSettings(
        unsigned int nCameraIndex,            EVideoResolution &eVideoResolution,
        EVideoAspectRatio &eVideoAspectRatio, unsigned int &nVideoFrequency,
        unsigned int &nCurrentExposure,       unsigned int &nMinExposure,
        unsigned int &nMaxExposure,           unsigned int &nCurrentFlashTime,
        unsigned int &nMinFlashTime,          unsigned int &nMaxFlashTime) const;

    bool GetCameraSyncOutSettings(
        unsigned int nCameraIndex,   unsigned int portNumber, ESyncOutFreqMode &eSyncOutMode,
        unsigned int &nSyncOutValue, float        &fSyncOutDutyCycle,
        bool         &bSyncOutNegativePolarity) const;

    bool GetCameraPosition(
        unsigned int nCameraIndex, SPoint &sPoint, float fvRotationMatrix[3][3]) const;

    bool GetCameraOrientation(
        unsigned int nCameraIndex, int &nOrientation) const;

    bool GetCameraResolution(
        unsigned int nCameraIndex,   unsigned int &nMarkerWidth,
        unsigned int &nMarkerHeight, unsigned int &nVideoWidth,
        unsigned int &nVideoHeight) const;

    bool GetCameraFOV(
        unsigned int nCameraIndex,  unsigned int &nMarkerLeft,  unsigned int &nMarkerTop,
        unsigned int &nMarkerRight, unsigned int &nMarkerBottom,
        unsigned int &nVideoLeft,   unsigned int &nVideoTop,
        unsigned int &nVideoRight,  unsigned int &nVideoBottom) const;

    bool GetCameraLensControlSettings(const unsigned int nCameraIndex, float* focus, float* aperture) const;
	bool GetCameraAutoExposureSettings(const unsigned int nCameraIndex, bool* autoExposureEnabled, float* autoExposureCompensation) const;
    bool GetCameraAutoWhiteBalance(const unsigned int nCameraIndex, bool* autoWhiteBalanceEnabled) const;

    EAxis        Get3DUpwardAxis() const;
    const char*  Get3DCalibrated() const;
    unsigned int Get3DLabeledMarkerCount() const;
    const char*  Get3DLabelName(unsigned int nMarkerIndex) const;
    unsigned int Get3DLabelColor(unsigned int nMarkerIndex) const;

    unsigned int Get3DBoneCount() const;
    const char*  Get3DBoneFromName(unsigned int boneIndex) const;
    const char*  Get3DBoneToName(unsigned int boneIndex) const;

    void         Get6DOFEulerNames(std::string &first, std::string &second, std::string &third) const;
    unsigned int Get6DOFBodyCount() const;
    const char*  Get6DOFBodyName(unsigned int nBodyIndex) const;
    unsigned int Get6DOFBodyColor(unsigned int nBodyIndex) const;
    unsigned int Get6DOFBodyPointCount(unsigned int nBodyIndex) const;
    bool         Get6DOFBodyPoint(unsigned int nBodyIndex, unsigned int nMarkerIndex, SPoint &sPoint) const;

    unsigned int GetGazeVectorCount() const;
    const char*  GetGazeVectorName(unsigned int nGazeVectorIndex) const;
    float        GetGazeVectorFrequency(unsigned int nGazeVectorIndex) const;

    unsigned int GetAnalogDeviceCount() const;
    bool         GetAnalogDevice(unsigned int nDeviceIndex, unsigned int &nDeviceID, unsigned int &nChannels,
                                 char* &pName, unsigned int &nFrequency, char* &pUnit,
                                 float &fMinRange, float &fMaxRange) const;
    const char*  GetAnalogLabel(unsigned int nDeviceIndex, unsigned int nChannelIndex) const;
    const char*  GetAnalogUnit(unsigned int nDeviceIndex, unsigned int nChannelIndex) const;

    void         GetForceUnits(char* &pLength, char* &pForce) const;
    unsigned int GetForcePlateCount() const;
    bool         GetForcePlate(unsigned int nPlateIndex, unsigned int &nID, unsigned int &nAnalogDeviceID,
                               unsigned int &nFrequency, char* &pType, char* &pName, float &fLength, float &fWidth) const;
    bool         GetForcePlateLocation(unsigned int nPlateIndex, SPoint sCorner[4]) const;
    bool         GetForcePlateOrigin(unsigned int nPlateIndex, SPoint &sOrigin) const;
    unsigned int GetForcePlateChannelCount(unsigned int nPlateIndex) const;
    bool         GetForcePlateChannel(unsigned int nPlateIndex, unsigned int nChannelIndex,
                                      unsigned int &nChannelNumber, float &fConversionFactor) const;
    bool         GetForcePlateCalibrationMatrix(unsigned int nPlateIndex, float fvCalMatrix[12][12], unsigned int* rows, unsigned int* columns) const;

    unsigned int GetImageCameraCount() const;
    bool         GetImageCamera(unsigned int nCameraIndex, unsigned int &nCameraID, bool &bEnabled,
                                CRTPacket::EImageFormat &eFormat, unsigned int &nWidth, unsigned int &nHeight,
                                float &fCropLeft, float &fCropTop, float &fCropRight, float &fCropBottom) const;

    unsigned int GetSkeletonCount() const;
    const char*  GetSkeletonName(unsigned int skeletonIndex);
    unsigned int GetSkeletonSegmentCount(unsigned int skeletonIndex);
    bool         GetSkeleton(unsigned int skeletonIndex, SSettingsSkeleton* skeleton);
    bool         GetSkeletonSegment(unsigned int skeletonIndex, unsigned int segmentIndex, SSettingsSkeletonSegment* segment);// parentIndex = -1 => No parent.


    bool SetSystemSettings(
        const unsigned int* pnCaptureFrequency, const float* pfCaptureTime,
        const bool* pbStartOnExtTrig, const bool* trigNO, const bool* trigNC, const bool* trigSoftware,
        const EProcessingActions* peProcessingActions, const EProcessingActions* peRtProcessingActions, const EProcessingActions* peReprocessingActions);

    bool SetExtTimeBaseSettings(
        const bool*         pbEnabled,            const ESignalSource* peSignalSource,
        const bool*         pbSignalModePeriodic, const unsigned int*  pnFreqMultiplier,
        const unsigned int* pnFreqDivisor,        const unsigned int*  pnFreqTolerance,
        const float*        pfNominalFrequency,   const bool*          pbNegativeEdge,
        const unsigned int* pnSignalShutterDelay, const float*         pfNonPeriodicTimeout);

    bool SetCameraSettings(
        const unsigned int nCameraID,        const ECameraMode* peMode,
        const float*       pfMarkerExposure, const float*       pfMarkerThreshold,
        const int*         pnOrientation);

    bool SetCameraVideoSettings(
        const unsigned int nCameraID,               const EVideoResolution* eVideoResolution,
        const EVideoAspectRatio* eVideoAspectRatio, const unsigned int* pnVideoFrequency,
        const float* pfVideoExposure,               const float* pfVideoFlashTime);

    bool SetCameraSyncOutSettings(
        const unsigned int  nCameraID,      const unsigned int portNumber, const ESyncOutFreqMode* peSyncOutMode,
        const unsigned int* pnSyncOutValue, const float*       pfSyncOutDutyCycle,
        const bool*         pbSyncOutNegativePolarity);

    bool SetCameraLensControlSettings(const unsigned int nCameraID, const float focus, const float aperture);
	bool SetCameraAutoExposureSettings(const unsigned int nCameraID, const bool autoExposure, const float compensation);
    bool SetCameraAutoWhiteBalance(const unsigned int nCameraID, const bool enable);

    bool SetImageSettings(
        const unsigned int  nCameraID, const bool*         pbEnable,    const CRTPacket::EImageFormat* peFormat,
        const unsigned int* pnWidth,   const unsigned int* pnHeight,    const float* pfLeftCrop,
        const float*        pfTopCrop, const float*        pfRightCrop, const float* pfBottomCrop);

    bool SetForceSettings(
        const unsigned int nPlateID,  const SPoint* psCorner1, const SPoint* psCorner2,
        const SPoint*      psCorner3, const SPoint* psCorner4);

    char* GetErrorString();

private:
    bool SendString(const char* pCmdStr, int nType);
    bool SendCommand(const char* pCmdStr);
    bool SendCommand(const char* pCmdStr, char* pCommandResponseStr, unsigned int timeout = cWaitForDataTimeout);
    bool SendXML(const char* pCmdStr);
    void AddXMLElementBool(CMarkup* oXML, const char* tTag, const bool* pbValue, const char* tTrue = "True", const char* tFalse = "False");
    void AddXMLElementBool(CMarkup* oXML, const char* tTag, const bool bValue, const char* tTrue = "True", const char* tFalse = "False");
    void AddXMLElementInt(CMarkup* oXML, const char* tTag, const int* pnValue);
    void AddXMLElementUnsignedInt(CMarkup* oXML, const char* tTag, const unsigned int* pnValue);
    void AddXMLElementFloat(CMarkup* oXML, const char* tTag, const float* pfValue, unsigned int pnDecimals = 6);
    bool CompareNoCase(std::string tStr1, const char* tStr2) const;
    bool ReceiveCalibrationSettings(int timeout = cWaitForDataTimeout);

private:
    CNetwork*                     mpoNetwork;
    CRTPacket*                    mpoRTPacket;
    char*                         maDataBuff;
	unsigned int				  mDataBuffSize;
    CRTPacket::EEvent             meLastEvent;
    CRTPacket::EEvent             meState;  // Same as meLastEvent but without EventCameraSettingsChanged
    int                           mnMinorVersion;
    int                           mnMajorVersion;
    bool                          mbBigEndian;
    bool                          mbIsMaster;
    SSettingsGeneral              msGeneralSettings;
    SSettings3D                   ms3DSettings;
    SSettings6DOF                 mvs6DOFSettings;
    std::vector<SGazeVector>      mvsGazeVectorSettings;
    std::vector<SAnalogDevice>    mvsAnalogDeviceSettings;
    SSettingsForce                msForceSettings;
    std::vector<SImageCamera>     mvsImageSettings;
    std::vector<SSettingsSkeleton> mSkeletonSettings;
    SCalibration                  mCalibrationSettings;
    char                          mSendBuffer[5000];
    char                          maErrorStr[1024];
    unsigned short                mnBroadcastPort;
    FILE*                         mpFileBuffer;
    std::vector<SDiscoverResponse> mvsDiscoverResponseList;
};


#endif // RTPROTOCOL_H