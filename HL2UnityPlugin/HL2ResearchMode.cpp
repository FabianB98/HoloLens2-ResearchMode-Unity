#include "pch.h"
#include "HL2ResearchMode.h"
#include "HL2ResearchMode.g.cpp"

#define SHORT_THROW_PIXEL_DIRECTIONS_TOPIC L"/hololensShortThrowPixelDirections"
#define LONG_THROW_PIXEL_DIRECTIONS_TOPIC L"/hololensLongThrowPixelDirections"
#define STEREO_CAMERA_PIXEL_DIRECTIONS_TOPIC L"/hololensStereoCameraPixelDirections"

#define SHORT_THROW_DEPTH_TOPIC L"/hololensShortThrowDepth"
#define LONG_THROW_DEPTH_TOPIC L"/hololensLongThrowDepth"
#define STEREO_IMAGE_TOPIC L"/hololensStereoImage"

#define PIXEL_DIRECTIONS_MESSAGE_TYPE L"hololens_msgs/PixelDirections"
#define STEREO_PIXEL_DIRECTIONS_MESSAGE_TYPE L"hololens_msgs/StereoPixelDirections"
#define DEPTH_FRAME_MESSAGE_TYPE L"hololens_msgs/DepthFrame"
#define STEREO_CAMERA_FRAME_MESSAGE_TYPE L"hololens_msgs/StereoCameraFrame"

extern "C"
HMODULE LoadLibraryA(
    LPCSTR lpLibFileName
);

static ResearchModeSensorConsent camAccessCheck;
static HANDLE camConsentGiven;
static ResearchModeSensorConsent imuAccessCheck;
static HANDLE imuConsentGiven;

using namespace DirectX;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Perception::Spatial::Preview;

//typedef std::chrono::duration<int64_t, std::ratio<1, 10'000'000>> HundredsOfNanoseconds;

// This function is copied from https://stackoverflow.com/a/57551892
void OutputDebugStringFormat(const char* zcFormat, ...)
{
    // initialize use of the variable argument array
    va_list vaArgs;
    va_start(vaArgs, zcFormat);

    // reliably acquire the size
    // from a copy of the variable argument array
    // and a functionally reliable call to mock the formatting
    va_list vaArgsCopy;
    va_copy(vaArgsCopy, vaArgs);
    const int iLen = std::vsnprintf(NULL, 0, zcFormat, vaArgsCopy);
    va_end(vaArgsCopy);

    // return a formatted string without risking memory mismanagement
    // and without assuming any compiler or platform specific behavior
    std::vector<char> zc(iLen + 1);
    std::vsnprintf(zc.data(), zc.size(), zcFormat, vaArgs);
    va_end(vaArgs);
    std::string strText(zc.data(), iLen);

    OutputDebugStringA(strText.c_str());
}

namespace winrt::HL2UnityPlugin::implementation
{
    HL2ResearchMode::HL2ResearchMode() 
    {
        // Load Research Mode library
        camConsentGiven = CreateEvent(nullptr, true, false, nullptr);
        imuConsentGiven = CreateEvent(nullptr, true, false, nullptr);
        HMODULE hrResearchMode = LoadLibraryA("ResearchModeAPI");
        HRESULT hr = S_OK;

        if (hrResearchMode)
        {
            typedef HRESULT(__cdecl* PFN_CREATEPROVIDER) (IResearchModeSensorDevice** ppSensorDevice);
            PFN_CREATEPROVIDER pfnCreate = reinterpret_cast<PFN_CREATEPROVIDER>(GetProcAddress(hrResearchMode, "CreateResearchModeSensorDevice"));
            if (pfnCreate)
            {
                winrt::check_hresult(pfnCreate(&m_pSensorDevice));
            }
            else
            {
                winrt::check_hresult(E_INVALIDARG);
            }
        }

        // get spatial locator of rigNode
        GUID guid;
        IResearchModeSensorDevicePerception* pSensorDevicePerception;
        winrt::check_hresult(m_pSensorDevice->QueryInterface(IID_PPV_ARGS(&pSensorDevicePerception)));
        winrt::check_hresult(pSensorDevicePerception->GetRigNodeId(&guid));
        pSensorDevicePerception->Release();
        m_locator = SpatialGraphInteropPreview::CreateLocatorForNode(guid);

        size_t sensorCount = 0;

        winrt::check_hresult(m_pSensorDevice->QueryInterface(IID_PPV_ARGS(&m_pSensorDeviceConsent)));
        winrt::check_hresult(m_pSensorDeviceConsent->RequestCamAccessAsync(HL2ResearchMode::CamAccessOnComplete));
        winrt::check_hresult(m_pSensorDeviceConsent->RequestIMUAccessAsync(HL2ResearchMode::ImuAccessOnComplete));

        m_pSensorDevice->DisableEyeSelection();

        winrt::check_hresult(m_pSensorDevice->GetSensorCount(&sensorCount));
        m_sensorDescriptors.resize(sensorCount);
        winrt::check_hresult(m_pSensorDevice->GetSensorDescriptors(m_sensorDescriptors.data(), m_sensorDescriptors.size(), &sensorCount));
    }

    void HL2ResearchMode::InitializeDepthSensor() 
    {
        for (auto sensorDescriptor : m_sensorDescriptors)
        {
            if (sensorDescriptor.sensorType == DEPTH_AHAT)
            {
                winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_depthSensor));
                winrt::check_hresult(m_depthSensor->QueryInterface(IID_PPV_ARGS(&m_pDepthCameraSensor)));
                winrt::check_hresult(m_pDepthCameraSensor->GetCameraExtrinsicsMatrix(&m_depthCameraPose));
                m_depthCameraPoseInvMatrix = XMMatrixInverse(nullptr, XMLoadFloat4x4(&m_depthCameraPose));
                break;
            }
        }
    }

    void HL2ResearchMode::InitializeLongDepthSensor()
    {
        for (auto sensorDescriptor : m_sensorDescriptors)
        {
            if (sensorDescriptor.sensorType == DEPTH_LONG_THROW)
            {
                winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_longDepthSensor));
                winrt::check_hresult(m_longDepthSensor->QueryInterface(IID_PPV_ARGS(&m_pLongDepthCameraSensor)));
                winrt::check_hresult(m_pLongDepthCameraSensor->GetCameraExtrinsicsMatrix(&m_longDepthCameraPose));
                m_longDepthCameraPoseInvMatrix = XMMatrixInverse(nullptr, XMLoadFloat4x4(&m_longDepthCameraPose));
                break;
            }
        }
    }

    void HL2ResearchMode::InitializeSpatialCamerasFront()
    {
        for (auto sensorDescriptor : m_sensorDescriptors)
        {
            if (sensorDescriptor.sensorType == LEFT_FRONT)
            {
                winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_LFSensor));
                winrt::check_hresult(m_LFSensor->QueryInterface(IID_PPV_ARGS(&m_LFCameraSensor)));
                winrt::check_hresult(m_LFCameraSensor->GetCameraExtrinsicsMatrix(&m_LFCameraPose));
                m_LFCameraPoseInvMatrix = XMMatrixInverse(nullptr, XMLoadFloat4x4(&m_LFCameraPose));
            }
            if (sensorDescriptor.sensorType == RIGHT_FRONT)
            {
                winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_RFSensor));
                winrt::check_hresult(m_RFSensor->QueryInterface(IID_PPV_ARGS(&m_RFCameraSensor)));
                winrt::check_hresult(m_RFCameraSensor->GetCameraExtrinsicsMatrix(&m_RFCameraPose));
                m_RFCameraPoseInvMatrix = XMMatrixInverse(nullptr, XMLoadFloat4x4(&m_RFCameraPose));
            }
        }
    }

    void HL2ResearchMode::InitializeAccelSensor()
    {
        for (auto sensorDescriptor : m_sensorDescriptors)
        {
            if (sensorDescriptor.sensorType == IMU_ACCEL)
            {
                winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_accelSensor));
            }
        }
    }

    void HL2ResearchMode::InitializeGyroSensor()
    {
        for (auto sensorDescriptor : m_sensorDescriptors)
        {
            if (sensorDescriptor.sensorType == IMU_GYRO)
            {
                winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_gyroSensor));
            }
        }
    }

    void HL2ResearchMode::InitializeMagSensor()
    {
        for (auto sensorDescriptor : m_sensorDescriptors)
        {
            if (sensorDescriptor.sensorType == IMU_MAG)
            {
                winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_magSensor));
            }
        }
    }

    void HL2ResearchMode::StartDepthSensorLoop(bool reconstructPointCloud)
    {
        //std::thread th1([this] {this->DepthSensorLoopTest(); });
        if (reconstructPointCloud && m_refFrame == nullptr)
        {
            m_refFrame = m_locator.GetDefault().CreateStationaryFrameOfReferenceAtCurrentLocation().CoordinateSystem();
        }
        m_reconstructShortThrowPointCloud = reconstructPointCloud;

        m_pDepthUpdateThread = new std::thread(HL2ResearchMode::DepthSensorLoop, this);
    }

    void HL2ResearchMode::DepthSensorLoop(HL2ResearchMode* pHL2ResearchMode)
    {
        // prevent starting loop for multiple times
        if (!pHL2ResearchMode->m_depthSensorLoopStarted)
        {
            pHL2ResearchMode->m_depthSensorLoopStarted = true;
        }
        else return;

        pHL2ResearchMode->m_depthSensor->OpenStream();

        try 
        {
            UINT64 lastTs = 0;
            while (pHL2ResearchMode->m_depthSensorLoopStarted)
            {
                IResearchModeSensorFrame* pDepthSensorFrame = nullptr;
                ResearchModeSensorResolution resolution;
                pHL2ResearchMode->m_depthSensor->GetNextBuffer(&pDepthSensorFrame);

                // process sensor frame
                pDepthSensorFrame->GetResolution(&resolution);
                pHL2ResearchMode->m_depthResolution = resolution;
                
                IResearchModeSensorDepthFrame* pDepthFrame = nullptr;
                winrt::check_hresult(pDepthSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame)));

                size_t outBufferCount = 0;
                const UINT16* pDepth = nullptr;
                pDepthFrame->GetBuffer(&pDepth, &outBufferCount);
                pHL2ResearchMode->m_depthBufferSize = outBufferCount;
                size_t outAbBufferCount = 0;
                const UINT16* pAbImage = nullptr;
                pDepthFrame->GetAbDepthBuffer(&pAbImage, &outAbBufferCount);

                auto pDepthTexture = std::make_unique<uint8_t[]>(outBufferCount);
                auto pAbTexture = std::make_unique<uint8_t[]>(outAbBufferCount);
                std::vector<float> pointCloud;

                ResearchModeSensorTimestamp timestamp;
                pDepthSensorFrame->GetTimeStamp(&timestamp);

                if (timestamp.HostTicks == lastTs) continue;
                lastTs = timestamp.HostTicks;

                // get tracking transform
                Windows::Perception::Spatial::SpatialLocation transToWorld = nullptr;
                if (pHL2ResearchMode->m_reconstructShortThrowPointCloud) 
                {
                    auto ts = PerceptionTimestampHelper::FromSystemRelativeTargetTime(HundredsOfNanoseconds(checkAndConvertUnsigned(timestamp.HostTicks)));
                    transToWorld = pHL2ResearchMode->m_locator.TryLocateAtTimestamp(ts, pHL2ResearchMode->m_refFrame);
                    if (transToWorld == nullptr) continue;
                }

                XMMATRIX depthToWorld = XMMatrixIdentity();
                if (pHL2ResearchMode->m_reconstructShortThrowPointCloud)
                    depthToWorld = pHL2ResearchMode->m_depthCameraPoseInvMatrix * SpatialLocationToDxMatrix(transToWorld);

                pHL2ResearchMode->mu.lock();
                auto roiCenterFloat = XMFLOAT3(pHL2ResearchMode->m_roiCenter[0], pHL2ResearchMode->m_roiCenter[1], pHL2ResearchMode->m_roiCenter[2]);
                auto roiBoundFloat = XMFLOAT3(pHL2ResearchMode->m_roiBound[0], pHL2ResearchMode->m_roiBound[1], pHL2ResearchMode->m_roiBound[2]);
                pHL2ResearchMode->mu.unlock();

                XMVECTOR roiCenter = XMLoadFloat3(&roiCenterFloat);
                XMVECTOR roiBound = XMLoadFloat3(&roiBoundFloat);
                
                for (UINT i = 0; i < resolution.Height; i++)
                {
                    for (UINT j = 0; j < resolution.Width; j++)
                    {
                        auto idx = resolution.Width * i + j;
                        UINT16 depth = pDepth[idx];
                        depth = (depth > 4090) ? 0 : depth - pHL2ResearchMode->m_depthOffset;

                        if (pHL2ResearchMode->m_reconstructShortThrowPointCloud)
                        {
                            // back-project point cloud within Roi
                            if (i > pHL2ResearchMode->depthCamRoi.kRowLower * resolution.Height && i < pHL2ResearchMode->depthCamRoi.kRowUpper * resolution.Height &&
                                j > pHL2ResearchMode->depthCamRoi.kColLower * resolution.Width && j < pHL2ResearchMode->depthCamRoi.kColUpper * resolution.Width &&
                                depth > pHL2ResearchMode->depthCamRoi.depthNearClip && depth < pHL2ResearchMode->depthCamRoi.depthFarClip)
                            {
                                float xy[2] = { 0, 0 };
                                float uv[2] = { j, i };
                                pHL2ResearchMode->m_pDepthCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);
                                auto pointOnUnitPlane = XMFLOAT3(xy[0], xy[1], 1);
                                auto tempPoint = (float)depth / 1000 * XMVector3Normalize(XMLoadFloat3(&pointOnUnitPlane));
                                // apply transformation
                                auto pointInWorld = XMVector3Transform(tempPoint, depthToWorld);

                                // filter point cloud based on region of interest
                                if (!pHL2ResearchMode->m_useRoiFilter ||
                                    (pHL2ResearchMode->m_useRoiFilter && XMVector3InBounds(pointInWorld - roiCenter, roiBound)))
                                {
                                    pointCloud.push_back(XMVectorGetX(pointInWorld));
                                    pointCloud.push_back(XMVectorGetY(pointInWorld));
                                    pointCloud.push_back(-XMVectorGetZ(pointInWorld));
                                }
                            }
                        }

                        // save depth map as grayscale texture pixel into temp buffer
                        if (depth == 0) { pDepthTexture.get()[idx] = 0; }
                        else { pDepthTexture.get()[idx] = (uint8_t)((float)depth / 1000 * 255); }

                        // save AbImage as grayscale texture pixel into temp buffer
                        UINT16 abValue = pAbImage[idx];
                        uint8_t processedAbValue = 0;
                        if (abValue > 1000) { processedAbValue = 0xFF; }
                        else { processedAbValue = (uint8_t)((float)abValue / 1000 * 255); }

                        pAbTexture.get()[idx] = processedAbValue;

                        // save the depth of center pixel
                        if (pHL2ResearchMode->m_reconstructShortThrowPointCloud && 
                            i == (UINT)(0.35 * resolution.Height) && j == (UINT)(0.5 * resolution.Width)
                            && pointCloud.size()>=3)
                        {
                            pHL2ResearchMode->m_centerDepth = depth;
                            if (depth > pHL2ResearchMode->depthCamRoi.depthNearClip && depth < pHL2ResearchMode->depthCamRoi.depthFarClip)
                            {
                                std::lock_guard<std::mutex> l(pHL2ResearchMode->mu);
                                pHL2ResearchMode->m_centerPoint[0] = *(pointCloud.end() - 3);
                                pHL2ResearchMode->m_centerPoint[1] = *(pointCloud.end() - 2);
                                pHL2ResearchMode->m_centerPoint[2] = *(pointCloud.end() - 1);
                            }
                        }
                    }
                }

                // save data
                {
                    std::lock_guard<std::mutex> l(pHL2ResearchMode->mu);

                    // save point cloud
                    if (pHL2ResearchMode->m_reconstructShortThrowPointCloud)
                    {
                        if (!pHL2ResearchMode->m_pointCloud)
                        {
                            OutputDebugString(L"Create Space for point cloud...\n");
                            pHL2ResearchMode->m_pointCloud = new float[outBufferCount * 3];
                        }

                        memcpy(pHL2ResearchMode->m_pointCloud, pointCloud.data(), pointCloud.size() * sizeof(float));
                        pHL2ResearchMode->m_pointcloudLength = pointCloud.size();
                    }
                    

                    // save raw depth map
                    if (!pHL2ResearchMode->m_depthMap)
                    {
                        OutputDebugString(L"Create Space for depth map...\n");
                        pHL2ResearchMode->m_depthMap = new UINT16[outBufferCount];
                    }
                    memcpy(pHL2ResearchMode->m_depthMap, pDepth, outBufferCount * sizeof(UINT16));

                    // save pre-processed depth map texture (for visualization)
                    if (!pHL2ResearchMode->m_depthMapTexture)
                    {
                        OutputDebugString(L"Create Space for depth map texture...\n");
                        pHL2ResearchMode->m_depthMapTexture = new UINT8[outBufferCount];
                    }
                    memcpy(pHL2ResearchMode->m_depthMapTexture, pDepthTexture.get(), outBufferCount * sizeof(UINT8));

                    // save raw AbImage
                    if (!pHL2ResearchMode->m_shortAbImage)
                    {
                        OutputDebugString(L"Create Space for short AbImage...\n");
                        pHL2ResearchMode->m_shortAbImage = new UINT16[outBufferCount];
                    }
                    memcpy(pHL2ResearchMode->m_shortAbImage, pAbImage, outBufferCount * sizeof(UINT16));

                    // save pre-processed AbImage texture (for visualization)
                    if (!pHL2ResearchMode->m_shortAbImageTexture)
                    {
                        OutputDebugString(L"Create Space for short AbImage texture...\n");
                        pHL2ResearchMode->m_shortAbImageTexture = new UINT8[outBufferCount];
                    }
                    memcpy(pHL2ResearchMode->m_shortAbImageTexture, pAbTexture.get(), outBufferCount * sizeof(UINT8));
                }
                pHL2ResearchMode->m_shortAbImageTextureUpdated = true;
                pHL2ResearchMode->m_depthMapTextureUpdated = true;
                if (pHL2ResearchMode->m_reconstructShortThrowPointCloud) pHL2ResearchMode->m_pointCloudUpdated = true;

                pDepthTexture.reset();

                // release space
                if (pDepthFrame) 
                {
                    pDepthFrame->Release();
                }
                if (pDepthSensorFrame)
                {
                    pDepthSensorFrame->Release();
                }
                
            }
        }
        catch (...)  {}
        pHL2ResearchMode->m_depthSensor->CloseStream();
        pHL2ResearchMode->m_depthSensor->Release();
        pHL2ResearchMode->m_depthSensor = nullptr;
        
    }

    void HL2ResearchMode::StartLongDepthSensorLoop(bool reconstructPointCloud, bool streamRawSensorDataToRosbridge)
    {
        if ((reconstructPointCloud || streamRawSensorDataToRosbridge) && m_refFrame == nullptr)
        {
            m_refFrame = m_locator.GetDefault().CreateStationaryFrameOfReferenceAtCurrentLocation().CoordinateSystem();
        }
        m_reconstructLongThrowPointCloud = reconstructPointCloud;
        m_streamRawLongThrowSensorDataToRosbridge = streamRawSensorDataToRosbridge;

        m_pLongDepthUpdateThread = new std::thread(HL2ResearchMode::LongDepthSensorLoop, this);
    }

    void HL2ResearchMode::LongDepthSensorLoop(HL2ResearchMode* pHL2ResearchMode)
    {
        // prevent starting loop for multiple times
        if (!pHL2ResearchMode->m_longDepthSensorLoopStarted)
        {
            pHL2ResearchMode->m_longDepthSensorLoopStarted = true;
        }
        else {
            return;
        }

        pHL2ResearchMode->m_longDepthSensor->OpenStream();
        
        pHL2ResearchMode->m_longThrowConnectedToRosbridge = false;
        Windows::Networking::Sockets::MessageWebSocket websocket;
        Windows::Storage::Streams::DataWriter dataWriter;
        bool websocketAndDataWriterInitialized = false;
        int failedFrames = 0;

        try
        {
            UINT64 lastTs = 0; bool printedResolution = false;
            while (pHL2ResearchMode->m_longDepthSensorLoopStarted)
            {
                IResearchModeSensorFrame* pDepthSensorFrame = nullptr;
                ResearchModeSensorResolution resolution;
                pHL2ResearchMode->m_longDepthSensor->GetNextBuffer(&pDepthSensorFrame);

                // process sensor frame
                pDepthSensorFrame->GetResolution(&resolution);
                pHL2ResearchMode->m_longDepthResolution = resolution;

                IResearchModeSensorDepthFrame* pDepthFrame = nullptr;
                winrt::check_hresult(pDepthSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame)));

                size_t outBufferCount = 0;
                const UINT16* pDepth = nullptr;
                const BYTE* pSigma = nullptr;
                pDepthFrame->GetSigmaBuffer(&pSigma, &outBufferCount);
                pDepthFrame->GetBuffer(&pDepth, &outBufferCount);

                const UINT16* pAbImage = nullptr;
                pDepthFrame->GetAbDepthBuffer(&pAbImage, &outBufferCount);

                pHL2ResearchMode->m_longDepthBufferSize = outBufferCount;

                auto pDepthTexture = std::make_unique<uint8_t[]>(outBufferCount);
                auto pAbTexture = std::make_unique<uint8_t[]>(outBufferCount);
                std::vector<float> pointCloud;

                ResearchModeSensorTimestamp timestamp;
                pDepthSensorFrame->GetTimeStamp(&timestamp);

                if (timestamp.HostTicks == lastTs) continue;
                lastTs = timestamp.HostTicks;

                // get tracking transform
                Windows::Perception::Spatial::SpatialLocation transToWorld = nullptr;
                if (pHL2ResearchMode->m_reconstructLongThrowPointCloud || pHL2ResearchMode->m_streamRawLongThrowSensorDataToRosbridge)
                {
                    auto ts = PerceptionTimestampHelper::FromSystemRelativeTargetTime(HundredsOfNanoseconds(checkAndConvertUnsigned(timestamp.HostTicks)));
                    transToWorld = pHL2ResearchMode->m_locator.TryLocateAtTimestamp(ts, pHL2ResearchMode->m_refFrame);
                    if (transToWorld == nullptr) continue;
                }
                XMMATRIX depthToWorld = XMMatrixIdentity();
                if (pHL2ResearchMode->m_reconstructLongThrowPointCloud || pHL2ResearchMode->m_streamRawLongThrowSensorDataToRosbridge)
                    depthToWorld = pHL2ResearchMode->m_longDepthCameraPoseInvMatrix * SpatialLocationToDxMatrix(transToWorld);

                pHL2ResearchMode->mu.lock();
                auto roiCenterFloat = XMFLOAT3(pHL2ResearchMode->m_roiCenter[0], pHL2ResearchMode->m_roiCenter[1], pHL2ResearchMode->m_roiCenter[2]);
                auto roiBoundFloat = XMFLOAT3(pHL2ResearchMode->m_roiBound[0], pHL2ResearchMode->m_roiBound[1], pHL2ResearchMode->m_roiBound[2]);
                pHL2ResearchMode->mu.unlock();

                XMVECTOR roiCenter = XMLoadFloat3(&roiCenterFloat);
                XMVECTOR roiBound = XMLoadFloat3(&roiBoundFloat);

                for (UINT i = 0; i < resolution.Height; i++)
                {
                    for (UINT j = 0; j < resolution.Width; j++)
                    {
                        auto idx = resolution.Width * i + j;
                        UINT16 depth = pDepth[idx];
                        depth = (pSigma[idx] & 0x80) ? 0 : depth - pHL2ResearchMode->m_depthOffset;

                        if (pHL2ResearchMode->m_reconstructLongThrowPointCloud)
                        {
                            // back-project point cloud within Roi
                            if (i > pHL2ResearchMode->depthCamRoi.kRowLower * resolution.Height && i < pHL2ResearchMode->depthCamRoi.kRowUpper * resolution.Height &&
                                j > pHL2ResearchMode->depthCamRoi.kColLower * resolution.Width && j < pHL2ResearchMode->depthCamRoi.kColUpper * resolution.Width &&
                                depth > 200)
                            {
                                float xy[2] = { 0, 0 };
                                float uv[2] = { j, i };
                                pHL2ResearchMode->m_pLongDepthCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);
                                auto pointOnUnitPlane = XMFLOAT3(xy[0], xy[1], 1);
                                auto tempPoint = (float)depth / 1000 * XMVector3Normalize(XMLoadFloat3(&pointOnUnitPlane));
                                // apply transformation
                                auto pointInWorld = XMVector3Transform(tempPoint, depthToWorld);

                                // filter point cloud based on region of interest
                                if (!pHL2ResearchMode->m_useRoiFilter ||
                                    (pHL2ResearchMode->m_useRoiFilter && XMVector3InBounds(pointInWorld - roiCenter, roiBound)))
                                {
                                    pointCloud.push_back(XMVectorGetX(pointInWorld));
                                    pointCloud.push_back(XMVectorGetY(pointInWorld));
                                    pointCloud.push_back(-XMVectorGetZ(pointInWorld));
                                }
                            }
                        }

                        // save as grayscale texture pixel into temp buffer
                        if (depth == 0) { pDepthTexture.get()[idx] = 0; }
                        else { pDepthTexture.get()[idx] = (uint8_t)((float)depth / 4000 * 255); }

                        // save AbImage as grayscale texture pixel into temp buffer
                        UINT16 abValue = pAbImage[idx];
                        uint8_t processedAbValue = 0;
                        if (abValue > 2000) { processedAbValue = 0xFF; }
                        else { processedAbValue = (uint8_t)((float)abValue / 2000 * 255); }

                        pAbTexture.get()[idx] = processedAbValue;

                        // save the depth of center pixel
                        if (pHL2ResearchMode->m_reconstructLongThrowPointCloud &&
                            i == (UINT)(0.35 * resolution.Height) && j == (UINT)(0.5 * resolution.Width)
                            && pointCloud.size() >= 3)
                        {
                            pHL2ResearchMode->m_centerDepth = depth;
                            if (depth > pHL2ResearchMode->depthCamRoi.depthNearClip && depth < pHL2ResearchMode->depthCamRoi.depthFarClip)
                            {
                                std::lock_guard<std::mutex> l(pHL2ResearchMode->mu);
                                pHL2ResearchMode->m_centerPoint[0] = *(pointCloud.end() - 3);
                                pHL2ResearchMode->m_centerPoint[1] = *(pointCloud.end() - 2);
                                pHL2ResearchMode->m_centerPoint[2] = *(pointCloud.end() - 1);
                            }
                        }
                    }
                }

                // stream data to Rosbridge
                if (pHL2ResearchMode->m_streamRawLongThrowSensorDataToRosbridge)
                {
                    if (!pHL2ResearchMode->m_longThrowConnectedToRosbridge)
                    {
                        if (websocketAndDataWriterInitialized)
                        {
                            try
                            {
                                dataWriter.DetachStream();
                                websocket.Close();
                                websocketAndDataWriterInitialized = false;
                            }
                            catch (...)
                            {
                                OutputDebugString(L"Oh no, something went wrong when closing the old websocket connection!");
                            }
                        }

                        try
                        {
                            Windows::Foundation::Uri uri = Windows::Foundation::Uri(pHL2ResearchMode->m_rosbridgeUri);
                            websocket = Windows::Networking::Sockets::MessageWebSocket();
                            websocket.Control().MessageType(Windows::Networking::Sockets::SocketMessageType::Utf8);

                            websocket.ConnectAsync(uri).get();
                            dataWriter = Windows::Storage::Streams::DataWriter(websocket.OutputStream());
                            websocketAndDataWriterInitialized = true;

                            dataWriter.WriteString(L"{\"op\":\"advertise\",\"topic\":\"");
                            dataWriter.WriteString(LONG_THROW_PIXEL_DIRECTIONS_TOPIC);
                            dataWriter.WriteString(L"\",\"type\":\"");
                            dataWriter.WriteString(PIXEL_DIRECTIONS_MESSAGE_TYPE);
                            dataWriter.WriteString(L"\"}");
                            dataWriter.StoreAsync().get();

                            dataWriter.WriteString(L"{\"op\":\"advertise\",\"topic\":\"");
                            dataWriter.WriteString(LONG_THROW_DEPTH_TOPIC);
                            dataWriter.WriteString(L"\",\"type\":\"");
                            dataWriter.WriteString(DEPTH_FRAME_MESSAGE_TYPE);
                            dataWriter.WriteString(L"\"}");
                            dataWriter.StoreAsync().get();

                            OutputDebugString(L"Established connection to a Rosbridge websocket!\n");
                            pHL2ResearchMode->m_longThrowConnectedToRosbridge = true;
                            pHL2ResearchMode->m_longThrowPixelDirectionsSent = false;
                            failedFrames = 0;
                        }
                        catch (...)
                        {
                            OutputDebugString(L"Connection to a Rosbridge websocket couldn't be established!\n");
                        }
                    }

                    if (pHL2ResearchMode->m_longThrowConnectedToRosbridge)
                    {
                        if (pHL2ResearchMode->m_streamLongThrowPixelDirectionsToRosbridge && !pHL2ResearchMode->m_longThrowPixelDirectionsSent)
                        {
                            std::vector<PixelDirection> pixelDirections;

                            // Iterate over all pixels of the depth camera image.
                            for (UINT v = 0; v < resolution.Height; v++)
                            {
                                for (UINT u = 0; u < resolution.Width; u++)
                                {
                                    // Calculate the direction (in camera view space) in which the current pixel points at.
                                    float xy[2] = { 0, 0 };
                                    float uv[2] = { static_cast<float>(u), static_cast<float>(v) };
                                    pHL2ResearchMode->m_pLongDepthCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);
                                    Windows::Foundation::Numerics::float3 direction = Windows::Foundation::Numerics::float3(xy[0], xy[1], 1.0);
                                    direction = Windows::Foundation::Numerics::normalize(direction);

                                    // Add the direction to the resulting pixel directions.
                                    pixelDirections.push_back(PixelDirection(u, v, direction));
                                }
                            }

                            OutputDebugStringFormat("Got %zu pixel directions!\n", pixelDirections.size());

                            // Send the pixel directions to ROS.
                            try
                            {
                                dataWriter.WriteString(L"{\"op\":\"publish\",\"topic\":\"");
                                dataWriter.WriteString(LONG_THROW_PIXEL_DIRECTIONS_TOPIC);
                                dataWriter.WriteString(L"\",\"msg\":{\"pixelDirections\":[");
                                for (size_t i = 0; i < pixelDirections.size(); i++)
                                {
                                    dataWriter.WriteString(L"{\"u\":");
                                    dataWriter.WriteString(to_hstring(pixelDirections[i].u));
                                    dataWriter.WriteString(L",\"v\":");
                                    dataWriter.WriteString(to_hstring(pixelDirections[i].v));
                                    dataWriter.WriteString(L",\"direction\":{\"x\":");
                                    dataWriter.WriteString(to_hstring(pixelDirections[i].direction.x));
                                    dataWriter.WriteString(L",\"y\":");
                                    dataWriter.WriteString(to_hstring(pixelDirections[i].direction.y));
                                    dataWriter.WriteString(L",\"z\":");
                                    dataWriter.WriteString(to_hstring(pixelDirections[i].direction.z));
                                    dataWriter.WriteString(L"}}");
                                    if (i < pixelDirections.size() - 1)
                                    {
                                        dataWriter.WriteString(L",");
                                    }
                                }
                                dataWriter.WriteString(L"]}}");
                                dataWriter.StoreAsync().get();

                                pHL2ResearchMode->m_longThrowPixelDirectionsSent = true;
                                failedFrames = 0;
                            }
                            catch (...)
                            {
                                OutputDebugString(L"Pixel directions couldn't be sent to the Rosbridge websocket!\n");
                                failedFrames++;
                                if (failedFrames >= 5)
                                {
                                    OutputDebugString(L"Failed sending data to the Rosbridge websocket for more than five times in a row! Assuming connection to be broken.\n");
                                    pHL2ResearchMode->m_longThrowConnectedToRosbridge = false;
                                }
                            }
                        }

                        // Encode depth image in Base64.
                        UINT8* depthBytes = (UINT8*)pDepth;
                        UINT32 depthPixelStride = 2;    // Depth images captured by the HoloLens 2 have 16 bit (= 2 byte) per pixel of depth information.
                        int32_t imageBufferSize = resolution.Width * resolution.Height * depthPixelStride;
                        std::string depthEncoded = base64_encode(depthBytes, imageBufferSize);

                        // Encode active brightness / reflectivity in Base64.
                        UINT8* reflectivityBytes = (UINT8*)pAbImage;
                        UINT32 reflectivityPixelStride = 2;     // Active brightness images captured by the HoloLens 2 have 16 bit (= 2 byte) per pixel of depth information.
                        imageBufferSize = resolution.Width * resolution.Height * reflectivityPixelStride;
                        std::string reflectivityEncoded = base64_encode(reflectivityBytes, imageBufferSize);

                        // Get the translation vector and the rotation quaternion from the transformation matrix.
                        XMVECTOR scaleVector;           // This should always be equal to a scale of 1.0 so we don't have to transmit it to ROS.
                        XMVECTOR rotationQuaternion;
                        XMVECTOR translationVector;
                        XMMatrixDecompose(&scaleVector, &rotationQuaternion, &translationVector, depthToWorld);

                        // Send everything (i.e. encoded depth image, width, height, pixel stride, translation and rotation) to ROS.
                        try
                        {
                            dataWriter.WriteString(L"{\"op\":\"publish\",\"topic\":\"");
                            dataWriter.WriteString(LONG_THROW_DEPTH_TOPIC);
                            dataWriter.WriteString(L"\",\"msg\":{\"depthMapWidth\":");
                            dataWriter.WriteString(to_hstring(resolution.Width));
                            dataWriter.WriteString(L",\"depthMapHeight\":");
                            dataWriter.WriteString(to_hstring(resolution.Height));
                            dataWriter.WriteString(L",\"depthMapPixelStride\":");
                            dataWriter.WriteString(to_hstring(depthPixelStride));
                            dataWriter.WriteString(L",\"reflectivityPixelStride\":");
                            dataWriter.WriteString(to_hstring(reflectivityPixelStride));
                            dataWriter.WriteString(L",\"base64encodedDepthMap\":\"");
                            dataWriter.WriteString(to_hstring(depthEncoded.c_str()));
                            dataWriter.WriteString(L"\",\"base64encodedReflectivity\":\"");
                            dataWriter.WriteString(to_hstring(reflectivityEncoded.c_str()));
                            dataWriter.WriteString(L"\",\"camToWorldTranslation\":{\"x\":");
                            dataWriter.WriteString(to_hstring(XMVectorGetX(translationVector)));
                            dataWriter.WriteString(L",\"y\":");
                            dataWriter.WriteString(to_hstring(XMVectorGetY(translationVector)));
                            dataWriter.WriteString(L",\"z\":");
                            dataWriter.WriteString(to_hstring(XMVectorGetZ(translationVector)));
                            dataWriter.WriteString(L"},\"camToWorldRotation\":{\"w\":");
                            dataWriter.WriteString(to_hstring(XMVectorGetW(rotationQuaternion)));
                            dataWriter.WriteString(L",\"x\":");
                            dataWriter.WriteString(to_hstring(XMVectorGetX(rotationQuaternion)));
                            dataWriter.WriteString(L",\"y\":");
                            dataWriter.WriteString(to_hstring(XMVectorGetY(rotationQuaternion)));
                            dataWriter.WriteString(L",\"z\":");
                            dataWriter.WriteString(to_hstring(XMVectorGetZ(rotationQuaternion)));
                            dataWriter.WriteString(L"}}}");
                            dataWriter.StoreAsync().get();  // Waiting for the data to be finished sending is definitely not performant. This needs to be changed in the future.

                            failedFrames = 0;
                        }
                        catch (...)
                        {
                            OutputDebugString(L"Depth image couldn't be sent to the Rosbridge websocket!\n");
                            failedFrames++;
                            if (failedFrames >= 5)
                            {
                                OutputDebugString(L"Failed sending data to the Rosbridge websocket for more than five times in a row! Assuming connection to be broken.\n");
                                pHL2ResearchMode->m_longThrowConnectedToRosbridge = false;
                            }
                        }
                    }
                }

                // save data
                {
                    std::lock_guard<std::mutex> l(pHL2ResearchMode->mu);

                    // save point cloud
                    if (pHL2ResearchMode->m_reconstructLongThrowPointCloud) 
                    {
                        if (!pHL2ResearchMode->m_longThrowPointCloud)
                        {
                            OutputDebugString(L"Create Space for point cloud (long throw)...\n");
                            pHL2ResearchMode->m_longThrowPointCloud = new float[outBufferCount * 3];
                        }

                        memcpy(pHL2ResearchMode->m_longThrowPointCloud, pointCloud.data(), pointCloud.size() * sizeof(float));
                        pHL2ResearchMode->m_longThrowPointcloudLength = pointCloud.size();
                    }

                    // save raw depth map
                    if (!pHL2ResearchMode->m_longDepthMap)
                    {
                        OutputDebugString(L"Create Space for long throw depth map...\n");
                        pHL2ResearchMode->m_longDepthMap = new UINT16[outBufferCount];
                    }
                    memcpy(pHL2ResearchMode->m_longDepthMap, pDepth, outBufferCount * sizeof(UINT16));

                    // save pre-processed depth map texture (for visualization)
                    if (!pHL2ResearchMode->m_longDepthMapTexture)
                    {
                        OutputDebugString(L"Create Space for long throw depth map texture...\n");
                        pHL2ResearchMode->m_longDepthMapTexture = new UINT8[outBufferCount];
                    }
                    memcpy(pHL2ResearchMode->m_longDepthMapTexture, pDepthTexture.get(), outBufferCount * sizeof(UINT8));

                    // save raw AbImage
                    if (!pHL2ResearchMode->m_longAbImage)
                    {
                        OutputDebugString(L"Create Space for long AbImage...\n");
                        pHL2ResearchMode->m_longAbImage = new UINT16[outBufferCount];
                    }
                    memcpy(pHL2ResearchMode->m_longAbImage, pAbImage, outBufferCount * sizeof(UINT16));

                    // save pre-processed AbImage texture (for visualization)
                    if (!pHL2ResearchMode->m_longAbImageTexture)
                    {
                        OutputDebugString(L"Create Space for long AbImage texture...\n");
                        pHL2ResearchMode->m_longAbImageTexture = new UINT8[outBufferCount];
                    }
                    memcpy(pHL2ResearchMode->m_longAbImageTexture, pAbTexture.get(), outBufferCount * sizeof(UINT8));
                }
                pHL2ResearchMode->m_longAbImageTextureUpdated = true;
                pHL2ResearchMode->m_longDepthMapTextureUpdated = true;
                if (pHL2ResearchMode->m_reconstructLongThrowPointCloud) pHL2ResearchMode->m_longThrowPointCloudUpdated = true;

                pDepthTexture.reset();

                // release space
                if (pDepthFrame) 
                {
                    pDepthFrame->Release();
                }
                if (pDepthSensorFrame)
                {
                    pDepthSensorFrame->Release();
                }
            }
        }
        catch (...) {}
        pHL2ResearchMode->m_longDepthSensor->CloseStream();
        pHL2ResearchMode->m_longDepthSensor->Release();
        pHL2ResearchMode->m_longDepthSensor = nullptr;

        if (websocketAndDataWriterInitialized)
        {
            try
            {
                dataWriter.DetachStream();
                websocket.Close();
                websocketAndDataWriterInitialized = false;
            }
            catch (...)
            {
                OutputDebugString(L"Oh no, something went wrong when closing the websocket connection!");
            }
        }
    }

    void HL2ResearchMode::StartSpatialCamerasFrontLoop(bool streamRawSensorDataToRosbridge)
    {
        if (m_refFrame == nullptr)
        {
            m_refFrame = m_locator.GetDefault().CreateStationaryFrameOfReferenceAtCurrentLocation().CoordinateSystem();
        }

        m_streamSpatialCamerasFrontSensorDataToRosbridge = streamRawSensorDataToRosbridge;

        m_pSpatialCamerasFrontUpdateThread = new std::thread(HL2ResearchMode::SpatialCamerasFrontLoop, this);
    }

    void HL2ResearchMode::SpatialCamerasFrontLoop(HL2ResearchMode* pHL2ResearchMode)
    {
        // prevent starting loop for multiple times
        if (!pHL2ResearchMode->m_spatialCamerasFrontLoopStarted)
        {
            pHL2ResearchMode->m_spatialCamerasFrontLoopStarted = true;
        }
        else {
            return;
        }

        pHL2ResearchMode->m_LFSensor->OpenStream();
        pHL2ResearchMode->m_RFSensor->OpenStream();

        pHL2ResearchMode->m_spatialCamerasFrontConnectedToRosbridge = false;
        Windows::Networking::Sockets::MessageWebSocket websocket;
        Windows::Storage::Streams::DataWriter dataWriter;
        bool websocketAndDataWriterInitialized = false;
        int failedFrames = 0;

        try
        {
            while (pHL2ResearchMode->m_spatialCamerasFrontLoopStarted)
            {
                IResearchModeSensorFrame* pLFCameraFrame = nullptr;
                IResearchModeSensorFrame* pRFCameraFrame = nullptr;
                ResearchModeSensorResolution LFResolution;
                ResearchModeSensorResolution RFResolution;
                pHL2ResearchMode->m_LFSensor->GetNextBuffer(&pLFCameraFrame);
				pHL2ResearchMode->m_RFSensor->GetNextBuffer(&pRFCameraFrame);

                // process sensor frame
                pLFCameraFrame->GetResolution(&LFResolution);
                pHL2ResearchMode->m_LFResolution = LFResolution;
                pRFCameraFrame->GetResolution(&RFResolution);
                pHL2ResearchMode->m_RFResolution = RFResolution;

                IResearchModeSensorVLCFrame* pLFFrame = nullptr;
                winrt::check_hresult(pLFCameraFrame->QueryInterface(IID_PPV_ARGS(&pLFFrame)));
                IResearchModeSensorVLCFrame* pRFFrame = nullptr;
                winrt::check_hresult(pRFCameraFrame->QueryInterface(IID_PPV_ARGS(&pRFFrame)));

                size_t LFOutBufferCount = 0;
                const BYTE *pLFImage = nullptr;
                pLFFrame->GetBuffer(&pLFImage, &LFOutBufferCount);
                pHL2ResearchMode->m_LFbufferSize = LFOutBufferCount;
				size_t RFOutBufferCount = 0;
				const BYTE *pRFImage = nullptr;
				pRFFrame->GetBuffer(&pRFImage, &RFOutBufferCount);
				pHL2ResearchMode->m_RFbufferSize = RFOutBufferCount;

                // get tracking transform
                ResearchModeSensorTimestamp timestamp_left, timestamp_right;
                pLFCameraFrame->GetTimeStamp(&timestamp_left);
                pRFCameraFrame->GetTimeStamp(&timestamp_right);

                auto ts_left = PerceptionTimestampHelper::FromSystemRelativeTargetTime(HundredsOfNanoseconds(checkAndConvertUnsigned(timestamp_left.HostTicks)));
                auto ts_right = PerceptionTimestampHelper::FromSystemRelativeTargetTime(HundredsOfNanoseconds(checkAndConvertUnsigned(timestamp_right.HostTicks)));
                
                // uncomment the block below if their transform is needed
                auto rigToWorld_l = pHL2ResearchMode->m_locator.TryLocateAtTimestamp(ts_left, pHL2ResearchMode->m_refFrame);
                auto rigToWorld_r = rigToWorld_l;
                if (ts_left.TargetTime() != ts_right.TargetTime()) {
                    rigToWorld_r = pHL2ResearchMode->m_locator.TryLocateAtTimestamp(ts_right, pHL2ResearchMode->m_refFrame);
                }
                
                if (rigToWorld_l == nullptr || rigToWorld_r == nullptr)
                {
                    continue;
                }
                
                auto LfToWorld = pHL2ResearchMode->m_LFCameraPoseInvMatrix * SpatialLocationToDxMatrix(rigToWorld_l);
				auto RfToWorld = pHL2ResearchMode->m_RFCameraPoseInvMatrix * SpatialLocationToDxMatrix(rigToWorld_r);

                // stream data to Rosbridge
                if (pHL2ResearchMode->m_streamSpatialCamerasFrontSensorDataToRosbridge)
                {
                    if (!pHL2ResearchMode->m_spatialCamerasFrontConnectedToRosbridge)
                    {
                        if (websocketAndDataWriterInitialized)
                        {
                            try
                            {
                                dataWriter.DetachStream();
                                websocket.Close();
                                websocketAndDataWriterInitialized = false;
                            }
                            catch (...)
                            {
                                OutputDebugString(L"Oh no, something went wrong when closing the old websocket connection!");
                            }
                        }

                        try
                        {
                            Windows::Foundation::Uri uri = Windows::Foundation::Uri(pHL2ResearchMode->m_rosbridgeUri);
                            websocket = Windows::Networking::Sockets::MessageWebSocket();
                            websocket.Control().MessageType(Windows::Networking::Sockets::SocketMessageType::Utf8);

                            websocket.ConnectAsync(uri).get();
                            dataWriter = Windows::Storage::Streams::DataWriter(websocket.OutputStream());
                            websocketAndDataWriterInitialized = true;

                            dataWriter.WriteString(L"{\"op\":\"advertise\",\"topic\":\"");
                            dataWriter.WriteString(STEREO_CAMERA_PIXEL_DIRECTIONS_TOPIC);
                            dataWriter.WriteString(L"\",\"type\":\"");
                            dataWriter.WriteString(STEREO_PIXEL_DIRECTIONS_MESSAGE_TYPE);
                            dataWriter.WriteString(L"\"}");
                            dataWriter.StoreAsync().get();

                            dataWriter.WriteString(L"{\"op\":\"advertise\",\"topic\":\"");
                            dataWriter.WriteString(STEREO_IMAGE_TOPIC);
                            dataWriter.WriteString(L"\",\"type\":\"");
                            dataWriter.WriteString(STEREO_CAMERA_FRAME_MESSAGE_TYPE);
                            dataWriter.WriteString(L"\"}");
                            dataWriter.StoreAsync().get();

                            OutputDebugString(L"Established connection to a Rosbridge websocket!\n");
                            pHL2ResearchMode->m_spatialCamerasFrontConnectedToRosbridge = true;
                            pHL2ResearchMode->m_spatialCamerasFrontPixelDirectionsSent = false;
                            failedFrames = 0;
                        }
                        catch (...)
                        {
                            OutputDebugString(L"Connection to a Rosbridge websocket couldn't be established!\n");
                        }
                    }

                    if (pHL2ResearchMode->m_spatialCamerasFrontConnectedToRosbridge)
                    {
                        if (pHL2ResearchMode->m_streamSpatialCamerasFrontPixelDirectionsToRosbridge && !pHL2ResearchMode->m_spatialCamerasFrontPixelDirectionsSent)
                        {
                            std::vector<PixelDirection> pixelDirectionsLeft;
                            std::vector<PixelDirection> pixelDirectionsRight;

                            // Iterate over all pixels of the left camera image.
                            for (UINT v = 0; v < LFResolution.Height; v++)
                            {
                                for (UINT u = 0; u < LFResolution.Width; u++)
                                {
                                    // Calculate the direction (in camera view space) in which the current pixel points at.
                                    float xy[2] = { 0, 0 };
                                    float uv[2] = { static_cast<float>(u), static_cast<float>(v) };
                                    pHL2ResearchMode->m_LFCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);
                                    Windows::Foundation::Numerics::float3 direction = Windows::Foundation::Numerics::float3(xy[0], xy[1], 1.0);
                                    direction = Windows::Foundation::Numerics::normalize(direction);

                                    // Add the direction to the resulting pixel directions.
                                    pixelDirectionsLeft.push_back(PixelDirection(u, v, direction));
                                }
                            }

                            // Iterate over all pixels of the right camera image.
                            for (UINT v = 0; v < RFResolution.Height; v++)
                            {
                                for (UINT u = 0; u < RFResolution.Width; u++)
                                {
                                    // Calculate the direction (in camera view space) in which the current pixel points at.
                                    float xy[2] = { 0, 0 };
                                    float uv[2] = { static_cast<float>(u), static_cast<float>(v) };
                                    pHL2ResearchMode->m_RFCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);
                                    Windows::Foundation::Numerics::float3 direction = Windows::Foundation::Numerics::float3(xy[0], xy[1], 1.0);
                                    direction = Windows::Foundation::Numerics::normalize(direction);

                                    // Add the direction to the resulting pixel directions.
                                    pixelDirectionsRight.push_back(PixelDirection(u, v, direction));
                                }
                            }

                            OutputDebugStringFormat("Got %zu (left camera) and %zu (right camera) pixel directions!\n", pixelDirectionsLeft.size(), pixelDirectionsRight.size());

                            // Send the pixel directions to ROS.
                            try
                            {
                                dataWriter.WriteString(L"{\"op\":\"publish\",\"topic\":\"");
                                dataWriter.WriteString(STEREO_CAMERA_PIXEL_DIRECTIONS_TOPIC);
                                dataWriter.WriteString(L"\",\"msg\":{\"pixelDirectionsLeft\":[");
                                for (size_t i = 0; i < pixelDirectionsLeft.size(); i++)
                                {
                                    dataWriter.WriteString(L"{\"u\":");
                                    dataWriter.WriteString(to_hstring(pixelDirectionsLeft[i].u));
                                    dataWriter.WriteString(L",\"v\":");
                                    dataWriter.WriteString(to_hstring(pixelDirectionsLeft[i].v));
                                    dataWriter.WriteString(L",\"direction\":{\"x\":");
                                    dataWriter.WriteString(to_hstring(pixelDirectionsLeft[i].direction.x));
                                    dataWriter.WriteString(L",\"y\":");
                                    dataWriter.WriteString(to_hstring(pixelDirectionsLeft[i].direction.y));
                                    dataWriter.WriteString(L",\"z\":");
                                    dataWriter.WriteString(to_hstring(pixelDirectionsLeft[i].direction.z));
                                    dataWriter.WriteString(L"}}");
                                    if (i < pixelDirectionsLeft.size() - 1)
                                    {
                                        dataWriter.WriteString(L",");
                                    }
                                }
                                dataWriter.WriteString(L"],\"pixelDirectionsRight\":[");
                                for (size_t i = 0; i < pixelDirectionsRight.size(); i++)
                                {
                                    dataWriter.WriteString(L"{\"u\":");
                                    dataWriter.WriteString(to_hstring(pixelDirectionsRight[i].u));
                                    dataWriter.WriteString(L",\"v\":");
                                    dataWriter.WriteString(to_hstring(pixelDirectionsRight[i].v));
                                    dataWriter.WriteString(L",\"direction\":{\"x\":");
                                    dataWriter.WriteString(to_hstring(pixelDirectionsRight[i].direction.x));
                                    dataWriter.WriteString(L",\"y\":");
                                    dataWriter.WriteString(to_hstring(pixelDirectionsRight[i].direction.y));
                                    dataWriter.WriteString(L",\"z\":");
                                    dataWriter.WriteString(to_hstring(pixelDirectionsRight[i].direction.z));
                                    dataWriter.WriteString(L"}}");
                                    if (i < pixelDirectionsRight.size() - 1)
                                    {
                                        dataWriter.WriteString(L",");
                                    }
                                }
                                dataWriter.WriteString(L"]}}");
                                dataWriter.StoreAsync().get();

                                pHL2ResearchMode->m_spatialCamerasFrontPixelDirectionsSent = true;
                                failedFrames = 0;
                            }
                            catch (...)
                            {
                                OutputDebugString(L"Pixel directions couldn't be sent to the Rosbridge websocket!\n");
                                failedFrames++;
                                if (failedFrames >= 5)
                                {
                                    OutputDebugString(L"Failed sending data to the Rosbridge websocket for more than five times in a row! Assuming connection to be broken.\n");
                                    pHL2ResearchMode->m_spatialCamerasFrontConnectedToRosbridge = false;
                                }
                            }
                        }

                        // Encode the two camera images in Base64.
                        UINT8* leftImageBytes = (UINT8*)pLFImage;
                        UINT32 leftPixelStride = 1;     // Camera images captured by the HoloLens 2 have 8 bit (= 1 byte) per pixel of visible light information.
                        int32_t imageBufferSize = LFResolution.Width * LFResolution.Height * leftPixelStride;
                        std::string leftImageEncoded = base64_encode(leftImageBytes, imageBufferSize);

                        UINT8* rightImageBytes = (UINT8*)pRFImage;
                        UINT32 rightPixelStride = 1;     // Camera images captured by the HoloLens 2 have 8 bit (= 1 byte) per pixel of visible light information.
                        imageBufferSize = RFResolution.Width * RFResolution.Height * rightPixelStride;
                        std::string rightImageEncoded = base64_encode(rightImageBytes, imageBufferSize);

                        // Get the translation vector and the rotation quaternion from the transformation matrix.
                        XMVECTOR scaleVectorLeft;           // This should always be equal to a scale of 1.0 so we don't have to transmit it to ROS.
                        XMVECTOR rotationQuaternionLeft;
                        XMVECTOR translationVectorLeft;
                        XMMatrixDecompose(&scaleVectorLeft, &rotationQuaternionLeft, &translationVectorLeft, LfToWorld);

                        XMVECTOR scaleVectorRight;           // This should always be equal to a scale of 1.0 so we don't have to transmit it to ROS.
                        XMVECTOR rotationQuaternionRight;
                        XMVECTOR translationVectorRight;
                        XMMatrixDecompose(&scaleVectorRight, &rotationQuaternionRight, &translationVectorRight, RfToWorld);

                        // Send everything (i.e. encoded images, width, height, pixel stride, translation and rotation) to ROS.
                        try
                        {
                            dataWriter.WriteString(L"{\"op\":\"publish\",\"topic\":\"");
                            dataWriter.WriteString(STEREO_IMAGE_TOPIC);
                            dataWriter.WriteString(L"\",\"msg\":{\"imageWidthLeft\":");
                            dataWriter.WriteString(to_hstring(LFResolution.Width));
                            dataWriter.WriteString(L",\"imageWidthRight\":");
                            dataWriter.WriteString(to_hstring(RFResolution.Width));
                            dataWriter.WriteString(L",\"imageHeightLeft\":");
                            dataWriter.WriteString(to_hstring(LFResolution.Height));
                            dataWriter.WriteString(L",\"imageHeightRight\":");
                            dataWriter.WriteString(to_hstring(RFResolution.Height));
                            dataWriter.WriteString(L",\"pixelStrideLeft\":");
                            dataWriter.WriteString(to_hstring(leftPixelStride));
                            dataWriter.WriteString(L",\"pixelStrideRight\":");
                            dataWriter.WriteString(to_hstring(rightPixelStride));
                            dataWriter.WriteString(L",\"base64encodedImageLeft\":\"");
                            dataWriter.WriteString(to_hstring(leftImageEncoded.c_str()));
                            dataWriter.WriteString(L"\",\"base64encodedImageRight\":\"");
                            dataWriter.WriteString(to_hstring(rightImageEncoded.c_str()));
                            dataWriter.WriteString(L"\",\"camToWorldTranslationLeft\":{\"x\":");
                            dataWriter.WriteString(to_hstring(XMVectorGetX(translationVectorLeft)));
                            dataWriter.WriteString(L",\"y\":");
                            dataWriter.WriteString(to_hstring(XMVectorGetY(translationVectorLeft)));
                            dataWriter.WriteString(L",\"z\":");
                            dataWriter.WriteString(to_hstring(XMVectorGetZ(translationVectorLeft)));
                            dataWriter.WriteString(L"},\"camToWorldTranslationRight\":{\"x\":");
                            dataWriter.WriteString(to_hstring(XMVectorGetX(translationVectorRight)));
                            dataWriter.WriteString(L",\"y\":");
                            dataWriter.WriteString(to_hstring(XMVectorGetY(translationVectorRight)));
                            dataWriter.WriteString(L",\"z\":");
                            dataWriter.WriteString(to_hstring(XMVectorGetZ(translationVectorRight)));
                            dataWriter.WriteString(L"},\"camToWorldRotationLeft\":{\"w\":");
                            dataWriter.WriteString(to_hstring(XMVectorGetW(rotationQuaternionLeft)));
                            dataWriter.WriteString(L",\"x\":");
                            dataWriter.WriteString(to_hstring(XMVectorGetX(rotationQuaternionLeft)));
                            dataWriter.WriteString(L",\"y\":");
                            dataWriter.WriteString(to_hstring(XMVectorGetY(rotationQuaternionLeft)));
                            dataWriter.WriteString(L",\"z\":");
                            dataWriter.WriteString(to_hstring(XMVectorGetZ(rotationQuaternionLeft)));
                            dataWriter.WriteString(L"},\"camToWorldRotationRight\":{\"w\":");
                            dataWriter.WriteString(to_hstring(XMVectorGetW(rotationQuaternionRight)));
                            dataWriter.WriteString(L",\"x\":");
                            dataWriter.WriteString(to_hstring(XMVectorGetX(rotationQuaternionRight)));
                            dataWriter.WriteString(L",\"y\":");
                            dataWriter.WriteString(to_hstring(XMVectorGetY(rotationQuaternionRight)));
                            dataWriter.WriteString(L",\"z\":");
                            dataWriter.WriteString(to_hstring(XMVectorGetZ(rotationQuaternionRight)));
                            dataWriter.WriteString(L"}}}");
                            dataWriter.StoreAsync().get();  // Waiting for the data to be finished sending is definitely not performant. This needs to be changed in the future.

                            failedFrames = 0;
                        }
                        catch (...)
                        {
                            OutputDebugString(L"Front facting spatial images (front left and front right) couldn't be sent to the Rosbridge websocket!\n");
                            failedFrames++;
                            if (failedFrames >= 5)
                            {
                                OutputDebugString(L"Failed sending data to the Rosbridge websocket for more than five times in a row! Assuming connection to be broken.\n");
                                pHL2ResearchMode->m_spatialCamerasFrontConnectedToRosbridge = false;
                            }
                        }
                    }
                }

                // save data
                {
                    std::lock_guard<std::mutex> l(pHL2ResearchMode->mu);

                    pHL2ResearchMode->m_lastSpatialFrame.LFFrame.timestamp = timestamp_left.HostTicks;
                    pHL2ResearchMode->m_lastSpatialFrame.RFFrame.timestamp = timestamp_right.HostTicks;

                    pHL2ResearchMode->m_lastSpatialFrame.LFFrame.timestamp_ft = ts_left.TargetTime().time_since_epoch().count();
                    pHL2ResearchMode->m_lastSpatialFrame.RFFrame.timestamp_ft = ts_right.TargetTime().time_since_epoch().count();


					// save LF and RF images
					if (!pHL2ResearchMode->m_lastSpatialFrame.LFFrame.image)
					{
						OutputDebugString(L"Create Space for Left Front Image...\n");
						pHL2ResearchMode->m_lastSpatialFrame.LFFrame.image = new UINT8[LFOutBufferCount];
					}
					memcpy(pHL2ResearchMode->m_lastSpatialFrame.LFFrame.image, pLFImage, LFOutBufferCount * sizeof(UINT8));

					if (!pHL2ResearchMode->m_lastSpatialFrame.RFFrame.image)
					{
						OutputDebugString(L"Create Space for Right Front Image...\n");
						pHL2ResearchMode->m_lastSpatialFrame.RFFrame.image = new UINT8[RFOutBufferCount];
					}
					memcpy(pHL2ResearchMode->m_lastSpatialFrame.RFFrame.image, pRFImage, RFOutBufferCount * sizeof(UINT8));
                }
				pHL2ResearchMode->m_LFImageUpdated = true;
				pHL2ResearchMode->m_RFImageUpdated = true;

                // release space
				if (pLFFrame) pLFFrame->Release();
				if (pRFFrame) pRFFrame->Release();

				if (pLFCameraFrame) pLFCameraFrame->Release();
				if (pRFCameraFrame) pRFCameraFrame->Release();
            }
        }
        catch (...) {}
        pHL2ResearchMode->m_LFSensor->CloseStream();
        pHL2ResearchMode->m_LFSensor->Release();
        pHL2ResearchMode->m_LFSensor = nullptr;

		pHL2ResearchMode->m_RFSensor->CloseStream();
		pHL2ResearchMode->m_RFSensor->Release();
		pHL2ResearchMode->m_RFSensor = nullptr;

        if (websocketAndDataWriterInitialized)
        {
            try
            {
                dataWriter.DetachStream();
                websocket.Close();
                websocketAndDataWriterInitialized = false;
            }
            catch (...)
            {
                OutputDebugString(L"Oh no, something went wrong when closing the websocket connection!");
            }
        }
    }

    void HL2ResearchMode::StartAccelSensorLoop()
    {
        m_pAccelUpdateThread = new std::thread(HL2ResearchMode::AccelSensorLoop, this);
    }

    void HL2ResearchMode::AccelSensorLoop(HL2ResearchMode* pHL2ResearchMode)
    {
        // prevent starting loop for multiple times
        if (!pHL2ResearchMode->m_accelSensorLoopStarted)
        {
            pHL2ResearchMode->m_accelSensorLoopStarted = true;
        }
        else {
            return;
        }

        try
        {
            winrt::check_hresult(pHL2ResearchMode->m_accelSensor->OpenStream());

            while (pHL2ResearchMode->m_accelSensorLoopStarted)
            {
                IResearchModeSensorFrame* pSensorFrame = nullptr;
                winrt::check_hresult(pHL2ResearchMode->m_accelSensor->GetNextBuffer(&pSensorFrame));
                
                IResearchModeAccelFrame* pModeAccelFrame = nullptr;
                winrt::check_hresult(pSensorFrame->QueryInterface(IID_PPV_ARGS(&pModeAccelFrame)));

                DirectX::XMFLOAT3 pSample;
                winrt::check_hresult(pModeAccelFrame->GetCalibratedAccelaration(&pSample));

                auto pAccelSample = std::make_unique<float[]>(3);

                // save data
                {
                    std::lock_guard<std::mutex> l(pHL2ResearchMode->mu);

                    // save raw accel sample
                    if (!pHL2ResearchMode->m_accelSample)
                    {
                        OutputDebugString(L"Create Space for accel sample...\n");
                        pHL2ResearchMode->m_accelSample = new float[3];
                    }
                    pHL2ResearchMode->m_accelSample[0] = pSample.x;
                    pHL2ResearchMode->m_accelSample[1] = pSample.y;
                    pHL2ResearchMode->m_accelSample[2] = pSample.z;
                }

                pHL2ResearchMode->m_accelSampleUpdated = true;

                pAccelSample.reset();

                // release space
                if (pModeAccelFrame) {
                    pModeAccelFrame->Release();
                }
                if (pSensorFrame)
                {
                    pSensorFrame->Release();
                }
            }
        }
        catch (...) {}

        pHL2ResearchMode->m_accelSensor->CloseStream();
        pHL2ResearchMode->m_accelSensor->Release();
        pHL2ResearchMode->m_accelSensor = nullptr;

    }

    void HL2ResearchMode::StartGyroSensorLoop()
    {
        m_pGyroUpdateThread = new std::thread(HL2ResearchMode::GyroSensorLoop, this);
    }

    void HL2ResearchMode::GyroSensorLoop(HL2ResearchMode* pHL2ResearchMode)
    {
        // prevent starting loop for multiple times
        if (!pHL2ResearchMode->m_gyroSensorLoopStarted)
        {
            pHL2ResearchMode->m_gyroSensorLoopStarted = true;
        }
        else {
            return;
        }

        try
        {
            winrt::check_hresult(pHL2ResearchMode->m_gyroSensor->OpenStream());

            while (pHL2ResearchMode->m_gyroSensorLoopStarted)
            {
                IResearchModeSensorFrame* pSensorFrame = nullptr;
                winrt::check_hresult(pHL2ResearchMode->m_gyroSensor->GetNextBuffer(&pSensorFrame));

                IResearchModeGyroFrame* pModeGyroFrame = nullptr;
                winrt::check_hresult(pSensorFrame->QueryInterface(IID_PPV_ARGS(&pModeGyroFrame)));

                DirectX::XMFLOAT3 pSample;
                winrt::check_hresult(pModeGyroFrame->GetCalibratedGyro(&pSample));

                auto pGyroSample = std::make_unique<float[]>(3);

                // save data
                {
                    std::lock_guard<std::mutex> l(pHL2ResearchMode->mu);

                    // save raw gyro sample
                    if (!pHL2ResearchMode->m_gyroSample)
                    {
                        OutputDebugString(L"Create Space for gyro sample...\n");
                        pHL2ResearchMode->m_gyroSample = new float[3];
                    }
                    pHL2ResearchMode->m_gyroSample[0] = pSample.x;
                    pHL2ResearchMode->m_gyroSample[1] = pSample.y;
                    pHL2ResearchMode->m_gyroSample[2] = pSample.z;
                }

                pHL2ResearchMode->m_gyroSampleUpdated = true;

                pGyroSample.reset();

                // release space
                if (pModeGyroFrame) {
                    pModeGyroFrame->Release();
                }
                if (pSensorFrame)
                {
                    pSensorFrame->Release();
                }
            }
        }
        catch (...) {}
        pHL2ResearchMode->m_gyroSensor->CloseStream();
        pHL2ResearchMode->m_gyroSensor->Release();
        pHL2ResearchMode->m_gyroSensor = nullptr;

    }

    void HL2ResearchMode::StartMagSensorLoop()
    {
        m_pMagUpdateThread = new std::thread(HL2ResearchMode::MagSensorLoop, this);
    }

    void HL2ResearchMode::MagSensorLoop(HL2ResearchMode* pHL2ResearchMode)
    {
        // prevent starting loop for multiple times
        if (!pHL2ResearchMode->m_magSensorLoopStarted)
        {
            pHL2ResearchMode->m_magSensorLoopStarted = true;
        }
        else {
            return;
        }

        try
        {
            winrt::check_hresult(pHL2ResearchMode->m_magSensor->OpenStream());

            while (pHL2ResearchMode->m_magSensorLoopStarted)
            {
                IResearchModeSensorFrame* pSensorFrame = nullptr;
                winrt::check_hresult(pHL2ResearchMode->m_magSensor->GetNextBuffer(&pSensorFrame));

                IResearchModeMagFrame* pModeMagFrame = nullptr;
                winrt::check_hresult(pSensorFrame->QueryInterface(IID_PPV_ARGS(&pModeMagFrame)));

                DirectX::XMFLOAT3 pSample;
                winrt::check_hresult(pModeMagFrame->GetMagnetometer(&pSample));

                auto pMagSample = std::make_unique<float[]>(3);

                // save data
                {
                    std::lock_guard<std::mutex> l(pHL2ResearchMode->mu);

                    // save raw gyro sample
                    if (!pHL2ResearchMode->m_magSample)
                    {
                        OutputDebugString(L"Create Space for mag sample...\n");
                        pHL2ResearchMode->m_magSample = new float[3];
                    }
                    pHL2ResearchMode->m_magSample[0] = pSample.x;
                    pHL2ResearchMode->m_magSample[1] = pSample.y;
                    pHL2ResearchMode->m_magSample[2] = pSample.z;
                }

                pHL2ResearchMode->m_magSampleUpdated = true;

                pMagSample.reset();

                // release space
                if (pModeMagFrame) {
                    pModeMagFrame->Release();
                }
                if (pSensorFrame)
                {
                    pSensorFrame->Release();
                }
            }
        }
        catch (...) {}
        pHL2ResearchMode->m_magSensor->CloseStream();
        pHL2ResearchMode->m_magSensor->Release();
        pHL2ResearchMode->m_magSensor = nullptr;

    }

    void HL2ResearchMode::CamAccessOnComplete(ResearchModeSensorConsent consent)
    {
        camAccessCheck = consent;
        SetEvent(camConsentGiven);
    }

    void HL2ResearchMode::ImuAccessOnComplete(ResearchModeSensorConsent consent)
    {
        imuAccessCheck = consent;
        SetEvent(imuConsentGiven);
    }

    inline UINT16 HL2ResearchMode::GetCenterDepth() {return m_centerDepth;}

    inline int HL2ResearchMode::GetDepthBufferSize() { return m_depthBufferSize; }

    inline bool HL2ResearchMode::DepthMapTextureUpdated() { return m_depthMapTextureUpdated; }

    inline bool HL2ResearchMode::ShortAbImageTextureUpdated() { return m_shortAbImageTextureUpdated; }

    inline bool HL2ResearchMode::LongAbImageTextureUpdated() { return m_longAbImageTextureUpdated; }

    inline bool HL2ResearchMode::PointCloudUpdated() { return m_pointCloudUpdated; }

    inline bool HL2ResearchMode::LongThrowPointCloudUpdated() { return m_longThrowPointCloudUpdated; }

    inline int HL2ResearchMode::GetLongDepthBufferSize() { return m_longDepthBufferSize; }

    inline bool HL2ResearchMode::LongDepthMapTextureUpdated() { return m_longDepthMapTextureUpdated; }

	inline bool HL2ResearchMode::LFImageUpdated() { return m_LFImageUpdated; }

	inline bool HL2ResearchMode::RFImageUpdated() { return m_RFImageUpdated; }

    inline bool HL2ResearchMode::AccelSampleUpdated() { return m_accelSampleUpdated; }

    inline bool HL2ResearchMode::GyroSampleUpdated() { return m_gyroSampleUpdated; }

    inline bool HL2ResearchMode::MagSampleUpdated() { return m_magSampleUpdated; }

    hstring HL2ResearchMode::PrintDepthResolution()
    {
        std::string res_c_ctr = std::to_string(m_depthResolution.Height) + "x" + std::to_string(m_depthResolution.Width) + "x" + std::to_string(m_depthResolution.BytesPerPixel);
        return winrt::to_hstring(res_c_ctr);
    }

    hstring HL2ResearchMode::PrintDepthExtrinsics()
    {
        std::stringstream ss;
        ss << "Extrinsics: \n" << MatrixToString(m_depthCameraPose);
        std::string msg = ss.str();
        std::wstring widemsg = std::wstring(msg.begin(), msg.end());
        OutputDebugString(widemsg.c_str());
        return winrt::to_hstring(msg);
    }

	hstring HL2ResearchMode::PrintLFResolution()
	{
		std::string res_c_ctr = std::to_string(m_LFResolution.Height) + "x" + std::to_string(m_LFResolution.Width) + "x" + std::to_string(m_LFResolution.BytesPerPixel);
		return winrt::to_hstring(res_c_ctr);
	}

	hstring HL2ResearchMode::PrintLFExtrinsics()
	{
		std::stringstream ss;
		ss << "Extrinsics: \n" << MatrixToString(m_LFCameraPose);
		std::string msg = ss.str();
		std::wstring widemsg = std::wstring(msg.begin(), msg.end());
		OutputDebugString(widemsg.c_str());
		return winrt::to_hstring(msg);
	}

	hstring HL2ResearchMode::PrintRFResolution()
	{
		std::string res_c_ctr = std::to_string(m_RFResolution.Height) + "x" + std::to_string(m_RFResolution.Width) + "x" + std::to_string(m_RFResolution.BytesPerPixel);
		return winrt::to_hstring(res_c_ctr);
	}

	hstring HL2ResearchMode::PrintRFExtrinsics()
	{
		std::stringstream ss;
		ss << "Extrinsics: \n" << MatrixToString(m_RFCameraPose);
		std::string msg = ss.str();
		std::wstring widemsg = std::wstring(msg.begin(), msg.end());
		OutputDebugString(widemsg.c_str());
		return winrt::to_hstring(msg);
	}

    std::string HL2ResearchMode::MatrixToString(DirectX::XMFLOAT4X4 mat)
    {
        std::stringstream ss;
        for (size_t i = 0; i < 4; i++)
        {
            for (size_t j = 0; j < 4; j++)
            {
                ss << mat(i, j) << ",";
            }
            ss << "\n";
        }
        return ss.str();
    }
    
    // Stop the sensor loop and release buffer space.
    // Sensor object should be released at the end of the loop function
    void HL2ResearchMode::StopAllSensorDevice()
    {
        m_depthSensorLoopStarted = false;
        //m_pDepthUpdateThread->join();
        if (m_depthMap) 
        {
            delete[] m_depthMap;
            m_depthMap = nullptr;
        }
        if (m_depthMapTexture) 
        {
            delete[] m_depthMapTexture;
            m_depthMapTexture = nullptr;
        }
        if (m_pointCloud) 
        {
            m_pointcloudLength = 0;
            delete[] m_pointCloud;
            m_pointCloud = nullptr;
        }
        if (m_shortAbImage) 
        {
            delete[] m_shortAbImage;
            m_shortAbImage = nullptr;
        }
        if (m_shortAbImageTexture)
        {
            delete[] m_shortAbImageTexture;
            m_shortAbImageTexture = nullptr;
        }
        if (m_longAbImage)
        {
            delete[] m_longAbImage;
            m_longAbImage = nullptr;
        }
        if (m_longAbImageTexture)
        {
            delete[] m_longAbImageTexture;
            m_longAbImageTexture = nullptr;
        }

        if (m_lastSpatialFrame.LFFrame.image) 
        {
            delete[] m_lastSpatialFrame.LFFrame.image;
            m_lastSpatialFrame.LFFrame.image = nullptr;
        }
        if (m_lastSpatialFrame.RFFrame.image)
        {
            delete[] m_lastSpatialFrame.RFFrame.image;
            m_lastSpatialFrame.RFFrame.image = nullptr;
        }

        m_longDepthSensorLoopStarted = false;
        if (m_longDepthMap)
        {
            delete[] m_longDepthMap;
            m_longDepthMap = nullptr;
        }
        if (m_longDepthMapTexture)
        {
            delete[] m_longDepthMapTexture;
            m_longDepthMapTexture = nullptr;
        }

        m_accelSensorLoopStarted = false;
        if (m_accelSample)
        {
            delete[] m_accelSample;
            m_accelSample = nullptr;
        }
        m_gyroSensorLoopStarted = false;
        if (m_gyroSample)
        {
            delete[] m_gyroSample;
            m_gyroSample = nullptr;
        }
        m_magSensorLoopStarted = false;
        if (m_magSample)
        {
            delete[] m_magSample;
            m_magSample = nullptr;
        }

		m_pSensorDevice->Release();
		m_pSensorDevice = nullptr;
		m_pSensorDeviceConsent->Release();
		m_pSensorDeviceConsent = nullptr;
    }

    com_array<uint16_t> HL2ResearchMode::GetDepthMapBuffer()
    {
        std::lock_guard<std::mutex> l(mu);
        if (!m_depthMap)
        {
            return com_array<uint16_t>();
        }
        com_array<UINT16> tempBuffer = com_array<UINT16>(m_depthMap, m_depthMap + m_depthBufferSize);
        
        return tempBuffer;
    }

    com_array<uint16_t> HL2ResearchMode::GetShortAbImageBuffer()
    {
        std::lock_guard<std::mutex> l(mu);
        if (!m_shortAbImage)
        {
            return com_array<uint16_t>();
        }
        com_array<UINT16> tempBuffer = com_array<UINT16>(m_shortAbImage, m_shortAbImage + m_depthBufferSize);

        return tempBuffer;
    }

    // Get depth map texture buffer. (For visualization purpose)
    com_array<uint8_t> HL2ResearchMode::GetDepthMapTextureBuffer()
    {
        std::lock_guard<std::mutex> l(mu);
        if (!m_depthMapTexture) 
        {
            return com_array<UINT8>();
        }
        com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(m_depthMapTexture), std::move_iterator(m_depthMapTexture + m_depthBufferSize));

        m_depthMapTextureUpdated = false;
        return tempBuffer;
    }

    // Get depth map texture buffer. (For visualization purpose)
    com_array<uint8_t> HL2ResearchMode::GetShortAbImageTextureBuffer()
    {
        std::lock_guard<std::mutex> l(mu);
        if (!m_shortAbImageTexture)
        {
            return com_array<UINT8>();
        }
        com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(m_shortAbImageTexture), std::move_iterator(m_shortAbImageTexture + m_depthBufferSize));

        m_shortAbImageTextureUpdated = false;
        return tempBuffer;
    }

    com_array<uint16_t> HL2ResearchMode::GetLongDepthMapBuffer()
    {
        std::lock_guard<std::mutex> l(mu);
        if (!m_longDepthMap)
        {
            return com_array<uint16_t>();
        }
        com_array<UINT16> tempBuffer = com_array<UINT16>(m_longDepthMap, m_longDepthMap + m_longDepthBufferSize);

        return tempBuffer;
    }


    com_array<uint16_t> HL2ResearchMode::GetLongAbImageBuffer()
    {
        std::lock_guard<std::mutex> l(mu);
        if (!m_longAbImage)
        {
            return com_array<uint16_t>();
        }
        com_array<UINT16> tempBuffer = com_array<UINT16>(m_longAbImage, m_longAbImage + m_longDepthBufferSize);

        return tempBuffer;
    }

    com_array<uint8_t> HL2ResearchMode::GetLongDepthMapTextureBuffer()
    {
        std::lock_guard<std::mutex> l(mu);
        if (!m_longDepthMapTexture)
        {
            return com_array<UINT8>();
        }
        com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(m_longDepthMapTexture), std::move_iterator(m_longDepthMapTexture + m_longDepthBufferSize));

        m_longDepthMapTextureUpdated = false;
        return tempBuffer;
    }

    // Get depth map texture buffer. (For visualization purpose)
    com_array<uint8_t> HL2ResearchMode::GetLongAbImageTextureBuffer()
    {
        std::lock_guard<std::mutex> l(mu);
        if (!m_longAbImageTexture)
        {
            return com_array<UINT8>();
        }
        com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(m_longAbImageTexture), std::move_iterator(m_longAbImageTexture + m_longDepthBufferSize));

        m_longAbImageTextureUpdated = false;
        return tempBuffer;
    }

    com_array<uint8_t> HL2ResearchMode::GetLFCameraBuffer(int64_t& ts)
	{
		std::lock_guard<std::mutex> l(mu);
		if (!m_lastSpatialFrame.LFFrame.image)
		{
			return com_array<UINT8>();
		}
        com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(m_lastSpatialFrame.LFFrame.image), std::move_iterator(m_lastSpatialFrame.LFFrame.image + m_LFbufferSize));
        ts = m_lastSpatialFrame.LFFrame.timestamp_ft;
        m_LFImageUpdated = false;
        return tempBuffer;
	}

    com_array<uint8_t> HL2ResearchMode::GetRFCameraBuffer(int64_t& ts)
	{
		std::lock_guard<std::mutex> l(mu);
		if (!m_lastSpatialFrame.RFFrame.image)
		{
            return com_array<UINT8>();
		}
        com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(m_lastSpatialFrame.RFFrame.image), std::move_iterator(m_lastSpatialFrame.RFFrame.image + m_RFbufferSize));
        ts = m_lastSpatialFrame.RFFrame.timestamp_ft;
        m_RFImageUpdated = false;
		return tempBuffer;
	}

    com_array<uint8_t> HL2ResearchMode::GetLRFCameraBuffer(int64_t& ts_left, int64_t& ts_right)
    {
        std::lock_guard<std::mutex> l(mu);
        if (!m_lastSpatialFrame.LFFrame.image || !m_lastSpatialFrame.RFFrame.image)
        {
            return com_array<UINT8>();
        }
        

        UINT8* rawTempBuffer = new UINT8[m_LFbufferSize + m_RFbufferSize];
        memcpy(rawTempBuffer, m_lastSpatialFrame.LFFrame.image, m_LFbufferSize);
        memcpy(rawTempBuffer + m_LFbufferSize, m_lastSpatialFrame.RFFrame.image, m_RFbufferSize);

        com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(rawTempBuffer), std::move_iterator(rawTempBuffer + m_LFbufferSize + m_RFbufferSize));
        ts_left = m_lastSpatialFrame.LFFrame.timestamp_ft;
        ts_right = m_lastSpatialFrame.RFFrame.timestamp_ft;

        //std::stringstream ss;
        //ss << "HostTicks: " << m_lastSpatialFrame.LFFrame.timestamp <<
        //    "\nUnix: " << m_lastSpatialFrame.LFFrame.timestamp_unix << 
        //    "\nFt: " << m_lastSpatialFrame.LFFrame.timestamp_ft << "\n";
        //std::string msg = ss.str();
        //std::wstring widemsg = std::wstring(msg.begin(), msg.end());
        //OutputDebugString(widemsg.c_str());

        m_LFImageUpdated = false;
        m_RFImageUpdated = false;
        return tempBuffer;
    }

    com_array<float> HL2ResearchMode::GetAccelSample()
    {
        std::lock_guard<std::mutex> l(mu);
        if (!m_accelSample)
        {
            return com_array<float>(3);
        }
        com_array<float> tempBuffer = com_array<float>(std::move_iterator(m_accelSample), std::move_iterator(m_accelSample + 3));
        m_accelSampleUpdated = false;
        return tempBuffer;
    }

    com_array<float> HL2ResearchMode::GetGyroSample()
    {
        std::lock_guard<std::mutex> l(mu);
        if (!m_gyroSample)
        {
            return com_array<float>(3);
        }
        com_array<float> tempBuffer = com_array<float>(std::move_iterator(m_gyroSample), std::move_iterator(m_gyroSample + 3));
        m_gyroSampleUpdated = false;
        return tempBuffer;
    }

    com_array<float> HL2ResearchMode::GetMagSample()
    {
        std::lock_guard<std::mutex> l(mu);
        if (!m_magSample)
        {
            return com_array<float>(3);
        }
        com_array<float> tempBuffer = com_array<float>(std::move_iterator(m_magSample), std::move_iterator(m_magSample + 3));
        m_magSampleUpdated = false;
        return tempBuffer;
    }

    // Get the buffer for point cloud in the form of float array.
    // There will be 3n elements in the array where the 3i, 3i+1, 3i+2 element correspond to x, y, z component of the i'th point. (i->[0,n-1])
    com_array<float> HL2ResearchMode::GetPointCloudBuffer()
    {
        std::lock_guard<std::mutex> l(mu);
        if (!m_reconstructShortThrowPointCloud || m_pointcloudLength == 0)
        {
            return com_array<float>();
        }
        com_array<float> tempBuffer = com_array<float>(std::move_iterator(m_pointCloud), std::move_iterator(m_pointCloud + m_pointcloudLength));
        m_pointCloudUpdated = false;
        return tempBuffer;
    }

    // Get the buffer for point cloud in the form of float array.
    // There will be 3n elements in the array where the 3i, 3i+1, 3i+2 element correspond to x, y, z component of the i'th point. (i->[0,n-1])
    com_array<float> HL2ResearchMode::GetLongThrowPointCloudBuffer()
    {
        std::lock_guard<std::mutex> l(mu);
        {
            std::stringstream ss;
            ss << "m_reconstructLongThrowPointCloud: " << m_reconstructLongThrowPointCloud << "\n";
            ss << "m_pointcloudLength: " << m_longThrowPointcloudLength << "\n";
            std::string msg = ss.str();
            std::wstring widemsg = std::wstring(msg.begin(), msg.end());
            OutputDebugString(widemsg.c_str());
        }
        if (!m_reconstructLongThrowPointCloud || m_longThrowPointcloudLength == 0)
        {
            return com_array<float>();
        };
        com_array<float> tempBuffer = com_array<float>(std::move_iterator(m_longThrowPointCloud), std::move_iterator(m_longThrowPointCloud + m_longThrowPointcloudLength));
        m_longThrowPointCloudUpdated = false;
        return tempBuffer;
    }

    // Get the 3D point (float[3]) of center point in depth map. Can be used to render depth cursor.
    com_array<float> HL2ResearchMode::GetCenterPoint()
    {
        std::lock_guard<std::mutex> l(mu);
        com_array<float> centerPoint = com_array<float>(std::move_iterator(m_centerPoint), std::move_iterator(m_centerPoint + 3));

        return centerPoint;
    }

    // Set the reference coordinate system. Need to be set before the sensor loop starts; otherwise, default coordinate will be used.
    void HL2ResearchMode::SetReferenceCoordinateSystem(winrt::Windows::Perception::Spatial::SpatialCoordinateSystem refCoord)
    {
        m_refFrame = refCoord;
    }

    void HL2ResearchMode::SetPointCloudRoiInSpace(float centerX, float centerY, float centerZ, float boundX, float boundY, float boundZ)
    {
        std::lock_guard<std::mutex> l(mu);

        m_useRoiFilter = true;
        m_roiCenter[0] = centerX;
        m_roiCenter[1] = centerY;
        m_roiCenter[2] = -centerZ;

        m_roiBound[0] = boundX;
        m_roiBound[1] = boundY;
        m_roiBound[2] = boundZ;
    }

    void HL2ResearchMode::SetPointCloudDepthOffset(uint16_t offset)
    {
        m_depthOffset = offset;
    }

    long long HL2ResearchMode::checkAndConvertUnsigned(UINT64 val)
    {
        assert(val <= kMaxLongLong);
        return static_cast<long long>(val);
    }

    XMMATRIX HL2ResearchMode::SpatialLocationToDxMatrix(SpatialLocation location) {
        auto rot = location.Orientation();
        auto quatInDx = XMFLOAT4(rot.x, rot.y, rot.z, rot.w);
        auto rotMat = XMMatrixRotationQuaternion(XMLoadFloat4(&quatInDx));
        auto pos = location.Position();
        auto posMat = XMMatrixTranslation(pos.x, pos.y, pos.z);
        return rotMat * posMat;
    }

}
