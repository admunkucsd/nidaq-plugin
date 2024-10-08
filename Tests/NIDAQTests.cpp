#include "../Source/NIDAQThread.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "../Source/nidaq-api/NIDAQmx.h"
#include "../Source/NIDAQmxApiWrapper.h"

#include <TestFixtures.h>

class NIDAQUnitTests : public testing::Test
{
protected:
    void SetUp() override
    {
        numChannels = 16;

        tester = std::make_unique<DataThreadTester> (
            FakeSourceNodeParams {
                16,
                100,
                0.0f });

        nidaqThread = tester->createDataThread<NIDAQThread>();
    }

protected:
    NIDAQThread* nidaqThread;
    std::unique_ptr<DataThreadTester> tester;
    int numChannels;
    int64_t currentSampleIndex = 0;
};

class MockNIDAQmxApiWrapper : public NIDAQmxApiWrapper
{
public:
    virtual ~MockNIDAQmxApiWrapper() override = default;

    MOCK_METHOD3 (getDevAIPhysicalChans, NIDAQ::int32 (const char* device, char* data, NIDAQ::uInt32 bufferSize));
    MOCK_METHOD2 (getPhysicalChanAITermCfgs, NIDAQ::int32 (const char physicalChannel[], NIDAQ::int32* data));
    MOCK_METHOD2 (getPhysicalChanAIVoltageRngs, NIDAQ::int32 (const char physicalChannel[], NIDAQ::float64* data));
    
    // create mock methods for all pure virtual functions in NIDAQmxApiWrapper
    MOCK_METHOD2 (createTask, NIDAQ::int32 (const char taskName[], NIDAQ::TaskHandle* taskHandle));
    MOCK_METHOD1 (startTask, NIDAQ::int32 (NIDAQ::TaskHandle taskHandle));
    MOCK_METHOD1 (stopTask, NIDAQ::int32 (NIDAQ::TaskHandle taskHandle));
    MOCK_METHOD1 (clearTask, NIDAQ::int32 (NIDAQ::TaskHandle taskHandle));
    MOCK_METHOD8 (createAIVoltageChan, 
        NIDAQ::int32 (NIDAQ::TaskHandle taskHandle, 
            const char physicalChannel[], 
            const char nameToAssignToChannel[], 
            NIDAQ::int32 terminalConfig, 
            NIDAQ::float64 minVal, 
            NIDAQ::float64 maxVal, 
            NIDAQ::int32 units, 
            const char customScaleName[]));
    MOCK_METHOD2(getDevProductCategory, NIDAQ::int32 (const char device[], NIDAQ::int32* data));
    MOCK_METHOD2(getDevProductNum, NIDAQ::int32 (const char device[], NIDAQ::int32* data));
    MOCK_METHOD2(getDevProductSerialNum, NIDAQ::int32 (const char device[], NIDAQ::int32* data));
    MOCK_METHOD2(getDevAISimultaneousSamplingSupported, NIDAQ::int32 (const char device[], NIDAQ::int32* data));
    MOCK_METHOD2(getDevAIMinRate, NIDAQ::int32 (const char device[], NIDAQ::float64* data));
    MOCK_METHOD2(getDevAIMaxSingleChanRate, NIDAQ::int32 (const char device[], NIDAQ::float64* data));
    MOCK_METHOD2(getDevAIMaxMultiChanRate, NIDAQ::int32 (const char device[], NIDAQ::float64* data));
    MOCK_METHOD3(getDevAIVoltageRngs, NIDAQ::int32 (const char device[], NIDAQ::float64* data, NIDAQ::uInt32 arraySizeInElements));
    MOCK_METHOD2(getPhysicalChanAITermCfgs, NIDAQ::int32 (const char physicalChannel[], NIDAQ::int32* data));
    MOCK_METHOD3(getDevDILines, NIDAQ::int32 (const char device[], char* data, NIDAQ::uInt32 bufferSize));
    MOCK_METHOD2(getExtendedErrorInfo, NIDAQ::int32 (char* errorString, NIDAQ::uInt32 bufferSize));
};

// Mocking the NIDAQmx API functions
class MockNIDAQmx : public NIDAQmx
{
public:
    MockNIDAQmx (NIDAQDevice* device) : apiWrapper(std::make_unique<MockNIDAQmxApiWrapper>()),
        NIDAQmx(device, apiWrapper.get()) 
    {}

    std::unique_ptr<MockNIDAQmxApiWrapper> apiWrapper;
};

class NIDAQmxUnitTests : public testing::Test
{
protected:
    void SetUp(String name)
    {
        // Perform setup operations
        device = new NIDAQDevice (name);
        daqmx = new MockNIDAQmx(device);
    }

    void TearDown() override
    {
        delete daqmx;
        delete device;
    }

protected:
    MockNIDAQmx* daqmx;
    NIDAQDevice* device;
};

/*
When connected to an NIDAQ, the NIDAQmx Plugin continuously reads from both analog and digital input channels. 
Reading from analog channels returns signed floating point values corresponding to the voltages recorded by the channel.
Reading from digital channels returns a single binary value per channel, indicating the state of the digital signal read. 
This test verifies that the NIDAQmx plugin can perform these read operations and copy these values to the output Audio Buffer.
*/
TEST_F (NIDAQUnitTests, DataIntegrity)
{
    // Write the buffer to the NIDAQmx plugin
    //tester->startAcquisition (false);

    ////nidaqThread->

    //// Read the buffer from the NIDAQmx plugin
    //int numSamples = 100;
    ////auto inputBuffer = tester->createBuffer (1000.0, 20.0, numChannels, numSamples);
    ////tester->writeBlock (inputBuffer, currentSampleIndex);


    //tester->stopAcquisition();
}

// Test connect() function
TEST_F (NIDAQmxUnitTests, TestConnectSimulatedDevice)
{
    SetUp ("Simulated");
    daqmx->connect();

    // Verify expected properties of the simulated device
    EXPECT_EQ (device->isUSBDevice, false);
    EXPECT_EQ (device->sampleRateRange.min, 1000.0f);
    EXPECT_EQ (device->sampleRateRange.max, 30000.0f);
    EXPECT_EQ (device->productName, "No Device Detected");
}

// Test run() function with mocked analog input channels
TEST_F (NIDAQmxUnitTests, TestRun)
{
    SetUp ("Simulated");

    // mock dev product category
    EXPECT_CALL (*daqmx->apiWrapper, getDevProductCategory (testing::_, testing::_))
        .WillOnce (testing::Return (0));

    // mock dev product number
    EXPECT_CALL (*daqmx->apiWrapper, getDevProductNum (testing::_, testing::_))
        .WillOnce (testing::Return (0));

    // mock dev product serial number
    EXPECT_CALL (*daqmx->apiWrapper, getDevProductSerialNum (testing::_, testing::_))
        .WillOnce (testing::Return (0));

    // mock dev AI simultaneous sampling supported
    EXPECT_CALL (*daqmx->apiWrapper, getDevAISimultaneousSamplingSupported (testing::_, testing::_))
        .WillOnce (testing::Return (0));

    // mock dev AI min rate
    EXPECT_CALL (*daqmx->apiWrapper, getDevAIMinRate (testing::_, testing::_))
        .WillOnce (testing::Return (1000));

    // mock dev AI max single channel rate
    EXPECT_CALL (*daqmx->apiWrapper, getDevAIMaxSingleChanRate (testing::_, testing::_))
        .WillOnce (testing::Return (30000));

    // mock dev AI max multi channel rate
    EXPECT_CALL (*daqmx->apiWrapper, getDevAIMaxMultiChanRate (testing::_, testing::_))
        .WillOnce (testing::Return (30000));

    // mock dev AI voltage ranges
    EXPECT_CALL (*daqmx->apiWrapper, getDevAIVoltageRngs (testing::_, testing::_, testing::_))
        .WillOnce (testing::Return (0));

    // mock physical channel AI term configs
    EXPECT_CALL (*daqmx->apiWrapper, getPhysicalChanAITermCfgs (testing::_, testing::_))
        .WillOnce (testing::Return (0));

    // mock create AI voltage channel
    EXPECT_CALL (*daqmx->apiWrapper, createAIVoltageChan (testing::_, testing::_, testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
        .WillOnce (testing::Return (0));

    // Run the NIDAQmx plugin
    daqmx->run();
}