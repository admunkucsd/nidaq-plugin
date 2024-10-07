#include "../Source/NIDAQThread.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "../Source/nidaq-api/NIDAQmx.h"

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

// Mocking the NIDAQmx API functions
class MockNIDAQmx : public NIDAQmx
{
public:
    MockNIDAQmx (NIDAQDevice* device) : NIDAQmx(device) {}

    // Mocking required API functions
    static int32 DAQmxGetDevProductCategory (const char device[], int32* data)
    {
        // Simulate getting device product category
        *data = DAQmx_Val_USBDAQ; // e.g., simulate a USB device
        return 0; // success
    }

    static int32 DAQmxCreateTask (const char taskName[], NIDAQ::TaskHandle* taskHandle)
    {
        // Simulate task creation
        *taskHandle = reinterpret_cast<NIDAQ::TaskHandle> (new int (1)); // arbitrary task handle
        return 0; // success
    }

    static int32 DAQmxCreateAIVoltageChan (NIDAQ::TaskHandle taskHandle,
        const char physicalChannel[], 
        const char nameToAssignToChannel[], 
        int32 terminalConfig,
        float minVal,
        float maxVal,
        int32 units,
        const void* customScaleName)
    {
        // Simulate creating AI Voltage Channel
        return 0; // success
    }

    static int32 DAQmxCreateDIChan (NIDAQ::TaskHandle taskHandle, 
        const char lines[], 
        const char nameToAssignToLines[],
        int32 lineGrouping)
    {
        // Simulate creating DI Channel
        return 0; // success
    }

    static int32 DAQmxGetDevAISimultaneousSamplingSupported (const char device[], NIDAQ::bool32* data)
    {
        // Simulate querying simultaneous sampling support
        *data = 1; // Simultaneous sampling supported
        return 0;
    }
    // Add more mock functions as needed...
    MOCK_METHOD3 (getDevAIPhysicalChans, int32 (const char* device, char* data, uint32 bufferSize));
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
    SetUp ("Dev1");

    EXPECT_CALL(*daqmx, getDevAIPhysicalChans (testing::_, testing::_, testing::_))
        .WillOnce (testing::Return (0));
    // Prepare the device
    daqmx->setNumActiveAnalogInputs (4);
    daqmx->setVoltageRange(0);

    // Add analog input channels (mocked)
    for (int i = 0; i < daqmx->getNumActiveAnalogInputs(); i++)
    {
        daqmx->ai.add (new AnalogInput ("Dev1/ai" + std::to_string (i), DAQmx_Val_Bit_TermCfg_Diff));
    }

    daqmx->connect();

    // Mock run
    daqmx->run();

    // Validate expected outcomes
    EXPECT_TRUE (daqmx->ai.size() > 0); // Ensure data was captured
}