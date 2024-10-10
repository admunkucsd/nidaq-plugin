#include "../Source/NIDAQThread.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include <TestFixtures.h>

using testing::_;
using testing::Return;

class MockNIDAQmxAPI : public NIDAQmxAPI
{
public:
    virtual ~MockNIDAQmxAPI() override = default;

    MOCK_METHOD3 (getDevAIPhysicalChans, NIDAQmxAPI::int32 (const char* device, char* data, NIDAQmxAPI::uInt32 bufferSize));
    MOCK_METHOD2 (getPhysicalChanAIVoltageRngs, NIDAQmxAPI::int32 (const char physicalChannel[], NIDAQmxAPI::float64* data));
    MOCK_METHOD2 (createTask, NIDAQmxAPI::int32 (const char taskName[], NIDAQmxAPI::TaskHandle* taskHandle));
    MOCK_METHOD1 (startTask, NIDAQmxAPI::int32 (NIDAQmxAPI::TaskHandle taskHandle));
    MOCK_METHOD1 (stopTask, NIDAQmxAPI::int32 (NIDAQmxAPI::TaskHandle taskHandle));
    MOCK_METHOD1 (clearTask, NIDAQmxAPI::int32 (NIDAQmxAPI::TaskHandle taskHandle));
    MOCK_METHOD8 (createAIVoltageChan, NIDAQmxAPI::int32 (NIDAQmxAPI::TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], NIDAQmxAPI::int32 terminalConfig, NIDAQmxAPI::float64 minVal, NIDAQmxAPI::float64 maxVal, NIDAQmxAPI::int32 units, const char customScaleName[]));
    MOCK_METHOD2 (getDevProductCategory, NIDAQmxAPI::int32 (const char device[], NIDAQmxAPI::int32* data));
    MOCK_METHOD2 (getDevProductNum, NIDAQmxAPI::int32 (const char device[], NIDAQmxAPI::uInt32* data));
    MOCK_METHOD2 (getDevSerialNum, NIDAQmxAPI::int32 (const char device[], NIDAQmxAPI::uInt32* data));
    MOCK_METHOD2 (getDevAISimultaneousSamplingSupported, NIDAQmxAPI::int32 (const char device[], NIDAQmxAPI::bool32* data));
    MOCK_METHOD2 (getDevAIMinRate, NIDAQmxAPI::int32 (const char device[], NIDAQmxAPI::float64* data));
    MOCK_METHOD2 (getDevAIMaxSingleChanRate, NIDAQmxAPI::int32 (const char device[], NIDAQmxAPI::float64* data));
    MOCK_METHOD2 (getDevAIMaxMultiChanRate, NIDAQmxAPI::int32 (const char device[], NIDAQmxAPI::float64* data));
    MOCK_METHOD3 (getDevAIVoltageRngs, NIDAQmxAPI::int32 (const char device[], NIDAQmxAPI::float64* data, NIDAQmxAPI::uInt32 arraySizeInElements));
    MOCK_METHOD2 (getPhysicalChanAITermCfgs, NIDAQmxAPI::int32 (const char physicalChannel[], NIDAQmxAPI::int32* data));
    MOCK_METHOD3 (getDevDILines, NIDAQmxAPI::int32 (const char device[], char* data, NIDAQmxAPI::uInt32 bufferSize));
    MOCK_METHOD2 (getExtendedErrorInfo, NIDAQmxAPI::int32 (char* errorString, NIDAQmxAPI::uInt32 bufferSize));
    MOCK_METHOD3 (getDevProductType, NIDAQmxAPI::int32 (const char device[], char* data, NIDAQmxAPI::uInt32 bufferSize));
    MOCK_METHOD3 (getAIResolution, NIDAQmxAPI::int32 (NIDAQmxAPI::TaskHandle taskHandle, const char channel[], NIDAQmxAPI::float64* data));
    MOCK_METHOD8 (readAnalogF64, NIDAQmxAPI::int32 (NIDAQmxAPI::TaskHandle taskHandle, int32 numSampsPerChan, float64 timeout, bool32 fillMode, float64* readArray, uInt32 arraySizeInSamps, int32* sampsPerChanRead, bool32* reserved));
    MOCK_METHOD8 (readDigitalU32, NIDAQmxAPI::int32 (NIDAQmxAPI::TaskHandle taskHandle, int32 numSampsPerChan, float64 timeout, bool32 fillMode, uInt32* readArray, uInt32 arraySizeInSamps, int32* sampsPerChanRead, bool32* reserved));
    MOCK_METHOD8 (readDigitalU16, NIDAQmxAPI::int32 (NIDAQmxAPI::TaskHandle taskHandle, int32 numSampsPerChan, float64 timeout, bool32 fillMode, uInt16* readArray, uInt32 arraySizeInSamps, int32* sampsPerChanRead, bool32* reserved));
    MOCK_METHOD8 (readDigitalU8, NIDAQmxAPI::int32 (NIDAQmxAPI::TaskHandle taskHandle, int32 numSampsPerChan, float64 timeout, bool32 fillMode, uInt8* readArray, uInt32 arraySizeInSamps, int32* sampsPerChanRead, bool32* reserved));
    MOCK_METHOD2 (getSysDevNames, NIDAQmxAPI::int32 (char* data, NIDAQmxAPI::uInt32 bufferSize));
};

class MockNIDAQmx : public NIDAQmx
{
public:
    MockNIDAQmx (NIDAQDevice* device, MockNIDAQmxAPI* wrapper) : NIDAQmx (device, wrapper)
    {
    }

    virtual ~MockNIDAQmx() override = default;
};

class NIDAQUnitTests : public testing::Test
{
protected:
    void SetUp (String name)
    {
        testing::FLAGS_gmock_verbose = "error";
        // Perform setup operations
        device = new NIDAQDevice (name);
        nidaqmx = new MockNIDAQmx (device, &apiWrapper);
    }

    void TearDown() override
    {
        if (nidaqmx)
            delete nidaqmx;
        if (device)
            delete device;
    }

protected:
    MockNIDAQmx* nidaqmx;
    NIDAQDevice* device;
    MockNIDAQmxAPI apiWrapper;
};

class NIDAQIntegrationTests : public testing::Test
{
protected:
    void SetUp() override
    {
        testing::FLAGS_gmock_verbose = "error";

        tester = std::make_unique<DataThreadTester> (
            FakeSourceNodeParams {
                numChannels,
                100,
                0.0f });
    }

    void TearDown() override
    {
    }

    void SetupMockCalls()
    {
        //ON_CALL (apiWrapper, getSysDevNames (_, _))
        //    .WillByDefault ([] (char* data, NIDAQmxAPI::uInt32 bufferSize)
        //                    {
        //    strcpy(data, "Dev1");
        //    LOGC("getSysDevNames called ", String(data));
        //    return 0; });
        //ON_CALL (apiWrapper, getDevProductType (_, _, _))
        //    .WillByDefault ([] (const char device[], char* data, NIDAQmxAPI::uInt32 bufferSize)
        //                    {
        //    strcpy(data, "Dev1");
        //    return 0; });
        //EXPECT_CALL (apiWrapper, readAnalogF64 (_, _, _, _, _, _, _, _))
        //    .WillOnce ([&] (NIDAQmxAPI::TaskHandle taskHandle,
        //                         NIDAQmxAPI::int32 numSampsPerChan,
        //                         NIDAQmxAPI::float64 timeout,
        //                         NIDAQmxAPI::bool32 fillMode,
        //                         NIDAQmxAPI::float64* readArray,
        //                         NIDAQmxAPI::uInt32 arraySizeInSamps,
        //                         NIDAQmxAPI::int32* sampsPerChanRead,
        //                         NIDAQmxAPI::bool32* reserved)
        //                    {
        //    numSampsPerChan = numSamplesPerChannel;

        //    for (int i = 0; i < numChannels; i++)
        //    {
        //        for(int j = 0; j < numSamplesPerChannel; j++)
        //        {
        //            readArray[i * numSamplesPerChannel + j] = i + j;
        //            sampleNumbers[i * numSamplesPerChannel + j] = static_cast<juce::int64> (i) + j;
        //        }
        //    }

        //    arraySizeInSamps = numChannels * numSamplesPerChannel;
        //    *sampsPerChanRead = numSamplesPerChannel;

        //    return 0; });
        //ON_CALL (apiWrapper, readDigitalU8 (_, _, _, _, _, _, _, _)).WillByDefault ([&] (NIDAQmxAPI::TaskHandle taskHandle, NIDAQmxAPI::int32 numSampsPerChan, NIDAQmxAPI::float64 timeout, NIDAQmxAPI::bool32 fillMode, NIDAQmxAPI::uInt8* readArray, NIDAQmxAPI::uInt32 arraySizeInSamps, NIDAQmxAPI::int32* sampsPerChanRead, NIDAQmxAPI::bool32* reserved)
        //                                                                            {
        //    numSampsPerChan = numSamplesPerChannel;

        //    for (int i = 0; i < numChannels; i++)
        //    {
        //        for(int j = 0; j < numSamplesPerChannel; j++)
        //        {
        //            readArray[i * numSamplesPerChannel + j] = i + j;
        //            sampleNumbers[i * numSamplesPerChannel + j] = static_cast<juce::int64> (i) + j;
        //        }
        //    }

        //    arraySizeInSamps = numChannels * numSamplesPerChannel;
        //    *sampsPerChanRead = numSamplesPerChannel;

        //    return 0; });
        //ON_CALL (apiWrapper, readDigitalU32 (_, _, _, _, _, _, _, _)).WillByDefault ([&] (NIDAQmxAPI::TaskHandle taskHandle, NIDAQmxAPI::int32 numSampsPerChan, NIDAQmxAPI::float64 timeout, NIDAQmxAPI::bool32 fillMode, NIDAQmxAPI::uInt32* readArray, NIDAQmxAPI::uInt32 arraySizeInSamps, NIDAQmxAPI::int32* sampsPerChanRead, NIDAQmxAPI::bool32* reserved)
        //                                                                             {
        //    numSampsPerChan = numSamplesPerChannel;

        //    for (int i = 0; i < numChannels; i++)
        //    {
        //        for(int j = 0; j < numSamplesPerChannel; j++)
        //        {
        //            readArray[i * numSamplesPerChannel + j] = i + j;
        //            sampleNumbers[i * numSamplesPerChannel + j] = static_cast<juce::int64> (i) + j;
        //        }
        //    }

        //    arraySizeInSamps = numChannels * numSamplesPerChannel;
        //    *sampsPerChanRead = numSamplesPerChannel;

        //    return 0; });
        //ON_CALL (apiWrapper, readDigitalU16 (_, _, _, _, _, _, _, _)).WillByDefault ([&] (NIDAQmxAPI::TaskHandle taskHandle, NIDAQmxAPI::int32 numSampsPerChan, NIDAQmxAPI::float64 timeout, NIDAQmxAPI::bool32 fillMode, NIDAQmxAPI::uInt16* readArray, NIDAQmxAPI::uInt32 arraySizeInSamps, NIDAQmxAPI::int32* sampsPerChanRead, NIDAQmxAPI::bool32* reserved)
        //                                                                             {
        //        numSampsPerChan = numSamplesPerChannel;

        //        for (int i = 0; i < numChannels; i++)
        //        {
        //            for(int j = 0; j < numSamplesPerChannel; j++)
        //            {
        //                readArray[i * numSamplesPerChannel + j] = i + j;
        //                sampleNumbers[i * numSamplesPerChannel + j] = static_cast<juce::int64> (i) + j;
        //            }
        //        }

        //        arraySizeInSamps = numChannels * numSamplesPerChannel;
        //        *sampsPerChanRead = numSamplesPerChannel;

        //        return 0; });

        //ON_CALL (apiWrapper, getDevProductCategory (_, _))
        //    .WillByDefault ([&] (const char[], NIDAQmxAPI::int32* data)
        //                    {
        //        *data = 1;  // Example category
        //        return 0; });
        //ON_CALL (apiWrapper, getDevProductNum (_, _))
        //    .WillByDefault ([&] (const char[], NIDAQmxAPI::uInt32* data)
        //                    {
        //        *data = 12345;  // Example product number
        //        return 0; });
        //ON_CALL (apiWrapper, getDevSerialNum (_, _))
        //    .WillByDefault ([&] (const char[], NIDAQmxAPI::uInt32* data)
        //                    {
        //        *data = 12345;  // Example serial number
        //        return 0; });
        //ON_CALL (apiWrapper, getDevAISimultaneousSamplingSupported (_, _))
        //    .WillByDefault ([&] (const char[], NIDAQmxAPI::bool32* data)
        //                    {
        //        *data = false;  // Example simultaneous sampling supported
        //        return 0; });
        //ON_CALL (apiWrapper, getDevAIMinRate (_, _))
        //    .WillByDefault ([&] (const char[], NIDAQmxAPI::float64* data)
        //                    {
        //        *data = 1000.0f;  // Example minimum rate
        //        return 0; });
        //ON_CALL (apiWrapper, getDevAIMaxSingleChanRate (_, _))
        //    .WillByDefault ([] (const char[], NIDAQmxAPI::float64* data)
        //                    {
        //        *data = 30000.0f;  // Example maximum single channel rate
        //        return 0; });
        //ON_CALL (apiWrapper, getDevAIMaxMultiChanRate (_, _))
        //    .WillByDefault ([&] (const char[], NIDAQmxAPI::float64* data)
        //                    {
        //        *data = 20000.0f;  // Example maximum multi-channel rate
        //        return 0; });
        //ON_CALL (apiWrapper, getDevAIVoltageRngs (_, _, _))
        //    .WillByDefault ([&] (const char[], NIDAQmxAPI::float64* data, NIDAQmxAPI::uInt32 arraySizeInElements)
        //                    {
        //        data[0] = -100.0f;
        //        data[1] = 100.0f;
        //        return 0; });
        //ON_CALL (apiWrapper, getDevAIPhysicalChans (_, _, _))
        //    .WillByDefault ([] (const char[], char* data, NIDAQmxAPI::uInt32 bufferSize)
        //                    {
        //        strcpy (data, "Dev1/ai0,Dev1/ai1,Dev1/ai2,Dev1/ai3,Dev1/ai4,Dev1/ai5,Dev1/ai6,Dev1/ai7");
        //        return 0; });
        //ON_CALL (apiWrapper, getPhysicalChanAITermCfgs (_, _))
        //    .WillByDefault ([] (const char[], NIDAQmxAPI::int32* data)
        //                    {
        //        *data = DAQmx_Val_RSE;  // Example terminal configuration
        //        return 0; });
        //ON_CALL (apiWrapper, getAIResolution (_, _, _))
        //    .WillByDefault ([&] (NIDAQmxAPI::TaskHandle taskHandle, const char channel[], NIDAQmxAPI::float64* data)
        //                    {
        //        *data = 12.0f;  // Example resolution
        //        return 0; });
        //ON_CALL (apiWrapper, getDevDILines (_, _, _))
        //    .WillByDefault ([] (const char device[], char* data, NIDAQmxAPI::uInt32 bufferSize)
        //                    {
        //        strcpy(data, "Dev1/port0/line0,Dev1/port0/line1,Dev1/port0/line2,Dev1/port0/line3,Dev1/port0/line4,Dev1/port0/line5,Dev1/port0/line6,Dev1/port0/line7");  // Example digital lines
        //        return 0; });
    }

    AudioBuffer<float> createBuffer (float startingValue, float step, int numChannels, int numSamples)
    {
        AudioBuffer<float> inputBuffer (numChannels, numSamples);

        // in microvolts
        float curValue = startingValue;

        for (int chidx = 0; chidx < numChannels; chidx++)
        {
            for (int sample_idx = 0; sample_idx < numSamples; sample_idx++)
            {
                inputBuffer.setSample (chidx, sample_idx, curValue);
                curValue += step;
            }
        }

        return inputBuffer;
    }

    void writeBlock (AudioBuffer<float>& buffer)
    {
        auto audioProcessor = (AudioProcessor*) tester->getSourceNode();
        auto dataStreams = tester->getSourceNode()->getDataStreams();

        ASSERT_EQ (dataStreams.size(), 1);

        auto streamId = dataStreams[0]->getStreamId();
        HeapBlock<char> data;
        size_t dataSize = SystemEvent::fillTimestampAndSamplesData (
            data,
            tester->getSourceNode(),
            streamId,
            currentSampleIndex,
            0,
            buffer.getNumSamples(),
            0);

        MidiBuffer eventBuffer;
        eventBuffer.addEvent (data, dataSize, 0);

        auto originalBuffer = buffer;
        audioProcessor->processBlock (buffer, eventBuffer);

        // Assert the buffer hasn't changed after process()
        ASSERT_EQ (buffer.getNumSamples(), originalBuffer.getNumSamples());
        ASSERT_EQ (buffer.getNumChannels(), originalBuffer.getNumChannels());

        for (int chidx = 0; chidx < buffer.getNumChannels(); chidx++)
        {
            for (int sampleIdx = 0; sampleIdx < buffer.getNumSamples(); ++sampleIdx)
                ASSERT_EQ (buffer.getSample (chidx, sampleIdx), originalBuffer.getSample (chidx, sampleIdx));
        }

        currentSampleIndex += buffer.getNumSamples();
    }

protected:
    NIDAQThread* nidaqThread;
    std::unique_ptr<DataThreadTester> tester;
    static constexpr NIDAQmxAPI::int32 numChannels = 16;
    static constexpr NIDAQmxAPI::int32 numSamplesPerChannel = 4;
    int64_t currentSampleIndex = 0;
    MockNIDAQmxAPI apiWrapper;
    
    int64 sampleNumbers[numChannels * numSamplesPerChannel];
    NIDAQmxAPI::float64 readArray[numChannels * numSamplesPerChannel];
};

/*
When connected to an NIDAQ, the NIDAQmx Plugin continuously reads from both analog and digital input channels. 
Reading from analog channels returns signed floating point values corresponding to the voltages recorded by the channel.
Reading from digital channels returns a single binary value per channel, indicating the state of the digital signal read. 
This test verifies that the NIDAQmx plugin can perform these read operations and copy these values to the output Audio Buffer.
*/
TEST_F (NIDAQIntegrationTests, DataIntegrity)
{
    tester->startAcquisition (false);

    int numSamples = 100;
    auto inputBuffer = createBuffer (1000.0, 20.0, 5, numSamples);
    writeBlock (inputBuffer);

    tester->stopAcquisition();
}

// Test connect() function
TEST_F (NIDAQUnitTests, ConnectSimulatedDevice)
{
    SetUp ("Simulated");

    nidaqmx->connect();

    // Verify expected properties of the simulated device
    EXPECT_EQ (device->isUSBDevice, false);
    EXPECT_EQ (device->sampleRateRange.min, 1000.0f);
    EXPECT_EQ (device->sampleRateRange.max, 30000.0f);
    EXPECT_EQ (device->productName, "No Device Detected");
}

TEST_F (NIDAQUnitTests, ConnectRealDevice)
{
    bool expectedUSBDevice = false;
    String expectedDeviceName = "Dev1";
    NIDAQmxAPI::int32 expectedCategory = 1;
    NIDAQmxAPI::uInt32 expectedDevProductNum = 12345;
    NIDAQmxAPI::uInt32 expectedDevSerialNum = 12345;
    NIDAQmxAPI::bool32 expectedSimAISamplingSupported = false;
    NIDAQmxAPI::float64 devAIMaxMultiChanRate = 20000.0f;
    NIDAQmxAPI::float64 expectedSampleRateRange[2] = { 1000.0f, devAIMaxMultiChanRate / DEFAULT_NUM_ANALOG_INPUTS };
    SettingsRange expectedVoltageRange (10.0f, 100.0f);
    NIDAQmxAPI::float64 expectedAIResolution = 16;
    NIDAQmxAPI::uInt32 expectedNumAIChannels = 1;
    NIDAQmxAPI::uInt32 expectedNumDIChannels = 1;

    ON_CALL (apiWrapper, getDevProductCategory (_, _))
        .WillByDefault ([&] (const char[], NIDAQmxAPI::int32* data)
                        {
                *data = expectedCategory;  // Example category
                return 0; });
    ON_CALL (apiWrapper, getDevProductNum (_, _))
        .WillByDefault ([&] (const char[], NIDAQmxAPI::uInt32* data)
                        {
                *data = expectedDevProductNum;  // Example product number
                return 0; });
    ON_CALL (apiWrapper, getDevSerialNum (_, _))
        .WillByDefault ([&] (const char[], NIDAQmxAPI::uInt32* data)
                        {
                *data = expectedDevSerialNum;  // Example serial number
                return 0; });
    ON_CALL (apiWrapper, getDevAISimultaneousSamplingSupported (_, _))
        .WillByDefault ([&] (const char[], NIDAQmxAPI::bool32* data)
                        {
                *data = expectedSimAISamplingSupported;  // Example simultaneous sampling supported
                return 0; });
    ON_CALL (apiWrapper, getDevAIMinRate (_, _))
        .WillByDefault ([&] (const char[], NIDAQmxAPI::float64* data)
                        {
                *data = expectedSampleRateRange[0];  // Example minimum rate
                return 0; });
    ON_CALL (apiWrapper, getDevAIMaxSingleChanRate (_, _))
        .WillByDefault ([] (const char[], NIDAQmxAPI::float64* data)
                        {
                *data = 30000.0f;  // Example maximum single channel rate
                return 0; });
    ON_CALL (apiWrapper, getDevAIMaxMultiChanRate (_, _))
        .WillByDefault ([&] (const char[], NIDAQmxAPI::float64* data)
                        {
                *data = devAIMaxMultiChanRate;  // Example maximum multi-channel rate
                return 0; });
    ON_CALL (apiWrapper, getDevAIVoltageRngs (_, _, _))
        .WillByDefault ([&] (const char[], NIDAQmxAPI::float64* data, NIDAQmxAPI::uInt32 arraySizeInElements)
                        {
                data[0] = expectedVoltageRange.min;
                data[1] = expectedVoltageRange.max;
                return 0; });
    ON_CALL (apiWrapper, getDevAIPhysicalChans (_, _, _))
        .WillByDefault ([] (const char[], char* data, NIDAQmxAPI::uInt32 bufferSize)
                        {
                strcpy (data, "Dev1/ai0");
                return 0; });
    ON_CALL (apiWrapper, getPhysicalChanAITermCfgs (_, _))
        .WillByDefault ([] (const char[], NIDAQmxAPI::int32* data)
                        {
                *data = DAQmx_Val_RSE;  // Example terminal configuration
                return 0; });
    ON_CALL (apiWrapper, getAIResolution (_, _, _))
        .WillByDefault ([&] (NIDAQmxAPI::TaskHandle taskHandle, const char channel[], NIDAQmxAPI::float64* data)
                        {
                *data = expectedAIResolution;  // Example resolution
                return 0; });
    ON_CALL (apiWrapper, getDevDILines (_, _, _))
        .WillByDefault ([] (const char device[], char* data, NIDAQmxAPI::uInt32 bufferSize)
                        {
                strcpy(data, "Dev1/port0/line0:7");  // Example digital lines
                return 0; });

    SetUp (expectedDeviceName);

    // Assert values set on the device
    EXPECT_EQ (device->isUSBDevice, expectedUSBDevice);
    EXPECT_EQ (device->getName(), expectedDeviceName);
    EXPECT_EQ (device->deviceCategory, expectedCategory);
    EXPECT_EQ (device->productNum, expectedDevProductNum);
    EXPECT_EQ (device->serialNum, expectedDevSerialNum);
    EXPECT_EQ (device->simAISamplingSupported, expectedSimAISamplingSupported);
    EXPECT_EQ (device->voltageRanges[0].min, expectedVoltageRange.min);
    EXPECT_EQ (device->voltageRanges[0].max, expectedVoltageRange.max);
    EXPECT_EQ (device->sampleRateRange.min, expectedSampleRateRange[0]);
    EXPECT_EQ (device->sampleRateRange.max, expectedSampleRateRange[1]);
    EXPECT_EQ (device->adcResolutions[0], expectedAIResolution);
    EXPECT_EQ (device->numAIChannels, expectedNumAIChannels);
    EXPECT_EQ (device->numDIChannels, expectedNumDIChannels);
}

// test channel configuration
TEST_F (NIDAQUnitTests, AnalogChannelConfiguration)
{
    SettingsRange expectedVoltageRange (10.0f, 100.0f);
    NIDAQmxAPI::float64 expectedAIResolution = 16;

    ON_CALL (apiWrapper, getDevAIVoltageRngs (_, _, _))
        .WillByDefault ([&] (const char[], NIDAQmxAPI::float64* data, NIDAQmxAPI::uInt32 arraySizeInElements)
                        {
            data[0] = expectedVoltageRange.min;
            data[1] = expectedVoltageRange.max;
            return 0; });
    ON_CALL (apiWrapper, getAIResolution (_, _, _))
        .WillByDefault ([&] (NIDAQmxAPI::TaskHandle taskHandle, const char channel[], NIDAQmxAPI::float64* data)
                        {
            *data = expectedAIResolution;  // Example resolution
            return 0; });

    SetUp ("Dev1");

    // Assert values set on the device
    EXPECT_EQ (device->voltageRanges[0].min, expectedVoltageRange.min);
    EXPECT_EQ (device->voltageRanges[0].max, expectedVoltageRange.max);
    EXPECT_EQ (device->adcResolutions[0], expectedAIResolution);
}

TEST_F (NIDAQUnitTests, DigitalChannelConfiguration)
{
    ON_CALL (apiWrapper, getDevDILines (_, _, _))
        .WillByDefault ([] (const char device[], char* data, NIDAQmxAPI::uInt32 bufferSize)
                        {
                strcpy(data, "Dev1/port0/line0:7");  // Example digital lines
                return 0; });

    SetUp ("Dev1");

    // Assert values set on the device
    EXPECT_EQ (device->digitalPortNames[0], "Dev1/port0");
    EXPECT_EQ (device->digitalPortStates[0], true);
    EXPECT_EQ (device->numDIChannels, 1);
}
