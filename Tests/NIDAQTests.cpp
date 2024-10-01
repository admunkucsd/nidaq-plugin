#include "../Source/NIDAQThread.h"
#include "gtest/gtest.h"

#include <TestFixtures.h>

class NIDAQTests : public testing::Test
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

/*
When connected to an NIDAQ, the NIDAQmx Plugin continuously reads from both analog and digital input channels. 
Reading from analog channels returns signed floating point values corresponding to the voltages recorded by the channel.
Reading from digital channels returns a single binary value per channel, indicating the state of the digital signal read. 
This test verifies that the NIDAQmx plugin can perform these read operations and copy these values to the output Audio Buffer.
*/
TEST_F (NIDAQTests, DataIntegrity)
{
    // Write the buffer to the NIDAQmx plugin
    tester->startAcquisition (false);

    //nidaqThread->

    // Read the buffer from the NIDAQmx plugin
    int numSamples = 100;
    //auto inputBuffer = tester->createBuffer (1000.0, 20.0, numChannels, numSamples);
    //tester->writeBlock (inputBuffer, currentSampleIndex);


    tester->stopAcquisition();
}