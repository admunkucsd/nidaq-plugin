#pragma once

#include <NIDAQmx.h>

class NIDAQmxApiWrapper
{
public:
    virtual NIDAQ::int32 getDevAIPhysicalChans (const char* device, char* data, NIDAQ::uInt32 bufferSize) = 0;
    virtual NIDAQ::int32 getPhysicalChanAITermCfgs (const char physicalChannel[], NIDAQ::int32* data) = 0;
};

class NIDAQmxApiWrapperImpl : public NIDAQmxApiWrapper
{
public:
    NIDAQ::int32 getDevAIPhysicalChans (const char* device, char* data, NIDAQ::uInt32 bufferSize) override
    {
        return NIDAQ::DAQmxGetDevAIPhysicalChans (device, data, bufferSize);
    }

    NIDAQ::int32 getPhysicalChanAITermCfgs (const char physicalChannel[], NIDAQ::int32* data) override
    {
        return NIDAQ::DAQmxGetPhysicalChanAITermCfgs (physicalChannel, data);
    }
};