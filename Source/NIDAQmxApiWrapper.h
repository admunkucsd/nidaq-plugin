#pragma once

#include "nidaq-api/NIDAQmx.h"

#include <DataThreadHeaders.h>

class TESTABLE NIDAQmxApiWrapper
{
public:
    using int32 = NIDAQ::int32;
    using TaskHandle = NIDAQ::TaskHandle;
    using uInt32 = NIDAQ::uInt32;
    using float64 = NIDAQ::float64;
    using bool32 = NIDAQ::bool32;
    using uInt64 = NIDAQ::uInt64;
    using uInt16 = NIDAQ::uInt16;
    using uInt8 = NIDAQ::uInt8;

public:
    virtual ~NIDAQmxApiWrapper();

    virtual int32 getDevAIPhysicalChans (const char* device, char* data, uInt32 bufferSize)
    {
        return NIDAQ::DAQmxGetDevAIPhysicalChans (device, data, bufferSize);
    }

    virtual int32 getPhysicalChanAITermCfgs (const char physicalChannel[], int32* data)
    {
        return NIDAQ::DAQmxGetPhysicalChanAITermCfgs (physicalChannel, data);
    }

    virtual int32 createTask (const char taskName[], TaskHandle* taskHandle)
    {
        return NIDAQ::DAQmxCreateTask (taskName, taskHandle);
    }

    virtual int32 createAIVoltageChan (TaskHandle taskHandle,
                                       const char physicalChannel[],
                                       const char nameToAssignToChannel[],
                                       int32 terminalConfig,
                                       float64 minVal,
                                       float64 maxVal,
                                       int32 units,
                                       const char customScaleName[])
    {
        return NIDAQ::DAQmxCreateAIVoltageChan (taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, customScaleName);
    }

    virtual int32 getTaskNumDevices (TaskHandle taskHandle, uInt32* data)
    {
        return NIDAQ::DAQmxGetTaskNumDevices (taskHandle, data);
    }

    virtual int32 getNthTaskDevice (TaskHandle taskHandle, uInt32 index, char buffer[], int32 bufferSize)
    {
        return NIDAQ::DAQmxGetNthTaskDevice (taskHandle, index, buffer, bufferSize);
    }

    virtual int32 getDevProductCategory (const char device[], int32* data)
    {
        return NIDAQ::DAQmxGetDevProductCategory (device, data);
    }

    virtual int32 getDevProductNum (const char device[], uInt32* data)
    {
        return NIDAQ::DAQmxGetDevProductNum (device, data);
    }

    virtual int32 getDevSerialNum (const char device[], uInt32* data)
    {
        return NIDAQ::DAQmxGetDevSerialNum (device, data);
    }

    virtual int32 getDevAISimultaneousSamplingSupported (const char device[], bool32* data)
    {
        return NIDAQ::DAQmxGetDevAISimultaneousSamplingSupported (device, data);
    }

    virtual int32 getDevAIMinRate (const char device[], float64* data)
    {
        return NIDAQ::DAQmxGetDevAIMinRate (device, data);
    }

    virtual int32 getDevAIMaxSingleChanRate (const char device[], float64* data)
    {
        return NIDAQ::DAQmxGetDevAIMaxSingleChanRate (device, data);
    }

    virtual int32 getDevAIMaxMultiChanRate (const char device[], float64* data)
    {
        return NIDAQ::DAQmxGetDevAIMaxMultiChanRate (device, data);
    }

    virtual int32 getDevAIVoltageRngs (const char device[], float64* data, uInt32 arraySizeInElements)
    {
        return NIDAQ::DAQmxGetDevAIVoltageRngs (device, data, arraySizeInElements);
    }

    virtual int32 getAIResolution (TaskHandle taskHandle, const char channel[], float64* data)
    {
        return NIDAQ::DAQmxGetAIResolution (taskHandle, channel, data);
    }

    virtual int32 stopTask (TaskHandle taskHandle)
    {
        return NIDAQ::DAQmxStopTask (taskHandle);
    }

    virtual int32 clearTask (TaskHandle taskHandle)
    {
        return NIDAQ::DAQmxClearTask (taskHandle);
    }

    virtual int32 startTask (TaskHandle taskHandle)
    {
        return NIDAQ::DAQmxStartTask (taskHandle);
    }

    virtual int32 getDevDILines (const char device[], char* data, uInt32 bufferSize)
    {
        return NIDAQ::DAQmxGetDevDILines (device, data, bufferSize);
    }

    virtual int32 getExtendedErrorInfo (char* errorString, uInt32 bufferSize)
    {
        return NIDAQ::DAQmxGetExtendedErrorInfo (errorString, bufferSize);
    }

    virtual int32 getSysDevNames (char* data, uInt32 bufferSize)
    {
        return NIDAQ::DAQmxGetSysDevNames (data, bufferSize);
    }

    virtual int32 getDevProductType (const char device[], char* data, uInt32 bufferSize)
    {
        return NIDAQ::DAQmxGetDevProductType (device, data, bufferSize);
    }

    virtual int32 cfgSampClkTiming (TaskHandle taskHandle, const char source[], float64 rate, int32 activeEdge, int32 sampleMode, uInt64 sampsPerChan)
    {
        return NIDAQ::DAQmxCfgSampClkTiming (taskHandle, source, rate, activeEdge, sampleMode, sampsPerChan);
    }

    virtual int32 getDevDIPorts (const char device[], char* data, uInt32 bufferSize)
    {
        return NIDAQ::DAQmxGetDevDIPorts (device, data, bufferSize);
    }

    virtual int32 createDIChan (TaskHandle taskHandle, const char lines[], const char nameToAssignToLines[], int32 lineGrouping)
    {
        return NIDAQ::DAQmxCreateDIChan (taskHandle, lines, nameToAssignToLines, lineGrouping);
    }

    virtual int32 readDigitalU32 (TaskHandle taskHandle, int32 numSampsPerChan, float64 timeout, bool32 fillMode, uInt32 readArray[], uInt32 arraySizeInSamps, int32* sampsPerChanRead, bool32* reserved)
    {
        return NIDAQ::DAQmxReadDigitalU32 (taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved);
    }

    virtual int32 readDigitalU16 (TaskHandle taskHandle, int32 numSampsPerChan, float64 timeout, bool32 fillMode, uInt16 readArray[], uInt32 arraySizeInSamps, int32* sampsPerChanRead, bool32* reserved)
    {
        return NIDAQ::DAQmxReadDigitalU16 (taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved);
    }

    virtual int32 readDigitalU8 (TaskHandle taskHandle, int32 numSampsPerChan, float64 timeout, bool32 fillMode, uInt8 readArray[], uInt32 arraySizeInSamps, int32* sampsPerChanRead, bool32* reserved)
    {
        return NIDAQ::DAQmxReadDigitalU8 (taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved);
    }

    virtual int32 readAnalogF64 (TaskHandle taskHandle, int32 numSampsPerChan, float64 timeout, int32 fillMode, float64 readArray[], uInt32 arraySizeInSamps, int32* sampsPerChanRead, bool32* reserved)
    {
        return NIDAQ::DAQmxReadAnalogF64 (taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved);
    }

    virtual int32 taskControl (TaskHandle taskHandle, int32 action)
    {
        return NIDAQ::DAQmxTaskControl (taskHandle, action);
    }
};