/*
------------------------------------------------------------------

This file is part of the Open Ephys GUI
Copyright (C) 2019 Allen Institute for Brain Science and Open Ephys

------------------------------------------------------------------

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef __NIDAQCOMPONENTS_H__
#define __NIDAQCOMPONENTS_H__

#include <DataThreadHeaders.h>
#include <stdio.h>
#include <string.h>
#include <optional>

#include "nidaq-api/NIDAQmx.h"

#define MAX_NUM_AI_CHANNELS 32
#define MAX_NUM_DI_CHANNELS 32

#define DEFAULT_NUM_ANALOG_INPUTS 8
#define DEFAULT_NUM_DIGITAL_INPUTS 8

#define NUM_SOURCE_TYPES 4
#define NUM_SAMPLE_RATES 17
#define CHANNEL_BUFFER_SIZE 500
#define ERR_BUFF_SIZE 2048

#define STR2CHR( jString ) ((jString).toUTF8())

class NIDAQmx;
class InputChannel;
class AnalogInput;
class DigitalInput;

enum SOURCE_TYPE {
	RSE = 0,
	NRSE,
	DIFF,
	PSEUDO_DIFF
};

struct SettingsRange {
	NIDAQ::float64 min, max;
	SettingsRange() : min(0), max(0) {}
	SettingsRange(NIDAQ::float64 min_, NIDAQ::float64 max_)
		: min(min_), max(max_) {}
};

class InputChannel
{
public:
	InputChannel(String name_) : name(name_) {};
	InputChannel() : name("") {};
	~InputChannel() {};

	String getName() { return name; }

	// Defines if the channel is available for use
	void setAvailable(bool available_) { available = available_; }
	bool isAvailable() { return available; }

	// Defines if the channel is enabled for use
	void setEnabled(bool enabled_) { enabled = enabled_; }
	bool isEnabled() { return enabled; }

private:

	String name;
	bool available = false;
	bool enabled = false;
};

class AnalogInput : public InputChannel
{

public:
	AnalogInput(String name, NIDAQ::int32 terminalConfig);
	AnalogInput() : InputChannel() {};
	~AnalogInput() {};

	SOURCE_TYPE getSourceType() { return sourceTypes[sourceTypeIndex]; }
	void setNextSourceType() { sourceTypeIndex = (sourceTypeIndex + 1) % sourceTypes.size(); }

private:
	int sourceTypeIndex = 0;
	Array<SOURCE_TYPE> sourceTypes;

};

class NIDAQDevice
{

public:

	NIDAQDevice(String name_) : name(name_) {};
	NIDAQDevice() {};
	~NIDAQDevice() {};

	String getName() { return name; }

	String productName;

	NIDAQ::int32 deviceCategory;
	NIDAQ::uInt32 productNum;
	NIDAQ::uInt32 serialNum;
	NIDAQ::uInt32 numAIChannels;
	NIDAQ::uInt32 numAOChannels;
	NIDAQ::uInt32 numDIChannels;
	NIDAQ::uInt32 numDOChannels;

	bool isUSBDevice;
	bool simAISamplingSupported;

	int digitalReadSize;

	SettingsRange sampleRateRange;

	Array<SettingsRange> voltageRanges;
	Array<NIDAQ::float64> adcResolutions;

private:

	String name;

};

class NIDAQmxDeviceManager
{
public:

	NIDAQmxDeviceManager() {};
	~NIDAQmxDeviceManager() {};

	void scanForDevices();

	int getNumAvailableDevices() { return devices.size(); }

	int getDeviceIndexFromName(String deviceName);
	NIDAQDevice* getDeviceAtIndex(int index) { return devices[index]; }

	NIDAQDevice* getDeviceFromName(String deviceName);

	friend class NIDAQThread;

private:

	OwnedArray<NIDAQDevice> devices;
	int activeDeviceIndex;
};

class NIDAQmx : public Thread
{
public:

	NIDAQmx(NIDAQDevice* device_);
	~NIDAQmx() {};

	/* Pointer to the active device */
	NIDAQDevice* device;

	/* Connects to the active device */
	void connect(); 

	String getProductName() { return device->productName; };
	String getSerialNumber() { return String(device->serialNum); };

	void setNumActiveAnalogInputs(int numActiveAnalogInputs_) { numActiveAnalogInputs = numActiveAnalogInputs_; };
	int getNumActiveAnalogInputs() { return numActiveAnalogInputs; };

	void setNumActiveDigitalInputs(int numActiveDigitalInputs_) { numActiveDigitalInputs = numActiveDigitalInputs_; };
	int getNumActiveDigitalInputs() { return numActiveDigitalInputs; };

	int getActiveDigitalLines();

	void setDigitalReadSize(int digitalReadSize_) { digitalReadSize = digitalReadSize_; };
	int getDigitalReadSize() { return digitalReadSize; };

	NIDAQ::float64 getSampleRate() { return sampleRates[sampleRateIndex]; };
	void setSampleRate(int index) { sampleRateIndex = index; };

	SettingsRange getVoltageRange() { return device->voltageRanges[voltageRangeIndex]; };
	void setVoltageRange(int index) { voltageRangeIndex = index; };

	SOURCE_TYPE getSourceTypeForInput(int analogIntputIndex) { return ai[analogIntputIndex]->getSourceType(); };
	void toggleSourceType(int analogInputIndex) { ai[analogInputIndex]->setNextSourceType(); }
	
	void setSyncStrategy(bool enableDigitalInSync) { digitalInSync = enableDigitalInSync; }

	void writeReferenceSampleToFile(int64 sampleIndex, double timestamp);

	void run();

	Array<NIDAQ::float64> sampleRates;

	OwnedArray<AnalogInput> 	ai;
	OwnedArray<InputChannel> 	di;

	friend class NIDAQThread;

private:
	/* Manages connected NIDAQ devices */
	ScopedPointer<NIDAQmxDeviceManager> dm;

	int deviceIndex = 0;
	int sampleRateIndex = 0;
	int voltageRangeIndex = 0;

	int digitalReadSize = 0;

	int numActiveAnalogInputs = DEFAULT_NUM_ANALOG_INPUTS; //8
	int numActiveDigitalInputs = DEFAULT_NUM_DIGITAL_INPUTS; //8

	NIDAQ::float64		ai_data[CHANNEL_BUFFER_SIZE * DEFAULT_NUM_ANALOG_INPUTS];

	NIDAQ::uInt8		di_data_8[CHANNEL_BUFFER_SIZE];  //PXI devices use 8-bit read
	NIDAQ::uInt16		di_data_16[CHANNEL_BUFFER_SIZE]; //some other devices may use 16-bit read? 
	NIDAQ::uInt32		di_data_32[CHANNEL_BUFFER_SIZE]; //USB devices use 32-bit read

	int64 ai_timestamp;
	uint64 eventCode;

	DataBuffer* aiBuffer;
    
    int64 referenceCount;
    int lastReferenceValue;

	bool digitalInSync;

	int digitalInSyncChannel;
	String referenceSampleFileSaveDirectory;
    
};

#endif  // __NIDAQCOMPONENTS_H__
