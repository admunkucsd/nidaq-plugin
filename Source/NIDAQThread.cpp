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

#include "NIDAQThread.h"
#include "NIDAQEditor.h"

DataThread* NIDAQThread::createDataThread(SourceNode *sn)
{
	return new NIDAQThread(sn);
}

GenericEditor* NIDAQThread::createEditor(SourceNode* sn)
{
    return new NIDAQEditor(sn, this, true);
}

NIDAQThread::NIDAQThread(SourceNode* sn) : DataThread(sn), recordingTimer(this)
{
	progressBar = new ProgressBar(initializationProgress);

	openConnection();

	int totalChans = getNumAnalogInputs();
	sourceBuffers.add(new DataBuffer(totalChans, 10000));

	mNIDAQ->aiBuffer = sourceBuffers.getLast();

	//Default to highest sample rate, largest voltage range
	sampleRateIndex = mNIDAQ->sampleRates.size() - 1;
	voltageRangeIndex = mNIDAQ->aiVRanges.size() - 1;
	
}

NIDAQThread::~NIDAQThread()
{
}

void NIDAQThread::openConnection()
{

	dm = new NIDAQmxDeviceManager();

	dm->scanForDevices();

	/* TODO: Prompt user which device to select: */
	/* For now we default to first detected device */
	mNIDAQ = new NIDAQmx(dm->devices[0].toUTF8());

	setSampleRate(mNIDAQ->sampleRates.size() - 1);

	inputAvailable = true;
	
	printf("Created new device: %s", mNIDAQ->deviceName);
}

void NIDAQThread::closeConnection()
{
}

int NIDAQThread::getNumAnalogInputs() const
{
	return mNIDAQ->ai.size();
}

int NIDAQThread::getNumDigitalInputs() const
{
	return mNIDAQ->di.size();
}

void NIDAQThread::toggleAIChannel(int index)
{
	mNIDAQ->aiChannelEnabled.set(index, !mNIDAQ->aiChannelEnabled[index]);
}

void NIDAQThread::setVoltageRange(int rangeIndex)
{
	/* Doesnt work like I think it does...why??? */
	voltageRangeIndex = rangeIndex;
	for (auto input : mNIDAQ->ai)
	{
		input.setVoltageRange(mNIDAQ->aiVRanges[rangeIndex]);
	}
	mNIDAQ->voltageRange = mNIDAQ->aiVRanges[rangeIndex];
}

void NIDAQThread::setSampleRate(int rateIndex)
{
	sampleRateIndex = rateIndex;
	mNIDAQ->samplerate = mNIDAQ->sampleRates[rateIndex];
}

int NIDAQThread::getVoltageRangeIndex()
{
	return voltageRangeIndex;
}

int NIDAQThread::getSampleRateIndex()
{
	return sampleRateIndex;
}

Array<String> NIDAQThread::getVoltageRanges()
{
	Array<String> voltageRanges;
	for (VRange range : mNIDAQ->aiVRanges)
	{
		voltageRanges.add(String(range.vmin) + "-" + String(range.vmax) + " V");
	}
	return voltageRanges;
}

Array<String> NIDAQThread::getSampleRates()
{
	Array<String> sampleRates;
	for (auto rate : mNIDAQ->sampleRates)
	{
		sampleRates.add(String(rate) + " kS/S");
	}
	return sampleRates;
}

/** Returns true if the data source is connected, false otherwise.*/
bool NIDAQThread::foundInputSource()
{
    return inputAvailable;
}

XmlElement NIDAQThread::getInfoXml()
{

	//TODO: 
	XmlElement nidaq_info("NI-DAQmx");
	XmlElement* api_info = new XmlElement("API");
	//api_info->setAttribute("version", api.version);
	nidaq_info.addChildElement(api_info);

	return nidaq_info;

}

String NIDAQThread::getInfoString()
{

	String infoString;

	infoString += "TEST";

	return infoString;

}

/** Initializes data transfer.*/
bool NIDAQThread::startAcquisition()
{
	//TODO:
	mNIDAQ->startThread();
	startThread();
    return true;
}

void NIDAQThread::timerCallback()
{
	//TODO:
}

void NIDAQThread::startRecording()
{

}

void NIDAQThread::stopRecording()
{
	//TODO:
}

/** Stops data transfer.*/
bool NIDAQThread::stopAcquisition()
{
	//TODO:
	if (isThreadRunning())
	{
		signalThreadShouldExit();
	}
	if (mNIDAQ->isThreadRunning())
	{
		mNIDAQ->signalThreadShouldExit();
	}
    return true;
}

void NIDAQThread::setSelectedInput()
{

}

void NIDAQThread::setDefaultChannelNames()
{
	//TODO:
}

bool NIDAQThread::usesCustomNames() const
{
	return false;
}

/** Returns the number of virtual subprocessors this source can generate */
unsigned int NIDAQThread::getNumSubProcessors() const
{
	//TODO?
	return 1;
}

/** Returns the number of continuous headstage channels the data source can provide.*/
int NIDAQThread::getNumDataOutputs(DataChannel::DataChannelTypes type, int subProcessorIdx) const
{
	if (subProcessorIdx > 0) return 0;

	int numChans = 0;

	if (type == DataChannel::ADC_CHANNEL)
	{
		numChans = getNumAnalogInputs();
	}
	/*
	if (type == DataChannel::EVENT_CHANNEL)
	{
		numChans = getNumDigitalInputs();
	}
	*/

	return numChans;
}

/** Returns the number of TTL channels that each subprocessor generates*/
int NIDAQThread::getNumTTLOutputs(int subProcessorIdx) const
{
	//TODO
	return 0;
}

/** Returns the sample rate of the data source.*/
float NIDAQThread::getSampleRate(int subProcessorIdx) const
{
	return mNIDAQ->samplerate;
}

/** Returns the volts per bit of the data source.*/
float NIDAQThread::getBitVolts(const DataChannel* chan) const
{
	//TODO
	return mNIDAQ->bitVolts;
}

void NIDAQThread::setTriggerMode(bool trigger)
{
    //TODO
}

void NIDAQThread::setRecordMode(bool record)
{
    //TODO
}

void NIDAQThread::setAutoRestart(bool restart)
{
	//TODO
}

void NIDAQThread::setDirectoryForInput(int id, File directory)
{
	//TODO
}

File NIDAQThread::getDirectoryForInput(int id)
{
	//TODO
	return File::getCurrentWorkingDirectory();
}

float NIDAQThread::getFillPercentage(int id)
{
	//TODO
	return 0.0f;
}

bool NIDAQThread::updateBuffer()
{
	return true;
}

RecordingTimer::RecordingTimer(NIDAQThread* t_)
{
	thread = t_;
}

void RecordingTimer::timerCallback()
{
	thread->startRecording();
	stopTimer();
}
