#include "NIDAQDataBuffer.h"

NIDAQDataBuffer::NIDAQDataBuffer(int chans, int size) : 
    DataBuffer(chans, size),
    abstractFifo(size),
    buffer(chans, size),
    numChans(chans)
{
    sampleNumberBuffer.malloc(size);
    timestampBuffer.malloc(size);
    eventCodeBuffer.malloc(size);
    timestampSampleBuffer.malloc(size);

    lastSampleNumber = 0;
    lastTimestamp = -1.0;
}

NIDAQDataBuffer::~NIDAQDataBuffer()
{

}

void NIDAQDataBuffer::clear()
{
    buffer.clear();
    abstractFifo.reset();

    lastSampleNumber = 0;
    lastTimestamp = -1.0;
}

int NIDAQDataBuffer::addToBuffer(
    float* data, 
    int64* sampleNumbers,
    double* timestamps,
    uint64* eventCodes,
    int numItems, 
    int chunkSize,
    std::optional<int64> timeStampSampleIndex)
{
    int startIndex1, blockSize1, startIndex2, blockSize2;

    abstractFifo.prepareToWrite(numItems, startIndex1, blockSize1, startIndex2, blockSize2);

    int bs[3] = { blockSize1, blockSize2, 0 };
    int si[2] = { startIndex1, startIndex2 };
    int cSize = 0;
    int idx = 0;
    int blkIdx;

    for (int i = 0; bs[i] != 0; ++i)
    {                                // for each of the dest blocks we can write to...
        blkIdx = 0;
        for (int j = 0; j < bs[i]; j += chunkSize)
        {                     // for each chunk...
            cSize = chunkSize <= bs[i] - j ? chunkSize : bs[i] - j;     // figure our how much you can write
            for (int chan = 0; chan < numChans; ++chan)         // write that much, per channel
            {
                buffer.copyFrom(chan,                           // (int destChannel)
                    si[i] + j,                      // (int destStartSample)
                    data + (idx * numChans) + chan, // (const float* source)
                    cSize);                         // (int num samples)
            }

            for (int k = 0; k < cSize; ++k)
            {
                sampleNumberBuffer[si[i] + blkIdx + k] = sampleNumbers[idx + k];
                timestampBuffer[si[i] + blkIdx + k] = timestamps[idx + k];
                eventCodeBuffer[si[i] + blkIdx + k] = eventCodes[idx + k];
                timestampSampleBuffer[si[i] + blkIdx + k] = timeStampSampleIndex;

            }
            idx += cSize;
            blkIdx += cSize;
        }
    }

    // finish write
    abstractFifo.finishedWrite(idx);

    return idx;
}

int NIDAQDataBuffer::getNumSamples() const
{
    return abstractFifo.getNumReady();
}

int NIDAQDataBuffer::readAllFromBuffer(
    AudioBuffer<float>& data,
    int64* blockSampleNumber,
    double* blockTimestamp,
    uint64* eventCodes,
    int maxSize,
    std::optional<int64>* timestampSampleIndex,
    int dstStartChannel,
    int numChannels)
{
    // check to see if the maximum size is smaller than the total number of available ints
    int numReady = abstractFifo.getNumReady();
    int numItems = (maxSize < numReady) ? maxSize : numReady;

    int startIndex1, blockSize1, startIndex2, blockSize2;
    abstractFifo.prepareToRead(numItems, startIndex1, blockSize1, startIndex2, blockSize2);

    int channelsToCopy = numChannels < 0 ? data.getNumChannels() : numChannels;

    if (blockSize1 > 0)
    {
        for (int chan = 0; chan < channelsToCopy; ++chan)
        {
            data.copyFrom(dstStartChannel + chan,            // destChan
                0,               // destStartSample
                buffer,          // source
                chan,            // sourceChannel
                startIndex1,     // sourceStartSample
                blockSize1);     // numSamples
        }

        memcpy(blockSampleNumber, sampleNumberBuffer + startIndex1, 8);
        memcpy(blockTimestamp, timestampBuffer + startIndex1, 8);
        memcpy(eventCodes, eventCodeBuffer + startIndex1, blockSize1 * 8);
        memcpy(timestampSampleIndex, timestampSampleBuffer + startIndex1, sizeof(std::optional<int64>));
    }
    else
    {
        // std::cout << "NO SAMPLES" << std::endl;
        memcpy(blockSampleNumber, &lastSampleNumber, 8);
        memcpy(blockTimestamp, &lastTimestamp, 8);
    }

    if (blockSize2 > 0)
    {
        for (int chan = 0; chan < channelsToCopy; ++chan)
        {
            data.copyFrom(dstStartChannel + chan,            // destChan
                blockSize1,      // destStartSample
                buffer,          // source
                chan,            // sourceChannel
                startIndex2,     // sourceStartSample
                blockSize2);     // numSamples
        }
        memcpy(eventCodes + blockSize1, eventCodeBuffer + startIndex2, blockSize2 * 8);
    }

    // std::cout << "START SAMPLE FOR READ: " << *blockSampleNumber << std::endl;

    if (numItems > 0)
    {

        lastSampleNumber = *blockSampleNumber;
        lastTimestamp = *blockTimestamp;

        // std::cout << "Updating last sample number: " << lastSampleNumber << std::endl;
    }

    abstractFifo.finishedRead(numItems);

    return numItems;
}

void NIDAQDataBuffer::resize(int chans, int size)
{
    buffer.setSize(chans, size);

    abstractFifo.setTotalSize(size);

    sampleNumberBuffer.malloc(size);
    timestampBuffer.malloc(size);
    eventCodeBuffer.malloc(size);
    timestampSampleBuffer.malloc(size);

    lastSampleNumber = 0;
    lastTimestamp = -1.0;

    numChans = chans;
}
