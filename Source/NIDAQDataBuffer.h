#pragma once

#include <DataThreadHeaders.h>

class NIDAQDataBuffer : public DataBuffer
{
public:
    /** Constructor */
    NIDAQDataBuffer(int chans, int size);

    /** Destructor */
    ~NIDAQDataBuffer();

    /** Clears the buffer.*/
    void clear();

    /** Add an array of floats to the buffer.

        @param data The data.
        @param sampleNumbers  Array of sample numbers (integers). Same length as numItems.
        @param timestamps  Array of timestamps (in seconds) (double). Same length as numItems.
        @param eventCodes Array of event codes. Same length as numItems.
        @param numItems Total number of samples per channel.
        @param chunkSize Number of consecutive samples per channel per chunk.
        1 by default. Typically 1 or numItems.

        @return The number of items actually written. May be less than numItems if
        the buffer doesn't have space.
    */
    int addToBuffer(float* data,
        int64* sampleNumbers,
        double* timestamps,
        uint64* eventCodes,
        int numItems,
        int chunkSize = 1,
        std::optional<int64> timeStampSampleIndex = std::nullopt);

    /** Returns the number of samples currently available in the buffer.*/
    int getNumSamples() const;

    /** Copies as many samples as possible from the DataBuffer to an AudioBuffer.*/
    int readAllFromBuffer(
        AudioBuffer<float>& data,
        int64* sampleNumbers,
        double* timestamps,
        uint64* eventCodes,
        int maxSize,
        std::optional<int64>* timestampSampleIndex,
        int dstStartChannel = 0,
        int numChannels = -1);

    /** Resizes the data buffer */
    void resize(int chans, int size);

private:
    AbstractFifo abstractFifo;
    AudioBuffer<float> buffer;

    HeapBlock<int64> sampleNumberBuffer;
    HeapBlock<double> timestampBuffer;
    HeapBlock<uint64> eventCodeBuffer;
    HeapBlock<std::optional<int64>> timestampSampleBuffer;

    int64 lastSampleNumber;
    double lastTimestamp;

    int numChans;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(NIDAQDataBuffer);
};