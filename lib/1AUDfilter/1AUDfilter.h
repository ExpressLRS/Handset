/** ----------------------------------------------------------------------------------------
 *  1AUD Filter
 * 
 *  Designed by Chris Thompson
 * 
 *  Implementation by James Kingdon
 * 
 *  The 1AUD filter is a dynamic filter that builds on the 1 Euro filter.
 *  Compared to 1E the 1AUD uses stacked PT1 filters as input to the derivative, enabling
 *  the use of higher derivative filter cut-off frequencies (and therefore faster tracking 
 *  of D), and stacked PT1 filters for the main filter, providing stronger (2nd order) 
 *  filtering of the input signal. Slew limiting of the input is also implemented.
 * 
 *  This software is released under the MIT license:
 * 
 *  Copyright 2020 C Thompson, J Kingdon
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy of this 
 *  software and associated documentation files (the "Software"), to deal in the Software 
 *  without restriction, including without limitation the rights to use, copy, modify, merge, 
 *  publish, distribute, sublicense, and/or sell copies of the Software, and to permit 
 *  persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all copies or 
 *  substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 *  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 *  PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 *  FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 *  ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 *  THE SOFTWARE.
 * ----------------------------------------------------------------------------------------
*/

#pragma once

#include <stdint.h>

#define ONEAUD_DEFAULT_DCUTOFF (50.0f)

/** Two PT1 filters with a common k
 *  Updates are processed by both PT1 sequentially, providing 2nd order filtering at the output
 */
class DoublePT1filter
{
private:
    float current1, current2, k;

public:
    DoublePT1filter();
    float update(const float x);
    float getK();
    void setK(const float newK);
    float getCurrent();
};

class OneAUDfilter
{
private:

    float prevInput, prevSmoothed;
    float minFreq, maxFreq;
    float beta;
    float sampleRate;
    float kScale;   // 2 * PI / sampleRate
    float dCutoff;
    float maxSlewPerSecond, maxSlewPerSample;
    bool slewDisabled;

    int32_t currentIntValue;

    // The filters for the derivative and output
    DoublePT1filter dFilt, oFilt;

    /** Limit the maximum per step delta in the input
     * This helps to remove the impact of measurement spikes in the data that
     * exceed what is physically possible in the domain space. The limit should be
     * higher than the noise threshold.
     */
    float slewLimit(const float x);

public:
    /** Constructor
     * @param minFreq       cutoff frequency when the input is unchanging
     * @param maxFreq       cutoff frequency when the input is rapidly changing
     * @param beta          the rate at which the cutoff frequency is raised when the input is changing
     * @param sampleRate    the frequency (Hz) at which the input data was sampled
     * @param dCutoffFreq   the cutoff frequency for the derivative filter. Optional, default ONEAUD_DEFAULT_DCUTOFF Hz
     * @param maxSlew       the maximum expected rate of change in steps per second. 0 disables slew limiting. Optional, default 0
     * @param initialValue  a hint for the expected value to allow quicker initial settling at startup. Optional, default 0
     */
    OneAUDfilter(const float minFreq, const float maxFreq, const float beta, const float sampleRate, 
                 const float dCutoffFreq = ONEAUD_DEFAULT_DCUTOFF,
                 const float maxSlew = 0.0f, const float initialValue = 0.0f);

    // change the sample rate that the filter is running at
    void setSampleRate(const float newSampleRate);

    // update the filter with a new value
    float update(const float newValue);

    // get the current filtered value
    float getCurrent();
    
    // get the current value of the dynamic cutoff frequency
    float getCutoff();

    // get the current filtered value as an int32_t
    int32_t getCurrentAsInt();

};
