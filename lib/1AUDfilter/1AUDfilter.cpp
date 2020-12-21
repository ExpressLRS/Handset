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

// #include <Arduino.h>
#include "1AUDfilter.h"

// Some compilers mis-optimise the use of float constants and get better performance
// with an explicit declaration
static const float FASTZERO = 0.0f;
static const float FAST2PI = 6.28318531f;

DoublePT1filter::DoublePT1filter()
{
    current1 = FASTZERO;
    current2 = FASTZERO;
}

float DoublePT1filter::update(const float x)
{
    current1 = current1 + k * (x - current1);
    current2 = current2 + k * (current1 - current2);
    return current2;
}

float DoublePT1filter::getCurrent()
{
    return current2;
}

float DoublePT1filter::getK()
{
    return k;
}

void DoublePT1filter::setK(const float newK)
{
    k = newK;
}

/**
 *  Note that the cutoff frequencies must be less than sampleRate/(2Pi) in order for the filter
 * to remain stable
 */
OneAUDfilter::OneAUDfilter(const float _minFreq, const float _maxFreq, const float _beta, const float _sampleRate, 
                           const float _dCutoff, const float _maxSlew, const float initialValue)
{
    // save the params
    minFreq = _minFreq;
    maxFreq = _maxFreq;
    beta = _beta;
    sampleRate = _sampleRate;
    maxSlewPerSecond = _maxSlew;
    dCutoff = _dCutoff;

    // calculate derived values
    kScale = FAST2PI / _sampleRate;
    maxSlewPerSample = _maxSlew / _sampleRate;
    slewDisabled = (_maxSlew == FASTZERO);

    // other setup
    prevInput = initialValue;
    prevSmoothed = FASTZERO;

    // set k for the derivative
    const float k =  dCutoff * kScale;
    // Serial.printf("d k %f\n", k);
    dFilt.setK(k);

    // Set k for the output filter
    const float kOut = minFreq * kScale;
    // Serial.printf("kOut %f\n", kOut);
    oFilt.setK(kOut);
}

void OneAUDfilter::setSampleRate(const float newSampleRate)
{
    sampleRate = newSampleRate;
    maxSlewPerSample = maxSlewPerSecond / sampleRate;

    // precompute 2 * PI / sampleRate and save for future use
    kScale = FAST2PI / sampleRate;

    float kD = dCutoff * kScale;
    // make sure kD is reasonable
    if (kD >= 1.0f) {
        kD = 0.99f;
    }
    dFilt.setK(kD);

    // The output PT1s get a new K at each update, so no need to set here
}

float OneAUDfilter::slewLimit(const float x)
{
    // static float avSlew = 0;

    if (slewDisabled) {
        return x;
    }

    const float s = x - prevInput;

    // avSlew -= avSlew/10.0f;
    // if (s>0.0f) {
    //     avSlew += s/10.0f;
    // } else {
    //     avSlew += (-s)/10.0f;
    // }

    if (s > maxSlewPerSample) {
        // Serial.printf("%f > %f (av %f)\n", s, maxSlewPerSample, avSlew/10.0f);
        return prevInput + maxSlewPerSample;
    } else if ((-s) > maxSlewPerSample) {
        // Serial.printf("%f < -%f (av %f)\n", s, maxSlewPerSample, avSlew/10.0f);
        return prevInput - maxSlewPerSample;
    }

    return x;
}

// update the filter with a new value
float OneAUDfilter::update(const float newValue)
{
    // slew limit the input
    const float limitedNew = slewLimit(newValue);

    // apply the derivative filter to the input
    const float df = dFilt.update(limitedNew);

    // ## get differential of filtered input
    const float dx = (df - prevSmoothed) * sampleRate;
    const float absDx = dx >= FASTZERO ? dx : -dx;

    // update prevSmoothed
    prevSmoothed = df;

    // ## adjust cutoff upwards using dx and Beta
    float fMain = minFreq + (beta * absDx);
    if (fMain > maxFreq) {
        fMain = maxFreq;
    }

    // ## get the k value for the cutoff 
    // kCutoff = 2*PI*cutoff/sampleRate. 
    // 2*PI/sampleRate has been pre-computed and held in kScale
    const float kMain = fMain * kScale;
    oFilt.setK(kMain);

    // apply the main filter to the input
    const float mf = oFilt.update(limitedNew);

    // save the current value as an int so that we can use it from an ISR
    currentIntValue = mf;

    // update the previous value
    prevInput = newValue;

    return mf;
}

// get the current filtered value
float OneAUDfilter::getCurrent()
{
    return oFilt.getCurrent();
}

float OneAUDfilter::getCutoff()
{
    return oFilt.getK() / kScale;
}


/** Since we can't use floats in ISRs, this function
 * provides a way to get the current value as an int
 */
int32_t OneAUDfilter::getCurrentAsInt()
{
    return currentIntValue;
}
