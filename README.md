# Express_CX
Firmware for an experimental handset based on ExpressLRS

You're welcome to look around, but the project is still very much a work in progress and isn't particularly consumable yet. It's purpose is to test the limits of current RC technology and is likely only of interest to a very small subset of pilots.

This project builds on the awesome ExpressLRS high performance RC link https://github.com/AlessandroAU/ExpressLRS

The idea is to try and provide the highest quality RC data to the flight controller, building on the low latency, low jitter capabilities of ExpressLRS. The primary target is quadcopters running BetaFlight - this is not intended to be a general purpose RC controller.

The gimbal data is filtered before being sampled at high (currently 128kHz per channel) frequency and then run through the 1AUD filter designed by Chris Thompson https://github.com/ctzsnooze/1AUD-filter. This allows customisable filter settngs to be applied for different use cases. Latency is reduced through increased packet rate, currently up to 1kHz.

In future I hope to experiment with calculating the first and second order derivatives for each channel and sending those over the link along with the absolute stick positions.
