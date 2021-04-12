# Express_CX
Firmware for a high performance handset based on ExpressLRS

The idea is to try and provide the highest quality RC data to the flight controller, building on the low latency, low jitter capabilities of ExpressLRS. The primary target is quadcopters running BetaFlight - this is not intended to be a general purpose RC controller.

The gimbal data is filtered before being sampled at high (currently 128kHz per channel) frequency and then run through the 1AUD filter designed by Chris Thompson https://github.com/ctzsnooze/1AUD-filter. This allows customisable filter settngs to be applied for different use cases. Latency is reduced through increased packet rate, currently up to 1kHz.
