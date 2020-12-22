# Express_CX
Firmware for an experimental handset based on ExpressLRS

This project builds on the awesome ExpressLRS high performance RC link https://github.com/AlessandroAU/ExpressLRS

The aim of the project is to explore the possibilities for providing the highest quality RC data, building on the low latency, low jitter capabilities of ExpressLRS. The primary target is quadcopters running BetaFlight - this is not intended to be a general purpose RC controller.

The aspects of quality being focussed on are fidelity of stick movement and latency. The raw gimbal data is filtered before being sampled at high (currently 128kHz per channel) frequency and then run through the 1AUD filter designed by Chris Thompson https://github.com/ctzsnooze/1AUD-filter. This allows customisable filter settngs to be applied for different use cases. Latency is reduced through increased packet rate, currently up to 1kHz.

In future I hope to use this platform to experiment with alternative encodings for the RC data, calculating the first and second order derivatives for each channel and sending those over the link along with the absolute stick positions.
