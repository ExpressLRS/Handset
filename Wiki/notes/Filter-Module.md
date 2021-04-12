![filter top view](https://github.com/JBKingdon/Express_CX/blob/master/images/pcbs/filter_top.jpg)

The filter module filters and buffers the signals from the gimbals before they are sampled by the ADC.

The project is on EasyEDA at https://easyeda.com/jbk1/expresscontroller

The circuit is from https://www.electronics-tutorials.ws/filter/second-order-filters.html

The cutoff frequency to use depends on both personal preference and how noisy the gimbals are. I'm currently using a cutoff of 339Hz which seems to work well, and plan to experiment with some higher frequency versions. The main purpose of having the filter on a module is to make it easy to test different cutoff frequencies.

339Hz can be achieved using 10k resistors and 47nF capacitors (or 15k with 33nF is similar at 322Hz). The footprints are for 0805 sized parts. For other cutoff frequencies there is a table of suggested values in the EasyEDA project description.

The pcb includes pads for changing Q using a resistor divider network in the feedback loop. I'm not currently using this, so the divider resistors on the back of the pcb can be left unpopulated. The ones on the front are still needed to complete the feedback circuit, but the value is not critical. I used 10k which would be a reasonable value if we wanted to modify Q later by adding the resistors on the back.

There are footprints for power supply decoupling capacitors on the back of the PCB, two 1210 and two 0805. The power rail is 5V so I'd recommend a 10V minimum rating on these capacitors. The larger ones can be anything in the 10 to 100 uF range, the smaller ones in the range 10 to 100 nF, ideally a mix of different values. The footprints don't have the polarity marked, so if using tantalum take extra care to get them the right way around! (but ceramics are cheaper and work fine here).

There is an assembly "cheat sheet" to help with where each component goes during assembly

![build cheat sheet](https://github.com/JBKingdon/Express_CX/blob/master/images/filter_cheat_sheet.png)

Rear view of the filter
![underside of filter](https://github.com/JBKingdon/Express_CX/blob/master/images/pcbs/filter_bottom.jpg)