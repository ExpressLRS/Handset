
The Express LRS Handset project is first and foremost an **experimental test bed** for pushing the envelope with what can be done with the ELRS radio link.  This handset will not replace your "daily driver", nor will it ever support radio protocols other than ELRS.  The Micro Controller Unit (MCU) does not run OpenTX.  The user interface is very rudimentary - simple text menus that are selected with a rotary encoder.  The firmware for this handset will never achieve "Release Candidate", let alone "Stable" status or be assigned a version number.  The software will be a continuous work in progress.

You will have to order and print parts, assemble, and solder this handset by hand as well as know how to build the firmware from source code and upload to the MCU.  

There will be pain.  There will be gnashing of teeth.  At this point is where we give you your first and final warning as passed down by ancient sailors:

***********************************************************
**Beyond this line there be Dragons . . . .**
***********************************************************

OK, now that we've set some expectations for this project we'll list some of the cool features of this handset and explain why you would want to go thru all of the effort to build instead of just purchasing a commercial handset.

1. Fly at the fastest and most consistent packet rate available.  Currently this is 500Hz, but this handset is capable of much faster.  Since the MCU doesn't have to deal with OpenTX, all of its CPU cycles are used to provide the most consistent ELRS signal to the receiver and thus your flight controller.  This is the main goal of this project, to increase the performance and stability of the ELRS radio link.  With this handset, you'll be right on the bleeding edge with the devs.
2. Uses FrSky M10 Hall Effect gimbals.  If you want the best radio signal, you start with the best data.
3. Hardware Filtering of gimbal data.  Select a level of hardware filtering by swapping out custom daughterboards to fine tune your feel.
4. Swappable RF deck.  RF deck is on a replaceable daughterboard so you can swap from 2.4GHz to 900MHz bands. (900 MHz modules currently not developed)  Can also swap to a lower powered RF module to save battery life if you don't need the higher power settings.
5. 1S2P 18650 LiPo battery.  Entire handset sips power from 2 LiPos in parallel.  Fly an entire weekend without charging.
6. Wireless Qi charging.  Build a charging cradle that will wirelessly charge your handset when you're not flying.
7. Internal Moxon antenna.  Transmit antenna is mounted internally for aesthetics, but can be mounted externally simply by drilling a hole.
8. Simplified operation.  Quad fliers rarely use 99% of the mixing functions of OpenTX since all of that is done in the Flight Controller.
9. Parts should be less than $100 U.S.
10. Did I mention wireless Qi charging?

At this time the best source of information on this project is our OneDrive folder:
https://1drv.ms/u/s!Ap0n6SxbLP9mspxu-dxmtthcFGTxvA?e=23NiK6

