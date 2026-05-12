SDMA driver for IMX6ULL

## (in)Frequently Asked Questions

### Why there are references to BP / DSP / StarCore in IMX6 Reference Manual?

It seems that SDMA documentation was copied from older devices using same IP core without proper care. Descriptions and
register were left with original terms that are not defined anywhere.

* BP - Baseband Processor/Platform (aka. StarCore/DSP)
* AP - Application Processor/Platform

There is limited amount of publicly available documentation but this [MXC300 fact-sheet] contains diagram that includes
SDMA with both application processor and DSP core. There are other diagrams here: [MOTOKRZR K3 – Theory of operation].

[MXC300 fact-sheet]: https://www.nxp.com/docs/en/fact-sheet/MXC300301FS.pdf
[MOTOKRZR K3 – Theory of operation]: https://firmware.center/firmware/Motorola/KRZR%20K3%20%28Sumba%29/Service%20Docs/MOTOKRZR_K3_Theory_of_Operation_v.1.0.pdf
