**j2497-keyhole**: A mitigation against injection of J2497 commands while permitting regulation-required LAMP messages.

The code here accompanies mitigations designed and developed by the National Motor Freight Traffic Association Inc. (NMFTA) to mitigate the issues disclosed in CISA ICS Advisory [ICSA-22-063-01 Trailer Power Line Communications 
(PLC) J2497](https://www.cisa.gov/uscert/ics/advisories/icsa-22-063-01). The many other mitigations are published into the public domain and [available in a paper hosted by the NMFTA](http://www.nmfta.org/documents/ctsrp/Actionable_Mitigations_Options_v9_DIST.pdf?v=1).

The mitigation here takes the form of a signal which can be transmitted on a J2497 segment for protection against injection of J2497 commands while permitting regulation-required LAMP messages.. The transmit can be by a DAC or also by bit-banging. From the [`j2497_keyhole.generate`](j2497_keyhole.py#L64) docstring:

    Use this function to get a complete set of keyhole mitigation signals. Play them on a loop to prevent all but the
    allowed messages from being received by any J2497 receiver on the powerline segment.

    The signals will probably need to be prepared for playback on your DAC. They will work even when played on a
    bit-banged DAC (1-bit / PWM etc.). Here's an example of preparing the signals for playback @ 1Msps on a signed
    8bit output:

    dac_ready = [(x * 127).astype('int8').tobytes() for x in j2497_keyhole.generate(1E6)]

    And these can be played back-to-back on a loop e.g.

    while True:
        for s in dac_ready:
            dac_device_driver_api.write(s)

    If you want to bit-bang the output, we have found that even the simplest PWM rule will work:

    bangs = [(x >= 0) for x in j2497_keyhole.generate(10E6)]

    Note that there is no dynamic generation required. The signals can be pre-computed and played back from non-volatile
    storage as well.
    [...]

The `main.py` file contains an example of transmitting the keyhole mitigation signal using cheap FL2K VGA adapters as transmitters (SDRs) by running and writing-to the `fl2k_file` executable in `PATH` from the [osmo-fl2k project](https://gitea.osmocom.org/sdr/osmo-fl2k/). This will work both on Linux and Windows.

## Installation

1. compile and install [osmo-fl2k](https://gitea.osmocom.org/sdr/osmo-fl2k/) following the project instructions , or install from pre-built binaries
2. install this project's python dependencies with `pip3 install -r requirements.txt`
3. run `main.py` to transmit the keyhole mitigation on a connected FL2K adapter

---

THE INFORMATION CONTAINED HEREIN IS 
PROVIDED “AS IS” WITHOUT WARRANTY OF ANY 
KIND, WHETHER WRITTEN OR ORAL, EITHER 
EXPRESSED OR IMPLIED, STATUTORY OR 
OTHERWISE, INCLUDING, BUT NOT LIMITED TO, THE 
IMPLIED WARRANTIES OF MERCHANTABILITY AND 
FITNESS FOR A PARTICULAR PURPOSE. THE ENTIRE 
RISK AS TO THE QUALITY AND PERFORMANCE OF 
THE INFORMATION IS WITH THE USER.