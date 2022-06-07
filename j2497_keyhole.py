# MIT License
#
# Copyright (c) 2022 National Motor Freight Traffic Association Inc.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import binascii
import itertools
import numpy as np
from scipy.signal import chirp

from j2497_common import SYNC_BITS, get_payload_bits, get_payload_chirps

DEFAULT_ALLOWED_MESSAGES = [b'\x0a\x00', ]  # LAMP ON only by default

DEFAULT_SUPPLIER_PARAMETERS = [
    {  # WABCO 0a00 measured @ after crc-corrupted 16byte payload door signal
        'label': 'wabco tcs ii 2s1m basic msh 400 500 101 0',
        'expected_delays': [45.0, 41.7, ],
        'extra_stop_bits': [2, 2],  # tends to do 2 extra stop bits followed by 2 extra stop bits (but can vary)
        'expected_phases': [-1, 1],  # tends to use one phase over the other but just use equal probability
    },
    {  # Bendix TABS6 0a00 measured @ after crc-corrupt 16B payload door signal
        'label': 'bendix tabs6 5014016 ES1301 K003236',
        'expected_delays': [47.2, 41.7, 40.6, ],
        'extra_stop_bits': [1, 0],  # tends to do 1 extra stop bits followed by 0 extra stop bit (but can vary)
        'expected_phases': [-1, 1],
    },
    # {  # Haldex TABS 0a00 measured @ after crc-corrupt 16B payload door signal
    #    'label': 'haldex tabs H16 0676',
    #    'expected_delays': [46.1, ],
    #    'extra_stop_bits': [1, 0],  # tends to do 1 extra stop bits followed by 0 extra stop bits (doesn't vary)
    #    'expected_phases': [-1, 1],  # only one phase observed in testing
    # },
    # because Haldex TABS doesn't queue messages to send, picking any expected delay is fine and because both of the
    # other parameters match the Bendix unit, it is sufficient to omit these supplier parameters
]

# There is a minimum period for the keyhole signals which was discovered during testing. Bendix TABS6 transmitters
# verify their sends and will retry if their transmission is corrupted, which is great! Except that they also have a
# priority inversion bug so if they can't successfully transmit a lower priority message e.g. 89c20302b502 any
# higher-priority messages e.g. 0a00 (LAMP ON) will queue. If the door+keyhole signals are transmitted too rapidly
# then the TABS6 will trigger this priority inversion bug and there will be _no_ required LAMP messages. We also send
# all-jam periods sometimes to reduce the chance of forever-retries due to keyholes corrupting the signals too
MIN_PERIOD_US = 32000
DEFAULT_PERIOD_US = MIN_PERIOD_US


def generate(sample_rate, allowed_messages=None, keyhole_supplier_parameters=None, period_us=None,
             calibration_mode=False):
    """
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

    The interframe delays from most J2497 transmitters depend on the length of the allowed messages,
    the crc-corrupted state of the door signal and the period of the signals. Changes to any of these should be
    followed by re-calibrating the measured delays and updating the supplier parameters.

    :param sample_rate: sample rate of resulting signal, must be at least 800KHz, 1MHz is good
    :param allowed_messages: messages to allow via keyholes
    :param keyhole_supplier_parameters: supplier keyhole parameter list
    :param period_us: period of the door+keyhole signals generated
    :param calibration_mode: if true, generate modified waveforms used to calibrate the supplier parameters
    :return: an iterator of np array signals of float32 values in [-1.0, 1.0]
    """
    if sample_rate < 800E3:
        raise ValueError("sample rate must be >= 800 KHz")
    if keyhole_supplier_parameters is None:
        keyhole_supplier_parameters = DEFAULT_SUPPLIER_PARAMETERS
    if allowed_messages is None:
        allowed_messages = DEFAULT_ALLOWED_MESSAGES
    if period_us is None:
        period_us = DEFAULT_PERIOD_US
    jam_amplitude = 1
    if calibration_mode:
        # to calibrate supplier parameters the keyholes must be suppressed to measure expected delays
        # and jams must be suppressed to receive J1708 and hence measure expected delays
        jam_amplitude = 0

    assert period_us >= MIN_PERIOD_US
    # it is important for transmitters that don't queue LAMP ON (e.g. Haldex) that multiples of the period of the
    # door+keyhole do not align with the 0.5s period of the LAMP messages sent. We take anything within a sync
    # symbol width as 'alignment'.
    period_samples = int(period_us * sample_rate / US_PER_SEC)
    remainder = (0.5 * sample_rate) % period_samples
    alignment_limit = len(SYNC_BITS) * BODY_BIT_TIME_US * sample_rate / US_PER_SEC
    assert remainder > alignment_limit
    assert period_samples - remainder > alignment_limit

    doors = _door_signals(sample_rate)
    # combining doors and keyholes with `next(cycle(doors))` in the for loop below is fine if there are more keyholes
    # than there are doors. This is a generator, so confirm that after the loop -- see below
    doors_len = len(list(doors))
    doors = itertools.cycle(_door_signals(sample_rate))

    keyhole_count = 0
    for keyhole in _keyhole_signals(sample_rate, allowed_messages, keyhole_supplier_parameters, calibration_mode):
        keyhole_count = keyhole_count + 1
        door_n_keyhole = np.append(next(doors), keyhole)
        assert len(door_n_keyhole) < period_samples
        late_jam = jam_amplitude * _get_jam(sample_rate, period_samples - len(door_n_keyhole))
        door_n_keyhole = np.append(door_n_keyhole, late_jam)

        yield door_n_keyhole
    # confirm that there were, in fact, at least as many keyholes than doors
    assert keyhole_count >= doors_len

    # We need to send all-jam periods sometimes to reduce the chance of triggering a priority inversion bug. See the
    # MIN_PERIOD_US comments for more details.
    all_jam = next(doors)
    assert len(all_jam) < period_samples
    the_jam = jam_amplitude * _get_jam(sample_rate, period_samples - len(all_jam))
    all_jam = np.append(all_jam, the_jam)
    yield all_jam

    return


def _door_signals(sample_rate):
    """
    Generates 'door' signals whose purpose is to hold J2497 transmitters in wait, causing them to queue their
    messages to be sent and thus grooming the expected delays to better the chances of a keyhole aligning perfectly
    with an allowed message.

    All the values in DEFAULT_SUPPLIER_PARAMETERS are measured using the values below. Any changes to the
    payload or CRC necessitate re-calibrating DEFAULT_SUPPLIER_PARAMETERS.

    :param sample_rate:
    :return: a numpy float32 array of samples valued in [-1.0, 1.0]
    """
    # TODO: vary MID `89` through all possible trailer ABS MIDs [ 0x89, 0x8a, 0x8b, 0xf6, 0xf7 ] to _also_
    #   perform an address denial mitigation at the same time as the keyhole protection. Will need to both use correct
    #   CRC and also re-calibrate the values in DEFAULT_SUPPLIER_PARAMETERS
    mids = [b'\x89', ]
    for mid in mids:
        door_bits = get_payload_bits(mid + binascii.unhexlify('fe0757aaaaaaaaaaaaaaaaaaaaa71c'),
                                     checksum=binascii.unhexlify('cc'))  # Correct CRC is `b4` 〜(￣▽￣〜)
        yield get_payload_chirps(door_bits, sample_rate)


US_PER_SEC = 1e6
UART_BIT_TIME_US = 104.17  # i.e. 9600bps
BODY_BIT_TIME_US = 100  # J2497 body bit time
SYNC_SYMBOL_TIME_US = (5  # bits in start sync symbol
                       ) * BODY_BIT_TIME_US

# Intellon ssc p485 measured J2497 -> UART latency. Needed because measured/expected delays are UART delays
FROM_J2497_OVER_TO_UART_OVER_US = 48.3
# time duration for crc and the rest of a message after the payload
TIME_AFTER_PAYLOAD_US = (1  # start bit
                         + 8  # bits in crc byte
                         + 1  # stop bit
                         + 7  # bits in end sync symbol
                         ) * BODY_BIT_TIME_US


def _keyhole_signals(sample_rate, allowed_messages, keyhole_supplier_parameters, calibration_mode):
    """
    Generates keyhole signals which will permit only J2497 messages with matching payloads in the allowed messages list.
    There are multiple possible keyholes which are generated according to the combinations of the supplier parameters
    given in keyhole_supplier_parameters.

    To calibrate your own keyhole_supplier_parameters, set calibration_mode to true and measure some J2497 waveforms!

    :param sample_rate:
    :param allowed_messages: messages to permit by matching keyholes
    :param keyhole_supplier_parameters: delays, extra stop bits and phases to match for keyholes specific to devices
    :param calibration_mode: set to true to make keyholes that can be used to calibrate keyhole_supplier_parameters vals
    :return: a numpy float32 array of samples valued in [-1.0, 1.0]
    """
    keyhole_amplitude = 1
    jam_amplitude = 1
    if calibration_mode:
        # to calibrate supplier parameters the keyholes must be suppressed to measure expected delays
        keyhole_amplitude = 0
        # and jams must be suppressed to receive J1708 and hence measure expected delays
        jam_amplitude = 0
    blank_after_payload = np.zeros(int(TIME_AFTER_PAYLOAD_US * sample_rate / US_PER_SEC), np.float32)

    for allowed_message in allowed_messages:
        for params in keyhole_supplier_parameters:
            current_expected_delays_bit_times = params['expected_delays']
            current_extra_stop_bits = params['extra_stop_bits']
            current_expected_phases = params['expected_phases']

            keyhole_bits = get_payload_bits(allowed_message, checksum=None,
                                            extra_stop_bits=current_extra_stop_bits,
                                            truncate_at_checksum=True)

            for current_expected_delay_bit_time in current_expected_delays_bit_times:
                keyhole_signal_start_us = current_expected_delay_bit_time * UART_BIT_TIME_US \
                                          + FROM_J2497_OVER_TO_UART_OVER_US \
                                          - UART_BIT_TIME_US \
                                          - SYNC_SYMBOL_TIME_US
                keyhole_signal_start_samples = int(keyhole_signal_start_us * sample_rate / US_PER_SEC)

                # TODO: maybe overlap the jam and door signal a little bit (1/2 a body bit time probably).
                #   For now terminate the jam as soon as the door signal starts.
                early_jam = jam_amplitude * _get_jam(sample_rate, keyhole_signal_start_samples)

                for current_phase in current_expected_phases:
                    # TODO: prepare keyhole with an arbitrary mask. In testing so far all LAMP ON receivers reject 0a00
                    #   messages with an invalid CRC; therefore it is acceptable to blank the CRC and end symbol. If
                    #   receivers (ABS tractor controllers) are found that receive 0a00 messages with invalid CRC then
                    #   blanking a subset of bits of the 0a00(f6) message will be necessary. The following append of the
                    #   valid payload with silence for the CRC and end sync symbol will need to be replaced with a more
                    #   general substitution of silence for a 'mask' (a set of bits). For 0a00 the mask will need to be
                    #   of some of the logical '1' bits in the MID 0a and some in the payload as well since the silence
                    #   gaps are decoded as '1' or '0' unpredictably but usually in the same consecutively.
                    keyhole_signal = np.append(
                        keyhole_amplitude * current_phase * get_payload_chirps(keyhole_bits, sample_rate),
                        blank_after_payload
                    )

                    keyhole_signal = np.append(early_jam, keyhole_signal)
                    yield keyhole_signal


# Any constant carrier in the range 300E3-400E3 works; however, this frequency was optimized by testing for the best
# corrupting constant carrier at 3/4 power of the target signal.
DEFAULT_JAM_FREQ = 376.379E3


def _get_jam(sample_rate, duration_samples, freq=DEFAULT_JAM_FREQ):
    """
    this is a really dumb and degenerate use of a chirp function to make a single component sinusoid ＞﹏＜

    :param sample_rate:
    :param duration_samples: duration of the signal in samples
    :param freq: frequency of the constant carrier interference signal
    :return: a numpy array of samples valued in [-1.0, 1.0]
    """
    constant_carrier = chirp(
        np.linspace(0, duration_samples / sample_rate, duration_samples),
        f0=freq, f1=freq, t1=duration_samples / sample_rate, phi=-90, method='linear')
    return constant_carrier
