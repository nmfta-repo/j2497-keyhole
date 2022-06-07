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

import numpy as np
from scipy.signal import chirp
from builtins import bytes, chr
import bitstring

INITIAL_PREAMBLE_BITS = bitstring.ConstBitArray(bin='00')
SYNC_BITS = bitstring.ConstBitArray(bin='11111')
START_BITS = bitstring.ConstBitArray(bin='0')
STOP_BITS = bitstring.ConstBitArray(bin='1')
ENDSYNC_BITS = bitstring.ConstBitArray(bin='1111111')


def get_preamble_bits(preamble_mid: bytes):
    mid_bits = bitstring.BitArray(bytes=preamble_mid)
    mid_bits.reverse()
    mid_bits.prepend(INITIAL_PREAMBLE_BITS)
    mid_bits.prepend(START_BITS)
    mid_bits.append(STOP_BITS)
    return mid_bits


# TODO: there are definitely more efficient ways to do the J1708 checksum
def get_checksum_bits(payload):
    msg = str(bitstring.ConstBitArray(bytes=payload).bin)
    checksum = 0
    for n in range(0, len(msg), 8):
        checksum = checksum + int(msg[n:n + 8], 2)

    # Two's Complement (10)
    binint = int("{0:b}".format(checksum))  # Convert to binary (1010)
    flipped = ~binint  # Flip the bits (-1011)
    flipped += 1  # Add one_bits (two's complement method) (-1010)
    intflipped = int(str(flipped), 2)  # Back to int (-10)
    intflipped = ((intflipped + (1 << 8)) % (1 << 8))  # Over to binary (246) <-- .uint
    intflipped = '{0:08b}'.format(intflipped)  # Format to one_bits byte (11110110) <-- same as -10.bin

    checksum_bits = bitstring.BitArray(bin=intflipped)
    return checksum_bits


def get_payload_bits(payload, checksum=None, extra_stop_bits=None, truncate_at_checksum=False):
    if extra_stop_bits is None:
        extra_stop_bits = [0, ]
    payload_bits = bitstring.BitArray()

    payload_bits.append(SYNC_BITS)
    char_count = 0
    for b_int in bytes(payload):
        b_bytes = bytes([b_int])
        b_bits = bitstring.BitArray(bytes=b_bytes)
        b_bits.reverse()

        payload_bits.append(START_BITS)  # start bit
        payload_bits.append(b_bits)  # bit-reversed byte
        payload_bits.append(STOP_BITS)  # stop bit
        extra_stop_bit = extra_stop_bits[-1] if char_count > len(extra_stop_bits) else extra_stop_bits[char_count]
        for i in range(extra_stop_bit):
            payload_bits.append(STOP_BITS)  # stop bit

    if truncate_at_checksum:
        return payload_bits

    if checksum is None:
        checksum_bits = get_checksum_bits(payload)
    else:
        checksum_bits = bitstring.BitArray(bytes=checksum)
    checksum_bits.reverse()

    payload_bits.append(START_BITS)
    payload_bits.append(checksum_bits)
    payload_bits.append(STOP_BITS)

    payload_bits.append(ENDSYNC_BITS)

    return payload_bits


def generate_single_chirp(samp_rate):
    wave = np.hstack((
        np.tile(np.hstack((
            chirp(np.linspace(0, 63E-6, int(63E-6 * samp_rate)),
                  f0=203E3, f1=400E3, t1=63E-6, phi=-90, method='linear'),
            chirp(np.linspace(63E-6, 67E-6, int(4E-6 * samp_rate)),
                  f0=400E3, f1=100E3, t1=67E-6, phi=-90, method='linear'),
            chirp(np.linspace(67E-6, 100E-6, int(33E-6 * samp_rate)),
                  f0=100E3, f1=200E3, t1=100E-6, phi=-90, method='linear')
        )), 1)
    ))
    target_len = int(100e-6 * samp_rate)
    wave = np.append(wave, np.zeros(np.max([0, target_len - len(wave)])))
    return wave


def generate_single_chirp_alt(samp_rate):
    wave = np.hstack((
        np.tile(np.hstack((
            chirp(np.linspace(0, 63E-6, int(63E-6 * samp_rate)),
                  f0=203E3, f1=394E3, t1=63E-6, phi=-90, method='linear'),
            chirp(np.linspace(63E-6, 67E-6, int(4E-6 * samp_rate)),
                  f0=400E3, f1=100E3, t1=67E-6, phi=-90, method='linear'),
            chirp(np.linspace(67E-6, 100E-6, int(33E-6 * samp_rate)),
                  f0=1E3, f1=216E3, t1=100E-6, phi=-30, method='linear')
        )), 1)
    ))
    target_len = int(100e-6 * samp_rate)
    wave = np.append(wave, np.zeros(np.max([0, target_len - len(wave)])))
    return wave


def generate_signal(j1708_message: bytes, samp_rate, j2497_preamble_mid_byte=None, j1708_checksum=None,
                    extra_stop_bits=None, truncate_at_checksum=False):
    if extra_stop_bits is None:
        extra_stop_bits = [0, ]
    local_chirp = generate_single_chirp(samp_rate)

    if j2497_preamble_mid_byte is None:
        j2497_preamble_mid_byte = chr(bytes(j1708_message)[0]).encode('latin-1')
    elif type(j2497_preamble_mid_byte) is int:
        j2497_preamble_mid_byte = bytes([j2497_preamble_mid_byte])
    wave = np.zeros(0, np.float32)

    j2497_preamble_bits = get_preamble_bits(j2497_preamble_mid_byte)
    wave = np.append(wave,
                        get_preamble_chirps(j2497_preamble_bits, samp_rate, local_chirp))

    j2497_payload_bits = get_payload_bits(j1708_message, j1708_checksum,
                                          extra_stop_bits=extra_stop_bits,
                                          truncate_at_checksum=truncate_at_checksum)
    wave = np.append(wave,
                        get_payload_chirps(j2497_payload_bits, samp_rate, local_chirp))

    return wave


def get_preamble_chirps(j2497_preamble_bits, samp_rate, local_chirp=None):
    if local_chirp is None:
        local_chirp = generate_single_chirp(samp_rate)
    wave = np.zeros(0, np.float32)
    for n in j2497_preamble_bits:
        if not n:
            wave = np.append(wave, local_chirp)
            wave = np.append(wave, np.zeros(int(samp_rate * 114e-6) - len(local_chirp)))
        else:
            wave = np.append(wave, np.zeros(int(samp_rate * 114e-6)))
    return wave


def get_payload_chirps(j2497_payload_bits, samp_rate, local_chirp=None):
    if local_chirp is None:
        local_chirp = generate_single_chirp(samp_rate)
    wave = np.zeros(0, np.float32)
    for n in j2497_payload_bits:
        if n:
            wave = np.append(wave, local_chirp)
        else:
            wave = np.append(wave, local_chirp * -1)
    return wave


def generate_signal_fromhex(j1708_message_hex, samp_rate, j2497_preamble_hex=None):
    j1708_message_hex = j1708_message_hex.replace(',', '')
    j1708_message = bytes(bytearray.fromhex(j1708_message_hex))
    if j2497_preamble_hex is not None:
        j2497_preamble = bytes(bytearray.fromhex(j2497_preamble_hex))
    else:
        j2497_preamble = chr(j1708_message[0]).encode('latin-1')
    return generate_signal(j1708_message, samp_rate, j2497_preamble_mid_byte=j2497_preamble)
