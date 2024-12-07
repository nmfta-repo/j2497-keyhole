#!/usr/bin/env python3

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
import subprocess
import errno

import j2497_keyhole

FL2K_FULL_SCALE = 127
FL2K_SAMP_RATE = 7777777  # The lowest FL2K sample rate that it will support

# Some zeros to 'warm-up' the fl2k transmitter before sending the J2497 waveform, otherwise
# waveform is corrupted by fl2k transmit
FL2K_WARMUP_SIZE = FL2K_SAMP_RATE * 4
# Some zeros to cool-down the fl2k transmitter, otherwise the waveform is corrupted by fl2k transmit
FL2K_COOLDOWN_SIZE = FL2K_SAMP_RATE * 4



def prep_signal(signal: np.ndarray):
    out = signal * FL2K_FULL_SCALE
    out = out.astype('int8').tobytes()
    return out


REPEAT = 50
RX_MIN_WAIT_US=15_000  # wait time to use as minimum between fast-as-possible messages sent
FL2K_WRITE_CHUNK_SIZE = 4096  # size of bytes to write at a time to the FL2K subprocess
if __name__ == '__main__':
    warmup = np.zeros(FL2K_WARMUP_SIZE, np.int8).tobytes()
    cooldown = np.zeros(FL2K_COOLDOWN_SIZE, np.int8).tobytes()
    chirps_chain = itertools.chain(
        j2497_keyhole.generate(sample_rate=FL2K_SAMP_RATE, calibration_mode=False),
    )

    # NB: you will need to run this once per boot to increase the USB buffers:
    #
    #     sudo sh -c 'echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb'
    #
    p = subprocess.Popen(['fl2k_file', '-s', str(FL2K_SAMP_RATE), '-r', '1', '-'],
                         stdin=subprocess.PIPE)
    t = p.stdin
    t.write(warmup)
    sys.stderr.write('warmed-up; please expect at least one "Underflow!" message\n')

    repeating_chirps = itertools.chain.from_iterable(itertools.repeat(tuple(chirps_chain), REPEAT))
    for chirps in repeating_chirps:
        l = len(chirps)
        try:
            for i in range(0, l, FL2K_WRITE_CHUNK_SIZE):
                t.write(prep_signal(chirps[i: i + FL2K_WRITE_CHUNK_SIZE if i + FL2K_WRITE_CHUNK_SIZE < l else l - 1]))
        except IOError as e:
            if e.errno == errno.EPIPE or e.errno == errno.EINVAL:
                break
            else:
                raise

    sys.stderr.write('cooldown\n')
    t.write(cooldown)

    p.stdin.close()
    p.kill()
    p.wait()
