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
FL2K_WARMUP_SIZE = int(FL2K_SAMP_RATE / 128.0)
# Some zeros to cool-down the fl2k transmitter, otherwise the waveform is corrupted by fl2k transmit
FL2K_COOLDOWN_SIZE = FL2K_SAMP_RATE

REPEAT = 4096


def prep_signal(signal: np.ndarray):
    out = signal * FL2K_FULL_SCALE
    out = out.astype('int8').tobytes()
    return out


if __name__ == '__main__':
    warmup = np.zeros(FL2K_WARMUP_SIZE, np.int8).tobytes()
    cooldown = np.zeros(FL2K_COOLDOWN_SIZE, np.int8).tobytes()
    doors_n_keyholes = [prep_signal(x) for x in
                        j2497_keyhole.generate(sample_rate=FL2K_SAMP_RATE, calibration_mode=False)]

    # NB: you will probably want to run this once per boot to increase the USB buffers:
    # sudo sh -c 'echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb'
    p = subprocess.Popen(['fl2k_file', '-s', str(FL2K_SAMP_RATE), '-r', '1', '-'],
                         stdin=subprocess.PIPE)
    t = p.stdin
    t.write(warmup)
    for i in range(0, REPEAT):
        for dnk in doors_n_keyholes:
            try:
                t.write(dnk)
            except IOError as e:
                if e.errno == errno.EPIPE or e.errno == errno.EINVAL:
                    break
                else:
                    raise
    t.write(cooldown)

    p.stdin.close()
    p.kill()
    p.wait()
