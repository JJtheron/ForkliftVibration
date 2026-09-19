#!/usr/bin/env python3
"""
Forklift whole-body vibration / shock event logger
Raspberry Pi + ADXL343 (I2C), ISO 2631-1 frequency weighting.

Captures continuous seat-pad acceleration, applies Wk (vertical) and Wd
(horizontal) weighting, accumulates VDV, and writes a waveform file for
every shock event that exceeds threshold.

    sudo apt install python3-numpy python3-scipy python3-smbus2
    pip install gpiozero

    python3 wbv_shock_logger.py --selftest     # verify weighting filters
    python3 wbv_shock_logger.py                # run

Wiring (I2C):
    ADXL343 VIN -> 3V3       SDA -> GPIO2      INT1 -> GPIO17  (watermark)
    ADXL343 GND -> GND       SCL -> GPIO3      INT2 -> GPIO27  (--hw-gate)
SDO to GND gives address 0x53; SDO to 3V3 gives 0x1D.
"""

import argparse
import csv
import math
import os
import queue
import signal
import sys
import threading
import time
from collections import deque
from datetime import datetime, timezone

import numpy as np
from scipy.signal import bilinear, tf2sos, sosfilt, sosfilt_zi, sosfreqz

# --------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------

I2C_BUS = 1
I2C_ADDR = 0x53
INT_GPIO = 17

# 800 Hz keeps bilinear warping error under ~4% at the 80 Hz band edge.
# At 400 Hz the Wk realisation is ~19% low at 80 Hz. Do not drop below 400.
ODR_HZ = 800.0
G_RANGE = 4              # +/- 4 g. Dock-plate hits will clip a 2 g part.
FIFO_WATERMARK = 24      # of 32. ~30 ms of data per interrupt at 800 Hz.

# Shock detection, on weighted acceleration
SHOCK_PEAK_MS2 = 2.5     #2.5 instantaneous weighted peak that counts as a shock
SHOCK_REARM_S = 0.5      #0.5 refractory period so one pothole is one event
PRE_TRIGGER_S = 1.0      #1.0  waveform captured before the trigger sample
POST_TRIGGER_S = 2.0     #2.0  ...and after

# Health guidance thresholds (ISO 2631-1 / EU 2002/44/EC), 8 h equivalent
EAV_AWMS2 = 0.5          # exposure action value, r.m.s.
ELV_AWMS2 = 1.15         # exposure limit value, r.m.s.
EAV_VDV = 9.1            # exposure action value, VDV
ELV_VDV = 21.0           # exposure limit value, VDV

# Multiplying factors for health, seated (ISO 2631-1 clause 7)
K_FACTORS = {"x": 1.4, "y": 1.4, "z": 1.0}

OUT_DIR = os.environ.get("WBV_OUT_DIR", "./wbv_events")
SUMMARY_INTERVAL_S = 60.0

# --- Motion gating -------------------------------------------------------
# Parked time must not dilute the metrics: an hour of standing still drags
# the r.m.s. down and makes a rough truck look compliant. While gated off,
# samples are still filtered (to keep filter state continuous) but are not
# accumulated and cannot trigger a shock capture.
#
# Software arbitration runs on WEIGHTED r.m.s., which is far finer than the
# chip's 62.5 mg threshold steps. The hardware ACT/INACT interrupt is used
# as a corroborating hint only.
GATE_WINDOW_S = 1.0        # r.m.s. window for the gate decision
GATE_IDLE_RMS = 0.05       # below this the truck is considered stopped
GATE_MOVING_RMS = 0.10     # above this it is moving again (2x hysteresis)
GATE_IDLE_HOLD_S = 30.0    # sustained quiet time before gating off

# Hardware activity/inactivity thresholds, m/s^2 on RAW acceleration.
# Quantised to 62.5 mg (0.613 m/s^2) steps by the part.
HW_ACT_MS2 = 1.23          # 2 LSB - wake
HW_INACT_MS2 = 0.61        # 1 LSB - sleep, must be below HW_ACT_MS2
HW_INACT_TIME_S = 30       # 1 s/LSB, max 255

THRESH_LSB_MS2 = 0.0625 * 9.80665      # 62.5 mg per count

G_MS2 = 9.80665
LSB_G = 0.0039           # full-resolution mode, all ranges
LSB_MS2 = LSB_G * G_MS2

# --------------------------------------------------------------------------
# ISO 2631-1 frequency weighting
# --------------------------------------------------------------------------
# Band-limiting and acceleration-velocity transition sections follow the
# analogue transfer functions in ISO 2631-1 Annex A directly.
#
# The Wk upward-step parameters below are a least-squares fit to the
# tabulated Wk values, accurate to ~3.7% in the analogue domain. Wd uses the
# Annex A parameters unmodified and reproduces the published table to <1%.
#
# >>> VALIDATE Wk AGAINST YOUR COPY OF THE STANDARD BEFORE REPORTING
# >>> ABSOLUTE COMPLIANCE NUMBERS. --selftest prints achieved vs reference.

TP = 2.0 * math.pi

WK_FIT = dict(f3=2.8335, Q4=1.3520, f5=2.5254, Q5=1.4154,
              f6=7.8046, Q6=0.5946, gain=0.48045)

# ISO 2631-1 principal weighting curves, third-octave centres.
#
# >>> THESE TABLES ARE A TRANSCRIPTION AND ARE NOT AUTHORITATIVE. Replace
# >>> them with the values printed in ISO 2631-1 Table 3 before you rely on
# >>> --selftest as evidence of anything. The tail entries (63, 80 Hz) are
# >>> the least trustworthy. They exist so the self-test has something to
# >>> compare against, not to define the filter.
REF_FREQS = np.array([0.5, 0.63, 0.8, 1, 1.25, 1.6, 2, 2.5, 3.15, 4, 5, 6.3,
                      8, 10, 12.5, 16, 20, 25, 31.5, 40, 50, 63, 80])
REF_WK = np.array([0.418, 0.459, 0.477, 0.482, 0.484, 0.494, 0.531, 0.631,
                   0.804, 0.967, 1.039, 1.054, 1.036, 0.988, 0.902, 0.768,
                   0.636, 0.513, 0.405, 0.314, 0.246, 0.186, 0.132])
REF_WD = np.array([0.853, 0.944, 0.992, 1.011, 1.008, 0.968, 0.890, 0.776,
                   0.642, 0.512, 0.409, 0.323, 0.253, 0.201, 0.160, 0.125,
                   0.099, 0.079, 0.062, 0.048, 0.037, 0.026, 0.016])


def _analog_sections(kind):
    """Return [(num, den), ...] in s for the weighting cascade."""
    w1, w2 = TP * 0.4, TP * 100.0                    # band limits, all curves
    secs = [
        ([1, 0, 0], [1, math.sqrt(2) * w1, w1 ** 2]),        # high pass
        ([w2 ** 2], [1, math.sqrt(2) * w2, w2 ** 2]),        # low pass
    ]
    if kind == "Wk":
        p = WK_FIT
        w3 = w4 = TP * p["f3"]
        secs.append(([w4 ** 2 / w3, w4 ** 2], [1, w4 / p["Q4"], w4 ** 2]))
        w5, w6, g = TP * p["f5"], TP * p["f6"], p["gain"]
        secs.append(([g / w5 ** 2, g / (p["Q5"] * w5), g],
                     [1 / w6 ** 2, 1 / (p["Q6"] * w6), 1.0]))
    elif kind == "Wd":
        w3 = w4 = TP * 2.0
        secs.append(([w4 ** 2 / w3, w4 ** 2], [1, w4 / 0.63, w4 ** 2]))
    else:
        raise ValueError(kind)
    return secs


def build_weighting_sos(kind, fs):
    """Digital second-order sections for weighting `kind` at rate `fs`."""
    out = []
    for b, a in _analog_sections(kind):
        bz, az = bilinear(np.atleast_1d(b).astype(float),
                          np.atleast_1d(a).astype(float), fs)
        out.append(tf2sos(bz, az))
    return np.vstack(out)


SIGNIFICANT_WEIGHT = 0.1   # below this a band contributes negligibly


def selftest(fs=ODR_HZ):
    print(f"ISO 2631-1 weighting realisation at fs = {fs:.0f} Hz")
    print("Reference columns are transcribed, not authoritative - see the "
          "note above REF_FREQS.\n")
    ok = True
    for kind, ref in (("Wk", REF_WK), ("Wd", REF_WD)):
        sos = build_weighting_sos(kind, fs)
        _, h = sosfreqz(sos, worN=REF_FREQS * TP / fs)
        got = np.abs(h)
        err = 100.0 * (got - ref) / ref
        sig = ref >= SIGNIFICANT_WEIGHT
        print(f"  {kind}   {'Hz':>7} {'got':>8} {'ref':>8} {'err%':>8}")
        for f, g, r, e, s in zip(REF_FREQS, got, ref, err, sig):
            mark = "" if s else "   (low weight, not judged)"
            print(f"        {f:7.2f} {g:8.3f} {r:8.3f} {e:8.1f}{mark}")
        worst = np.max(np.abs(err[sig]))
        print(f"        worst deviation where weight >= "
              f"{SIGNIFICANT_WEIGHT}: {worst:.1f}%\n")
        if worst > 10.0:
            ok = False
    print("PASS" if ok else "FAIL - check fs and filter parameters")
    return 0 if ok else 1


# --------------------------------------------------------------------------
# ADXL343 driver
# --------------------------------------------------------------------------

REG_DEVID = 0x00
REG_THRESH_ACT = 0x24
REG_THRESH_INACT = 0x25
REG_TIME_INACT = 0x26
REG_ACT_INACT_CTL = 0x27
REG_BW_RATE = 0x2C
REG_POWER_CTL = 0x2D
REG_INT_ENABLE = 0x2E
REG_INT_MAP = 0x2F
REG_INT_SOURCE = 0x30
REG_DATA_FORMAT = 0x31
REG_DATAX0 = 0x32
REG_FIFO_CTL = 0x38
REG_FIFO_STATUS = 0x39

RATE_CODES = {100.0: 0x0A, 200.0: 0x0B, 400.0: 0x0C, 800.0: 0x0D, 1600.0: 0x0E}
RANGE_CODES = {2: 0x00, 4: 0x01, 8: 0x02, 16: 0x03}


class ADXL343:
    def __init__(self, bus=I2C_BUS, addr=I2C_ADDR):
        from smbus2 import SMBus, i2c_msg
        self._i2c_msg = i2c_msg
        self.bus = SMBus(bus)
        self.addr = addr
        devid = self._r8(REG_DEVID)
        if devid != 0xE5:
            raise RuntimeError(f"DEVID 0x{devid:02X}, expected 0xE5 "
                               f"(check address and wiring)")

    def _w8(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    def _r8(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def configure(self, odr=ODR_HZ, g_range=G_RANGE, watermark=FIFO_WATERMARK):
        if odr not in RATE_CODES:
            raise ValueError(f"unsupported ODR {odr}")
        self._w8(REG_POWER_CTL, 0x00)                 # standby while configuring
        self._w8(REG_BW_RATE, RATE_CODES[odr])
        # FULL_RES keeps 3.9 mg/LSB at every range; bit 3 = FULL_RES
        self._w8(REG_DATA_FORMAT, 0x08 | RANGE_CODES[g_range])
        self._w8(REG_FIFO_CTL, 0x00)                  # bypass, flushes FIFO
        self._w8(REG_FIFO_CTL, 0x80 | (watermark & 0x1F))   # stream mode
        self._w8(REG_INT_MAP, 0x00)                   # all interrupts -> INT1
        self._w8(REG_INT_ENABLE, 0x02)                # watermark only
        self._r8(REG_INT_SOURCE)                      # clear latched state
        self._w8(REG_POWER_CTL, 0x08)                 # measure

    def configure_activity_gate(self, act_ms2=HW_ACT_MS2,
                                inact_ms2=HW_INACT_MS2,
                                inact_time_s=HW_INACT_TIME_S):
        """Enable AC-coupled activity/inactivity detection on INT2.

        AC coupling subtracts a running reference, so the part reacts to
        CHANGE rather than to orientation - without it, gravity on the
        vertical axis sits permanently above any sensible threshold.

        Thresholds quantise to 62.5 mg. Returns the actual values written.
        """
        act = max(1, min(255, round(act_ms2 / THRESH_LSB_MS2)))
        inact = max(1, min(255, round(inact_ms2 / THRESH_LSB_MS2)))
        if inact >= act:
            inact = max(1, act - 1)     # preserve hysteresis after rounding
        t_inact = max(1, min(255, int(round(inact_time_s))))

        self._w8(REG_THRESH_ACT, act)
        self._w8(REG_THRESH_INACT, inact)
        self._w8(REG_TIME_INACT, t_inact)
        # bit7 ACT ac-coupled, bits 6:4 ACT x/y/z
        # bit3 INACT ac-coupled, bits 2:0 INACT x/y/z
        self._w8(REG_ACT_INACT_CTL, 0xFF)
        # watermark stays on INT1 (bit clear); activity+inactivity -> INT2
        self._w8(REG_INT_MAP, 0x18)
        self._w8(REG_INT_ENABLE, 0x02 | 0x10 | 0x08)
        self._r8(REG_INT_SOURCE)
        print(f">>>>>>>>>{act * THRESH_LSB_MS2}, {inact * THRESH_LSB_MS2}, {t_inact}<<<<<<<<<<")
        return act * THRESH_LSB_MS2, inact * THRESH_LSB_MS2, t_inact

    def int_source(self):
        """Read and clear the interrupt source register.

        Returns (activity, inactivity) as booleans. Note this also clears
        the watermark bit, which is harmless - the watermark bit re-asserts
        while the FIFO stays above the level.
        """
        src = self._r8(REG_INT_SOURCE)
        return bool(src & 0x10), bool(src & 0x08)

    def fifo_count(self):
        return self._r8(REG_FIFO_STATUS) & 0x3F

    def read_samples(self, n):
        """Pop n samples from the FIFO. Returns (n,3) float array in m/s^2.

        Each 6-byte burst from DATAX0 pops exactly one FIFO entry, so the
        samples must be read one at a time; the part needs >5 us between
        consecutive reads for the FIFO to present the next entry.
        """
        raw = np.empty((n, 3), dtype=np.int16)
        for i in range(n):
            write = self._i2c_msg.write(self.addr, [REG_DATAX0])
            read = self._i2c_msg.read(self.addr, 6)
            self.bus.i2c_rdwr(write, read)
            b = bytes(read)
            raw[i] = np.frombuffer(b, dtype="<i2", count=3)
            time.sleep(6e-6)
        return raw.astype(np.float64) * LSB_MS2

    def close(self):
        try:
            self._w8(REG_INT_ENABLE, 0x00)
            self._w8(REG_POWER_CTL, 0x00)
        finally:
            self.bus.close()


# --------------------------------------------------------------------------
# Streaming metrics
# --------------------------------------------------------------------------

class AxisProcessor:
    """Weighting filter plus running r.m.s. / VDV / peak for one axis."""

    def __init__(self, kind, fs, k):
        self.sos = build_weighting_sos(kind, fs)
        self.zi = None                 # primed on first block, avoids a step
        self.fs = fs
        self.k = k
        self.sum_sq = 0.0
        self.sum_4th = 0.0
        self.n = 0
        self.peak = 0.0

    def process(self, x, accumulate=True):
        """Filter a block. Metrics accumulate only when `accumulate`.

        The filter always runs, even when gated off: dropping samples would
        discard the filter state and ring a 0.4 Hz high-pass on every
        resume, which looks exactly like a shock.
        """
        if self.zi is None:
            self.zi = sosfilt_zi(self.sos) * x[0]
        y, self.zi = sosfilt(self.sos, x, zi=self.zi)
        yk = y * self.k
        if accumulate:
            self.sum_sq += float(np.sum(yk ** 2))
            self.sum_4th += float(np.sum(yk ** 4))
            self.n += yk.size
            self.peak = max(self.peak, float(np.max(np.abs(yk))))
        return yk

    def accumulate_block(self, yk):
        """Fold an already-filtered, k-scaled block into the metrics."""
        self.sum_sq += float(np.sum(yk ** 2))
        self.sum_4th += float(np.sum(yk ** 4))
        self.n += yk.size
        self.peak = max(self.peak, float(np.max(np.abs(yk))))

    @property
    def rms(self):
        return math.sqrt(self.sum_sq / self.n) if self.n else 0.0

    @property
    def vdv(self):
        return (self.sum_4th / self.fs) ** 0.25 if self.n else 0.0

    @property
    def crest(self):
        r = self.rms
        return self.peak / r if r > 1e-9 else 0.0

    def projected(self, exposure_hours):
        """Normalise to an 8 h day given actual daily exposure duration.

        `exposure_hours` is how long the operator is on the truck per day,
        NOT how long this measurement ran. The measurement is assumed
        representative of that exposure.

        A(8) = aw * sqrt(T_exposure / 8h)    - r.m.s. already time-averages
        VDV(8) = VDV_meas * (T_exposure / T_measured)^(1/4)
        """
        if not self.n:
            return 0.0, 0.0
        t_meas = self.n / self.fs
        t_exp = exposure_hours * 3600.0
        aw8 = self.rms * math.sqrt(t_exp / (8.0 * 3600.0))
        vdv8 = self.vdv * (t_exp / t_meas) ** 0.25
        return aw8, vdv8


class MotionGate:
    """Decides whether the truck is moving, so parked time is excluded.

    Software decision on weighted r.m.s. with 2x hysteresis, plus an
    optional corroborating hardware inactivity flag. The chip's 62.5 mg
    threshold steps are too coarse to arbitrate alone, but the hardware
    flag is a useful second opinion: requiring both to agree before gating
    off avoids dropping a genuinely smooth but moving stretch.
    """

    def __init__(self, fs, use_hw=False, enabled=True):
        self.fs = fs
        self.use_hw = use_hw
        self.enabled = enabled
        self.win = max(1, int(GATE_WINDOW_S * fs))
        self.buf = deque(maxlen=self.win)
        self.buf_sq = 0.0
        self.moving = True          # assume moving until proven otherwise
        self.quiet_since = None
        self.hw_inactive = False
        self.moving_s = 0.0
        self.idle_s = 0.0
        self.transitions = 0

    def hw_event(self, activity, inactivity):
        if activity:
            self.hw_inactive = False
        elif inactivity:
            self.hw_inactive = True

    def update(self, t0, weighted):
        """Feed one block, return True if it should count."""
        dt = weighted.shape[0] / self.fs
        if not self.enabled:
            self.moving_s += dt
            return True
        # running r.m.s. over the window, magnitude across axes
        mag = np.sqrt(np.sum(weighted ** 2, axis=1))
        for v in mag:
            if len(self.buf) == self.buf.maxlen and self.buf:
                self.buf_sq -= self.buf[0] ** 2
            self.buf.append(v)
            self.buf_sq += v ** 2
        rms = math.sqrt(max(0.0, self.buf_sq) / len(self.buf))

        if self.moving:
            quiet = rms < GATE_IDLE_RMS and (self.hw_inactive or not self.use_hw)
            if quiet:
                if self.quiet_since is None:
                    self.quiet_since = t0
                elif t0 - self.quiet_since >= GATE_IDLE_HOLD_S:
                    self.moving = False
                    self.transitions += 1
                    self.quiet_since = None
            else:
                self.quiet_since = None
        else:
            if rms > GATE_MOVING_RMS:
                self.moving = True
                self.transitions += 1

        if self.moving:
            self.moving_s += dt
        else:
            self.idle_s += dt
        return self.moving

    @property
    def duty(self):
        total = self.moving_s + self.idle_s
        return self.moving_s / total if total > 0 else 0.0


class ShockLogger:
    def __init__(self, fs, out_dir=OUT_DIR, on_event=None, g_range=G_RANGE):
        self.fs = fs
        self.out_dir = out_dir
        self.on_event = on_event
        # full-scale in m/s^2; raw samples at the rail mean the peak is a
        # lower bound, not a measurement
        self.clip_ms2 = 0.98 * g_range * G_MS2
        os.makedirs(out_dir, exist_ok=True)
        self.pre = int(PRE_TRIGGER_S * fs)
        self.post = int(POST_TRIGGER_S * fs)
        # Bounded by SAMPLES, not blocks. Blocks arrive in FIFO-sized chunks,
        # so a deque(maxlen=pre) would hold pre*blocksize samples.
        self.ring = deque()
        self.ring_n = 0
        self.pending = None            # list of blocks after a trigger
        self.pending_n = 0
        self.trigger_offset = 0        # sample index of trigger within capture
        self.last_trigger = -1e9
        self.count = 0
        self.index_path = os.path.join(out_dir, "events.csv")
        if not os.path.exists(self.index_path):
            with open(self.index_path, "w", newline="") as f:
                csv.writer(f).writerow(
                    ["utc", "event", "peak_ms2", "axis", "crest", "file"])

    # Note on event counting: while a capture is in progress the detector is
    # not re-armed, so the effective refractory period is
    # max(SHOCK_REARM_S, POST_TRIGGER_S). Two hits 1.2 s apart land in one
    # waveform file and count as one event. Dock plates often produce a pair
    # (on and off the plate), so shorten POST_TRIGGER_S if you need them
    # counted separately.

    def _push_ring(self, block):
        self.ring.append(block)
        self.ring_n += block.shape[0]
        while self.ring and self.ring_n - self.ring[0].shape[0] >= self.pre:
            self.ring_n -= self.ring.popleft().shape[0]

    def feed(self, t0, raw, weighted):
        """raw/weighted are (n,3); t0 is the timestamp of raw[0]."""
        block = np.hstack([weighted, raw])
        if self.pending is not None:
            self.pending.append(block)
            self.pending_n += block.shape[0]
            if self.pending_n - self.trigger_offset >= self.post:
                self._flush()
            return None

        peaks = np.abs(weighted)
        idx = int(np.argmax(np.max(peaks, axis=1)))
        peak = float(np.max(peaks[idx]))
        if peak < SHOCK_PEAK_MS2 or t0 - self.last_trigger < SHOCK_REARM_S:
            self._push_ring(block)
            return None

        self.last_trigger = t0
        axis = "xyz"[int(np.argmax(peaks[idx]))]
        self.trigger_meta = (t0 + idx / self.fs, peak, axis)
        self.pending = list(self.ring) + [block]
        self.pending_n = self.ring_n + block.shape[0]
        self.trigger_offset = self.ring_n + idx
        self.ring.clear()
        self.ring_n = 0
        return peak

    def _flush(self):
        data = np.vstack(self.pending)
        lo = max(0, self.trigger_offset - self.pre)
        data = data[lo:self.trigger_offset + self.post]
        self.pending = None
        self.pending_n = 0
        t, peak, axis = self.trigger_meta
        self.count += 1
        stamp = datetime.fromtimestamp(t, timezone.utc)
        name = f"shock_{stamp:%Y%m%dT%H%M%S}_{self.count:05d}.npz"
        path = os.path.join(self.out_dir, name)
        w = data[:, :3]
        raw = data[:, 3:]
        rms = float(np.sqrt(np.mean(w ** 2)))
        crest = peak / rms if rms > 1e-9 else 0.0
        raw_peak = float(np.max(np.abs(raw)))
        clipped = raw_peak >= self.clip_ms2
        # this event's own contribution to the running VDV
        vdv_contrib = float((np.sum(w ** 4) / self.fs) ** 0.25)
        np.savez_compressed(path, fs=self.fs, t_trigger=t,
                            weighted=w.astype(np.float32),
                            raw=raw.astype(np.float32),
                            peak_ms2=peak, axis=axis, crest=crest,
                            raw_peak_ms2=raw_peak, clipped=clipped,
                            vdv_contrib=vdv_contrib)
        with open(self.index_path, "a", newline="") as f:
            csv.writer(f).writerow([stamp.isoformat(), self.count,
                                    f"{peak:.3f}", axis, f"{crest:.2f}", name])
        if clipped:
            print(f"  WARNING: raw peak {raw_peak:.1f} m/s^2 hit the "
                  f"+/-{self.clip_ms2/G_MS2:.0f} g rail - peak is a floor, "
                  f"not a value", file=sys.stderr, flush=True)
        print(f"  shock #{self.count}  {peak:.2f} m/s^2 on {axis}  "
              f"crest {crest:.1f}  -> {name}", flush=True)
        if self.on_event:
            try:
                self.on_event(dict(seq=self.count, t=t, peak=peak, axis=axis,
                                   crest=crest, vdv_contrib=vdv_contrib,
                                   raw_peak=raw_peak, clipped=clipped,
                                   path=path))
            except Exception as exc:
                print(f"  on_event failed: {exc}", file=sys.stderr, flush=True)

    def close(self):
        if self.pending is not None:
            self._flush()


# --------------------------------------------------------------------------
# Acquisition
# --------------------------------------------------------------------------

def run(args):
    fs = args.odr
    dev = ADXL343(args.bus, args.addr)
    dev.configure(odr=fs, g_range=args.range, watermark=args.watermark)
    print(f"ADXL343 up: {fs:.0f} Hz, +/-{args.range} g, "
          f"watermark {args.watermark}")

    axes = {
        "x": AxisProcessor("Wd", fs, K_FACTORS["x"]),
        "y": AxisProcessor("Wd", fs, K_FACTORS["y"]),
        "z": AxisProcessor("Wk", fs, K_FACTORS["z"]),
    }
    # ---- GPS ---------------------------------------------------------
    track = None
    gps_reader = None
    store = None
    if args.db:
        import DataBaseInterface as store_mod
        dsn = {"host": args.db_host, "database": args.db_name}
        if args.db_port:
            dsn["port"] = args.db_port
        if args.db_user:
            dsn["user"] = args.db_user
        if args.db_password:
            dsn["password"] = args.db_password
        if args.db_socket:
            dsn["unix_socket"] = args.db_socket
        if args.db_defaults_file and os.path.exists(args.db_defaults_file):
            dsn["read_default_file"] = args.db_defaults_file
        while True:
            try:
                store = store_mod.Store(dsn=dsn).connect()
                break
            except Exception as exc:
                print(f"Waiting for database... {exc}", file=sys.stderr,
                      flush=True)
                time.sleep(3)
        sid = store.start_session(dict(
            logger_id=args.logger_id, truck_id=args.truck_id,
            operator_ref=args.operator_ref,
            started_at=time.time(),
            sample_rate_hz=fs, accel_range_g=args.range,
            fifo_watermark=args.watermark,
            shock_threshold_ms2=SHOCK_PEAK_MS2,
            pre_trigger_s=PRE_TRIGGER_S, post_trigger_s=POST_TRIGGER_S,
            weighting_version=store_mod.WEIGHTING_VERSION,
            k_factor_x=K_FACTORS["x"], k_factor_y=K_FACTORS["y"],
            k_factor_z=K_FACTORS["z"],
            gating_enabled=not args.no_gate,
            config={k: v for k, v in vars(args).items()
                    if k != "db_password"},
        ))
        store.start()
        print(f"MySQL session {sid} ({store.session_uuid})")
    if args.gps:
        import GPS_interface as gps_mod
        try:
            gps_reader = gps_mod.GPSReader(
                port=args.gps_port, rate_ms=args.gps_rate_ms,
                on_fix=(store.add_fix if store else None)).open()
            gps_reader.start()
            track = gps_reader.track
            print(f"{track}track1<<<<<<<<<<<<<<<<<<<<<<<<<<<")
            print(f"GPS on {args.gps_port} at "
                  f"{1000/args.gps_rate_ms:.0f} Hz")
        except Exception as exc:
            print(f"GPS unavailable ({exc}); events will not be geotagged",
                  file=sys.stderr)
    def on_shock(ev):
        if store is None:
            return
        import DataBaseInterface as store_mod
        print(f"{track}track2<<<<<<<<<<<<<<<<<<<<<<<<<<<")
        fix, src, err = (track.at(ev["t"]) if track else (None, "none", None))
        print(f"{fix}, {src}, {err}track3<<<<<<<<<<<<<<<<<<<<<<<<<<<")
        store.add_event(store_mod.build_event_row(
            ev["seq"], ev["t"], ev["peak"], ev["axis"], ev["crest"],
            ev["vdv_contrib"], ev["raw_peak"], ev["clipped"],
            fix, src, err, ev["path"]))
    shocks = ShockLogger(fs, args.out_dir, on_event=on_shock,
                         g_range=args.range)
    gate = MotionGate(fs, use_hw=args.hw_gate, enabled=not args.no_gate)
    if args.no_gate:
        print("Motion gating DISABLED - parked time will dilute the metrics")
    print(f"{args.hw_gate, args.no_gate}, InitMotionGate<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
    if args.hw_gate and not args.no_gate:
        a, i, t = dev.configure_activity_gate()
        print(f"Hardware gate: wake >{a:.2f}, sleep <{i:.2f} m/s^2 raw "
              f"for {t} s (62.5 mg steps)")
    q = queue.Queue(maxsize=256)
    stop = threading.Event()

    def drain():
        """Pull whatever the FIFO holds and queue it with a timestamp."""
        n = dev.fifo_count()
        if n <= 0:
            return
        t_end = time.time()
        block = dev.read_samples(n)
        try:
            q.put_nowait((t_end - n / fs, block))
        except queue.Full:
            print("  WARNING: processing fell behind, block dropped",
                  file=sys.stderr, flush=True)
    gate_pin = None
    if args.hw_gate and not args.no_gate:
        try:
            from gpiozero import DigitalInputDevice
            gate_pin = DigitalInputDevice(args.gate_gpio, pull_up=False)
            gate_pin.when_activated = lambda: gate.hw_event(*dev.int_source())
            print(f"Activity/inactivity interrupt on GPIO{args.gate_gpio}")
        except Exception as exc:
            print(f"INT2 setup failed ({exc}); software gate only",
                  file=sys.stderr)
            gate.use_hw = False
    trigger = None
    if not args.poll:
        try:
            from gpiozero import DigitalInputDevice
            trigger = DigitalInputDevice(args.int_gpio, pull_up=False)
            trigger.when_activated = lambda: drain()
            print(f"Watermark interrupt on GPIO{args.int_gpio}")
        except Exception as exc:
            print(f"Interrupt setup failed ({exc}); falling back to polling",
                  file=sys.stderr)
    def poller():
        period = args.watermark / fs * 0.5
        while not stop.is_set():
            drain()
            time.sleep(period)
    if trigger is None:
        threading.Thread(target=poller, daemon=True).start()
        print("Polling FIFO")

    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    print(f"Logging to {args.out_dir}. Ctrl-C to stop.\n")
    started = time.time()
    period_start = started
    next_summary = started + SUMMARY_INTERVAL_S
    try:
        while not stop.is_set():
            try:
                t0, block = q.get(timeout=0.5)
            except queue.Empty:
                continue
            # Filter first without accumulating: the gate decision needs
            # the weighted signal it is about to judge.
            weighted = np.column_stack([
                axes["x"].process(block[:, 0], accumulate=False),
                axes["y"].process(block[:, 1], accumulate=False),
                axes["z"].process(block[:, 2], accumulate=False),
            ])
            was_moving = gate.moving
            moving = gate.update(t0, weighted)
            if moving:
                for idx, name in enumerate(("x", "y", "z")):
                    axes[name].accumulate_block(weighted[:, idx])
                shocks.feed(t0, block, weighted)
            elif was_moving:
                shocks.close()          # finish any capture in flight
                print(f"  gated off - stopped at {elapsed_str(time.time()-started)}",
                      flush=True)
            if moving and not was_moving:
                print(f"  gated on - moving at {elapsed_str(time.time()-started)}",
                      flush=True)

            if time.time() >= next_summary:
                now = time.time()
                row = summarise(axes, shocks, now - started,
                                args.exposure_hours, gate)
                if store is not None:
                    import DataBaseInterface as store_mod
                    store.add_interval((store_mod.utc(period_start),
                                        store_mod.utc(now)) + row)
                period_start = now
                next_summary += SUMMARY_INTERVAL_S
    finally:
        # Order matters. shocks.close() can still emit a final event
        # through on_event, and the last interval has to be queued, so the
        # store is the very last thing shut down.
        stop.set()
        if trigger is not None:
            trigger.close()
        if gate_pin is not None:
            gate_pin.close()
        if gps_reader is not None:
            gps_reader.stop()
        shocks.close()
        dev.close()
        print()
        now = time.time()
        row = summarise(axes, shocks, now - started,
                        args.exposure_hours, gate, final=True)
        if store is not None:
            import DataBaseInterface as store_mod
            store.add_interval((store_mod.utc(period_start),
                                store_mod.utc(now)) + row)
            store.close()
            print(f"  stored: {store.written['event']} events, "
                  f"{store.written['fix']} fixes, "
                  f"{store.written['interval']} intervals"
                  + (f", {store.dropped} dropped" if store.dropped else "")
                  + (f", last error: {store.last_error}"
                     if store.last_error else ""))
    print("11<<<<<<<<<<<<<<<<<<<<<<<<<")
    return 0


def elapsed_str(sec):
    return f"{int(sec)//60:d}m{int(sec)%60:02d}s"


def summarise(axes, shocks, elapsed, exposure_hours, gate, final=False):
    print(f"--- {'final' if final else 'interim'} @ {elapsed/60:.1f} min "
          f"| {shocks.count} shocks "
          f"| moving {gate.moving_s/60:.1f} min, idle {gate.idle_s/60:.1f} min "
          f"({100*gate.duty:.0f}% duty)")
    worst = max(axes.values(), key=lambda a: a.rms)
    for name, a in axes.items():
        aw8, vdv8 = a.projected(exposure_hours)
        print(f"  {name}: aw {a.rms:5.3f}  VDV {a.vdv:6.3f}  "
              f"peak {a.peak:5.2f}  crest {a.crest:4.1f}  "
              f"| 8h aw {aw8:5.3f}  8h VDV {vdv8:6.3f}")
    aw8, vdv8 = worst.projected(exposure_hours)
    flags = []
    if aw8 >= ELV_AWMS2:
        flags.append("aw OVER LIMIT")
    elif aw8 >= EAV_AWMS2:
        flags.append("aw over action value")
    if vdv8 >= ELV_VDV:
        flags.append("VDV OVER LIMIT")
    elif vdv8 >= EAV_VDV:
        flags.append("VDV over action value")
    print(f"  dominant axis over a {exposure_hours:g} h day: "
          f"A(8) {aw8:.3f} m/s^2, VDV(8) {vdv8:.2f}"
          + (f"  <-- {'; '.join(flags)}" if flags else ""))
    if worst.crest > 9.0:
        print("  crest factor > 9: shock-dominated, treat VDV as the "
              "governing metric")
    print(flush=True)

    dom = max(axes, key=lambda k: axes[k].rms)
    return (axes["x"].rms, axes["y"].rms, axes["z"].rms,
            axes["x"].vdv, axes["y"].vdv, axes["z"].vdv,
            axes["x"].peak, axes["y"].peak, axes["z"].peak,
            dom, worst.crest,
            gate.moving_s, gate.idle_s, shocks.count,
            exposure_hours, aw8, vdv8)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--selftest", action="store_true",
                   help="print weighting filter response and exit")
    p.add_argument("--bus", type=int, default=I2C_BUS)
    p.add_argument("--addr", type=lambda s: int(s, 0), default=I2C_ADDR)
    p.add_argument("--int-gpio", type=int, default=INT_GPIO)
    p.add_argument("--odr", type=float, default=ODR_HZ, choices=list(RATE_CODES))
    p.add_argument("--range", type=int, default=G_RANGE, choices=list(RANGE_CODES))
    p.add_argument("--watermark", type=int, default=FIFO_WATERMARK)
    p.add_argument("--poll", action="store_true",
                   help="poll the FIFO instead of using the INT1 line")
    p.add_argument("--hw-gate", action="store_true",
                   help="also require the ADXL343 inactivity interrupt to "
                        "agree before gating off (needs INT2 wired)")
    p.add_argument("--no-gate", action="store_true",
                   help="disable motion gating; count parked time too")
    p.add_argument("--gate-gpio", type=int, default=27,
                   help="GPIO for INT2 when --hw-gate is used (default 27)")
    p.add_argument("--exposure-hours", type=float, default=8.0,
                   help="hours per day the operator is actually on the truck; "
                        "drives the A(8)/VDV(8) normalisation (default 8)")
    p.add_argument("--out-dir", default=OUT_DIR)
    g = p.add_argument_group("GPS")
    g.add_argument("--gps", action="store_true",
                   help="read the Ultimate GPS HAT and geotag events")
    g.add_argument("--gps-port", default="/dev/serial0")
    g.add_argument("--gps-rate-ms", type=int, default=200,
                   help="fix interval; 200 = 5 Hz (default), 1000 = 1 Hz")
    d = p.add_argument_group("database")
    d.add_argument("--db", action="store_true",
                   help="write sessions, fixes and events to MySQL")
    d.add_argument("--db-host", default=os.environ.get("DB_HOST", "localhost"))
    d.add_argument("--db-port", type=int,
                   default=int(os.environ.get("DB_PORT", "3306")))
    d.add_argument("--db-user", default=os.environ.get("DB_USER"))
    d.add_argument("--db-password", default=os.environ.get("DB_PASSWORD"))
    d.add_argument("--db-name", default=os.environ.get("DB_NAME", "wbv"))
    d.add_argument("--db-socket", default="/var/run/mysqld/mysqld.sock")
    d.add_argument("--db-defaults-file",
                   default=os.path.expanduser("~/.my.cnf"),
                   help="MySQL option file with credentials (preferred over "
                        "passing a password on the command line)")
    d.add_argument("--truck-id", type=int)
    d.add_argument("--logger-id", type=int)
    d.add_argument("--operator-ref")
    args = p.parse_args()

    if args.selftest:
        return selftest(args.odr)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
