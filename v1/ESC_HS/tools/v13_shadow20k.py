#!/usr/bin/env python3
"""Build 26101703: safely export one frozen 256x32B 20kHz V13 shadow trace.

Serial mode (close SerialReader first):
    python tools_v13_shadow20k.py --serial COM4 --arm --seconds 20 --out v13_20k.csv
Offline decode:
    python tools_v13_shadow20k.py --bin v13_capture.bin --out v13_20k.csv
The script stops after --seconds, and Ctrl+C cleanly writes any received bytes.
The firmware's *motor* is NOT stopped by closing the serial port.

Requires: pyserial only in serial mode. `numpy` is optional for detailed
statistics. Real current-estimation RMSE requires independent ground truth.
"""
import argparse
import csv
import json
import math
import struct
import time
from pathlib import Path

FRAME = struct.Struct('<I6h8H')
assert FRAME.size == 32
RATE_HZ = 20000
N = 256


def packets(buf):
    off = 0
    while True:
        ix = buf.find(b'\xaa\x55', off)
        if ix < 0 or ix + 7 > len(buf):
            break
        yield buf[ix+2], struct.unpack_from('<I', buf, ix+3)[0]
        off = ix + 7


def records_from_raw(data):
    parts = {}
    done = {}
    current = None
    for addr, val in packets(data):
        if addr == 0xD1:
            current = val if val < N else None
            if current is not None:
                parts.setdefault(current,{})
        elif 0xD2 <= addr <= 0xD9 and current is not None:
            slot = parts[current]
            slot[addr-0xD2] = val
            if len(slot) == 8:
                done[current] = FRAME.unpack(b''.join(struct.pack('<I',slot[j]) for j in range(8)))
                current = None
    return done


def rows(records):
    for idx, v in sorted(records.items()):
        tick, ru, rv, rw, fu, fv, fw, du, dv, dw, bus, a, ev, cy, flags = v
        raw = [i * 0.004 for i in (ru, rv, rw)]
        filt = [i * 0.004 for i in (fu, fv, fw)]
        mean = sum(raw)/3.0
        proj = [r - mean for r in raw]
        yield {'capture_index':idx,'isr_tick':tick,'time_s':idx/RATE_HZ,
            **{f'raw_{p}_A':raw[j] for j,p in enumerate('uvw')},
            **{f'v13_{p}_A':filt[j] for j,p in enumerate('uvw')},
            **{f'projection_{p}_A':proj[j] for j,p in enumerate('uvw')},
            **{f'duty_{p}':d/65535.0 for p,d in zip('uvw',(du,dv,dw))},
            'bus_v':bus/1000.0, 'alpha':a/65535.0,
            'voltage_event':ev/65535.0,'v13_cycles':cy,
            'pwm_valid':int(bool(flags & 1)),
            'invalid_input':int(bool(flags & 2)),
            'raw_observer20k_enabled':int(bool(flags & 4))}


def summarize(dataset):
    out = {'samples':len(dataset), 'expected_samples':N, 'sample_rate_hz':RATE_HZ,
           'has_independent_current_ground_truth':False,
           'true_current_estimation_RMSE':None}
    if not dataset:
        return out
    try:
        import numpy as np
    except ImportError:
        out['notice'] = 'Install numpy to calculate numeric comparisons (pip install numpy).'
        return out
    ticks = np.array([r['isr_tick'] for r in dataset], dtype='int64')
    out['tick_gaps'] = int(np.count_nonzero(np.diff(ticks) != 2))
    out['v13_cycles_median'] = float(np.median([r['v13_cycles'] for r in dataset]))
    out['v13_cycles_max'] = int(max(r['v13_cycles'] for r in dataset))
    out['v13_alpha_median'] = float(np.median([r['alpha'] for r in dataset]))
    out['v13_voltage_event_median'] = float(np.median([r['voltage_event'] for r in dataset]))
    a = {key:np.array([[r[f'{key}_{p}_A'] for p in 'uvw'] for r in dataset],dtype=float)
         for key in ('raw','projection','v13')}
    # Clarke alpha-beta (three currents), not dq rotated by a non-ground-truth angle.
    def clarke(x):
        return np.column_stack(((2*x[:,0]-x[:,1]-x[:,2])/3,
                               (x[:,1]-x[:,2])/math.sqrt(3)))
    signals = {key:clarke(arr) for key,arr in a.items()}
    for k, x in signals.items():
        out[k+'_clarke_step_rms_A'] = float(np.sqrt(np.mean(np.sum(np.diff(x,axis=0)**2,axis=1))))
        out[k+'_three_phase_sum_rms_A'] = float(np.sqrt(np.mean(np.sum(a[k],axis=1)**2)))
        out[k+'_vector_rms_A'] = float(np.sqrt(np.mean(np.sum(x*x,axis=1))))
        if len(x)>=16:
            detrend = x-np.linspace(x[0],x[-1],len(x))
            spectrum = np.fft.rfft(detrend,axis=0)
            f = np.fft.rfftfreq(len(x),1/RATE_HZ)
            band = (f>=2000) & (f<=RATE_HZ/2)
            # FFT-derived band energy is NOT ground-truth error variance.
            out[k+'_high_frequency_2k_to_10k_rms_proxy_A'] = float(
                np.sqrt(np.sum(np.abs(spectrum[band])**2)/len(x)**2*2))
    out['projection_to_v13_vector_difference_rms_A'] = float(np.sqrt(
        np.mean(np.sum((signals['projection']-signals['v13'])**2,axis=1))))
    out['warning'] = ('Noise proxies and output differences are NOT current-estimation RMSE. '
        'There is no independent current ground truth. 12.8ms capture may be too short '
        'for precise electrical-frequency phase-lag measurement at low RPM.')
    return out


def main():
    ap=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    source=ap.add_mutually_exclusive_group(required=True)
    source.add_argument('--serial',help='serial port (COM4 on Windows)')
    source.add_argument('--bin',help='previously recorded UART raw binary')
    ap.add_argument('--arm',action='store_true',help='send 15 07 BC 00 00 to trigger a new capture')
    ap.add_argument('--seconds',type=float,default=20.0,help='serial acquisition duration; 20s recommended')
    ap.add_argument('--baud',type=int,default=460800)
    ap.add_argument('--out',default='v13_shadow20k.csv')
    args=ap.parse_args()
    if args.serial:
        import serial
        data=bytearray()
        try:
            with serial.Serial(args.serial,args.baud,timeout=0.2) as s:
                s.reset_input_buffer()
                if args.arm:s.write(bytes.fromhex('15 07 BC 00 00'))
                end=time.monotonic()+args.seconds
                while time.monotonic()<end:
                    data.extend(s.read(4096))
        except KeyboardInterrupt:
            print('Stopped with Ctrl+C. Saving any captured bytes.')
        data=bytes(data)
        Path(args.out).with_suffix('.bin').write_bytes(data)
    else:
        data=Path(args.bin).read_bytes()
    got=records_from_raw(data)
    table=list(rows(got))
    if table:
        with open(args.out,'w',newline='',encoding='utf-8') as f:
            writer=csv.DictWriter(f,fieldnames=list(table[0]))
            writer.writeheader();writer.writerows(table)
    report=summarize(table)
    Path(args.out).with_suffix('.summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(f'Capture {len(got)}/{N} samples; CSV {args.out}; report {Path(args.out).with_suffix(".summary.json")}')
    print(json.dumps(report,indent=2))
    if len(got)<N:
        print('Incomplete: check firmware build 26101703, mode=1, capture_state=2; '
              'close SerialReader; use --arm and --seconds 20.')

if __name__=='__main__':main()
