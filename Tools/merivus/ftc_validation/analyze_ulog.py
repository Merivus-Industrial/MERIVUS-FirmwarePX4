#!/usr/bin/env python3
"""Numerical FTC evidence from recorded uORB, never from commanded scenario labels."""
import argparse
import json
from pathlib import Path
import numpy as np
from pyulog import ULog

REQUIRED = ('motor_health_status', 'ftc_model_status', 'ftc_effectiveness_matrix',
            'ftc_allocation_shadow', 'ftc_control_authority', 'ftc_extreme_state',
            'ftc_recovery_status', 'ftc_system_status', 'ftc_simulation_status',
            'ftc_allocation_status', 'ftc_arbitration_status')


def statistics(values):
    array = np.asarray(values, dtype=float)
    finite = array[np.isfinite(array)]
    return dict(samples=len(array), finite=len(finite), minimum=float(np.min(finite)) if len(finite) else None,
                maximum=float(np.max(finite)) if len(finite) else None,
                mean=float(np.mean(finite)) if len(finite) else None,
                p50=float(np.percentile(finite, 50)) if len(finite) else None,
                p95=float(np.percentile(finite, 95)) if len(finite) else None)


def analyze(path, start=0, end=float('inf')):
    log = ULog(str(path))
    topics = {d.name: d.data for d in log.data_list if d.multi_id == 0}
    result = dict(ulog=str(path), missing_topics=[name for name in REQUIRED if name not in topics],
                  window=[start, end], topics={})
    for name in REQUIRED:
        if name not in topics:
            continue
        data = topics[name]
        selected = (data['timestamp']*1e-6 >= start) & (data['timestamp']*1e-6 <= end)
        times = data['timestamp'][selected]*1e-6
        info = dict(count=int(np.sum(selected)), fields={})
        if len(times) > 1:
            info.update(hz=float((len(times)-1)/(times[-1]-times[0])), maximum_interval=float(np.max(np.diff(times))))
        for field, values in data.items():
            if field.startswith('timestamp'):
                continue
            info['fields'][field] = statistics(values[selected])
        result['topics'][name] = info
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('ulog')
    parser.add_argument('--start', type=float, default=0)
    parser.add_argument('--end', type=float, default=float('inf'))
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    result = analyze(args.ulog, args.start, args.end)
    Path(args.output).write_text(json.dumps(result, indent=2, allow_nan=True))
    print('missing_topics:', result['missing_topics'])
    for name, topic in result['topics'].items():
        print(name, 'count', topic['count'], 'Hz', round(topic.get('hz', 0), 2))
        for field, values in topic['fields'].items():
            if field in ('excitation','condition_number','valid','baseline_learned','timing_aligned','saturated',
                         'current_observable','authority_limited','update_count','impact_detected','loc_state',
                         'active','degraded_mask','failed_mask','state'):
                print(' ', field, values)
