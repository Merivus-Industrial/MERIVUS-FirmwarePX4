#!/usr/bin/env python3
"""Offline nominal torque/IMU coherence across delay candidates."""
import argparse
import csv
from pathlib import Path
import numpy as np
from pyulog import ULog

def filtered(values, times):
    low = values[0].copy()
    mean = low.copy()
    output = np.zeros_like(values)
    for k in range(1, len(values)):
        dt = np.clip(times[k]-times[k-1], .001, .1)
        low += dt/(.2+dt) * (values[k]-low)
        output[k] = low-mean
        mean += dt/(2+dt)*(low-mean)
    return output

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('ulog')
    parser.add_argument('--output', required=True)
    parser.add_argument('--gyroscopic', action='store_true')
    parser.add_argument('--iris-model', action='store_true')
    parser.add_argument('--start', type=float, default=45)
    parser.add_argument('--end', type=float, default=125)
    args = parser.parse_args()
    data = {d.name:d.data for d in ULog(args.ulog).data_list if d.multi_id == 0}
    model, motors, imu, matrix = [data[k] for k in ('ftc_model_status','actuator_motors','vehicle_angular_velocity','ftc_effectiveness_matrix')]
    time = model['timestamp']*1e-6
    response_time = model['timestamp_sample']*1e-6
    selected = (time >= args.start) & (time <= args.end)
    B = np.array([[matrix['effectiveness[{}]'.format(a*16+i)][-1] for i in range(4)] for a in range(3)])
    if args.iris_model:
        B[0] *= np.array([.22/.245,.2/.1875,.22/.245,.2/.1875])
        B[1] *= .13/.1515
        B[2] *= .06/.05
    imu_i = np.clip(np.searchsorted(imu['timestamp']*1e-6,time,side='right')-1,0,len(imu['timestamp'])-1)
    measured = np.array([imu['xyz_derivative[{}]'.format(a)][imu_i] for a in range(3)]).T
    if args.gyroscopic:
        omega = np.array([imu['xyz[{}]'.format(a)][imu_i] for a in range(3)]).T
        inertia = np.array([.02,.02,.04])
        measured += np.cross(omega,omega*inertia)/inertia
    y = filtered(measured,time)
    with Path(args.output).open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(('delay_ms','axis','gain','coherence_r2','rmse','response_rms','nominal_torque_rms','condition'))
        for delay in range(0,101,10):
            indices = np.clip(np.searchsorted(motors['timestamp']*1e-6,response_time-delay*.001,side='right')-1,0,len(motors['timestamp'])-1)
            controls = np.nan_to_num(np.array([motors['control[{}]'.format(i)][indices] for i in range(4)]).T)
            if args.iris_model:
                controls = (controls*controls + .2*controls)/1.2
            du = filtered(controls,time)
            phi = du[:,None,:]*B[None,:,:]
            info = np.einsum('tai,taj->ij',phi[selected],phi[selected])
            nominal = du@B.T
            for axis in range(3):
                x,z = nominal[selected,axis],y[selected,axis]
                gain = x@z/max(x@x,1e-12)
                corr = (x@z)**2/max((x@x)*(z@z),1e-12)
                row=(delay,axis,gain,corr,np.sqrt(np.mean((gain*x-z)**2)),np.sqrt(np.mean(z*z)),np.sqrt(np.mean(x*x)),np.linalg.cond(info))
                writer.writerow(row)
                if delay in (0,40,80): print(row)
    status = data['vehicle_status']['timestamp']*1e-6
    print('vehicle_status interval percentiles',np.percentile(np.diff(status),[50,95,99,100]))
