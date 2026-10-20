"""Read-only diagnostics for the 205819 run and the preceding baseline."""
import json
import math
import re
import sqlite3
import struct
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "bed13_full_20261009_012734"))
from analyze_runs import CdrReader

K = math.pi * 0.152 / (60 * 19)  # rotor rpm -> wheel surface m/s
NAMES = {1: "nurse", 2: "scan_order", 3: "bed1_nav", 4: "bed1_scan", 5: "bed1_dispense",
         6: "bed3_nav", 7: "bed3_scan", 8: "bed3_dispense", 9: "home_nav", 10: "done",
         12: "bed1_dock", 13: "bed3_dock", 14: "wait", 15: "bed1_wait"}


def body(rpm):
    return np.asarray([(rpm[0] - rpm[2]) * .5 * K, (rpm[1] - rpm[3]) * .5 * K])


def load(root):
    rows = [json.loads(s) for s in (root / "timeline.jsonl").open(encoding="utf8")]
    base = rows[0]["wall_ns"] / 1e9
    topics = defaultdict(list)
    transitions = []
    for r in rows:
        r["t"] = r["wall_ns"] / 1e9 - base
        if r["kind"] == "task_state":
            transitions.append((r["t"], r["current"]))
        if "topic" in r:
            if r["kind"] == "string":
                try:
                    r["d"] = json.loads(r["data"])
                except (ValueError, TypeError):
                    r["d"] = {}
            topics[r["topic"]].append(r)
    intervals = [(a[0], b[0], a[1]) for a, b in zip(transitions, transitions[1:])]
    runs = []
    for start, state in transitions:
        if state in (1, 3) and (not runs or start > runs[-1][1]):
            end = next((t for t, s in transitions if t > start and s == 10), None)
            if end:
                runs.append((start, end))
    con = sqlite3.connect(f"file:{(root/'rosbag/rosbag_0.db3').as_posix()}?mode=ro", uri=True)
    poses = []
    for t, raw in con.execute("SELECT m.timestamp,m.data FROM messages m JOIN topics t ON t.id=m.topic_id WHERE t.name='/medical_nav/robot_pose' ORDER BY m.timestamp"):
        reader = CdrReader(raw)
        stamp, frame = reader.header()
        p = [reader.f64() for _ in range(7)]
        yaw = math.atan2(2*(p[6]*p[5]+p[3]*p[4]), 1-2*(p[4]**2+p[5]**2))
        poses.append((t / 1e9-base, p[0], p[1], yaw, stamp-base))
    path_topics = {}
    for topic in ('/plan', '/plan_smoothed', '/transformed_global_plan'):
        decoded = []
        for t, raw in con.execute(
                'SELECT m.timestamp,m.data FROM messages m JOIN topics t '
                'ON t.id=m.topic_id WHERE t.name=? ORDER BY m.timestamp', (topic,)):
            stamp, frame, points = CdrReader(raw).path()
            decoded.append((t/1e9-base, stamp-base, frame, points))
        path_topics[topic] = decoded
    paths = [(t, points) for t, _, _, points in path_topics['/plan_smoothed']]
    con.close()
    events = []
    for line in (root / "journal.log").read_text(encoding="utf8", errors="replace").splitlines():
        match = re.search(r"\[(\d{10}\.\d+)\].*?\]: (.*)$", line)
        if match:
            t = float(match[1])-base
            if rows[0]["t"] <= t <= rows[-1]["t"]:
                events.append((t, match[2]))
    return dict(root=root, base=base, topics=topics, poses=np.asarray(poses),
                paths=paths, path_topics=path_topics, intervals=intervals,
                runs=runs, events=events)


def selected(data, topic, start, end):
    return [r for r in data["topics"][topic] if start <= r["t"] < end]


def event_counts(data, start, end):
    events = [s for t, s in data["events"] if start <= t < end]
    return {"optimizer_fail": sum("Optimizer fail" in s for s in events),
            "smoother_collision": sum("Smoothed path leads" in s for s in events),
            "planner_fail": sum("failed to generate a valid path" in s for s in events),
            "stop_entries": sum("Robot to stop" in s for s in events),
            "approach_entries": sum("Robot to approach" in s for s in events),
            "missed_loops": sum("control loop missed" in s.lower() for s in events)}


def quant(values):
    return [round(float(x), 4) for x in np.quantile(values, [.5, .9, .99])] if len(values) else []


def nearest_pose(data, t):
    p = data["poses"]
    return p[np.argmin(abs(p[:, 0]-t))]


def report(data):
    print("DATA", data["root"].name, "base", data["base"], "runs", data["runs"])
    print("CORNER", Counter((r["d"].get("enabled"), r["d"].get("limited")) for r in data["topics"]["/medical_nav/corner_speed_status"]))
    for i, (start, end) in enumerate(data["runs"], 1):
        print("RUN", i, round(end-start, 3), event_counts(data, start, end))
        for a,b,s in data["intervals"]:
            if a < start or b > end+.01:
                continue
            odom = selected(data, "/medical_nav/wheel_odom", a, b)
            cmd = selected(data, "/cmd_vel_nav", a, b)
            diag = selected(data, "/medical_nav/wheel_diagnostics", a, b)
            p = data["poses"][(data["poses"][:, 0]>=a)&(data["poses"][:,0]<b)]
            entry = dict(state=NAMES.get(s,s), start=round(a,3), duration=round(b-a,3),
                         wheel_peak=round(max((r["speed"] for r in odom),default=0),3),
                         cmd_peak=round(max((r["speed"] for r in cmd),default=0),3),
                         events=event_counts(data,a,b),
                         saturation=sum(bool(r["d"].get("current_saturation_mask")) for r in diag),
                         offline=sum(r["d"].get("online_mask")!=15 for r in diag))
            if len(p):
                entry.update(x_range=[round(float(p[:,1].min()),3),round(float(p[:,1].max()),3)],
                             y_range=[round(float(p[:,2].min()),3),round(float(p[:,2].max()),3)],
                             exit_xy=[round(float(z),3) for z in p[-1,1:3]],
                             distance=round(float(np.linalg.norm(np.diff(p[:,1:3],axis=0),axis=1).sum()),3))
                target = {1:(1.7,0),3:(5.4,2.2),6:(5.4,-2.2),9:(0,0),12:(5.4,2.2),13:(5.4,-2.2)}.get(s)
                if target:
                    entry["goal_plane_overshoot_xy"]=[round(max(0,float(p[:,1].max()-target[0]) if s!=9 else -float(p[:,1].min())),3),
                                                      round(max(0,float(p[:,2].max()-target[1]) if s in (1,3,12,9) else float(target[1]-p[:,2].min())),3)]
            if s in (1,3,6,9):
                bed = selected(data,"/medical_nav/bed_approach_status",a,b)
                entry["bed_limited_counts"]=[sum(bool(r["d"].get(k)) for r in bed) for k in ("x_limited","side_limited")]
                br = selected(data,"/medical_nav/bridge_cmd_debug",a,b)
                entry["bridge_limited_ratio"] = round(sum(bool(r["d"].get("gate_release",{}).get("limiting")) for r in br)/max(1,len(br)),3)
            print(json.dumps(entry,ensure_ascii=False))
    diag = data["topics"]["/medical_nav/wheel_diagnostics"]
    print("DIAG",len(diag),"online",Counter(r["d"].get("online_mask") for r in diag),"saturated",sum(bool(r["d"].get("current_saturation_mask")) for r in diag))
    print("EVENTS",event_counts(data,0,1e9))


def motor_rows(data):
    rows = data["topics"]["/medical_nav/wheel_diagnostics"]
    result=[]
    epochs=[]
    for r in rows:
        cs=r["d"]["stamp_cs"]
        if not epochs or cs < epochs[-1][-1]["d"]["stamp_cs"]:
            epochs.append([])
        epochs[-1].append(r)
    for epoch in epochs:
        offset=min(r["t"]-r["d"]["stamp_cs"]*.01 for r in epoch)
        for r in epoch:
            d=r["d"]
            corrected=d["stamp_cs"]*.01+offset
            result.append((corrected,*body(d["target_rpm"]),*body(d["measured_rpm"]),r["t"],d["stamp_cs"]))
    return np.asarray(result)


def lag_report(data):
    motor=motor_rows(data)
    print("MOTOR_TRANSPORT_ESTIMATE",quant(motor[:,5]-motor[:,0]),"max",round(float(max(motor[:,5]-motor[:,0])),4))
    for i,(start,end) in enumerate(data["runs"],1):
        for a,b,s in data["intervals"]:
            if s not in (1,3,6,9,12,13) or a<start or b>end+.01:
                continue
            m=motor[(motor[:,0]>=a)&(motor[:,0]<b)]
            for axis in range(2):
                lags=np.arange(-.15,.651,.01)
                errors=[]
                for lag in lags:
                    q=m[(m[:,0]>=a+.65)&(m[:,0]<b-.65)]
                    if len(q)<5: errors.append(float('inf')); continue
                    pred=np.interp(q[:,0]-lag,m[:,0],m[:,1+axis])
                    errors.append(float(np.mean((pred-q[:,3+axis])**2)))
                best=int(np.argmin(errors))
                error=m[:,1+axis]-m[:,3+axis]
                print('LAG',i,NAMES[s],'xy'[axis],round(float(lags[best]),3),'rmse',round(math.sqrt(errors[best]),3),'zero_lag_rmse',round(float(np.sqrt(np.mean(error**2))),3))
    return motor


def plot(data,motor):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(2,2,figsize=(13,10),constrained_layout=True)
    colors={1:'gray',3:'tab:blue',6:'tab:orange',9:'tab:green',12:'tab:red',13:'tab:purple'}
    for ax,(start,end),idx in zip(axs.flat,data['runs'],range(1,5)):
        for a,b,s in data['intervals']:
            if a<start or b>end+.01 or s not in colors:continue
            p=data['poses'][(data['poses'][:,0]>=a)&(data['poses'][:,0]<b)]
            ax.plot(p[:,1],p[:,2],color=colors[s],label=NAMES[s],lw=1.7)
            selected_paths=[p for t,p in data['paths'] if a<t<b]
            for path in selected_paths[::max(1,len(selected_paths)//4)]:
                if path: ax.plot(*zip(*path),color=colors[s],alpha=.15,lw=.7)
        ax.scatter([0,1.7,5.4,5.4],[0,0,2.2,-2.2],marker='x',color='black')
        ax.set(title=f'Run {idx}: {end-start:.2f} s',xlabel='map X (m)',ylabel='map Y (m)')
        ax.axis('equal');ax.grid(alpha=.25);ax.legend(fontsize=8)
    fig.savefig(data['root']/'overshoot_xy.png',dpi=160);plt.close(fig)
    fig,axs=plt.subplots(4,2,figsize=(15,13),constrained_layout=True)
    for idx,(start,end) in enumerate(data['runs']):
        for axis in range(2):
            ax=axs[idx,axis]
            m=motor[(motor[:,0]>=start)&(motor[:,0]<end)]
            ax.plot(m[:,0]-start,m[:,1+axis],label='MCU post-ramp target',lw=1.1)
            ax.plot(m[:,0]-start,m[:,3+axis],label='measured wheel speed',lw=1.1)
            br=selected(data,'/medical_nav/bridge_cmd_debug',start,end)
            tt=[r['t']-start for r in br];vv=[r['d'].get('sent_cmd',{}).get(('vx','vy')[axis],0) for r in br]
            ax.plot(tt,vv,label='NUC sent (NAV only)',alpha=.55,lw=.8)
            for a,b,s in data['intervals']:
                if a>=start and b<=end+.01 and s in (12,13):ax.axvspan(a-start,b-start,alpha=.15,color='red')
            ax.set(title=f'Run {idx+1}: body {"XY"[axis]}',xlabel='seconds since launch',ylabel='m/s');ax.grid(alpha=.25)
    axs[0,0].legend(fontsize=8)
    fig.savefig(data['root']/'overshoot_speed.png',dpi=160);plt.close(fig)


def command_array(data, topic, key=None):
    rows = data['topics'][topic]
    if key:
        return np.asarray([(r['t'], r['d'][key]['vx'], r['d'][key]['vy']) for r in rows])
    return np.asarray([(r['t'], r['vx'], r['vy']) for r in rows])


def path_distance(point, path):
    path = np.asarray(path)
    if not len(path):
        return float('nan')
    if len(path) == 1:
        return float(np.linalg.norm(path[0]-point))
    delta = np.diff(path, axis=0)
    length2 = np.sum(delta**2, axis=1)
    fraction = np.clip(np.sum((point-path[:-1])*delta, axis=1) /
                       np.maximum(length2, 1e-12), 0, 1)
    return float(np.min(np.linalg.norm(path[:-1]+fraction[:, None]*delta-point, axis=1)))


def turn_report(data, make_plot=False):
    """Compare topic receipt times; MCU times have an estimated epoch offset.

    Path deviation is relative to the frozen pre-reversal controller path,
    so replanning cannot hide an existing excursion. Recorded static TF
    and bridge source confirm identity map->odom for this dataset.
    """
    motor = motor_rows(data)
    layers = {
        'MPPI': command_array(data, '/cmd_vel_nav'),
        'velocity_smoother': command_array(data, '/cmd_vel'),
        'bed_home_limiter': command_array(data, '/cmd_vel_home_limited'),
        'collision_monitor': command_array(data, '/cmd_vel_safe'),
        'bridge_sent': command_array(data, '/medical_nav/bridge_cmd_debug', 'sent_cmd'),
        'MCU_target_est': motor[:, :3],
        'measured_est': motor[:, [0, 3, 4]],
    }
    examples = [('Run 3 Bed1', 119.2, 120.9, 1),
                ('Run 4 Bed3', 189.6, 191.3, -1)]
    if make_plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    for idx, (name, start, end, old_sign) in enumerate(examples):
        raw = layers['MPPI']
        changed = raw[(raw[:, 0]>=start)&(raw[:, 0]<end)&(old_sign*raw[:, 2]<0)]
        reversal = float(changed[0, 0])
        reference_t, _, frame, reference = max(
            (r for r in data['path_topics']['/transformed_global_plan']
             if r[0]<reversal), key=lambda r: r[0])
        if frame not in ('map', 'odom'):
            raise ValueError('Controller path requires a frame transform: '+frame)
        poses = data['poses'][(data['poses'][:, 0]>=reversal)&(data['poses'][:, 0]<end)]
        peak = poses[np.argmax(old_sign*poses[:, 2])]
        entry = nearest_pose(data, reversal)
        print('TURN', name, 'reversal', round(reversal, 4),
              'local_time', __import__('datetime').datetime.fromtimestamp(data['base']+reversal).isoformat(),
              'pose_at_reversal', np.round(entry[1:3], 4).tolist(),
              'peak_time', round(float(peak[0]), 4), 'peak_pose', np.round(peak[1:3], 4).tolist(),
              'continued_map_y', round(float(old_sign*(peak[2]-entry[2])), 4),
              'reference_time', round(reference_t, 4), 'reference_frame', frame,
              'peak_distance_to_frozen_plan', round(path_distance(peak[1:3], reference), 4))
        for topic, records in data['path_topics'].items():
            frozen = max((r for r in records if r[0]<reversal), key=lambda r:r[0])
            print('  PATH', topic, 'time', round(frozen[0], 4),
                  'frame', frozen[2], 'points', len(frozen[3]),
                  'entry_distance', round(path_distance(entry[1:3], frozen[3]), 4),
                  'peak_distance', round(path_distance(peak[1:3], frozen[3]), 4))
        for label, array in layers.items():
            switched = array[(array[:, 0]>=reversal)&(array[:, 0]<end)&(old_sign*array[:, 2]<0)]
            crossing = float(switched[0, 0]) if len(switched) else None
            grid = np.arange(reversal, float(peak[0]), .005)
            indices = np.searchsorted(array[:, 0], grid, side='right')-1
            residual_area = float(np.sum(np.maximum(old_sign*array[indices, 2], 0))*.005)
            print('  LAYER', label, 'opposite_time', round(crossing, 4) if crossing else None,
                  'delay', round(crossing-reversal, 4) if crossing else None,
                  'old_direction_integral_body_y', round(residual_area, 4))
            if make_plot and label != 'bed_home_limiter':
                window = array[(array[:, 0]>=start)&(array[:, 0]<end)]
                axs[idx, 0].plot(window[:, 0]-reversal, window[:, 2], label=label, lw=1.1)
        print('  EVENTS', [(round(t-reversal, 4), s) for t,s in data['events']
                          if start<t<end and any(k in s for k in ('Optimizer fail', 'Robot to approach', 'route is blocked'))])
        if make_plot:
            axs[idx, 0].axvline(0, color='black', ls='--', lw=.8)
            axs[idx, 0].axhline(0, color='black', lw=.7)
            axs[idx, 0].set(title=name+' body Y speed', xlabel='seconds from MPPI reversal', ylabel='m/s')
            axs[idx, 0].grid(alpha=.25)
            path = np.asarray(reference)
            poses = data['poses'][(data['poses'][:, 0]>=start)&(data['poses'][:, 0]<end)]
            axs[idx, 1].plot(path[:, 0], path[:, 1], label='controller path before reversal', color='gray')
            axs[idx, 1].plot(poses[:, 1], poses[:, 2], label='recorded pose', color='tab:red')
            axs[idx, 1].scatter(*entry[1:3], marker='o', label='MPPI reversal')
            axs[idx, 1].scatter(*peak[1:3], marker='x', label='furthest old-direction pose')
            axs[idx, 1].set(xlim=(poses[:, 1].min()-.25, poses[:, 1].max()+.5),
                           ylim=(poses[:, 2].min()-.25, poses[:, 2].max()+.25),
                           title=name+' path excursion', xlabel='map X (m)', ylabel='map Y (m)')
            axs[idx, 1].set_aspect('equal', adjustable='box')
            axs[idx, 1].grid(alpha=.25)
            axs[idx, 1].legend(fontsize=8)
    if make_plot:
        axs[0, 0].legend(fontsize=8)
        fig.savefig(data['root']/'overshoot_turn_layers.png', dpi=170)
        plt.close(fig)


def feedback_report(data):
    """Receipt-time velocity comparison, not a calibrated latency estimate."""
    raw = command_array(data, '/medical_nav/wheel_odom')
    fused = command_array(data, '/odom')
    for i, (start, end) in enumerate(data['runs'], 1):
        segments = [(a, b) for a, b, state in data['intervals']
                    if start <= a and b <= end+.01 and state in (1, 3, 6, 9)]
        for axis in range(2):
            lags = np.arange(-.10, .301, .01)
            errors = []
            samples = []
            for a, b in segments:
                samples.extend(fused[(fused[:, 0]>=a+.4)&(fused[:, 0]<b-.4)])
            samples = np.asarray(samples)
            if not len(samples):
                continue
            for lag in lags:
                prediction = np.interp(samples[:, 0]-lag, raw[:, 0], raw[:, 1+axis])
                errors.append(float(np.mean((prediction-samples[:, 1+axis])**2)))
            best = int(np.argmin(errors))
            print('FEEDBACK_FIT', i, 'xy'[axis], 'lag', round(float(lags[best]), 3),
                  'rmse', round(math.sqrt(errors[best]), 3))
    for at in (190.084, 190.184, 190.384, 190.584, 190.784):
        values = []
        for label, array in (('wheel', raw), ('fused', fused)):
            row = array[np.argmin(abs(array[:, 0]-at))]
            values.append((label, round(float(row[0]), 4), round(float(row[2]), 4)))
        print('FEEDBACK_TURN_SAMPLE', at, values)
    for row in selected(data, '/medical_nav/bridge_status', 189.6, 190.9):
        status = row['d']
        stm = status.get('stm32', {})
        offset = status.get('ops_offset_mm', {})
        if 'x_mm' in stm and 'y_mm' in stm:
            corrected = ((stm['y_mm']+offset.get('y', 0))/1000,
                         -(stm['x_mm']+offset.get('x', 0))/1000)
            print('SPARSE_OPS', round(row['t'], 4), 'corrected_map_xy', corrected,
                  'odom_source', status.get('odom_source'),
                  'yaw_source', status.get('yaw_source'))


if __name__=='__main__':
    root=Path(sys.argv[1]).resolve() if len(sys.argv)>1 else Path(__file__).resolve().parent
    data=load(root)
    if '--turns' in sys.argv:
        turn_report(data, '--plot' in sys.argv)
    elif '--feedback' in sys.argv:
        feedback_report(data)
    else:
        report(data);motor=lag_report(data)
        if '--plot' in sys.argv:plot(data,motor)
