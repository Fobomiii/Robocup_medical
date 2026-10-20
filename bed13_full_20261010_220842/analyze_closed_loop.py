"""Read-only comparison of recorded commands, feedback and path excursions.

Times are NUC recorder receipt times unless explicitly marked MCU-estimated.
Reports and figures are diagnostics, not changes to robot configuration.
"""
import json
import math
import re
import sqlite3
import sys
from collections import Counter
from datetime import datetime
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'bed13_full_20261010_205819'))
from analyze_overshoot import (CdrReader, NAMES, command_array, event_counts,
                               load, motor_rows, path_distance)

NAV = (1, 3, 6, 9)
LAYERS = {'MPPI': '/cmd_vel_nav', 'smoother': '/cmd_vel',
          'bed_home': '/cmd_vel_home_limited', 'safe': '/cmd_vel_safe',
          'wheel': '/medical_nav/wheel_odom', 'odom': '/odom'}


def sample(array, times):
    idx = np.clip(np.searchsorted(array[:, 0], times, side='right') - 1,
                  0, len(array) - 1)
    return array[idx, 1:3]


def local(data, offset):
    return datetime.fromtimestamp(data['base'] + offset).isoformat(timespec='milliseconds')


def segments(data, run):
    return [(a, b, s) for a, b, s in data['intervals']
            if a >= run[0] and b <= run[1] + .01]


def summary(data):
    arrays = {name: command_array(data, topic) for name, topic in LAYERS.items()}
    result = []
    for run_id, run in enumerate(data['runs'], 1):
        item = {'run': run_id, 'start': local(data, run[0]), 'end': local(data, run[1]),
                'duration': run[1]-run[0], 'events': event_counts(data, *run), 'stages': []}
        for a, b, state in segments(data, run):
            grid = np.arange(a, b, .01)
            stage = {'state': state, 'name': NAMES[state], 'start_t': a,
                     'duration': b-a, 'speeds': {}}
            for name, array in arrays.items():
                speed = np.linalg.norm(sample(array, grid), axis=1)
                stage['speeds'][name] = {'mean': float(speed.mean()),
                    'median': float(np.median(speed)), 'peak': float(speed.max()),
                    'below_0.1_s': float(np.sum(speed < .1)*.01),
                    'above_1.5_s': float(np.sum(speed > 1.5)*.01)}
            if state in NAV:
                stage['events'] = event_counts(data, a, b)
                home = sample(arrays['bed_home'], grid)
                smoothed = sample(arrays['smoother'], grid)
                safe = sample(arrays['safe'], grid)
                stage['bed_home_reduction_s'] = float(np.sum(
                    np.linalg.norm(smoothed, axis=1)-np.linalg.norm(home, axis=1) > .05)*.01)
                stage['collision_reduction_s'] = float(np.sum(
                    np.linalg.norm(home, axis=1)-np.linalg.norm(safe, axis=1) > .05)*.01)
                stage['bed_flags'] = dict(Counter(
                    (str(r['d'].get('x_limited')) + '/' + str(r['d'].get('side_limited')))
                    for r in data['topics']['/medical_nav/bed_approach_status'] if a<=r['t']<b))
                p=data['poses'][(data['poses'][:, 0]>=a)&(data['poses'][:, 0]<b)]
                stage['pose_length'] = float(np.linalg.norm(np.diff(p[:, 1:3], axis=0), axis=1).sum())
                stage['entry_exit'] = [p[0, 1:3].tolist(), p[-1, 1:3].tolist()]
            item['stages'].append(stage)
        statuses=[r['d'] for r in data['topics']['/medical_nav/bridge_status']
                  if run[0]<=r['t']<run[1]]
        item['odom_sources']=dict(Counter(r.get('odom_source') for r in statuses))
        item['twist_sources']=dict(Counter(r.get('twist_source') for r in statuses))
        diag=[r['d'] for r in data['topics']['/medical_nav/wheel_diagnostics']
              if run[0]<=r['t']<run[1]]
        item['online']=dict(Counter(str(r.get('online_mask')) for r in diag))
        item['saturation_frames']=sum(bool(r.get('current_saturation_mask')) for r in diag)
        result.append(item)
    return result


def odom_headers(data):
    con=sqlite3.connect(f"file:{(data['root']/'rosbag/rosbag_0.db3').as_posix()}?mode=ro", uri=True)
    decoded=[]
    for t, raw in con.execute('SELECT m.timestamp,m.data FROM messages m JOIN topics t '
                             'ON t.id=m.topic_id WHERE t.name=? ORDER BY m.timestamp', ('/odom',)):
        r=CdrReader(raw)
        stamp, _=r.header()
        r.string()
        for _ in range(43): r.f64()
        vx, vy=r.f64(), r.f64()
        decoded.append((t/1e9-data['base'], stamp-data['base'], vx, vy))
    con.close()
    return np.asarray(decoded)


def averaged_odom(odom, duration):
    result=[]
    front=0
    for i, row in enumerate(odom):
        while front<i and row[1]-odom[front, 1] > duration:
            front+=1
        result.append((row[0], *odom[front:i+1, 2:4].mean(axis=0)))
    return np.asarray(result)


def clamp_velocity(current, target):
    target=np.clip(target, -3.0, 3.0)
    accel=(np.abs(target)>=np.abs(current))&(target*current>=0)
    step=np.where(accel, np.asarray([2.5, 2.2])/20,
                  np.asarray([2.0, 2.6])/20)
    return current+np.clip(target-current, -step, step)


def fits(data):
    raw=command_array(data, '/cmd_vel_nav')
    smooth=command_array(data, '/cmd_vel')
    odom=odom_headers(data)
    nav=[(a,b) for run in data['runs'] for a,b,s in segments(data,run) if s in NAV]
    selected=np.asarray([i for i,t in enumerate(smooth[:,0])
                         if i>0 and any(a+.3<t<b-.1 for a,b in nav)])
    times=smooth[selected, 0]
    actual=smooth[selected, 1:3]
    candidates=[]
    for duration in (0, .02, .05, .1, .15):
        average=averaged_odom(odom, duration)
        for shift in (-.03,-.02,-.01,0,.01,.02):
            # Recorder receipt order is not subscriber processing order.
            # Positive shift admits a slightly later recorder callback.
            cur=sample(average, times+shift)
            target=sample(raw, times)
            prediction=clamp_velocity(cur,target)
            error=actual-prediction
            candidates.append({'duration':duration,'receipt_shift':shift,
                'rmse':float(np.sqrt(np.mean(error**2))),
                'median_abs_error':float(np.median(np.abs(error))),
                'p90_abs_error':float(np.quantile(np.abs(error),.9))})
    candidates.sort(key=lambda r:r['rmse'])
    # One-step fit against recorded previous output; not a race-time forecast.
    prediction=clamp_velocity(smooth[selected-1,1:3],sample(raw,times))
    error=actual-prediction
    open_fit={'rmse':float(np.sqrt(np.mean(error**2))),
              'median_abs_error':float(np.median(np.abs(error))),
              'p90_abs_error':float(np.quantile(np.abs(error),.9))}
    nurse=next((a,b) for a,b,s in segments(data,data['runs'][-1]) if s==1)
    ramp=[]
    for t in np.arange(nurse[0]+.5,nurse[0]+2.01,.25):
        values={'t_from_start':round(float(t-nurse[0]),2)}
        for name,topic in LAYERS.items():
            values[name]=np.round(sample(command_array(data,topic),[t])[0],4).tolist()
        ramp.append(values)
    return {'closed_loop_candidates':candidates[:8], 'open_loop_one_step':open_fit,
            'smoother_tick_median':float(np.median(np.diff(smooth[selected,0]))),
            'odom_tick_median':float(np.median(np.diff(odom[:,0]))), 'nurse_ramp':ramp}


def turn_candidates(data):
    raw=command_array(data, '/cmd_vel_nav')
    motor=motor_rows(data)
    wheel=motor[:,[0,3,4]]
    result=[]
    for run_id,run in enumerate(data['runs'],1):
        for a,b,s in segments(data,run):
            if s not in (3,6,9): continue
            for axis in (1,2):
                q=raw[(raw[:,0]>a+.6)&(raw[:,0]<b-.4)]
                last=-10
                for i in range(1,len(q)):
                    # Ignore sign chatter with almost no established old-axis motion.
                    if q[i-1,axis]*q[i,axis]>=0: continue
                    t=q[i,0]
                    sign=np.sign(q[i-1,axis])
                    cur=sample(wheel,[t])[0,axis-1]
                    if sign*cur<.2 or t-last<.35: continue
                    prior=sample(wheel,[t-.2])[0,axis-1]
                    if sign*prior<.2: continue
                    p=data['poses'][(data['poses'][:,0]>=t)&(data['poses'][:,0]<min(t+1.6,b))]
                    entry=data['poses'][np.argmin(abs(data['poses'][:,0]-t))]
                    peak=p[np.argmax(sign*p[:,axis])]
                    reference=max((r for r in data['path_topics']['/transformed_global_plan']
                                   if r[0]<t),key=lambda r:r[0])
                    result.append({'run':run_id,'state':NAMES[s],'axis':'xy'[axis-1],
                        't':float(t),'local':local(data,t),'old_sign':int(sign),
                        'wheel_at_reversal':float(cur),'mp_command':q[i,1:3].tolist(),
                        'continued_axis':float(sign*(peak[axis]-entry[axis])),
                        'peak_t':float(peak[0]),'entry_xy':entry[1:3].tolist(),
                        'peak_xy':peak[1:3].tolist(),'ref_t':reference[0],
                        'ref_frame':reference[2],
                        'entry_path_distance':path_distance(entry[1:3],reference[3]),
                        'peak_path_distance':path_distance(peak[1:3],reference[3])})
                    last=t
    return result


def turn_detail(data, t, axis, old_sign, end):
    motor=motor_rows(data)
    arrays={name:command_array(data,topic) for name,topic in LAYERS.items()}
    arrays['bridge']=command_array(data,'/medical_nav/bridge_cmd_debug','sent_cmd')
    arrays['MCU_target_est']=motor[:,:3]
    arrays['wheel_est']=motor[:,[0,3,4]]
    idx=1 if axis=='x' else 2
    records={}
    for name,array in arrays.items():
        q=array[(array[:,0]>=t)&(array[:,0]<end)&(old_sign*array[:,idx]<-.01)]
        records[name]={'opposite_delay':float(q[0,0]-t) if len(q) else None,
                       'at_reversal':sample(array,[t])[0].tolist()}
    print('TURN_LAYERS',json.dumps({'t':t,'layers':records},ensure_ascii=False))
    raw=arrays['MPPI']
    returns=raw[(raw[:,0]>t)&(raw[:,0]<end)&(old_sign*raw[:,idx]>.01)]
    episode_end=float(returns[0,0]) if len(returns) else end
    # Do not attribute motion following a second reversal to the first one.
    p=data['poses'][(data['poses'][:,0]>=t)&(data['poses'][:,0]<episode_end)]
    entry=data['poses'][np.argmin(abs(data['poses'][:,0]-t))]
    peak=p[np.argmax(old_sign*p[:,idx])]
    grid=np.arange(t,peak[0],.005)
    area=np.maximum(old_sign*sample(arrays['wheel_est'],grid)[:,idx-1],0).sum()*.005
    ref=max((r for r in data['path_topics']['/transformed_global_plan'] if r[0]<t),key=lambda r:r[0])
    print('TURN_PHYSICAL',json.dumps({'axis':axis,'episode_end':episode_end,
          'peak_t':float(peak[0]),'continued_map_axis':float(old_sign*(peak[idx]-entry[idx])),
          'wheel_old_body_axis_integral':float(area),
          'entry_path_distance':path_distance(entry[1:3],ref[3]),
          'peak_path_distance':path_distance(peak[1:3],ref[3]),
          'entry_xy':entry[1:3].tolist(),'peak_xy':peak[1:3].tolist()},ensure_ascii=False))
    for at in np.arange(t-.1,end,.2):
        row={'t':round(float(at),4)}
        for name in ('MPPI','smoother','safe','bridge','MCU_target_est','wheel_est'):
            row[name]=round(float(sample(arrays[name],[at])[0,idx-1]),4)
        q=data['poses'][np.argmin(abs(data['poses'][:,0]-at))]
        row['pose_xy']=np.round(q[1:3],4).tolist()
        print('TURN_SAMPLE',json.dumps(row))
    print('TURN_EVENTS',[(round(float(at-t),4),msg) for at,msg in data['events']
                        if t-.3<at<end and any(k in msg for k in
                        ('Optimizer fail','Robot to approach','route is blocked'))])
    return arrays


def turn_plots(data):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    examples=[('Bed1 to Bed3, body Y',39.75440955162048,'y',-1,42.4),
              ('Return Home, body X',56.794556617736816,'x',-1,58.3)]
    fig,axs=plt.subplots(2,2,figsize=(14,10),constrained_layout=True)
    for i,(label,t,axis,sign,end) in enumerate(examples):
        arrays=turn_detail(data,t,axis,sign,end)
        idx=1 if axis=='x' else 2
        for name in ('MPPI','smoother','safe','MCU_target_est','wheel_est'):
            q=arrays[name];q=q[(q[:,0]>=t-.5)&(q[:,0]<end)]
            axs[i,0].plot(q[:,0]-t,q[:,idx],label=name,lw=1.2)
        axs[i,0].axhline(0,color='gray',lw=.8)
        axs[i,0].axvline(0,color='gray',ls='--',lw=.8)
        axs[i,0].set(title=label,xlabel='seconds from first MPPI reversal',ylabel='m/s')
        axs[i,0].grid(alpha=.25);axs[i,0].legend(fontsize=8)
        ref=max((r for r in data['path_topics']['/transformed_global_plan'] if r[0]<t),key=lambda r:r[0])
        p=data['poses'][(data['poses'][:,0]>=t-.5)&(data['poses'][:,0]<end)]
        entry=data['poses'][np.argmin(abs(data['poses'][:,0]-t))]
        raw=arrays['MPPI']
        returns=raw[(raw[:,0]>t)&(raw[:,0]<end)&(sign*raw[:,idx]>.01)]
        episode_end=float(returns[0,0]) if len(returns) else end
        after=p[(p[:,0]>=t)&(p[:,0]<episode_end)];peak=after[np.argmax(sign*after[:,idx])]
        path=np.asarray(ref[3])
        axs[i,1].plot(path[:,0],path[:,1],color='gray',label='frozen pre-reversal controller path')
        axs[i,1].plot(p[:,1],p[:,2],color='tab:red',label='direct OPS pose')
        axs[i,1].scatter(*entry[1:3],label='MPPI reversal')
        axs[i,1].scatter(*peak[1:3],marker='x',label='old-direction extreme')
        axs[i,1].set(xlabel='map X (m)',ylabel='map Y (m)',title=label+' excursion',
                     xlim=(p[:,1].min()-.3,p[:,1].max()+.6),
                     ylim=(p[:,2].min()-.25,p[:,2].max()+.25))
        axs[i,1].set_aspect('equal',adjustable='box');axs[i,1].grid(alpha=.25)
        axs[i,1].legend(fontsize=8)
    fig.savefig(ROOT/'turn_layers.png',dpi=170);plt.close(fig)


def dock_report(data):
    # Within each calibration epoch, displacement is unaffected by OPS offsets.
    cal=[(t,s) for t,s in data['events'] if 'STP23L calibrated' in s]
    for a,b,s in segments(data,data['runs'][0]):
        if s not in (12,13): continue
        stamps=[a]+[t for t,_ in cal if a<t<b]+[b]
        for left,right in zip(stamps,stamps[1:]):
            p=data['poses'][(data['poses'][:,0]>=left+.04)&(data['poses'][:,0]<right-.01)]
            print('DOCK_EPOCH',NAMES[s],round(left,3),round(right,3),
                  'xy_min',p[:,1:3].min(axis=0).tolist(),'xy_max',p[:,1:3].max(axis=0).tolist(),
                  'entry',p[0,1:3].tolist(),'exit',p[-1,1:3].tolist())
        for t,msg in cal:
            if a<t<b:
                pre=data['poses'][np.searchsorted(data['poses'][:,0],t)-1]
                post=data['poses'][np.searchsorted(data['poses'][:,0],t+.03)]
                print('CALIBRATION_JUMP',t,msg,'pre',pre[1:3].tolist(),'post',post[1:3].tolist())
    for r in data['topics']['/medical_nav/bridge_status']:
        if 30.9<r['t']<33 or 45.1<r['t']<47.3:
            d=r['d']
            print('DOCK_STATUS',round(r['t'],3),json.dumps({k:d.get(k) for k in
                ('stm32','ops_offset_mm','stp23l','twist_source')},ensure_ascii=False))


def audit(data, previous):
    import yaml
    def sections(d):
        text=(d['root']/'snapshot.txt').read_text(encoding='utf8')
        parts=re.split(r'========== ([A-Z0-9 ]+) ==========\s*',text)
        return {parts[i]:parts[i+1] for i in range(1,len(parts)-1,2)}
    left,right=sections(previous),sections(data)
    for section in ('CONTROLLER','COLLISION MONITOR','STM32 BRIDGE'):
        a=yaml.safe_load(left[section]);b=yaml.safe_load(right[section])
        def compare(x,y,path=''):
            if isinstance(x,dict) and isinstance(y,dict):
                for key in sorted(x.keys()|y.keys()): compare(x.get(key),y.get(key),path+'/'+key)
            elif x!=y: print('PARAMETER_DIFF',section,path,repr(x),'->',repr(y))
        compare(a,b)
    con=sqlite3.connect(f"file:{(data['root']/'rosbag/rosbag_0.db3').as_posix()}?mode=ro",uri=True)
    static={}
    for _,raw in con.execute('SELECT m.timestamp,m.data FROM messages m JOIN topics t '
                             'ON t.id=m.topic_id WHERE t.name=? ORDER BY m.timestamp',('/tf_static',)):
        r=CdrReader(raw)
        for _ in range(r.u32()):
            _,frame=r.header();child=r.string();values=[r.f64() for _ in range(7)]
            static[frame+'/'+child]=values
    print('STATIC_TF',json.dumps(static))
    sensor_rows=[]
    for t,raw in con.execute('SELECT m.timestamp,m.data FROM messages m JOIN topics t '
                             'ON t.id=m.topic_id WHERE t.name=? ORDER BY m.timestamp',('/medical_nav/stp23l',)):
        d=json.loads(CdrReader(raw).string());sensor_rows.append((t/1e9-data['base'],d))
    print('STP_FORMAT',sensor_rows[0] if sensor_rows else None)
    stage_rows=segments(data,data['runs'][0])
    for a,b,s in stage_rows:
        if s not in (12,13): continue
        rows=[(t,d) for t,d in sensor_rows if a-.5<t<b+.5]
        # Scalar ranges in millimetres are independent of OPS offset jumps.
        print('DOCK_SENSOR_ROWS',NAMES[s],json.dumps(rows[::max(1,len(rows)//12)],ensure_ascii=False))
        dispense=next((left,right) for left,right,state in stage_rows
                      if state==(5 if s==12 else 8) and left>=b-.01)
        side='a' if s==12 else 'c'
        for label,left,right in (('dock',a,b),('dispense_hold',*dispense),
                                 ('dock_and_hold',a,dispense[1])):
            for key,target in (('b',355.7),(side,1155.7)):
                values=[]
                for t,d in sensor_rows:
                    sensor=d.get('sensors',{}).get(key,{})
                    value=sensor.get('sensor_distance_mm')
                    if left<=t<right and sensor.get('valid') and isinstance(value,(int,float)):
                        values.append((t,value))
                if values:
                    minimum=min(values,key=lambda row:row[1])
                    maximum=max(values,key=lambda row:row[1])
                    print('DOCK_SENSOR_EXTREMES',json.dumps({
                        'state':NAMES[s],'window':label,'sensor':key,
                        'raw_target_mm':target,'minimum':minimum,'maximum':maximum,
                        'closest_beyond_target_mm':max(0,target-minimum[1]),
                        'last':values[-1]},ensure_ascii=False))
    con.close()


def plots(data, baseline):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(4,1,figsize=(14,11),constrained_layout=True)
    start,end=data['runs'][0]
    for ax,(a,b,s) in zip(axs,((a,b,s) for a,b,s in segments(data,(start,end)) if s in NAV)):
        for name in ('MPPI','smoother','safe','wheel'):
            q=command_array(data,LAYERS[name])
            q=q[(q[:,0]>=a)&(q[:,0]<b)]
            ax.plot(q[:,0]-a,np.linalg.norm(q[:,1:3],axis=1),label=name,lw=1.2)
        for t,msg in data['events']:
            if a<t<b and 'Optimizer fail' in msg:
                ax.axvline(t-a,color='red',ls=':',alpha=.7)
        ax.set(title=f'{NAMES[s]}: {b-a:.3f} s',xlabel='seconds from state entry',ylabel='speed (m/s)')
        ax.grid(alpha=.25)
    axs[0].legend(ncol=4)
    fig.savefig(ROOT/'speed_chain.png',dpi=160);plt.close(fig)
    fig,axs=plt.subplots(1,2,figsize=(12,4.6),constrained_layout=True)
    for ax,d,run,label in ((axs[0],baseline,baseline['runs'][-1],'Previous run 4, OPEN_LOOP'),
                         (axs[1],data,data['runs'][0],'Latest CLOSED_LOOP')):
        a,b=next((a,b) for a,b,s in segments(d,run) if s==1)
        for name in ('MPPI','smoother','safe','wheel'):
            q=command_array(d,LAYERS[name]);q=q[(q[:,0]>=a)&(q[:,0]<b)]
            ax.plot(q[:,0]-a,q[:,1],label=name,lw=1.3)
        ax.set(title=f'{label}: nurse {b-a:.3f} s',xlabel='seconds from state entry',ylabel='body X (m/s)',ylim=(-.08,1.22))
        ax.grid(alpha=.25);ax.legend(fontsize=8)
    fig.savefig(ROOT/'nurse_ramp_comparison.png',dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,7),constrained_layout=True)
    for a,b,s in segments(data,data['runs'][0]):
        if s not in NAV+(12,13): continue
        p=data['poses'][(data['poses'][:,0]>=a)&(data['poses'][:,0]<b)]
        ax.plot(p[:,1],p[:,2],label=NAMES[s],lw=1.5)
    ax.scatter([0,1.7,5.4,5.4],[0,0,2.2,-2.2],c='black',marker='x')
    ax.set(xlabel='map X (m)',ylabel='map Y (m)',title='Latest direct OPS trajectory (calibration jumps included)')
    ax.axis('equal');ax.grid(alpha=.25);ax.legend(fontsize=8)
    fig.savefig(ROOT/'latest_xy.png',dpi=160);plt.close(fig)


if __name__=='__main__':
    latest=load(ROOT)
    previous=load(ROOT.parent/'bed13_full_20261010_205819')
    if '--fit' in sys.argv:
        print('LATEST_FIT',json.dumps(fits(latest),ensure_ascii=False))
        print('PREVIOUS_FIT',json.dumps(fits(previous),ensure_ascii=False))
    elif '--turns' in sys.argv:
        print(json.dumps(turn_candidates(latest),ensure_ascii=False,indent=2))
    elif '--details' in sys.argv:
        turn_detail(latest,23.532440423965454,'y',1,25.1)
        turn_plots(latest)
    elif '--dock' in sys.argv:
        dock_report(latest)
    elif '--audit' in sys.argv:
        audit(latest,previous)
    else:
        result={'latest':summary(latest),'previous':summary(previous)[-2:]}
        print(json.dumps(result,ensure_ascii=False,indent=2))
        (ROOT/'diagnostics.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
        if '--plot' in sys.argv: plots(latest,previous)
