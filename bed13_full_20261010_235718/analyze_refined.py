"""Additional offline evidence. This script never changes robot configuration."""
import json
import sqlite3
import numpy as np
import matplotlib.pyplot as plt
from analyze_ghosts_paths import (ROOT, load, maps, clock, before, map_at, xy_grid,
                                line_values, map_plot, path_distance, calibration_metrics)
from analyze_closed_loop import LAYERS, segments, sample, command_array, motor_rows


def diagnostic_samples(data):
    result = {}
    for topic, rows in data['topics'].items():
        if 'filter_status' in topic or 'cone_compensation' in topic:
            result[topic] = [(round(r['t'], 3), r['d']) for r in rows[::max(1, len(rows)//4)]]
    return result


def turns(data):
    motor = motor_rows(data)
    arrays = {name: command_array(data, topic) for name, topic in LAYERS.items()}
    arrays['bridge'] = command_array(data, '/medical_nav/bridge_cmd_debug', 'sent_cmd')
    arrays['MCU_target_est'] = motor[:, :3]
    arrays['wheel_est'] = motor[:, [0, 3, 4]]
    raw = arrays['MPPI']
    result = []
    fig, axs = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    for row, (run, t, axis, sign, end) in enumerate(((2,108.554,'y',-1,110.2),
                                                    (3,153.765,'y',-1,155.5))):
        idx = 1 if axis == 'x' else 2
        # Use the recorded positive command itself, not a rounded seed time.
        reversal = raw[(raw[:,0]>=t-.002)&(raw[:,0]<t+.1)&(sign*raw[:,idx]<-.01)][0]
        t = float(reversal[0])
        returns = raw[(raw[:,0]>t+.002)&(raw[:,0]<end)&(sign*raw[:,idx]>.01)]
        episode_end = float(returns[0,0]) if len(returns) else end
        records = {}
        for name, a in arrays.items():
            q = a[(a[:,0]>=t-.002)&(a[:,0]<episode_end)&(sign*a[:,idx]<-.01)]
            records[name] = dict(opposite_delay=float(q[0,0]-t) if len(q) else None,
                                 at_reversal=sample(a, [t])[0].tolist())
        p = data['poses'][(data['poses'][:,0]>=t)&(data['poses'][:,0]<episode_end)]
        entry = data['poses'][np.argmin(abs(data['poses'][:,0]-t))]
        peak = p[np.argmax(sign*p[:,idx])]
        full = data['poses'][(data['poses'][:,0]>=t)&(data['poses'][:,0]<end)]
        full_peak = full[np.argmax(sign*full[:,idx])]
        ref = before_path(data['path_topics']['/transformed_global_plan'], t)
        grid = np.arange(t, peak[0], .005)
        area = float(np.maximum(sign*sample(arrays['wheel_est'], grid)[:,idx-1],0).sum()*.005)
        samples=[]
        for at in np.arange(t-.1, end, .2):
            s={'time':clock(data,at),'dt':round(float(at-t),3)}
            for name in ('MPPI','smoother','safe','bridge','MCU_target_est','wheel_est'):
                s[name]=round(float(sample(arrays[name],[at])[0,idx-1]),4)
            samples.append(s)
        result.append(dict(run=run,time=clock(data,t),t=t,axis=axis,episode_end=episode_end,
            layers=records,continued_map_axis=float(sign*(peak[idx]-entry[idx])),
            wheel_old_body_axis_integral=area,peak_t=float(peak[0]),
            full_interval_end=end, full_interval_axis_excursion=float(sign*(full_peak[idx]-entry[idx])),
            full_interval_peak_t=float(full_peak[0]), yaw_range_deg=np.degrees(full[:,3]).tolist()[::max(1,len(full)//5)],
            frozen_ref_t=ref[0],frozen_ref_frame=ref[2],
            entry_path_distance=path_distance(entry[1:3],ref[3]),
            peak_path_distance=path_distance(peak[1:3],ref[3]),samples=samples,
            events=[(clock(data,at),msg) for at,msg in data['events'] if t-.3<at<end and
                    any(k in msg for k in ('Optimizer fail','Robot to approach','route is blocked'))]))
        for name in ('MPPI','smoother','safe','MCU_target_est','wheel_est'):
            q=arrays[name];q=q[(q[:,0]>=t-.5)&(q[:,0]<end)]
            axs[row,0].plot(q[:,0]-t,q[:,idx],label=name,lw=1.3)
        axs[row,0].axvline(0,color='gray',ls='--');axs[row,0].axhline(0,color='gray',lw=.8)
        axs[row,0].set(title=f'Run {run}: Bed3 direction reversal',xlabel='seconds after MPPI reversal',ylabel='body Y (m/s)')
        axs[row,0].legend(fontsize=8);axs[row,0].grid(alpha=.2)
        path=np.asarray(ref[3]);axs[row,1].plot(path[:,0],path[:,1],label='frozen controller path',color='gray')
        q=data['poses'][(data['poses'][:,0]>=t-.5)&(data['poses'][:,0]<end)]
        axs[row,1].plot(q[:,1],q[:,2],color='black',label='OPS')
        axs[row,1].scatter(*entry[1:3],label='MPPI reversal');axs[row,1].scatter(*peak[1:3],marker='x',label='old direction extreme')
        axs[row,1].set(xlim=(q[:,1].min()-.25,q[:,1].max()+.8),ylim=(q[:,2].min()-.2,q[:,2].max()+.4),
                         xlabel='map X (m)',ylabel='map Y (m)')
        axs[row,1].set_aspect('equal');axs[row,1].legend(fontsize=8);axs[row,1].grid(alpha=.2)
    fig.savefig(ROOT/'overshoot_evidence.png',dpi=160);plt.close(fig)
    return result


def before_path(rows,t):
    return max((r for r in rows if r[0]<=t),key=lambda r:r[0])


def ghost_analysis(data,costmaps):
    rows=costmaps['/global_costmap/costmap_raw']
    # Early maps are useful references, not ground truth for moving objects.
    baseline=rows[0]
    x,y=xy_grid(baseline)
    roi=(x>1.0)&(x<3.0)&(y>-.6)&(y<1.6)
    records=[]
    for m in rows:
        records.append(dict(t=m['t'],time=clock(data,m['t']),
            roi_lethal=int(((m['a']==254)&roi).sum()),
            roi_hard=int(((m['a']>=253)&roi).sum()),
            roi_extra_lethal=int(((m['a']==254)&(baseline['a']!=254)&roi).sum())))
    cals=[c for c in calibration_metrics(data,costmaps) if 'bed3' in c['msg'].lower()]
    fig,axs=plt.subplots(2,3,figsize=(16,10),constrained_layout=True)
    checks=[]
    for j,c in enumerate(cals):
        for k,dt in enumerate((-.5,1.0)):
            m=map_at(rows,c['t']+dt);map_plot(axs[k,j],m)
            p=data['poses'][np.argmin(abs(data['poses'][:,0]-m['t']))]
            axs[k,j].add_patch(plt.Circle(p[1:3],.23,fill=False,color='black'))
            axs[k,j].add_patch(plt.Rectangle((1,-.6),2,2.2,fill=False,color='orange',lw=2))
            axs[k,j].set(title=f"Run {c['run']}, calibration {dt:+.1f}s\n{clock(data,m['t'])}",xlim=(-.5,6.5),ylim=(-3.5,3.5))
            axs[k,j].set_aspect('equal')
        early=map_at(rows,c['t']-1)
        late=map_at(rows,c['t']+1)
        old=(early['a']==254)&roi&(baseline['a']!=254)
        retained=(late['a']==254)&old
        checks.append(dict(run=c['run'],cal_time=c['time'],before=clock(data,early['t']),after=clock(data,late['t']),
            before_extra_lethal=int(old.sum()),retained_after=int(retained.sum()),
            new_lethal=int(((late['a']==254)&(early['a']!=254)&roi).sum())))
    fig.savefig(ROOT/'ghost_full_maps.png',dpi=150);plt.close(fig)
    fig,ax=plt.subplots(figsize=(12,3),constrained_layout=True)
    ax.plot([r['t'] for r in records],[r['roi_extra_lethal'] for r in records],label='additional lethal cells in nurse rear ROI')
    for c in cals:ax.axvline(c['t'],ls='--',label=f"Bed3 run {c['run']}")
    ax.set(xlabel='seconds after timeline start',ylabel='cells',title='Historical obstacle marks around nurse station')
    ax.legend(fontsize=8);ax.grid(alpha=.2);fig.savefig(ROOT/'ghost_cell_history.png',dpi=150);plt.close(fig)
    return dict(reference_time=clock(data,baseline['t']),roi=[1,3,-.6,1.6],checks=checks,history=records)


def tail_paths(data,costmaps):
    out=[]
    for run,(start,end) in enumerate(data['runs'],1):
        a,b,_=next(s for s in segments(data,(start,end)) if s[2]==6)
        for t,stamp,frame,path in data['path_topics']['/plan']:
            if not a<t<b or not path:continue
            p=np.asarray(path);m=before(costmaps['/global_costmap/costmap_raw'],t)
            pose=data['poses'][np.argmin(abs(data['poses'][:,0]-t))]
            if pose[2]>.0:continue
            chord=np.linspace(p[0],p[-1],400);c=line_values(m,chord)
            out.append(dict(run=run,time=clock(data,t),t=t,pose=pose[1:3].tolist(),
                path_start=p[0].tolist(),goal=p[-1].tolist(),path_points=p.tolist(),
                direct_max=int(c.max()),direct_hard_samples=int((c>=253).sum()),
                direct_mean=float(c.mean()),direct_length=float(np.linalg.norm(p[-1]-p[0])),
                path_length=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum()),
                start_distance_from_pose=float(np.linalg.norm(p[0]-pose[1:3]))))
    return out


def main():
    data=load(ROOT);costmaps=maps(data)
    con=sqlite3.connect(f"file:{(ROOT/'rosbag/rosbag_0.db3').as_posix()}?mode=ro",uri=True)
    print('MAPTOPICS',con.execute("select name,type from topics where name like '%map%'").fetchall());con.close()
    result=dict(samples=diagnostic_samples(data),ghosts=ghost_analysis(data,costmaps),
                turns=turns(data),tail_paths=tail_paths(data,costmaps))
    (ROOT/'refined_diagnostics.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
    print('SAMPLES',json.dumps(result['samples'],ensure_ascii=False))
    print('GHOST_CHECKS',result['ghosts']['checks'])
    for t in result['turns']:
        print('TURN',json.dumps({k:v for k,v in t.items() if k!='samples'},ensure_ascii=False))
    for p in result['tail_paths']:
        print('TAIL',json.dumps({k:v for k,v in p.items() if k!='path_points'},ensure_ascii=False))


if __name__=='__main__':main()
