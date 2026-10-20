"""Offline diagnostics only: no robot/configuration changes.

Receipt times are recorder times; MCU diagnostic times use an estimated
per-reset clock offset. Costmap messages are validated against their lengths.
"""
import json
import math
import re
import sqlite3
import struct
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'bed13_full_20261010_205819'))
sys.path.insert(0, str(ROOT.parent / 'bed13_full_20261010_220842'))
from analyze_overshoot import (CdrReader, NAMES, command_array, event_counts,
                               load, motor_rows, path_distance)
from analyze_closed_loop import sample, segments, summary, turn_candidates


def clock(data, t):
    return datetime.fromtimestamp(data['base'] + t).strftime('%H:%M:%S.%f')[:-3]


def maps(data):
    con = sqlite3.connect(f"file:{(ROOT/'rosbag/rosbag_0.db3').as_posix()}?mode=ro", uri=True)
    result = {}
    for topic in ('/global_costmap/costmap_raw', '/local_costmap/costmap_raw'):
        rows = []
        for t, raw in con.execute('SELECT m.timestamp,m.data FROM messages m JOIN topics t '
                                  'ON t.id=m.topic_id WHERE t.name=? ORDER BY m.timestamp', (topic,)):
            r = CdrReader(raw)
            stamp, frame = r.header()
            r.i32(); r.u32(); r.i32(); r.u32()
            layer = r.string()
            r.align(4)
            resolution = struct.unpack_from('<f', r.data, r.offset)[0]
            r.offset += 4
            nx, ny = r.u32(), r.u32()
            origin = [r.f64() for _ in range(7)]
            n = r.u32()
            assert n == nx * ny, (topic, n, nx, ny, r.offset)
            assert len(raw) == r.offset + n, (len(raw), r.offset, n)
            a = np.frombuffer(raw, dtype=np.uint8, offset=r.offset, count=n).reshape(ny, nx).copy()
            rows.append(dict(t=t/1e9-data['base'], stamp=stamp-data['base'], frame=frame,
                             layer=layer, resolution=float(resolution), origin=origin[:2], a=a))
        result[topic] = rows
    con.close()
    return result


def before(rows, t):
    return rows[max(0, np.searchsorted([r['t'] for r in rows], t, side='right')-1)]


def map_at(rows, t):
    return min(rows, key=lambda r: abs(r['t'] - t))


def xy_grid(m):
    ny, nx = m['a'].shape
    return np.meshgrid(m['origin'][0] + (np.arange(nx)+.5)*m['resolution'],
                       m['origin'][1] + (np.arange(ny)+.5)*m['resolution'])


def line_values(m, points):
    points = np.asarray(points)
    q = np.floor((points-np.asarray(m['origin']))/m['resolution']).astype(int)
    good = (q[:, 0]>=0)&(q[:, 0]<m['a'].shape[1])&(q[:, 1]>=0)&(q[:, 1]<m['a'].shape[0])
    result = np.full(len(q), 255, dtype=np.uint8)
    result[good] = m['a'][q[good, 1], q[good, 0]]
    return result


def line_metrics(m, path):
    p = np.asarray(path)
    if len(p)<2:
        return {}
    line = np.linspace(p[0], p[-1], max(2, int(np.linalg.norm(p[-1]-p[0])/.01)+1))
    c = line_values(m, line)
    pc = line_values(m, p)
    ds = np.linalg.norm(np.diff(p, axis=0), axis=1)
    yaw = 0.0
    # Travel yaw is held near zero. Report this as an optimistic geometric model.
    d = np.diff(p, axis=0)
    t = np.maximum(abs(d[:, 0]), abs(d[:, 1])).sum()/3
    dt = np.max(abs(p[-1]-p[0]))/3
    dev = max(path_distance(q, [p[0],p[-1]]) for q in p)
    return dict(path_length=float(ds.sum()), direct_length=float(np.linalg.norm(p[-1]-p[0])),
                optimistic_time=float(t), optimistic_direct_time=float(dt),
                direct_max_cost=int(c.max()), direct_blocked_samples=int((c>=253).sum()),
                direct_mean_cost=float(c.mean()), path_mean_cost=float(pc.mean()),
                path_max_cost=int(pc.max()), chord_deviation=float(dev),
                start=p[0].tolist(), goal=p[-1].tolist())


def contact_sheet():
    cap = cv2.VideoCapture(str(ROOT.parent/'RVIZ1011.mp4'))
    fps = cap.get(cv2.CAP_PROP_FPS)
    n = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    times = np.arange(0, n/fps, 8)
    sheet = Image.new('RGB', (4*454, math.ceil(len(times)/4)*360), 'white')
    draw = ImageDraw.Draw(sheet)
    for i,t in enumerate(times):
        cap.set(cv2.CAP_PROP_POS_MSEC, float(t)*1000)
        ok, frame = cap.read()
        if not ok: continue
        im = Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
        im.thumbnail((454,334))
        x,y=(i%4)*454,(i//4)*360
        sheet.paste(im,(x,y+26));draw.text((x+5,y+5),f'Video {t:.0f} s',fill='black')
    sheet.save(ROOT/'video_contact_sheet.jpg')
    cap.release()
    return dict(fps=fps, frames=n, duration=n/fps)


def calibration_metrics(data, costmaps):
    events = [(t,msg) for t,msg in data['events'] if 'STP23L calibrated' in msg]
    result=[]
    old=np.zeros(2)
    previous_run = None
    for t,msg in events:
        off=re.search(r'offset=\(([-\d.]+), ([-\d.]+)\)',msg)
        if not off: continue
        offset=np.array([float(off[1]),float(off[2])])
        run=next((i for i,(a,b) in enumerate(data['runs'],1) if a<t<b),0)
        # STM32 resets between trials also reset bridge calibration offsets.
        if run != previous_run:
            old = np.zeros(2)
            previous_run = run
        delta=offset-old
        old=offset
        p=data['poses']
        pre=p[np.searchsorted(p[:,0],t)-1]
        post=p[min(len(p)-1,np.searchsorted(p[:,0],t+.04))]
        entry=dict(t=t,time=clock(data,t),run=run,msg=msg,offset_mm=offset.tolist(),
                   delta_offset_mm=delta.tolist(), expected_ros_jump=[delta[1]/1000,-delta[0]/1000],
                   pose_before=pre[1:3].tolist(),pose_after=post[1:3].tolist(),
                   sampled_pose_delta=(post[1:3]-pre[1:3]).tolist(),costmaps={})
        for topic,rows in costmaps.items():
            stats=[]
            for dt in (-1.5,-.5,0,.5,1,2,4,6):
                m=map_at(rows,t+dt)
                x,y=xy_grid(m)
                # Rear region measured in fixed map coordinates for comparison.
                roi=(x>pre[1]-3)&(x<pre[1]-.4)&(y>pre[2]-1.5)&(y<pre[2]+1.5)
                a=m['a']
                stats.append(dict(dt=dt,map_t=m['t'],lethal=int((a==254).sum()),
                                  hard=int((a>=253).sum()),inflated=int(((a>0)&(a<253)).sum()),
                                  rear_lethal=int(((a==254)&roi).sum()),
                                  rear_hard=int(((a>=253)&roi).sum())))
            entry['costmaps'][topic]=stats
        result.append(entry)
    return result


def map_plot(ax,m):
    a=m['a'];r=m['resolution'];o=m['origin']
    # Cost is plotted directly; lethal marks use red and soft inflation blues.
    from matplotlib.colors import ListedColormap
    colors=plt.cm.Blues(np.linspace(0,1,256));colors[0]=[1,1,1,1]
    colors[253]=[.55,0,.8,1];colors[254]=[1,0,0,1];colors[255]=[.5,.5,.5,1]
    ax.imshow(a,origin='lower',extent=[o[0],o[0]+a.shape[1]*r,o[1],o[1]+a.shape[0]*r],
              cmap=ListedColormap(colors),vmin=0,vmax=255,interpolation='nearest')


def plots(data,costmaps,cals):
    bed=[c for c in cals if 'bed3' in c['msg'].lower()]
    if not bed:
        bed=cals[1::2]
    fig,axs=plt.subplots(len(bed),4,figsize=(16,4*len(bed)),constrained_layout=True)
    for row,c in enumerate(bed):
        for j,dt in enumerate((-.7,.6,2,5)):
            ax=axs[row,j];m=map_at(costmaps['/global_costmap/costmap_raw'],c['t']+dt)
            map_plot(ax,m)
            pose=data['poses'][np.argmin(abs(data['poses'][:,0]-m['t']))]
            ax.add_patch(plt.Circle(pose[1:3],.23,fill=False,color='black',lw=1.5))
            ax.set(xlim=(-.5,6.5),ylim=(-3.5,3.5),title=f"Run {c['run']}, calibration {dt:+.1f}s\n{clock(data,m['t'])}")
            ax.set_aspect('equal');ax.grid(alpha=.2)
    fig.savefig(ROOT/'bed3_costmaps.png',dpi=160);plt.close(fig)
    fig,axs=plt.subplots(1,len(data['runs']),figsize=(6*len(data['runs']),7),constrained_layout=True)
    plan_metrics=[]
    for ax,(start,end),i in zip(np.atleast_1d(axs),data['runs'],range(1,len(data['runs'])+1)):
        a,b,_=next(q for q in segments(data,(start,end)) if q[2]==6)
        rows=[r for r in data['path_topics']['/plan'] if a<=r[0]<b]
        m=before(costmaps['/global_costmap/costmap_raw'],a+.5)
        map_plot(ax,m)
        for j,(t,stamp,frame,path) in enumerate(rows):
            if not path:continue
            metrics=line_metrics(before(costmaps['/global_costmap/costmap_raw'],t),path)
            metrics.update(run=i,t=t,time=clock(data,t),map_age=t-before(costmaps['/global_costmap/costmap_raw'],t)['t'])
            plan_metrics.append(metrics)
            if j%max(1,len(rows)//5)==0:
                p=np.asarray(path);ax.plot(p[:,0],p[:,1],lw=1,label=f'{t-a:.1f}s')
        p=data['poses'][(data['poses'][:,0]>=a)&(data['poses'][:,0]<b)]
        ax.plot(p[:,1],p[:,2],color='black',lw=2,label='actual OPS')
        ax.set(xlim=(2.5,6.2),ylim=(-3,2.8),title=f'Run {i}: Bed1 to Bed3 {b-a:.2f}s',xlabel='map X (m)',ylabel='map Y (m)')
        ax.set_aspect('equal');ax.legend(fontsize=8);ax.grid(alpha=.2)
    fig.savefig(ROOT/'bed3_paths.png',dpi=160);plt.close(fig)
    return plan_metrics


def main():
    data=load(ROOT)
    costmaps=maps(data)
    video=contact_sheet()
    cals=calibration_metrics(data,costmaps)
    plans=plots(data,costmaps,cals)
    result={'base':data['base'],'video':video,'runs':summary(data),'calibrations':cals,
            'bed3_plans':plans,'turn_candidates':turn_candidates(data)}
    (ROOT/'diagnostics.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
    print('VIDEO',video)
    print('MAPS',{k:(len(v),v[0]['a'].shape,v[0]['origin'],v[0]['resolution']) for k,v in costmaps.items()})
    for r in result['runs']:
        print('RUN',r['run'],r['start'],round(r['duration'],3),r['events'])
        for s in r['stages']:
            print('STAGE',s['name'],round(s['duration'],3), 'wheel mean/peak',
                  round(s['speeds']['wheel']['mean'],3),round(s['speeds']['wheel']['peak'],3),
                  s.get('events',{}))
    for c in cals:
        print('CAL',c['time'],c['msg'],'delta',c['delta_offset_mm'],'observed',c['sampled_pose_delta'])
        print('CAL_MAP',[(r['dt'],r['lethal'],r['rear_lethal']) for r in c['costmaps']['/global_costmap/costmap_raw']])
    for r in plans:
        print('PLAN',json.dumps(r))
    for r in result['turn_candidates']:
        if r['continued_axis']>.12:print('TURN',json.dumps(r))


if __name__=='__main__':
    main()
