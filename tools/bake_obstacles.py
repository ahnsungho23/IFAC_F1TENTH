import json, math, sys
import numpy as np
from PIL import Image

REPO='/home/sungho/Documents/GitHub/2026_IFAC'
RES=0.05; OX=0.0; OY=0.0
wp=json.load(open(f'{REPO}/offline_trajectory_generator/output/map/global_waypoints.json'))['global_traj_wpnts_iqp']['wpnts']

def at(s):
    """가장 가까운 두 waypoint 선형보간."""
    s=s%wp[-1]['s_m']
    for i in range(len(wp)-1):
        if wp[i]['s_m']<=s<=wp[i+1]['s_m']:
            a,b=wp[i],wp[i+1]
            t=(s-a['s_m'])/max(1e-9,b['s_m']-a['s_m'])
            psi=a['psi_rad']+t*math.atan2(math.sin(b['psi_rad']-a['psi_rad']),math.cos(b['psi_rad']-a['psi_rad']))
            return (a['x_m']+t*(b['x_m']-a['x_m']), a['y_m']+t*(b['y_m']-a['y_m']), psi,
                    a['d_left']+t*(b['d_left']-a['d_left']), a['d_right']+t*(b['d_right']-a['d_right']))
    p=wp[-1]; return p['x_m'],p['y_m'],p['psi_rad'],p['d_left'],p['d_right']

# (s, d, 한 변[m], 설명)
OBSTACLES=[
    (16.50,  0.30, 0.25, "고속 직선 좌측치우침 (v=5.0, 우측 통과 창 0.23)"),
    (31.60,  0.35, 0.25, "고속 직선 좌측 치우침"),
    (10.55, -0.50, 0.25, "넓은 구간 우측 (좌측 여유 2.8m)"),
    (25.60,  0.60, 0.25, "곡선 좌측 (kappa 0.31)"),
    (40.60,  0.00, 0.25, "감속 직선 중앙"),
    ( 6.03,  0.55, 0.25, "좁은 곡선 벽쪽 (kappa 0.44, 우측 통과 창 0.21)"),
]

img=np.array(Image.open(f'{REPO}/src/monte_carlo_localization/maps/map.png')).copy()
H,W=img.shape
print(f"{'s':>6} {'d':>6} {'좌여유':>7} {'우여유':>7}  설명")
for s,d,size,note in OBSTACLES:
    x,y,psi,dl,dr=at(s)
    cx=x-d*math.sin(psi); cy=y+d*math.cos(psi)
    half=size/2.0
    left_gap = dl-(d+half); right_gap=(d-half)+dr
    print(f"{s:6.2f} {d:6.2f} {left_gap:7.2f} {right_gap:7.2f}  {note}")
    n=int(math.ceil(half/RES))
    for dr_ in range(-n,n+1):
        for dc in range(-n,n+1):
            wx=cx+dc*RES; wy=cy+dr_*RES
            if abs(wx-cx)>half or abs(wy-cy)>half: continue
            c=int(round((wx-OX)/RES)); r=int(round(H-1-(wy-OY)/RES))
            if 0<=r<H and 0<=c<W: img[r,c]=0

out=sys.argv[1]
Image.fromarray(img).save(out+'.png')
open(out+'.yaml','w').write(
    f"image: {out.split('/')[-1]}.png\nmode: trinary\nresolution: 0.05\norigin: [0.0, 0.0, 0]\n"
    "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
print("\n저장:", out+'.png / .yaml')
# 자차 시작 지점: 장애물에서 먼 직선
x,y,psi,_,_=at(34.0)
print(f"\n자차 시작 (s=34.0): sx={x:.6f} sy={y:.6f} stheta={psi:.6f}")
