#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 用法: python3 gen_yaml.py [bag路径]   默认 ~/bags/d435i_test.bag
import os, sys, rosbag

BAG = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/bags/d455_test1.bag')
OUT_DIR = os.path.expanduser('~/slam_test')
OUT = os.path.join(OUT_DIR, 'D455.yaml')
os.makedirs(OUT_DIR, exist_ok=True)
print('读取 bag:', BAG)

bag = rosbag.Bag(BAG)
all_topics = list(bag.get_type_and_topic_info().topics.keys())

def pick_ci(side):
    for t in all_topics:
        if side in t and t.endswith('camera_info'): return t
    raise SystemExit('找不到 %s camera_info，话题列表:\n%s' % (side, '\n'.join(all_topics)))

def pick_img(side):
    for t in all_topics:
        if side in t and t.endswith('image_rect_raw'): return t
    raise SystemExit('找不到 %s 图像话题，话题列表:\n%s' % (side, '\n'.join(all_topics)))

ci_L, ci_R, img_L = pick_ci('infra1'), pick_ci('infra2'), pick_img('infra1')
print('使用话题: %s | %s' % (ci_L, ci_R))

def first(topic):
    for t, msg, _ in bag.read_messages(topics=[topic]): return msg
    raise SystemExit('bag 内无消息: ' + topic)

mL, mR = first(ci_L), first(ci_R)
fx, fy, cx, cy = mL.K[0], mL.K[4], mL.K[2], mL.K[5]
D = list(mL.D)[:4] + [0.0] * max(0, 4 - len(mL.D))
P = mR.P
baseline = abs(P[3]) / P[0] if P[0] > 0 else 0.095
if P[3] == 0: print('警告: 右目 P[3]=0，基线退回 D455 默认 0.095m')

stamps = []
for t, msg, _ in bag.read_messages(topics=[img_L]):
    stamps.append(msg.header.stamp.to_sec())
    if len(stamps) >= 101: break
fps = 30
if len(stamps) >= 5:
    dts = sorted(b - a for a, b in zip(stamps, stamps[1:]))
    fps = int(round(1.0 / dts[len(dts) // 2]))

bf = baseline * fx
W, H = mL.width, mL.height
bag.close()

yaml = """%%YAML:1.0
# ===== gen_yaml.py 自动生成 (D455), 来源: %(bag)s =====
Camera.type: "PinHole"
Camera1.fx: %(fx).6f
Camera1.fy: %(fy).6f
Camera1.cx: %(cx).6f
Camera1.cy: %(cy).6f
Camera1.k1: %(k1).8f
Camera1.k2: %(k2).8f
Camera1.p1: %(p1).8f
Camera1.p2: %(p2).8f
Camera.fx: %(fx).6f
Camera.fy: %(fy).6f
Camera.cx: %(cx).6f
Camera.cy: %(cy).6f
Camera.k1: %(k1).8f
Camera.k2: %(k2).8f
Camera.p1: %(p1).8f
Camera.p2: %(p2).8f
Camera.width: %(w)d
Camera.height: %(h)d
Camera.fps: %(fps)d
# bf = baseline * fx
Camera.bf: %(bf).4f
Camera.RGB: 1
ThDepth: 40.0
ORBextractor.nFeatures: 1200
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0
""" % dict(bag=BAG, fx=fx, fy=fy, cx=cx, cy=cy, k1=D[0], k2=D[1], p1=D[2], p2=D[3], w=W, h=H, fps=fps, bf=bf)

open(OUT, 'w').write(yaml)
print('==== 摘要 ====')
print('%dx%d @ %d fps' % (W, H, fps))
print('fx=%.3f fy=%.3f cx=%.3f cy=%.3f' % (fx, fy, cx, cy))
print('D =', D)
print('baseline = %.4f m, bf = %.2f' % (baseline, bf))
print('已生成:', OUT)
