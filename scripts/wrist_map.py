#!/usr/bin/env python3
"""Map wrist-camera AprilTag detections to the table (ray-plane z=14.9, block tops) using a wrist calibration.
Usage: ~/venvs/realsense/bin/python scripts/wrist_map.py config/wrist_cam_*.json SCAN_DIR  (uses the *_0.jpg frame per stop)
"""
import json, sys, cv2, numpy as np
sys.path.insert(0, 'scripts'); import kinematics as K
cal, scan = sys.argv[1], sys.argv[2]
c = json.load(open(cal)); R = np.array(c['R_cam_flange']); t = np.array(c['t_cam_flange_mm'])
sc = json.load(open(scan + '/scan.json'))
det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), cv2.aruco.DetectorParameters())
for r in sc['records']:
    if not r['file'].endswith('_0.jpg'): continue
    g = cv2.resize(cv2.imread(scan + '/' + r['file'], 0), None, fx=1.5, fy=1.5, interpolation=cv2.INTER_CUBIC)
    cs, ids, _ = det.detectMarkers(g)
    if ids is None: continue
    Tf = K.fk_frames(r['angles'])[-1]
    for cc, i in zip(cs, ids.ravel()):
        u, v = cc.reshape(4, 2).mean(0) / 1.5
        d_cam = np.array([(u - c['cx']) / c['f'], (v - c['cy']) / c['f'], 1.0])
        d_fl, o_fl = R.T @ d_cam, -R.T @ t
        o = Tf[:3, :3] @ o_fl + Tf[:3, 3]; d = Tf[:3, :3] @ d_fl
        s = (14.9 - o[2]) / d[2]; p = o + s * d
        print(f"{r['file']:16s} tag {i} px ({u:5.0f},{v:5.0f}) -> xy ({p[0]:7.1f},{p[1]:7.1f}) r {np.hypot(p[0],p[1]):5.0f} az {np.degrees(np.arctan2(p[1],p[0])):6.1f}")
