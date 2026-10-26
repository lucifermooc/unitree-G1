#!/usr/bin/env python3
"""离线模拟跳变锁死解除（带反馈 + 采信后验证），规则与 lightning loc_guard.h 的 JumpRelock 一致。

用法：python3 -I tools/loc_debug/sim_relock.py <run_loc_online glog INFO 文件...> [K=5 min_travel=0.5 ...]

每帧 NDT 结果视为地图系绝对位姿（日志里的），LIO 预测 guess 按"已采信修正 D × 日志 guess"调整：
采信后输出跟着修正走，之后的残差按修正后的输出重算——这就能看出采信后 NDT 是否和新输出一致。
采信后 verify 帧内被拒 >= verify_fail 帧 -> 判采信错误：撤回修正、停用解除（等人工重定位）。
"""
import math, re, sys

P = dict(K=5, min_score=1.2, score_ratio=0.95, cons_m=0.15, cons_deg=2.0, min_travel=0.5,
         max_m=1.0, max_deg=10.0, reset_m=0.10, hint=15, gate_m=0.2, gate_deg=3.0, verify=5, verify_fail=3)
files = [a for a in sys.argv[1:] if '=' not in a]
for a in sys.argv[1:]:
    if '=' in a:
        k, v = a.split('='); P[k] = int(float(v)) if isinstance(P[k], int) else float(v)


def wrap(a): return (a + 180) % 360 - 180
def comp(a, b):
    c, s = math.cos(math.radians(a[2])), math.sin(math.radians(a[2]))
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))
def inv(a):
    c, s = math.cos(math.radians(a[2])), math.sin(math.radians(a[2]))
    return (-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), -a[2])


def load(f):
    frames = []; ndt = None; conv = False
    for l in open(f, errors='ignore'):
        if 'lidar loc converged' in l or 'not converged within' in l: conv = True
        m = re.search(r'^I\d+ ([\d:.]{12}).*lidar_loc.cc:\d+\] confidence: ([\d.]+), t: +([-\d.e]+) +([-\d.e]+) +[-\d.e]+, succ: \d, rpy\(deg\): [-\d.e]+ [-\d.e]+ ([-\d.e]+)', l)
        if m: ndt = (m[1], float(m[2]), (float(m[3]), float(m[4]), float(m[5]))); continue
        m = re.search(r'loc guard frame: t [\d.]+ lo ([-\d.]+) ([-\d.]+) ([-\d.]+) out ([-\d.]+) ([-\d.]+) ([-\d.]+) ', l)
        if m and ndt:
            frames.append((ndt[0], ndt[1], ndt[2], tuple(map(float, m.groups()[:3])), tuple(map(float, m.groups()[3:6])), conv))
            ndt = None
    return frames


for f in files:
    fr = load(f)
    D = (0.0, 0.0, 0.0)        # 已采信的地图系修正（作用在日志输出上）
    run = []; cnt = 0; hist = []; ev = []; verify = None; disabled = False; hint = None; resid_after = []
    for k in range(1, len(fr)):
        t, sc, nd, lo, out, cv = fr[k]
        lo0, out0 = fr[k - 1][3], fr[k - 1][4]
        guess = comp(D, comp(out0, comp(inv(lo0), lo)))
        jm = math.hypot(nd[0] - guess[0], nd[1] - guess[1]); jd = abs(wrap(nd[2] - guess[2]))
        rej = cv and (jm > P['gate_m'] or jd > P['gate_deg'])
        hist = (hist + [sc])[-50:]; med = sorted(hist)[len(hist) // 2]
        if verify is not None:
            verify['n'] += 1; verify['fail'] += rej; resid_after.append(jm)
            if verify['fail'] >= P['verify_fail']:
                D = comp(inv(verify['C']), D); disabled = True
                ev.append((t, 'REVERT', verify['C'])); verify = None; run = []; cnt = 0
                continue
            if verify['n'] >= P['verify']:
                ev[-1] = ev[-1] + (f"验证通过：之后{P['verify']}帧残差中位 {sorted(resid_after[-P['verify']:])[P['verify']//2]:.3f} m",)
                verify = None
        if not rej:
            if jm < P['reset_m']: run = []; cnt = 0
            continue
        C = comp(nd, inv(guess)); run = (run + [(C, jm, jd, lo, sc)])[-P['K']:]; cnt += 1
        if cnt >= P['hint'] and hint is None: hint = t
        if disabled or len(run) < P['K'] or verify is not None: continue
        mx = sum(r[0][0] for r in run) / len(run); my = sum(r[0][1] for r in run) / len(run)
        myaw = math.degrees(math.atan2(sum(math.sin(math.radians(r[0][2])) for r in run), sum(math.cos(math.radians(r[0][2])) for r in run)))
        sm = max(math.hypot(r[0][0] - mx, r[0][1] - my) for r in run); sd = max(abs(wrap(r[0][2] - myaw)) for r in run)
        tr = math.hypot(run[-1][3][0] - run[0][3][0], run[-1][3][1] - run[0][3][1])
        mm = sum(r[1] for r in run) / len(run); md = sum(r[2] for r in run) / len(run)
        if min(r[4] for r in run) >= max(P['min_score'], P['score_ratio'] * med) and sm <= P['cons_m'] and sd <= P['cons_deg'] \
                and tr >= P['min_travel'] and mm <= P['max_m'] and md <= P['max_deg']:
            Cfull = comp(nd, inv(guess))
            D = comp(Cfull, D)
            ev.append((t, 'RELOCK', Cfull, mm, md, tr)); verify = dict(n=0, fail=0, C=Cfull); run = []; cnt = 0
    name = f.split('/')[-1][-24:]
    rel = [e for e in ev if e[1] == 'RELOCK']; rev = [e for e in ev if e[1] == 'REVERT']
    print(f"== {name} 帧{len(fr)} 解锁{len(rel)} 撤回{len(rev)} 停用{'是' if disabled else '否'} 首次需重定位提示 {hint or '-'}")
    for e in ev[:6]:
        if e[1] == 'RELOCK':
            print(f"   {e[0]} 解锁 修正 {e[3]:.2f} m / {e[4]:.1f}°  净位移 {e[5]:.2f} m  {e[6] if len(e) > 6 else '（验证未完成）'}")
        else:
            print(f"   {e[0]} 撤回（采信后又连续被拒），停用自动解除")
