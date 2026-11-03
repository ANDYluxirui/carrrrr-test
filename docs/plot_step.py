#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把串口抓到的 step.csv 画成阶跃响应曲线，并算出三个评分指标：
  上升时间 / 稳态误差 / 抖动

只依赖 Pillow（pip install pillow），不需要 matplotlib。

用法：
  python plot_step.py step.csv
  python plot_step.py step.csv -o my_plot.png

CSV 格式（car_pid 默认输出）：
  # t,target,vL,vR,uL,uR
  0,0,0,0,0,0
  50,300,12,9,340,330
"""
import argparse
import os
import sys

from PIL import Image, ImageDraw, ImageFont

FONT_CANDIDATES = [
    "C:/Windows/Fonts/msyh.ttc",
    "C:/Windows/Fonts/simhei.ttf",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/System/Library/Fonts/PingFang.ttc",
]


def font(size):
    for p in FONT_CANDIDATES:
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                pass
    return ImageFont.load_default()


def load(path):
    t, tgt, vl, vr = [], [], [], []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for row in f:
            row = row.strip()
            if not row or row.startswith("#"):
                continue
            parts = row.replace(";", ",").split(",")
            if len(parts) < 4:
                continue
            try:
                vals = [float(p) for p in parts[:4]]
            except ValueError:
                continue
            t.append(vals[0] / 1000.0)
            tgt.append(vals[1])
            vl.append(vals[2])
            vr.append(vals[3])
    if len(t) < 10:
        sys.exit("没读到有效数据 —— 检查 CSV 格式和串口波特率（115200）")
    return t, tgt, vl, vr


def analyze(t, tgt, v):
    jumps = [abs(tgt[i + 1] - tgt[i]) for i in range(len(tgt) - 1)]
    if max(jumps) < 1e-6:
        return None
    i = jumps.index(max(jumps)) + 1
    target = tgt[i]

    # 只分析这一段台阶：找到下一次跳变就截断
    thr = abs(target) * 0.2 + 1.0
    j = len(tgt)
    for k in range(i + 1, len(tgt)):
        if abs(tgt[k] - tgt[k - 1]) > thr:
            j = k
            break

    st = [x - t[i] for x in t[i:j]]
    sv = v[i:j]
    if len(sv) < 10:
        return None

    end = st[-1]
    ss = [sv[k] for k in range(len(st)) if st[k] > end * 0.7]
    steady = sum(ss) / len(ss) if ss else float("nan")
    err = abs(steady - target) / abs(target) * 100 if target else float("nan")
    jit = (sum((x - steady) ** 2 for x in ss) / len(ss)) ** 0.5 if ss else float("nan")

    lo = sv[0] + 0.1 * (target - sv[0])
    hi = sv[0] + 0.9 * (target - sv[0])
    rise = None
    for k in range(len(sv)):
        if (hi > lo and sv[k] >= hi) or (hi < lo and sv[k] <= hi):
            rise = st[k]
            break
    return dict(i=i, st=st, sv=sv, target=target, rise=rise,
                steady=steady, err=err, jit=jit)


def draw(t, tgt, vl, vr, r, out):
    W, H = 1280, 700
    L, R, T, B = 90, 40, 60, 90
    img = Image.new("RGB", (W, H), "white")
    d = ImageDraw.Draw(img)
    f_t, f_a, f_s, f_m = font(26), font(15), font(13), font(14)

    pw, ph = W - L - R, H - T - B
    xs = [min(t), max(t)]
    ys = [min(min(vl), min(vr), min(tgt)), max(max(vl), max(vr), max(tgt))]
    pad = (ys[1] - ys[0]) * 0.12 + 1
    ys = [ys[0] - pad, ys[1] + pad]
    if ys[1] - ys[0] < 1:
        ys = [-1, 1]

    def PX(x):
        return L + (x - xs[0]) / (xs[1] - xs[0] + 1e-9) * pw

    def PY(y):
        return T + ph - (y - ys[0]) / (ys[1] - ys[0] + 1e-9) * ph

    d.text((L, 16), "PID 速度环 · 阶跃响应", font=f_t, fill=(25, 55, 95))

    # 网格
    for k in range(6):
        y = ys[0] + (ys[1] - ys[0]) * k / 5
        d.line([(L, PY(y)), (L + pw, PY(y))], fill=(232, 235, 238))
        d.text((L - 8 - d.textlength(f"{y:.0f}", font=f_s), PY(y) - 7), f"{y:.0f}",
               font=f_s, fill=(120, 120, 120))
    for k in range(6):
        x = xs[0] + (xs[1] - xs[0]) * k / 5
        d.line([(PX(x), T), (PX(x), T + ph)], fill=(232, 235, 238))
        d.text((PX(x) - 12, T + ph + 8), f"{x:.1f}", font=f_s, fill=(120, 120, 120))
    d.rectangle([L, T, L + pw, T + ph], outline=(150, 150, 150), width=2)
    d.text((L + pw - 60, T + ph + 34), "时间 / s", font=f_a, fill=(80, 80, 80))
    d.text((12, T + ph // 2 - 30), "mm/s", font=f_a, fill=(80, 80, 80))

    # 上升段底色
    if r and r["rise"] is not None:
        t0 = t[r["i"]]
        d.rectangle([PX(t0), T, PX(t0 + r["rise"]), T + ph], fill=(253, 240, 214))

    # 目标
    pts = [(PX(t[k]), PY(tgt[k])) for k in range(len(t))]
    for a, b in zip(pts[:-1], pts[1:]):
        d.line([a, b], fill=(150, 150, 150), width=2)
    # 实测
    d.line([(PX(t[k]), PY(vl[k])) for k in range(len(t))], fill=(192, 57, 43), width=3)
    d.line([(PX(t[k]), PY(vr[k])) for k in range(len(t))], fill=(41, 128, 185), width=2)

    # 图例
    lx, ly = L + pw - 250, T + 14
    d.line([(lx, ly + 7), (lx + 34, ly + 7)], fill=(150, 150, 150), width=2)
    d.text((lx + 42, ly), "目标速度", font=f_a, fill=(70, 70, 70))
    d.line([(lx, ly + 30), (lx + 34, ly + 30)], fill=(192, 57, 43), width=3)
    d.text((lx + 42, ly + 23), "左轮实测", font=f_a, fill=(70, 70, 70))
    d.line([(lx, ly + 53), (lx + 34, ly + 53)], fill=(41, 128, 185), width=2)
    d.text((lx + 42, ly + 46), "右轮实测", font=f_a, fill=(70, 70, 70))

    if r:
        rise_txt = f"{r['rise']*1000:.0f} ms" if r["rise"] is not None else "未到 90%"
        ok1 = r["rise"] is not None and r["rise"] < 0.25
        ok2 = r["err"] < 3.0
        ok3 = r["jit"] < 15.0
        lines = [
            (f"上升时间 : {rise_txt}", ok1),
            (f"稳态误差 : {r['err']:.1f} %", ok2),
            (f"抖动     : {r['jit']:.1f} mm/s", ok3),
            ("【全部达标】" if (ok1 and ok2 and ok3) else "【还有指标没达标】", ok1 and ok2 and ok3),
        ]
        bx, by = L + 16, T + ph - 118
        d.rectangle([bx, by, bx + 300, by + 100], fill=(255, 252, 240), outline=(220, 200, 150))
        for k, (txt, ok) in enumerate(lines):
            d.text((bx + 12, by + 10 + k * 23), txt, font=f_m,
                   fill=(30, 120, 60) if ok else (190, 90, 30))

    img.save(out)
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("-o", "--out", default="step_response.png")
    a = ap.parse_args()

    t, tgt, vl, vr = load(a.csv)
    r = analyze(t, tgt, vl)
    if r is None:
        sys.exit("没找到目标跳变 —— 确认 DEMO_MODE=1（阶跃模式）")
    draw(t, tgt, vl, vr, r, a.out)

    print(f"已生成 {a.out}")
    print(f"  上升时间 : {r['rise']*1000:.0f} ms" if r['rise'] is not None else "  上升时间 : 未到 90%")
    print(f"  稳态值   : {r['steady']:.1f} mm/s  (目标 {r['target']:.0f})")
    print(f"  稳态误差 : {r['err']:.1f} %")
    print(f"  抖动     : {r['jit']:.1f} mm/s")


if __name__ == "__main__":
    main()
