#!/usr/bin/env python3
"""Draws the RflyTools-for-macOS app icons (original artwork) as 1024 px PNGs.

Uses numpy + OpenCV from Python38Env. Shapes are drawn at 2x and downsampled for anti-aliasing.
Usage: python make_icons.py OUT_DIR
"""
import math, os, sys

import cv2
import numpy as np

S = 2048  # drawing resolution
OUT = 1024


def canvas():
    return np.zeros((S, S, 4), np.uint8)


def squircle_mask(margin=100, radius=370):
    m = np.zeros((S, S), np.uint8)
    a, b = margin * 2, S - margin * 2
    r = radius
    cv2.rectangle(m, (a + r, a), (b - r, b), 255, -1)
    cv2.rectangle(m, (a, a + r), (b, b - r), 255, -1)
    for cx, cy in [(a + r, a + r), (b - r, a + r), (a + r, b - r), (b - r, b - r)]:
        cv2.circle(m, (cx, cy), r, 255, -1, cv2.LINE_AA)
    return m


def gradient(top, bottom):
    t = np.linspace(0, 1, S)[:, None, None]
    g = (1 - t) * np.array(top[::-1], float) + t * np.array(bottom[::-1], float)  # RGB in, BGR out
    return np.repeat(g, S, axis=1)


def base(top, bottom):
    img = canvas()
    img[..., :3] = gradient(top, bottom).astype(np.uint8)
    img[..., 3] = squircle_mask()
    return img


def bgr(rgb):
    return (rgb[2], rgb[1], rgb[0], 255)


def quad(img, cx, cy, size, color, prop_color=None, thick=None):
    """Quadcopter, top view, X configuration."""
    thick = thick or int(size * 0.09)
    prop_color = prop_color or color
    d = size * 0.62
    for sx, sy in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
        px, py = int(cx + sx * d), int(cy + sy * d)
        cv2.line(img, (cx, cy), (px, py), bgr(color), thick, cv2.LINE_AA)
        cv2.circle(img, (px, py), int(size * 0.36), bgr(prop_color), int(thick * 0.8), cv2.LINE_AA)
        cv2.circle(img, (px, py), int(thick * 0.75), bgr(color), -1, cv2.LINE_AA)
    cv2.circle(img, (cx, cy), int(size * 0.2), bgr(color), -1, cv2.LINE_AA)


def text(img, s, cx, cy, scale, color, thick):
    f = cv2.FONT_HERSHEY_DUPLEX
    (w, h), _ = cv2.getTextSize(s, f, scale, thick)
    cv2.putText(img, s, (int(cx - w / 2), int(cy + h / 2)), f, scale, bgr(color), thick, cv2.LINE_AA)


def clip(img):
    """Keep drawing inside the squircle."""
    m = squircle_mask()
    img[..., 3] = np.minimum(img[..., 3], m)
    return img


def save(img, path):
    clip(img)
    small = cv2.resize(img, (OUT, OUT), interpolation=cv2.INTER_AREA)
    cv2.imwrite(path, small)


# ---------------------------------------------------------------------------- icons

def icon_rflysim3d():
    img = base((70, 140, 230), (150, 205, 255))
    # Ground plane with perspective grid.
    hz = 1150
    ground = np.array([[200, hz], [S - 200, hz], [S - 200, S - 200], [200, S - 200]], np.int32)
    cv2.fillPoly(img, [ground], bgr((70, 150, 70)), cv2.LINE_AA)
    vx = S // 2
    for k in range(-6, 7):
        cv2.line(img, (vx + k * 60, hz), (vx + k * 520, S), bgr((120, 190, 110)), 10, cv2.LINE_AA)
    y = hz
    step = 40
    while y < S:
        cv2.line(img, (0, int(y)), (S, int(y)), bgr((120, 190, 110)), 10, cv2.LINE_AA)
        step *= 1.45
        y += step
    # Drone flying above, with a shadow.
    cv2.ellipse(img, (1030, 1560), (260, 70), 0, 0, 360, (40, 90, 40, 255), -1, cv2.LINE_AA)
    quad(img, 1024, 760, 440, (255, 255, 255), (230, 240, 255))
    return img


def icon_coptersim():
    img = base((250, 252, 252), (215, 235, 235))
    quad(img, 1024, 1024, 700, (20, 190, 175), (20, 190, 175), 70)
    return img


def chevrons(top, bottom, accent, label):
    img = base(top, bottom)
    for k, y in enumerate([620, 900]):
        pts = np.array([[440, y + 300], [1024, y], [1608, y + 300]], np.int32)
        cv2.polylines(img, [pts], False, bgr(accent), 150, cv2.LINE_AA)
    text(img, label, 1024, 1560, 8.5, (255, 255, 255), 30)
    return img


def icon_sitl():
    return chevrons((105, 115, 245), (70, 70, 200), (215, 220, 255), "SITL")


def icon_hitl():
    img = chevrons((245, 140, 60), (200, 85, 40), (255, 225, 195), "HITL")
    # A small autopilot board between the chevrons.
    cv2.rectangle(img, (820, 1180), (1228, 1330), bgr((40, 40, 45)), -1, cv2.LINE_AA)
    for x in range(850, 1210, 60):
        cv2.rectangle(img, (x, 1150), (x + 26, 1180), bgr((230, 200, 90)), -1)
    return img


def icon_python():
    img = base((45, 50, 60), (25, 28, 34))
    cv2.rectangle(img, (300, 330), (1748, 460), bgr((70, 75, 90)), -1)
    for i, c in enumerate([(240, 90, 80), (240, 190, 60), (90, 200, 90)]):
        cv2.circle(img, (400 + i * 110, 395), 36, bgr(c), -1, cv2.LINE_AA)
    pts = np.array([[420, 760], [700, 960], [420, 1160]], np.int32)
    cv2.polylines(img, [pts], False, bgr((80, 220, 120)), 90, cv2.LINE_AA)
    cv2.rectangle(img, (800, 1110), (1250, 1190), bgr((80, 220, 120)), -1)
    text(img, "py", 1024, 1560, 9.0, (80, 170, 255), 34)
    return img


def icon_flightgear():
    img = base((40, 120, 220), (200, 225, 245))
    # Runway in perspective.
    rw = np.array([[930, 1050], [1118, 1050], [1500, S - 250], [548, S - 250]], np.int32)
    cv2.fillPoly(img, [np.array([[200, 1050], [S - 200, 1050], [S, S], [0, S]], np.int32)], bgr((95, 140, 80)))
    cv2.fillPoly(img, [rw], bgr((70, 70, 75)), cv2.LINE_AA)
    for k in range(6):
        y0 = 1080 + k * k * 22 + k * 60
        cv2.line(img, (1024, int(y0)), (1024, int(y0 + 30 + k * 18)), bgr((250, 250, 250)), 18 + k * 4)
    quad(img, 1024, 620, 380, (255, 255, 255))
    return img


def icon_3ddisplay():
    img = base((235, 240, 245), (200, 210, 220))
    cv2.rectangle(img, (330, 430), (1718, 1360), bgr((40, 45, 55)), -1, cv2.LINE_AA)
    sky = gradient((90, 160, 240), (170, 215, 255))[480:1310, 380:1668]
    img[480:1310, 380:1668, :3] = sky.astype(np.uint8)
    cv2.rectangle(img, (380, 1050), (1668, 1310), bgr((80, 160, 80)), -1)
    quad(img, 1024, 800, 260, (255, 255, 255))
    cv2.rectangle(img, (920, 1360), (1128, 1560), bgr((150, 155, 165)), -1)
    cv2.rectangle(img, (700, 1560), (1348, 1630), bgr((150, 155, 165)), -1, cv2.LINE_AA)
    return img


def icon_game():
    img = base((30, 25, 60), (90, 40, 120))
    # Race gate ring with a drone flying through it.
    cv2.ellipse(img, (1024, 1000), (620, 560), 0, 0, 360, bgr((255, 120, 60)), 110, cv2.LINE_AA)
    cv2.ellipse(img, (1024, 1000), (620, 560), 0, 200, 340, bgr((255, 200, 90)), 40, cv2.LINE_AA)
    quad(img, 1024, 1000, 330, (255, 255, 255), (200, 230, 255))
    # Motion streaks.
    for k, y in enumerate([900, 1000, 1100]):
        cv2.line(img, (260, y), (560 - k * 40, y), bgr((120, 220, 255)), 34, cv2.LINE_AA)
    return img



def icon_import():
    img = base((70, 80, 95), (35, 40, 50))
    # Installer disc with a down arrow into a tray: "import data from your RflySim ISO".
    cv2.circle(img, (1024, 760), 430, bgr((200, 205, 215)), -1, cv2.LINE_AA)
    cv2.circle(img, (1024, 760), 430, bgr((240, 242, 248)), 26, cv2.LINE_AA)
    cv2.circle(img, (1024, 760), 110, bgr((70, 80, 95)), -1, cv2.LINE_AA)
    cv2.ellipse(img, (1024, 760), (300, 300), 0, 200, 260, bgr((250, 250, 255)), 40, cv2.LINE_AA)
    arrow = np.array([[904, 980], [1144, 980], [1144, 1240], [1294, 1240], [1024, 1500], [754, 1240], [904, 1240]], np.int32)
    cv2.fillPoly(img, [arrow], bgr((90, 210, 130)), cv2.LINE_AA)
    cv2.polylines(img, [np.array([[420, 1380], [420, 1680], [1628, 1680], [1628, 1380]], np.int32)], False, bgr((240, 242, 248)), 90, cv2.LINE_AA)
    return img

ICONS = {
    "ImportData": icon_import,
    "RflyGame": icon_game,
    "RflySim3D": icon_rflysim3d,
    "CopterSim": icon_coptersim,
    "SITLRun": icon_sitl,
    "HITLRun": icon_hitl,
    "Python38Env": icon_python,
    "FlightGear-F450": icon_flightgear,
    "3DDisplay": icon_3ddisplay,
}

if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "build/icons"
    os.makedirs(out, exist_ok=True)
    for name, fn in ICONS.items():
        save(fn(), os.path.join(out, name + ".png"))
        print("icon:", name)
