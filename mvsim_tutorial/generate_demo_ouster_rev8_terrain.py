#!/usr/bin/env python3
"""Generates the terrain images of demo_ouster_rev8.world.xml:

- demo_ouster_rev8_elevation.png: grayscale height map (1 px = 1 m), with a
  flat village in the middle, hills around it, and the ground under the roads
  smoothed out.
- demo_ouster_rev8_texture.jpg: color texture of the whole terrain: grass,
  bare soil on steep slopes, asphalt roads with lane markings, sidewalks, a
  paved square and dirt footpaths.

Both images are indexed as image(row, col) = world(x, y), with the minimum
corner at (-L/2, -L/2). The road, path and square geometry here must match
the objects placed in demo_ouster_rev8.world.xml.

Requires numpy and Pillow. Run it from any directory:

    python3 generate_demo_ouster_rev8_terrain.py
"""

import math
import os

import numpy as np
from PIL import Image

L = 160.0  # terrain side [m]
ELEV_RES = 1.0  # height map resolution [m/px]
TEX_PX = 2048  # texture side [px]
MAX_Z = 10.0  # must match <elevation_image_max_z> in the world file
SEED = 42

ROAD_HALF_WIDTH = 3.5  # two lanes of 3.5 m
SIDEWALK_WIDTH = 2.5
FOOTPATH_HALF_WIDTH = 0.8


def rounded_rect(x0, y0, x1, y1, r, step=1.0):
    """Closed polyline of a rounded rectangle, counterclockwise."""
    pts = []
    corners = [(x1 - r, y0 + r, -90), (x1 - r, y1 - r, 0),
               (x0 + r, y1 - r, 90), (x0 + r, y0 + r, 180)]
    for cx, cy, a0 in corners:
        n = max(2, int(0.5 * math.pi * r / step))
        for k in range(n + 1):
            a = math.radians(a0 + 90.0 * k / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    pts.append(pts[0])
    return pts


def circle(cx, cy, r, step=1.0):
    n = max(8, int(2 * math.pi * r / step))
    return [(cx + r * math.cos(2 * math.pi * k / n),
             cy + r * math.sin(2 * math.pi * k / n)) for k in range(n + 1)]


# Roads (center lines):
RING_ROAD = rounded_rect(-45, -35, 45, 35, 15)
MAIN_STREET = [(-45, 0), (45, 0)]
HILL_ROAD = [(45, 14), (55, 15), (63, 19), (68, 26), (70, 35), (67, 43),
             (63, 49), (60, 53)]
ROADS = [RING_ROAD, MAIN_STREET, HILL_ROAD]

# Paved square, north of the main street: [x0, x1] x [y0, y1]
SQUARE = (-12.0, 12.0, 8.0, 24.0)

# Dirt footpaths: a loop on the south-west hill, and one from the ring road
# up to it.
HILL_LOOP = circle(-58, -50, 9)
FOOTPATHS = [HILL_LOOP, [(-38, -32), (-44, -38), (-50, -44)]]

# Flat hilltop at the end of the hill road:
HILLTOP = (60.0, 54.0, 8.0)  # x, y, radius


def segment_distance(px, py, poly):
    """Distance from points (px, py) to a polyline, and arc length of the
    closest point along it."""
    best = np.full(px.shape, np.inf)
    best_s = np.zeros(px.shape)
    s0 = 0.0
    for (ax, ay), (bx, by) in zip(poly[:-1], poly[1:]):
        dx = bx - ax
        dy = by - ay
        seg_len2 = dx * dx + dy * dy
        if seg_len2 < 1e-12:
            continue
        t = np.clip(((px - ax) * dx + (py - ay) * dy) / seg_len2, 0.0, 1.0)
        d = np.hypot(px - (ax + t * dx), py - (ay + t * dy))
        closer = d < best
        best = np.where(closer, d, best)
        best_s = np.where(closer, s0 + t * math.sqrt(seg_len2), best_s)
        s0 += math.sqrt(seg_len2)
    return best, best_s


def smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3 - 2 * t)


def grid(n, res):
    c = -0.5 * L + res * np.arange(n)
    return np.meshgrid(c, c, indexing="ij")  # [ix, iy]


def value_noise(rng, shape, cells):
    """Smooth isotropic random noise in [0,1], with features about
    1/`cells` of the image side (Gaussian-filtered white noise)."""
    white = rng.standard_normal(shape)
    fx = np.fft.fftfreq(shape[0])[:, None]
    fy = np.fft.fftfreq(shape[1])[None, :]
    sigma_f = cells / (2.0 * max(shape))
    filt = np.exp(-(fx * fx + fy * fy) / (2 * sigma_f * sigma_f))
    n = np.real(np.fft.ifft2(np.fft.fft2(white) * filt))
    n = (n - n.mean()) / (n.std() + 1e-12)
    return np.clip(0.5 + 0.18 * n, 0.0, 1.0)


def box_blur(z, radius):
    k = 2 * radius + 1
    p = np.pad(z, radius, mode="edge")
    c = np.cumsum(np.cumsum(p, axis=0), axis=1)
    c = np.pad(c, ((1, 0), (1, 0)))
    return (c[k:, k:] - c[:-k, k:] - c[k:, :-k] + c[:-k, :-k]) / (k * k)


def road_distance(x, y):
    d = np.full(x.shape, np.inf)
    for road in ROADS:
        d = np.minimum(d, segment_distance(x, y, road)[0])
    return d


def terrain_height(x, y, rng):
    hills = [(62, 55, 9.0, 16), (-60, -50, 7.0, 14), (-62, 58, 6.0, 13),
             (8, -74, 4.0, 14), (-78, 5, 3.0, 12), (75, -55, 5.0, 15)]
    z = np.zeros(x.shape)
    for cx, cy, h, s in hills:
        z += h * np.exp(-((x - cx) ** 2 + (y - cy) ** 2) / (2 * s * s))
    z += 1.2 * value_noise(rng, x.shape, 8) + 0.4 * value_noise(rng, x.shape, 24)

    # Flat village, blending into the hills:
    rho = np.maximum(np.abs(x) / 38.0, np.abs(y) / 28.0)
    z *= smoothstep(1.0, 1.35, rho)

    # Flat lookout on the hilltop:
    hx, hy, hr = HILLTOP
    d_top = np.hypot(x - hx, y - hy)
    w_top = 1.0 - smoothstep(hr, hr + 6.0, d_top)
    z_top = z[np.unravel_index(np.argmin(d_top), d_top.shape)]
    z = w_top * z_top + (1 - w_top) * z

    # Smooth ground under the roads (no cross slope bumps):
    d = road_distance(x, y)
    w = 1.0 - smoothstep(ROAD_HALF_WIDTH + 1.0, ROAD_HALF_WIDTH + 6.0, d)
    z = w * box_blur(z, 5) + (1 - w) * z
    return z


def main():
    out_dir = os.path.dirname(os.path.abspath(__file__))
    rng = np.random.default_rng(SEED)

    # ---- Height map ----
    n = int(round(L / ELEV_RES)) + 1
    x, y = grid(n, ELEV_RES)
    z = terrain_height(x, y, rng)
    z -= z.min()
    # Use the full 8-bit range: the simulator maps [min,max] -> [0, MAX_Z].
    print(f"Max terrain height: {z.max():.2f} m (scaled to {MAX_Z} m)")
    img = np.round(255.0 * z / z.max()).astype(np.uint8)
    Image.fromarray(img).save(os.path.join(out_dir, "demo_ouster_rev8_elevation.png"))
    z_m = MAX_Z * img.astype(float) / 255.0

    # ---- Texture ----
    res = L / TEX_PX
    tx, ty = np.meshgrid(-0.5 * L + res * (0.5 + np.arange(TEX_PX)),
                         -0.5 * L + res * (0.5 + np.arange(TEX_PX)), indexing="ij")
    trng = np.random.default_rng(SEED + 1)

    def noise(cells, amp):
        return amp * (value_noise(trng, tx.shape, cells) - 0.5)

    fine = trng.random(tx.shape) - 0.5

    # Grass, with patches of dry grass:
    dry = smoothstep(0.45, 0.75, value_noise(trng, tx.shape, 20))[..., None]
    grass = np.array([70, 115, 45]) * (1 - dry) + np.array([140, 135, 70]) * dry
    tex = grass * (1 + noise(60, 0.35)[..., None] + 0.25 * fine[..., None])

    # Bare soil on steep slopes (slope from the height map):
    gx, gy = np.gradient(z_m, ELEV_RES)
    slope = np.hypot(gx, gy)
    ii = np.clip(((tx + 0.5 * L) / ELEV_RES).round().astype(int), 0, n - 1)
    jj = np.clip(((ty + 0.5 * L) / ELEV_RES).round().astype(int), 0, n - 1)
    soil_w = smoothstep(0.25, 0.45, slope[ii, jj] + noise(30, 0.2))[..., None]
    soil = np.array([125, 100, 70]) * (1 + noise(80, 0.3)[..., None] + 0.2 * fine[..., None])
    tex = tex * (1 - soil_w) + soil * soil_w

    def paint(mask, color, rel_noise=0.15):
        nonlocal tex
        c = np.array(color, float) * (1 + rel_noise * fine[..., None] + noise(100, 0.1)[..., None])
        tex = np.where(mask[..., None], c, tex)

    # Dirt footpaths:
    for p in FOOTPATHS:
        d, _ = segment_distance(tx, ty, p)
        paint(d < FOOTPATH_HALF_WIDTH + noise(200, 0.4), (135, 110, 80), 0.25)

    # Sidewalks along the main street (inside the village):
    in_village = np.abs(tx) < 31
    sw = in_village & (np.abs(ty) > ROAD_HALF_WIDTH) & (np.abs(ty) < ROAD_HALF_WIDTH + SIDEWALK_WIDTH)
    paint(sw, (165, 162, 155), 0.08)
    joints = sw & ((np.mod(tx, 1.25) < res) | (np.abs(np.abs(ty) - ROAD_HALF_WIDTH - SIDEWALK_WIDTH + 0.05) < res))
    paint(joints, (115, 112, 108), 0.05)

    # Paved square, with tile joints and a darker border:
    x0, x1, y0, y1 = SQUARE
    sq = (tx > x0) & (tx < x1) & (ty > y0) & (ty < y1)
    paint(sq, (190, 175, 150), 0.1)
    paint(sq & ((np.mod(tx, 1.0) < res) | (np.mod(ty, 1.0) < res)), (140, 128, 110), 0.05)
    border = sq & ((tx < x0 + 0.6) | (tx > x1 - 0.6) | (ty < y0 + 0.6) | (ty > y1 - 0.6))
    paint(border, (120, 95, 80), 0.08)

    # Roads: asphalt, solid white edge lines, dashed white center line.
    dist = [segment_distance(tx, ty, road) for road in ROADS]
    on_road = np.zeros(tx.shape, bool)
    for d, _ in dist:
        on_road |= d < ROAD_HALF_WIDTH
    paint(on_road, (62, 62, 64), 0.2)
    for k, (d, s) in enumerate(dist):
        # No markings across junctions with the other roads:
        others = np.zeros(tx.shape, bool)
        for k2, (d2, _) in enumerate(dist):
            if k2 != k:
                others |= d2 < ROAD_HALF_WIDTH + 0.5
        edge = np.abs(d - (ROAD_HALF_WIDTH - 0.35)) < 0.08
        center = (d < 0.07) & (np.mod(s, 9.0) < 3.0)
        paint((edge | center) & ~others, (225, 225, 220), 0.05)

    tex = np.clip(tex, 0, 255).astype(np.uint8)
    Image.fromarray(tex).save(os.path.join(out_dir, "demo_ouster_rev8_texture.jpg"), quality=85)
    print("Done.")


if __name__ == "__main__":
    main()
