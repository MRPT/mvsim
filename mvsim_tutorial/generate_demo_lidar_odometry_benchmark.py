#!/usr/bin/env python3
"""Generates demo_lidar_odometry_benchmark.{world.xml,trajectory.txt}: a
procedurally-generated scenario for mvsim-dataset-gen (see
docs/mvsim-dataset-gen.rst) intended as a synthetic benchmark for testing
LiDAR odometry pipelines.

- Terrain: gently rolling hills (a 3-term sine/cosine sum), as an inline
  <elevation_data_matrix>.
- Path: a curvy, non-looping (x strictly increasing) >200m trajectory, as a
  2D "t x y" waypoints file for mvsim-dataset-gen's terrain-following input
  mode.
- Obstacles: cylindrical "pillars" and box "crates" scattered near the path
  (within the LiDAR's range) but never blocking it.

Run this script, then:

    mvsim-dataset-gen mvsim_tutorial/demo_lidar_odometry_benchmark.world.xml \\
        --trajectory mvsim_tutorial/demo_lidar_odometry_benchmark.trajectory.txt \\
        -o /tmp/lidar_odom_benchmark.rawlog --noiseless

Re-run this generator (optionally with a different --seed) to regenerate
the two checked-in files if this script itself changes.
"""

import argparse
import math
import os
import random


def terrain_z(x: float, y: float) -> float:
	"""Gently rolling terrain: a few sine/cosine terms, chosen so the worst-
	case slope stays under ~20 degrees (the small-angle terrain-following
	approximation in TrajectorySource::load2DWithTerrain() is only accurate
	for gentle terrain, not cliffs or stairs)."""
	return (
		3.0 * math.sin(2 * math.pi * x / 100.0)
		+ 2.0 * math.cos(2 * math.pi * y / 120.0)
		+ 1.2 * math.sin(2 * math.pi * (x + y) / 55.0)
	)


def path_y(x: float) -> float:
	"""Curvy, non-looping path: y as a function of x (strictly monotonic x
	trivially guarantees no self-intersection), two sine harmonics for a
	varying-curvature S-curve."""
	return 14.0 * math.sin(2 * math.pi * x / 70.0) + 6.0 * math.sin(2 * math.pi * x / 24.0)


def build_elevation_matrix(map_half: float, resolution: float) -> tuple[list[list[float]], int]:
	n = int(round(2 * map_half / resolution)) + 1  # rows = cols
	rows = []
	for ix in range(n):
		x = -map_half + ix * resolution
		rows.append([terrain_z(x, -map_half + iy * resolution) for iy in range(n)])
	return rows, n


def build_waypoints(x0: float, x1: float, step: float, speed: float) -> list[tuple[float, float, float]]:
	"""Resamples the (x, path_y(x)) curve at even arc-length spacing."""
	dense_n = 20000
	dense_pts = [(x0 + (x1 - x0) * i / dense_n, 0.0) for i in range(dense_n + 1)]
	dense_pts = [(x, path_y(x)) for x, _ in dense_pts]

	cum_len = [0.0]
	for i in range(1, len(dense_pts)):
		x0_, y0_ = dense_pts[i - 1]
		x1_, y1_ = dense_pts[i]
		cum_len.append(cum_len[-1] + math.hypot(x1_ - x0_, y1_ - y0_))
	total_len = cum_len[-1]

	waypoints = []
	target = 0.0
	j = 0
	while target <= total_len:
		while j < len(cum_len) - 1 and cum_len[j + 1] < target:
			j += 1
		if j >= len(dense_pts) - 1:
			x, y = dense_pts[-1]
		else:
			l0, l1 = cum_len[j], cum_len[j + 1]
			frac = (target - l0) / (l1 - l0) if l1 > l0 else 0.0
			x0_, y0_ = dense_pts[j]
			x1_, y1_ = dense_pts[j + 1]
			x = x0_ + frac * (x1_ - x0_)
			y = y0_ + frac * (y1_ - y0_)
		waypoints.append((target / speed, x, y))
		target += step

	print(f"Path length: {total_len:.1f} m, duration: {waypoints[-1][0]:.1f} s, {len(waypoints)} waypoints")
	return waypoints


def place_blocks(waypoints, map_half: float, target_count: int, rng: random.Random):
	"""Scatters cylinder/box obstacles near the path, clear of it and of
	each other."""
	path_xy = [(x, y) for (_, x, y) in waypoints]

	def min_dist_to_path(px, py):
		return min(math.hypot(px - x, py - y) for (x, y) in path_xy)

	blocks = []
	attempts = 0
	while len(blocks) < target_count and attempts < 5000:
		attempts += 1
		s_idx = rng.randint(0, len(waypoints) - 1)
		_, px, py = waypoints[s_idx]
		i0 = max(0, s_idx - 2)
		i1 = min(len(waypoints) - 1, s_idx + 2)
		_, tx0, ty0 = waypoints[i0]
		_, tx1, ty1 = waypoints[i1]
		dx, dy = tx1 - tx0, ty1 - ty0
		norm = math.hypot(dx, dy)
		if norm < 1e-6:
			continue
		dx, dy = dx / norm, dy / norm
		nx, ny = -dy, dx  # perpendicular to the path
		offset = rng.choice([-1, 1]) * rng.uniform(6.0, 40.0)
		bx, by = px + nx * offset, py + ny * offset

		if abs(bx) > map_half - 5 or abs(by) > map_half - 5:
			continue

		kind = rng.choice(["cylinder", "cylinder", "box"])
		if kind == "cylinder":
			radius = round(rng.uniform(0.3, 1.2), 2)
			clearance = radius + 3.5
		else:
			lx = round(rng.uniform(1.0, 3.0), 2)
			ly = round(rng.uniform(1.0, 3.0), 2)
			clearance = max(lx, ly) * 0.7 + 3.5

		if min_dist_to_path(bx, by) < clearance:
			continue
		if any(math.hypot(bx - b["x"], by - b["y"]) < 3.0 for b in blocks):
			continue

		z = terrain_z(bx, by)
		if kind == "cylinder":
			blocks.append(
				{
					"id": len(blocks),
					"kind": "cylinder",
					"x": bx,
					"y": by,
					"z": z,
					"radius": radius,
					"length": round(rng.uniform(2.0, 5.0), 2),
				}
			)
		else:
			blocks.append(
				{
					"id": len(blocks),
					"kind": "box",
					"x": bx,
					"y": by,
					"z": z,
					"lx": lx,
					"ly": ly,
					"lz": round(rng.uniform(1.5, 4.0), 2),
					"yaw": round(rng.uniform(0, 360), 1),
				}
			)

	print(f"Placed {len(blocks)} blocks after {attempts} attempts")
	return blocks


def elevation_matrix_xml(rows: list[list[float]]) -> str:
	# ';' is a row *separator*, not a terminator: a trailing ';' before ']'
	# (with only whitespace in between) makes MRPT's matrix parser see a
	# phantom empty last row and silently fail.
	row_strs = ["\t\t\t" + " ".join(f"{v:.3f}" for v in r) for r in rows]
	return "[\n" + ";\n".join(row_strs) + "\n\t\t]"


def blocks_xml(blocks: list[dict]) -> tuple[str, str]:
	cyl_classes: dict = {}
	box_classes: dict = {}
	instances = []
	for b in blocks:
		name = f"block{b['id']:03d}"
		if b["kind"] == "cylinder":
			cls = f'pillar_r{b["radius"]}_l{b["length"]}'
			cyl_classes[(b["radius"], b["length"])] = True
			instances.append(
				f'\t<block name="{name}" class="{cls}">\n'
				f'\t\t<init_pose3d>{b["x"]:.2f} {b["y"]:.2f} {b["z"]:.3f} 0 0 0</init_pose3d>\n'
				f"\t</block>\n"
			)
		else:
			cls = f'crate_{b["lx"]}x{b["ly"]}x{b["lz"]}'
			box_classes[(b["lx"], b["ly"], b["lz"])] = True
			instances.append(
				f'\t<block name="{name}" class="{cls}">\n'
				f'\t\t<init_pose3d>{b["x"]:.2f} {b["y"]:.2f} {b["z"]:.3f} '
				f'{b["yaw"]} 0 0</init_pose3d>\n'
				f"\t</block>\n"
			)

	classes_lines = []
	for radius, length in cyl_classes:
		classes_lines.append(
			f'\t<block:class name="pillar_r{radius}_l{length}">\n'
			f'\t\t<geometry type="cylinder" radius="{radius}" length="{length}" vertex_count="12" />\n'
			f"\t</block:class>\n"
		)
	for lx, ly, lz in box_classes:
		classes_lines.append(
			f'\t<block:class name="crate_{lx}x{ly}x{lz}">\n'
			f'\t\t<geometry type="box" lx="{lx}" ly="{ly}" lz="{lz}" />\n'
			f"\t</block:class>\n"
		)
	return "".join(classes_lines), "".join(instances)


WORLD_TEMPLATE = """<mvsim_world version="1.0">
\t<!--
\t  LiDAR-odometry benchmark dataset for mvsim-dataset-gen: a >200m curvy
\t  (non-looping) trajectory over gently rolling terrain, with cylindrical
\t  and box obstacles scattered near the path within the LiDAR's 60m range.
\t  Procedurally generated by generate_demo_lidar_odometry_benchmark.py.
\t  2D trajectory + terrain-following input mode.

\t  Usage:
\t    mvsim-dataset-gen mvsim_tutorial/demo_lidar_odometry_benchmark.world.xml \\\\
\t        --trajectory mvsim_tutorial/demo_lidar_odometry_benchmark.trajectory.txt \\\\
\t        -o /tmp/lidar_odom_benchmark.rawlog --noiseless
\t-->

\t<simul_timestep>0</simul_timestep>

\t<!-- ========================
\t\t   Terrain
\t     ======================== -->
\t<element class="elevation_map">
\t\t<resolution>{resolution}</resolution>
\t\t<elevation_data_matrix>
{elevation}
\t\t</elevation_data_matrix>
\t\t<mesh_color>#a0c8a0</mesh_color>
\t</element>

\t<!-- ========================
\t\t   Obstacles
\t     ======================== -->
{block_classes}
{blocks}
\t<!-- ============================= Vehicle ============================= -->
\t<vehicle:class name="sensor_platform">
\t\t<dynamics class="differential">
\t\t\t<l_wheel pos="0.0  0.5" mass="4.0" width="0.20" diameter="0.40" />
\t\t\t<r_wheel pos="0.0 -0.5" mass="4.0" width="0.20" diameter="0.40" />
\t\t\t<chassis mass="15.0" zmin="0.05" zmax="0.6"> </chassis>
\t\t\t<!-- Kinematics/controller are irrelevant: mvsim-dataset-gen never
\t\t\t     steps physics, it only reads the sensor XML parameters and
\t\t\t     the vehicle's <pose_3d> sensor mount offset. -->
\t\t\t<controller class="twist_ideal"> </controller>
\t\t</dynamics>
\t\t<friction class="default"> <mu>0.8</mu> </friction>
\t</vehicle:class>

\t<vehicle name="r1" class="sensor_platform">
\t\t<init_pose>{veh_x:.2f} {veh_y:.2f} 0</init_pose>

\t\t<include file="../definitions/velodyne-vlp16.sensor.xml"
\t\t  sensor_x="0.0" sensor_y="0" sensor_z="0.75" sensor_yaw="0"
\t\t  sensor_name="lidar1"
\t\t  sensor_rpm="600"
\t\t  max_range="{max_range}"
\t\t/>
\t</vehicle>

</mvsim_world>
"""


def main():
	ap = argparse.ArgumentParser(description=__doc__)
	ap.add_argument("--seed", type=int, default=42)
	ap.add_argument("--map-half", type=float, default=140.0, help="Terrain half-extent [m]")
	ap.add_argument("--resolution", type=float, default=2.5, help="Elevation grid resolution [m]")
	ap.add_argument("--x0", type=float, default=-110.0)
	ap.add_argument("--x1", type=float, default=110.0)
	ap.add_argument("--waypoint-step", type=float, default=0.5, help="Arc-length spacing [m]")
	ap.add_argument("--speed", type=float, default=1.5, help="Nominal path speed [m/s]")
	ap.add_argument("--n-blocks", type=int, default=55)
	ap.add_argument("--max-range", type=float, default=60.0)
	args = ap.parse_args()

	rng = random.Random(args.seed)

	rows, _n = build_elevation_matrix(args.map_half, args.resolution)
	waypoints = build_waypoints(args.x0, args.x1, args.waypoint_step, args.speed)
	blocks = place_blocks(waypoints, args.map_half, args.n_blocks, rng)

	elevation = elevation_matrix_xml(rows)
	block_classes, block_instances = blocks_xml(blocks)

	_, veh_x, veh_y = waypoints[0]
	world = WORLD_TEMPLATE.format(
		resolution=args.resolution,
		elevation=elevation,
		block_classes=block_classes,
		blocks=block_instances,
		veh_x=veh_x,
		veh_y=veh_y,
		max_range=args.max_range,
	)

	out_dir = os.path.dirname(os.path.abspath(__file__))
	world_path = os.path.join(out_dir, "demo_lidar_odometry_benchmark.world.xml")
	traj_path = os.path.join(out_dir, "demo_lidar_odometry_benchmark.trajectory.txt")

	with open(world_path, "w") as f:
		f.write(world)
	with open(traj_path, "w") as f:
		f.write("# Curvy, non-looping, >200m benchmark trajectory for mvsim-dataset-gen.\n")
		f.write("# t x y\n")
		for t, x, y in waypoints:
			f.write(f"{t:.3f} {x:.4f} {y:.4f}\n")

	print(f"Wrote {world_path}")
	print(f"Wrote {traj_path}")
	print(f"Initial waypoint: {waypoints[0]}")
	print(f"Final waypoint: {waypoints[-1]}")


if __name__ == "__main__":
	main()
