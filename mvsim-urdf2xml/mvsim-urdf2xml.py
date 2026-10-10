#!/usr/bin/env python3
# +-------------------------------------------------------------------------+
# |                       MultiVehicle simulator (libmvsim)                 |
# |                                                                         |
# | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
# | Distributed under 3-clause BSD License                                  |
# |   See COPYING                                                           |
# +-------------------------------------------------------------------------+
"""
mvsim-urdf2xml: generates an MVSim vehicle class XML file from a URDF robot
description, or checks an existing one against it.

The simulator itself only reads MVSim XML files: this standalone tool keeps
the URDF as the single source of truth for the robot geometry (wheels,
chassis, sensor poses, visual meshes), while MVSim-specific parameters
(dynamics class, controller, friction, sensor models) come from a small
mapping file (YAML).

Input URDF files must be already expanded (run xacro first).

Usage:
  mvsim-urdf2xml robot.urdf mapping.yaml -o robot.vehicle.xml
  mvsim-urdf2xml robot.urdf mapping.yaml --check robot.vehicle.xml

See the documentation for the mapping file format.
"""

import argparse
import math
import os
import re
import sys
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape, quoteattr

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


# --------------------------------------------------------------------------
# Minimal 3D math (4x4 homogeneous matrices as nested lists)
# --------------------------------------------------------------------------

def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def mat_from_xyz_rpy(xyz, rpy):
    """URDF convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)"""
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
        [-sp, cp * sr, cp * cr, xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def mat_to_xyz_ypr(m):
    """Returns (x,y,z), (yaw,pitch,roll) [rad], same convention as URDF rpy
    and MRPT CPose3D."""
    pitch = math.atan2(-m[2][0], math.hypot(m[0][0], m[1][0]))
    if abs(math.cos(pitch)) > 1e-9:
        yaw = math.atan2(m[1][0], m[0][0])
        roll = math.atan2(m[2][1], m[2][2])
    else:  # gimbal lock
        yaw = math.atan2(-m[0][1], m[1][1])
        roll = 0.0
    return (m[0][3], m[1][3], m[2][3]), (yaw, pitch, roll)


def transform_point(m, p):
    return tuple(m[i][0] * p[0] + m[i][1] * p[1] + m[i][2] * p[2] + m[i][3]
                 for i in range(3))


IDENTITY = mat_from_xyz_rpy((0, 0, 0), (0, 0, 0))


def fmt(v):
    """Compact float formatting (avoids "-0")"""
    s = f'{v:.6f}'.rstrip('0').rstrip('.')
    return '0' if s in ('-0', '') else s


# --------------------------------------------------------------------------
# URDF model
# --------------------------------------------------------------------------

def parse_origin(elem):
    o = elem.find('origin') if elem is not None else None
    xyz = (0.0, 0.0, 0.0)
    rpy = (0.0, 0.0, 0.0)
    if o is not None:
        if 'xyz' in o.attrib:
            xyz = tuple(float(v) for v in o.attrib['xyz'].split())
        if 'rpy' in o.attrib:
            rpy = tuple(float(v) for v in o.attrib['rpy'].split())
    return mat_from_xyz_rpy(xyz, rpy)


class Urdf:
    def __init__(self, text):
        self.root = ET.fromstring(text)
        if self.root.tag != 'robot':
            raise ValueError('Not a URDF file: root element must be <robot>')
        if self.root.find('.//{http://www.ros.org/wiki/xacro}macro') is not None or \
                'xacro:' in text:
            raise ValueError(
                'This looks like a xacro file: expand it first (e.g. "xacro robot.urdf.xacro")')
        self.name = self.root.attrib.get('name', 'robot')
        self.links = {l.attrib['name']: l for l in self.root.findall('link')}
        self.joints = {j.attrib['name']: j for j in self.root.findall('joint')}
        # child link -> joint
        self.parent_joint = {}
        for j in self.joints.values():
            self.parent_joint[j.find('child').attrib['link']] = j

    def joint_child(self, joint_name):
        return self.joints[joint_name].find('child').attrib['link']

    def link_pose(self, link, base):
        """Pose of `link` wrt `base` (4x4), following the chain of joints at
        their zero position. Returns (matrix, list of non-fixed joints)."""
        chain = []
        cur = link
        m = IDENTITY
        movable = []
        while cur != base:
            j = self.parent_joint.get(cur)
            if j is None:
                raise ValueError(f'Link "{link}" is not a descendant of "{base}"')
            m = mat_mul(parse_origin(j), m)
            if j.attrib.get('type') != 'fixed':
                movable.append(j.attrib['name'])
            chain.append(j.attrib['name'])
            cur = j.find('parent').attrib['link']
        return m, movable

    def mass(self, link):
        m = self.links[link].find('inertial/mass')
        return float(m.attrib['value']) if m is not None else 0.0

    def cylinder(self, link):
        """(radius, length) of the first cylinder collision (or visual)
        geometry of a link, or None"""
        for tag in ('collision', 'visual'):
            for e in self.links[link].findall(tag):
                c = e.find('geometry/cylinder')
                if c is not None:
                    return float(c.attrib['radius']), float(c.attrib['length'])
        return None

    def box(self, link):
        """(origin 4x4, (sx,sy,sz)) of the first box collision (or visual)
        geometry of a link, or None"""
        for tag in ('collision', 'visual'):
            for e in self.links[link].findall(tag):
                b = e.find('geometry/box')
                if b is not None:
                    size = tuple(float(v) for v in b.attrib['size'].split())
                    return parse_origin(e), size
        return None


# --------------------------------------------------------------------------
# package:// resolution
# --------------------------------------------------------------------------

def resolve_uri(uri, package_paths):
    if uri.startswith('file://'):
        return uri[len('file://'):]
    m = re.match(r'package://([^/]+)/(.*)', uri)
    if not m:
        return uri
    pkg, rel = m.group(1), m.group(2)
    if pkg in package_paths:
        return os.path.join(package_paths[pkg], rel)
    for env in ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH'):
        for prefix in os.environ.get(env, '').split(os.pathsep):
            d = os.path.join(prefix, 'share', pkg)
            if prefix and os.path.isdir(d):
                return os.path.join(d, rel)
    for prefix in os.environ.get('ROS_PACKAGE_PATH', '').split(os.pathsep):
        d = os.path.join(prefix, pkg)
        if prefix and os.path.isdir(d):
            return os.path.join(d, rel)
    raise ValueError(
        f'Cannot resolve "{uri}": add "{pkg}" to "package_paths" in the mapping file')


# --------------------------------------------------------------------------
# Generation
# --------------------------------------------------------------------------

WHEEL_TAGS = {
    'differential': ['l_wheel', 'r_wheel'],
    'differential_3_wheels': ['l_wheel', 'r_wheel', 'caster_wheel'],
    'differential_4_wheels': ['lf_wheel', 'rf_wheel', 'lr_wheel', 'rr_wheel'],
    'ackermann': ['rl_wheel', 'rr_wheel', 'fl_wheel', 'fr_wheel'],
}


class Model:
    """Everything extracted from the URDF + mapping, in MVSim terms"""

    def __init__(self, urdf, mapping):
        self.urdf = urdf
        self.map = mapping
        self.warnings = []
        self.base = mapping.get('base_link', 'base_link')
        if self.base not in urdf.links:
            raise ValueError(f'base_link "{self.base}" not found in the URDF')
        self.dynamics = mapping.get('dynamics', 'differential')
        if self.dynamics not in WHEEL_TAGS:
            raise ValueError(f'Unsupported dynamics "{self.dynamics}"')

        self.wheels = self._wheels()
        self.sensors = self._sensors()
        self.chassis = self._chassis()

    def warn(self, msg):
        self.warnings.append(msg)

    def _wheels(self):
        wheels = []
        wmap = self.map.get('wheels', {})
        for tag in WHEEL_TAGS[self.dynamics]:
            if tag not in wmap:
                raise ValueError(f'Missing wheel "{tag}" in mapping "wheels"')
            joint = wmap[tag]
            if joint not in self.urdf.joints:
                raise ValueError(f'Wheel joint "{joint}" not found in the URDF')
            link = self.urdf.joint_child(joint)
            m, movable = self.urdf.link_pose(link, self.base)
            (x, y, z), _ = mat_to_xyz_ypr(m)
            cyl = self.urdf.cylinder(link)
            if cyl is None:
                raise ValueError(f'Wheel link "{link}" has no cylinder geometry')
            radius, width = cyl
            if abs(z - radius) > 0.01:
                self.warn(
                    f'Wheel "{joint}" center is at z={z:.3f} wrt "{self.base}", but its '
                    f'radius is {radius:.3f}: MVSim vehicle frames are on the ground, so '
                    f'"{self.base}" should be a frame on the ground (e.g. base_footprint), '
                    f'or TFs will differ by {radius - z:.3f} m in z')
            wheels.append({'tag': tag, 'joint': joint, 'x': x, 'y': y, 'z': z,
                           'diameter': 2 * radius, 'width': width,
                           'mass': self.urdf.mass(link) or 1.0})
        return wheels

    def _sensors(self):
        sensors = []
        for s in self.map.get('sensors', []):
            link = s['link']
            if link not in self.urdf.links:
                raise ValueError(f'Sensor link "{link}" not found in the URDF')
            m, movable = self.urdf.link_pose(link, self.base)
            if movable:
                self.warn(f'Sensor link "{link}" is attached through non-fixed joints '
                          f'{movable}: using their zero position')
            (x, y, z), (yaw, pitch, roll) = mat_to_xyz_ypr(m)
            sensors.append({'link': link, 'include': s['include'], 'args': s.get('args', {}),
                            'x': x, 'y': y, 'z': z, 'yaw': yaw, 'pitch': pitch,
                            'roll': roll})
        return sensors

    def _chassis(self):
        cfg = self.map.get('chassis', {}) or {}
        wheel_links = {self.urdf.joint_child(w['joint']) for w in self.wheels}
        sensor_links = {s['link'] for s in self.sensors}

        mass = cfg.get('mass')
        if mass is None:
            mass = sum(self.urdf.mass(l) for l in self.urdf.links
                       if l not in wheel_links and l not in sensor_links)
            if mass <= 0:
                mass = 10.0
                self.warn('No inertial masses in the URDF: using a chassis mass of 10 kg')

        shape = None
        zmin = zmax = None
        link = cfg.get('link')
        if link:
            b = self.urdf.box(link)
            if b is None:
                raise ValueError(f'Chassis link "{link}" has no box geometry')
            origin, (sx, sy, sz) = b
            m, _ = self.urdf.link_pose(link, self.base)
            m = mat_mul(m, origin)
            corners = [transform_point(m, (dx * sx / 2, dy * sy / 2, dz * sz / 2))
                       for dx in (-1, 1) for dy in (-1, 1) for dz in (-1, 1)]
            xs = [c[0] for c in corners]
            ys = [c[1] for c in corners]
            zs = [c[2] for c in corners]
            shape = [(min(xs), min(ys)), (max(xs), min(ys)), (max(xs), max(ys)),
                     (min(xs), max(ys))]
            zmin, zmax = min(zs), max(zs)
        else:
            # Bounding box of the wheels:
            xs = [w['x'] for w in self.wheels]
            ys = [w['y'] for w in self.wheels]
            r = max(w['diameter'] for w in self.wheels) / 2
            shape = [(min(xs) - r, min(ys)), (max(xs) + r, min(ys)), (max(xs) + r, max(ys)),
                     (min(xs) - r, max(ys))]
        zmin = cfg.get('zmin', zmin if zmin is not None else 0.05)
        zmax = cfg.get('zmax', zmax if zmax is not None else 0.4)
        return {'mass': mass, 'shape': shape, 'zmin': max(0.0, zmin), 'zmax': zmax}

    def visuals(self, out_dir):
        """<visual> entries for the meshes of non-wheel links"""
        if not self.map.get('visual', True):
            return []
        package_paths = self.map.get('package_paths', {}) or {}
        wheel_links = {self.urdf.joint_child(w['joint']) for w in self.wheels}
        out = []
        for name, link in self.urdf.links.items():
            if name in wheel_links:
                continue
            for v in link.findall('visual'):
                mesh = v.find('geometry/mesh')
                if mesh is None:
                    continue
                try:
                    lm, _ = self.urdf.link_pose(name, self.base)
                except ValueError:
                    continue
                m = mat_mul(lm, parse_origin(v))
                (x, y, z), (yaw, pitch, roll) = mat_to_xyz_ypr(m)
                path = resolve_uri(mesh.attrib['filename'], package_paths)
                scale = [float(s) for s in mesh.attrib.get('scale', '1 1 1').split()]
                if max(scale) - min(scale) > 1e-9:
                    self.warn(f'Non-uniform mesh scale in link "{name}": using {scale[0]}')
                out.append({'uri': path, 'x': x, 'y': y, 'z': z, 'yaw': yaw,
                            'pitch': pitch, 'roll': roll, 'scale': scale[0]})
        return out


def generate_xml(model, out_dir='.'):
    mp = model.map
    cls = mp.get('vehicle_class', model.urdf.name)
    lines = []
    lines.append('<!-- Generated by mvsim-urdf2xml from the URDF of '
                 f'"{model.urdf.name.replace("--", "- -")}". Do not edit by hand: regenerate it. -->')
    lines.append(f'<vehicle:class name={quoteattr(cls)}>')
    lines.append(f'  <dynamics class={quoteattr(model.dynamics)}>')
    for w in model.wheels:
        lines.append(
            f'    <{w["tag"]} pos="{fmt(w["x"])} {fmt(w["y"])}" mass="{fmt(w["mass"])}" '
            f'width="{fmt(w["width"])}" diameter="{fmt(w["diameter"])}" '
            f'joint_name={quoteattr(w["joint"])} />')
    ch = model.chassis
    lines.append(f'    <chassis mass="{fmt(ch["mass"])}" zmin="{fmt(ch["zmin"])}" '
                 f'zmax="{fmt(ch["zmax"])}">')
    lines.append('      <shape>')
    for (x, y) in ch['shape']:
        lines.append(f'        <pt>{fmt(x)} {fmt(y)}</pt>')
    lines.append('      </shape>')
    lines.append('    </chassis>')
    controller = mp.get('controller')
    if controller:
        lines.extend('    ' + l for l in controller.strip().splitlines())
    lines.append('  </dynamics>')
    friction = mp.get('friction')
    if friction:
        lines.extend('  ' + l for l in friction.strip().splitlines())
    for v in model.visuals(out_dir):
        lines.append('  <visual>')
        lines.append(f'    <model_uri>{escape(v["uri"])}</model_uri>')
        lines.append(f'    <model_scale>{fmt(v["scale"])}</model_scale>')
        lines.append(f'    <model_offset_x>{fmt(v["x"])}</model_offset_x>')
        lines.append(f'    <model_offset_y>{fmt(v["y"])}</model_offset_y>')
        lines.append(f'    <model_offset_z>{fmt(v["z"])}</model_offset_z>')
        lines.append(f'    <model_yaw>{fmt(math.degrees(v["yaw"]))}</model_yaw>')
        lines.append(f'    <model_pitch>{fmt(math.degrees(v["pitch"]))}</model_pitch>')
        lines.append(f'    <model_roll>{fmt(math.degrees(v["roll"]))}</model_roll>')
        lines.append('  </visual>')
    for s in model.sensors:
        attrs = {'file': s['include'], 'sensor_name': s['link'],
                 'sensor_x': fmt(s['x']), 'sensor_y': fmt(s['y']), 'sensor_z': fmt(s['z']),
                 'sensor_yaw': fmt(math.degrees(s['yaw'])),
                 'sensor_pitch': fmt(math.degrees(s['pitch'])),
                 'sensor_roll': fmt(math.degrees(s['roll']))}
        for k, v in s['args'].items():
            attrs[k] = str(v)
        lines.append('  <include ' + ' '.join(f'{k}={quoteattr(v)}' for k, v in attrs.items()) +
                     ' />')
    lines.append('</vehicle:class>')
    return '\n'.join(lines) + '\n'


# --------------------------------------------------------------------------
# --check mode
# --------------------------------------------------------------------------

def check_xml(model, xml_text, tol_pos=1e-3, tol_ang_deg=0.1):
    """Compares an existing MVSim vehicle XML against the URDF. Returns a list
    of mismatch descriptions (empty if all OK)."""
    errors = []
    # Allow the "vehicle:class" tag name:
    root = ET.fromstring(xml_text.replace('vehicle:class', 'vehicle_class'))
    dyn = root.find('dynamics')
    if dyn is None:
        return ['No <dynamics> tag found']

    def num(s, what):
        try:
            return float(s)
        except (TypeError, ValueError):
            errors.append(f'{what}: cannot evaluate "{s}" (skipped)')
            return None

    for w in model.wheels:
        e = dyn.find(w['tag'])
        if e is None:
            errors.append(f'Wheel <{w["tag"]}> missing')
            continue
        pos = e.attrib.get('pos', '').split()
        if len(pos) >= 2:
            px, py = num(pos[0], f'{w["tag"]} pos.x'), num(pos[1], f'{w["tag"]} pos.y')
            for v, ref, what in ((px, w['x'], 'x'), (py, w['y'], 'y')):
                if v is not None and abs(v - ref) > tol_pos:
                    errors.append(f'Wheel <{w["tag"]}> pos.{what}: XML={v} URDF={ref:.4f}')
        d = num(e.attrib.get('diameter'), f'{w["tag"]} diameter')
        if d is not None and abs(d - w['diameter']) > tol_pos:
            errors.append(f'Wheel <{w["tag"]}> diameter: XML={d} URDF={w["diameter"]:.4f}')
        jn = e.attrib.get('joint_name', w['tag'] + '_joint')
        if jn != w['joint']:
            errors.append(f'Wheel <{w["tag"]}> joint_name: XML="{jn}" URDF="{w["joint"]}"')

    # Sensors: <include ... sensor_name=""> or <sensor name=""><pose_3d>
    xml_sensors = {}
    for inc in root.iter('include'):
        name = inc.attrib.get('sensor_name')
        if name:
            xml_sensors[name] = [inc.attrib.get(k, '0') for k in
                                 ('sensor_x', 'sensor_y', 'sensor_z', 'sensor_yaw',
                                  'sensor_pitch', 'sensor_roll')]
    for sen in root.iter('sensor'):
        p = sen.find('pose_3d')
        if sen.attrib.get('name') and p is not None:
            xml_sensors[sen.attrib['name']] = (p.text or '').split()

    for s in model.sensors:
        vals = xml_sensors.get(s['link'])
        if vals is None:
            errors.append(f'Sensor "{s["link"]}" not found in the XML')
            continue
        ref = [s['x'], s['y'], s['z'], math.degrees(s['yaw']), math.degrees(s['pitch']),
               math.degrees(s['roll'])]
        names = ['x', 'y', 'z', 'yaw', 'pitch', 'roll']
        for i, name in enumerate(names):
            v = num(vals[i] if i < len(vals) else '0', f'sensor "{s["link"]}" {name}')
            if v is None:
                continue
            tol = tol_pos if i < 3 else tol_ang_deg
            diff = abs(v - ref[i])
            if i >= 3:
                diff = abs((diff + 180) % 360 - 180)
            if diff > tol:
                errors.append(
                    f'Sensor "{s["link"]}" {name}: XML={v} URDF={ref[i]:.4f}')
    return errors


# --------------------------------------------------------------------------

def load_mapping(path):
    with open(path, 'r') as f:
        text = f.read()
    if yaml is None:
        raise RuntimeError('python3-yaml is required to read the mapping file')
    return yaml.safe_load(text) or {}


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog='mvsim-urdf2xml',
        description='Generates an MVSim vehicle class XML from a URDF (already expanded '
                    'with xacro), or checks an existing one against it.')
    ap.add_argument('urdf', help='Input URDF file ("-" for stdin)')
    ap.add_argument('mapping', help='Mapping file (YAML)')
    ap.add_argument('-o', '--output', help='Output vehicle XML file (default: stdout)')
    ap.add_argument('--check', metavar='VEHICLE_XML',
                    help='Instead of generating, compare this existing vehicle XML file '
                         'against the URDF and report mismatches (exit code 1 if any)')
    args = ap.parse_args(argv)

    urdf_text = sys.stdin.read() if args.urdf == '-' else open(args.urdf).read()
    try:
        model = Model(Urdf(urdf_text), load_mapping(args.mapping))
    except (ValueError, KeyError) as e:
        print(f'Error: {e}', file=sys.stderr)
        return 2

    for w in model.warnings:
        print(f'Warning: {w}', file=sys.stderr)

    if args.check:
        errors = check_xml(model, open(args.check).read())
        for e in errors:
            print(f'Mismatch: {e}')
        if not errors:
            print('OK: the vehicle XML matches the URDF')
        return 1 if errors else 0

    out_dir = os.path.dirname(os.path.abspath(args.output)) if args.output else os.getcwd()
    xml = generate_xml(model, out_dir)
    if args.output:
        with open(args.output, 'w') as f:
            f.write(xml)
    else:
        sys.stdout.write(xml)
    return 0


if __name__ == '__main__':
    sys.exit(main())
