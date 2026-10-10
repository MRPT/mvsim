#!/usr/bin/env python3
# Unit tests for mvsim-urdf2xml (no ROS needed).

import importlib.util
import math
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(HERE, '..', 'mvsim-urdf2xml', 'mvsim-urdf2xml.py')
spec = importlib.util.spec_from_file_location('urdf2xml', TOOL)
u2x = importlib.util.module_from_spec(spec)
spec.loader.exec_module(u2x)

URDF = os.path.join(HERE, 'urdf2xml', 'test_robot.urdf')
MAPPING = os.path.join(HERE, 'urdf2xml', 'test_mapping.yaml')


def load_model(mapping_changes=None):
    mapping = u2x.load_mapping(MAPPING)
    for k, v in (mapping_changes or {}).items():
        mapping[k] = v
    return u2x.Model(u2x.Urdf(open(URDF).read()), mapping)


class TestUrdf2Xml(unittest.TestCase):
    def test_rpy_roundtrip(self):
        for rpy in [(0.1, -0.4, 2.0), (0, 0.3, 0.5), (-1.2, 0.7, -3.0)]:
            m = u2x.mat_from_xyz_rpy((1, 2, 3), rpy)
            xyz, (y, p, r) = u2x.mat_to_xyz_ypr(m)
            m2 = u2x.mat_from_xyz_rpy(xyz, (r, p, y))
            for i in range(4):
                for j in range(4):
                    self.assertAlmostEqual(m[i][j], m2[i][j], places=9)

    def test_wheels(self):
        model = load_model()
        self.assertEqual(model.warnings, [])
        w = {x['tag']: x for x in model.wheels}
        self.assertAlmostEqual(w['lf_wheel']['x'], 0.3)
        self.assertAlmostEqual(w['lf_wheel']['y'], 0.3)
        self.assertAlmostEqual(w['lf_wheel']['z'], 0.15)  # on the ground
        self.assertAlmostEqual(w['lf_wheel']['diameter'], 0.3)
        self.assertAlmostEqual(w['lf_wheel']['width'], 0.08)
        self.assertAlmostEqual(w['lf_wheel']['mass'], 1.5)
        self.assertEqual(w['rr_wheel']['joint'], 'rear_right_wheel_joint')

    def test_base_not_on_ground_warns(self):
        model = load_model({'base_link': 'base_link'})
        self.assertTrue(any('on the ground' in s for s in model.warnings))

    def test_sensor_pose(self):
        s = load_model().sensors[0]
        # Through base_joint (z+0.15), then a pitched mount, then a yawed link:
        self.assertAlmostEqual(s['x'], 0.2 + 0.05 * math.sin(0.3))
        self.assertAlmostEqual(s['y'], 0.0)
        self.assertAlmostEqual(s['z'], 0.15 + 0.3 + 0.05 * math.cos(0.3))
        expected = u2x.mat_mul(u2x.mat_from_xyz_rpy((0, 0, 0), (0, 0.3, 0)),
                               u2x.mat_from_xyz_rpy((0, 0, 0), (0, 0, 0.5)))
        got = u2x.mat_from_xyz_rpy((0, 0, 0), (s['roll'], s['pitch'], s['yaw']))
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(expected[i][j], got[i][j], places=9)

    def test_chassis(self):
        ch = load_model().chassis
        self.assertAlmostEqual(ch['mass'], 20.0)  # wheels excluded
        self.assertAlmostEqual(ch['zmin'], 0.15)
        self.assertAlmostEqual(ch['zmax'], 0.35)
        xs = [p[0] for p in ch['shape']]
        self.assertAlmostEqual(min(xs), -0.4)
        self.assertAlmostEqual(max(xs), 0.4)

    def test_generate_and_check(self):
        model = load_model()
        xml = u2x.generate_xml(model)
        self.assertIn('joint_name="front_left_wheel_joint"', xml)
        self.assertIn('sensor_name="lidar"', xml)
        self.assertEqual(u2x.check_xml(model, xml), [])
        # A modified XML must be reported:
        bad = xml.replace('diameter="0.3"', 'diameter="0.32"', 1)
        errors = u2x.check_xml(model, bad)
        self.assertEqual(len(errors), 1)
        self.assertIn('diameter', errors[0])
        bad = xml.replace('sensor_z="', 'sensor_z="1', 1)
        self.assertTrue(any('lidar' in e for e in u2x.check_xml(model, bad)))

    def test_special_characters_are_escaped(self):
        mapping = u2x.load_mapping(MAPPING)
        mapping['vehicle_class'] = 'a&b'
        mapping['sensors'][0]['args'] = {'topic': 'x"y<z>&w'}
        model = u2x.Model(u2x.Urdf(open(URDF).read()), mapping)
        xml = u2x.generate_xml(model)
        root = u2x.ET.fromstring(xml.replace('vehicle:class', 'vehicle_class'))  # well-formed
        self.assertEqual(root.attrib['name'], 'a&b')
        inc = [e for e in root.iter('include') if e.attrib.get('sensor_name') == 'lidar'][0]
        self.assertEqual(inc.attrib['topic'], 'x"y<z>&w')
        self.assertEqual(u2x.check_xml(model, xml), [])

    def test_cli(self):
        with tempfile.TemporaryDirectory() as d:
            out = os.path.join(d, 'out.vehicle.xml')
            self.assertEqual(u2x.main([URDF, MAPPING, '-o', out]), 0)
            self.assertEqual(u2x.main([URDF, MAPPING, '--check', out]), 0)

    def test_errors(self):
        with self.assertRaises(ValueError):
            load_model({'wheels': {'lf_wheel': 'nonexistent'}})
        with self.assertRaises(ValueError):
            u2x.Urdf('<robot xmlns:xacro="http://www.ros.org/wiki/xacro">'
                     '<xacro:macro name="a"/></robot>')


if __name__ == '__main__':
    unittest.main()
