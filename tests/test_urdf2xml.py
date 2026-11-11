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
CAR_URDF = os.path.join(HERE, '..', 'mvsim_tutorial', 'ros2_control', 'car_ackermann.urdf')


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
        # A sensor on the chassis link does not remove its mass:
        mapping = u2x.load_mapping(MAPPING)
        mapping['sensors'].append({'link': 'base_link', 'include': 'gnss.sensor.xml'})
        model = u2x.Model(u2x.Urdf(open(URDF).read()), mapping)
        self.assertAlmostEqual(model.chassis['mass'], 20.0)
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
        self.assertIn('<base_frame_id>base_footprint</base_frame_id>', xml)
        self.assertEqual(u2x.check_xml(model, xml), [])
        no_frame = xml.replace('<base_frame_id>base_footprint</base_frame_id>', '')
        self.assertEqual(len(u2x.check_xml(model, no_frame)), 1)
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

    def test_optional_sensor(self):
        mapping = u2x.load_mapping(MAPPING)
        mapping['sensors'][0]['if'] = '${use_lidar|false}'
        mapping['sensors'][0]['args'] = {'raytrace_3d': True}
        model = u2x.Model(u2x.Urdf(open(URDF).read()), mapping)
        xml = u2x.generate_xml(model)
        root = u2x.ET.fromstring(xml.replace('vehicle:class', 'vehicle_class'))
        cond = root.find('if')
        self.assertEqual(cond.attrib['condition'], '${use_lidar|false}')
        self.assertEqual(cond.find('include').attrib['sensor_name'], 'lidar')
        self.assertEqual(cond.find('include').attrib['raytrace_3d'], 'true')
        self.assertEqual(u2x.check_xml(model, xml), [])

    def test_cli(self):
        with tempfile.TemporaryDirectory() as d:
            out = os.path.join(d, 'out.vehicle.xml')
            self.assertEqual(u2x.main([URDF, MAPPING, '-o', out]), 0)
            self.assertEqual(u2x.main([URDF, MAPPING, '--check', out]), 0)

    def test_ackermann(self):
        urdf = u2x.Urdf(open(CAR_URDF).read())
        mapping = {'dynamics': 'ackermann',
                   'wheels': {t: t + '_joint' for t in ('rl_wheel', 'rr_wheel', 'fl_wheel',
                                                         'fr_wheel')},
                   'steering_joints': ['fl_steer_joint', 'fr_steer_joint']}
        model = u2x.Model(urdf, mapping)
        st = model.steering
        self.assertAlmostEqual(st['f_wheels_x'], 1.3)
        self.assertAlmostEqual(st['f_wheels_d'], 2.0)
        # Inner wheel limit -> equivalent steering angle (car_ackermann: 30 deg):
        self.assertAlmostEqual(st['max_steer_ang_deg'], 30.0, places=2)
        self.assertAlmostEqual(st['max_steer_rate_deg'], 60.0, places=2)
        xml = u2x.generate_xml(model)
        self.assertIn('<f_wheels_x>1.3</f_wheels_x>', xml)
        self.assertEqual(u2x.check_xml(model, xml), [])
        bad = xml.replace('<f_wheels_d>2</f_wheels_d>', '<f_wheels_d>2.2</f_wheels_d>')
        self.assertEqual(len(u2x.check_xml(model, bad)), 1)
        # The inner wheel limits are on the side of each wheel (here, swapped):
        swapped = open(CAR_URDF).read().replace(
            'lower="-0.3804" upper="0.8044"', 'lower="-0.6" upper="0.2"').replace(
            'lower="-0.8044" upper="0.3804"', 'lower="-0.2" upper="0.6"')
        st = u2x.Model(u2x.Urdf(swapped), mapping).steering
        self.assertAlmostEqual(
            st['max_steer_ang_deg'],
            math.degrees(math.atan(1 / (1 / math.tan(0.2) + 0.5 * 2.0 / 1.3))))
        # Invalid steering joints or limits:
        for joints in (7, True, ['fl_steer_joint', 'fl_steer_joint']):
            with self.assertRaises(ValueError):
                u2x.Model(urdf, dict(mapping, steering_joints=joints))
        zero = open(CAR_URDF).read().replace('lower="-0.3804" upper="0.8044"',
                                             'lower="0" upper="0"')
        with self.assertRaises(ValueError):
            u2x.Model(u2x.Urdf(zero), mapping)
        # A central steering joint limit is the equivalent angle itself:
        mapping['steering_joints'] = 'fl_steer_joint'
        model = u2x.Model(urdf, mapping)
        self.assertAlmostEqual(model.steering['max_steer_ang_deg'], math.degrees(0.3804))
        # No steering joints: only the axle position
        del mapping['steering_joints']
        self.assertEqual(set(u2x.Model(urdf, mapping).steering), {'f_wheels_x', 'f_wheels_d'})

    def test_tutorial_examples_up_to_date(self):
        tutorial = os.path.join(HERE, '..', 'mvsim_tutorial')
        for name, xml in (('jackal', 'jackal_from_urdf'), ('skid_steer_4wd', 'skid_steer_4wd')):
            model = u2x.Model(
                u2x.Urdf(open(os.path.join(tutorial, 'ros2_control', name + '.urdf')).read()),
                u2x.load_mapping(os.path.join(tutorial, 'urdf2xml', name + '_mapping.yaml')))
            xml_text = open(os.path.join(tutorial, 'urdf2xml', xml + '.vehicle.xml')).read()
            self.assertEqual(u2x.check_xml(model, xml_text), [], name)
        self.assertEqual(model.base, 'base_footprint')
        self.assertIn('<base_frame_id>base_footprint</base_frame_id>', xml_text)

    def test_errors(self):
        with self.assertRaises(ValueError):
            load_model({'wheels': {'lf_wheel': 'nonexistent'}})
        with self.assertRaises(ValueError):
            u2x.Urdf('<robot xmlns:xacro="http://www.ros.org/wiki/xacro">'
                     '<xacro:macro name="a"/></robot>')
        # An expanded URDF may still declare the xacro namespace or mention it:
        u2x.Urdf('<robot name="r" xmlns:xacro="http://www.ros.org/wiki/xacro">'
                 '<!-- xacro: generated --><link name="base_link"/></robot>')
        # Input errors: exit status 2, not 1 (mismatches)
        self.assertEqual(u2x.main(['/nonexistent.urdf', MAPPING]), 2)
        self.assertEqual(u2x.main([URDF, MAPPING, '--check', '/nonexistent.xml']), 2)
        # Unresolved mesh URIs:
        with tempfile.TemporaryDirectory() as d:
            urdf = os.path.join(d, 'r.urdf')
            with open(urdf, 'w') as f:
                f.write(open(URDF).read().replace(
                    '</robot>', '<link name="m"><visual><geometry><mesh filename='
                    '"package://no_such_package/m.dae"/></geometry></visual></link>'
                    '<joint name="mj" type="fixed"><parent link="base_link"/>'
                    '<child link="m"/></joint></robot>'))
            self.assertEqual(u2x.main([urdf, MAPPING, '-o', os.path.join(d, 'o.xml')]), 2)


if __name__ == '__main__':
    unittest.main()
