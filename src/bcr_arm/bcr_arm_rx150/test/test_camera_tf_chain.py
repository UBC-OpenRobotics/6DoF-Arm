#!/usr/bin/env python3
"""The camera mount pose renders, parses, and is the same in sim and hardware.

camera_mount_pose.xacro is one shared file precisely so the sim and hardware
descriptions cannot drift apart, and it is mostly a long comment explaining how
the numbers were measured. Both of those are easy to break in ways nothing
notices until bring-up:

  - an XML comment may not contain "--", so an innocent dash in the prose makes
    the whole robot_description unparseable;
  - a value edited in one description instead of the shared file silently gives
    sim and hardware two different cameras.

These tests fail in CI instead, which is the point.
"""

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

URDF_DIR = Path(__file__).resolve().parents[1] / 'urdf'
SHARED = URDF_DIR / 'camera_mount_pose.xacro'
DESCRIPTIONS = {
    'hardware': (URDF_DIR / 'rx150_realsense_camera.urdf.xacro',
                 'rx150_camera_mount'),
    'sim': (URDF_DIR / 'rx150_gripper_depth_camera.urdf.xacro',
            'rx150_gripper_camera_mount'),
}


def _render(path):
    """Run xacro the way robot_state_publisher does, returning the XML."""
    result = subprocess.run(
        ['xacro', str(path), 'robot_name:=rx150'],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        pytest.fail('xacro failed on %s:\n%s' % (path.name, result.stderr))
    return result.stdout


def _mount_origin(xml_text, joint_name):
    root = ET.fromstring(xml_text)
    for joint in root.iter('joint'):
        if joint.get('name') == joint_name:
            origin = joint.find('origin')
            assert origin is not None, '%s has no <origin>' % joint_name
            return origin.get('xyz'), origin.get('rpy')
    pytest.fail('no joint named %s in the rendered description' % joint_name)


def test_shared_file_is_well_formed_xml():
    """A "--" anywhere in the comment makes this unparseable. It has bitten."""
    ET.fromstring(SHARED.read_text())


@pytest.mark.parametrize('name', sorted(DESCRIPTIONS))
def test_description_renders_and_parses(name):
    path, joint = DESCRIPTIONS[name]
    ET.fromstring(_render(path))


@pytest.mark.parametrize('name', sorted(DESCRIPTIONS))
def test_mount_joint_uses_the_shared_pose(name):
    """The joint must carry the shared properties, not its own literals.

    Checked against the SOURCE, not the rendered output. This used to assert
    that the rendered xyz appeared verbatim in camera_mount_pose.xacro, which
    only held while that file stored a hand-summed literal; the pose is now
    computed there from a datum plus measurements, so the rendered number
    legitimately appears nowhere. Reading the source keeps the actual rule --
    the numbers live in one file -- and stops the test from dictating how that
    file arrives at them.
    """
    path, joint = DESCRIPTIONS[name]
    source = ET.fromstring(path.read_text())
    for element in source.iter('joint'):
        if element.get('name', '').endswith(joint.split('_', 1)[1]):
            origin = element.find('origin')
            assert origin is not None, '%s has no <origin>' % joint
            assert origin.get('xyz') == '${cam_mount_xyz}', (
                '%s sets xyz="%s" instead of ${cam_mount_xyz}. The mounting pose '
                'belongs in camera_mount_pose.xacro so sim and hardware cannot '
                'drift apart.' % (joint, origin.get('xyz'))
            )
            assert origin.get('rpy') == '${cam_mount_rpy}', (
                '%s sets rpy="%s" instead of ${cam_mount_rpy}.'
                % (joint, origin.get('rpy'))
            )
            return
    pytest.fail('no mount joint matching %s in %s' % (joint, path.name))


def test_sim_and_hardware_agree():
    """The entire reason the shared file exists."""
    origins = {
        name: _mount_origin(_render(path), joint)
        for name, (path, joint) in DESCRIPTIONS.items()
    }
    assert origins['sim'] == origins['hardware'], (
        'sim and hardware disagree about where the camera is: %s' % origins
    )


def test_mount_pose_is_three_numbers():
    """Guards against a typo turning a coordinate into text."""
    for name, (path, joint) in DESCRIPTIONS.items():
        xyz, rpy = _mount_origin(_render(path), joint)
        for label, value in (('xyz', xyz), ('rpy', rpy)):
            parts = value.split()
            assert len(parts) == 3, '%s %s has %d values' % (name, label, len(parts))
            for part in parts:
                float(part)
