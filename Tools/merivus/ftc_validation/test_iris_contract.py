#!/usr/bin/env python3
"""Check the Iris allocator/output contract against the pinned Gazebo model."""
import math
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

REPO = Path(__file__).resolve().parents[3]


class IrisContract(unittest.TestCase):
    def test_geometry_and_thrust_linearization(self):
        airframe = REPO / 'ROMFS/px4fmu_common/init.d-posix/airframes/10015_gazebo-classic_iris'
        params = {k: float(v) for k, v in re.findall(r'^param set-default (\w+) ([-.\d]+)$', airframe.read_text(), re.M)}
        model = ET.parse(REPO / 'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf').getroot().find('model')
        motors = [p for p in model.findall('plugin') if p.find('motorNumber') is not None]
        self.assertEqual(len(motors), int(params['CA_ROTOR_COUNT']))
        for motor in motors:
            i = int(motor.findtext('motorNumber'))
            prefix = 'CA_ROTOR{}_'.format(i)
            pose = list(map(float, model.find("link[@name='{}']/pose".format(motor.findtext('linkName'))).text.split()))
            self.assertAlmostEqual(params[prefix+'PX'], pose[0])
            self.assertAlmostEqual(params[prefix+'PY'], -pose[1])
            sign = 1 if motor.findtext('turningDirection') == 'ccw' else -1
            self.assertAlmostEqual(params[prefix+'KM'], sign*float(motor.findtext('momentConstant')))
            channel = model.find(".//channel[@name='rotor{}']".format(i+1))
            scale = float(channel.findtext('input_scaling'))
            idle = float(channel.findtext('zero_position_armed'))
            self.assertEqual(float(channel.findtext('input_offset')), 0)
            k = float(motor.findtext('motorConstant'))
            net_max = k*((idle+scale)**2-idle**2)
            self.assertAlmostEqual(params[prefix+'CT'], net_max)
            factor = params['THR_MDL_FAC']
            for thrust in (0, .1, .25, .5, .75, 1):
                signal = (math.sqrt((1-factor)**2+4*factor*thrust)-(1-factor))/(2*factor)
                actual = k*((idle+scale*signal)**2-idle**2)/net_max
                self.assertAlmostEqual(actual, thrust, places=7)


if __name__ == '__main__':
    unittest.main()
