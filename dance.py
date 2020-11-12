# write dance waypoints
import numpy as np

# disco dance
disco = {'waypoints': [[np.pi/2, 0, np.pi/4, 0, 0], [-np.pi/2, 0, -np.pi/4, 0, 0]],
        'gripper_states': [True, True]
        }

# lasso dance
lasso = {'waypoints': [[np.pi, 0, 0.8973788022994995, 0.6795535087585449, np.pi/2], [-np.pi, 0, 0.8973788022994995, 0.6795535087585449, -2.9866607189178467]],
        'gripper_states': [True, True]
        }

# left robot arm swing
l_arm_swing = {'waypoints': [[np.pi/2, 0, 0, -0.6320000886917114, np.pi/2], [np.pi/2, -0.01840776950120926, -0.0782330259680748, -2.015650749206543, np.pi/2], [1.3575730323791504, 0.015339808538556099, -0.06749515980482101, -1.5999419689178467, np.pi/2]],
                'gripper_states': [True, True, True]
            }

# right robot arm swing
r_arm_swing = {'waypoints': [[-np.pi/2, 0, 0, -0.9004467725753784, np.pi/2], [-np.pi/2, -0.11965050548315048, -0.1426602154970169, -2.043262481689453, np.pi/2], [-1.4419419765472412, -0.09357283264398575, -0.10124273598194122, -1.5953400135040283, np.pi/2]],
                'gripper_states': [True, True, True]
            }

# dance list of dictionaries
dances = [l_arm_swing, r_arm_swing, disco, lasso]
# sequence of dances
dance_sequence = [0, 0, 1, 1]

# waypoints of final dance
dance_waypoints = []
dance_gripper = []

# create final dance
for i in dance_sequence:
    # extend list
    dance_waypoints.extend(dances[i]['waypoints'])
    dance_gripper.extend(dances[i]['gripper_states'])


if __name__ == "__main__":
    print(dance_waypoints)
    print('----------------------------------------------')
    print(dance_waypoints)


