# write dance waypoints


# disco dance
disco = {'waypoints': [],
        'gripper_states': []
        }

# lasso dance
lasso = {'waypoints': [],
        'gripper_states': []
        }

# left robot arm swing
l_arm_swing = {'waypoints': [[1.3606410026550293, -0.004601942375302315, -0.09970875084400177, -0.6320000886917114, 1.448077917098999], [1.3468351364135742, -0.01840776950120926, -0.0782330259680748, -2.015650749206543, 1.448077917098999], [1.3575730323791504, 0.015339808538556099, -0.06749515980482101, -1.5999419689178467, 1.448077917098999]],
                'gripper_states': [True, True, True]
            }

# right robot arm swing
r_arm_swing = {'waypoints': [[-1.494097352027893, -0.09817477315664291, -0.11965050548315048, -0.9004467725753784, 1.5447187423706055], [-1.477223515510559, -0.11965050548315048, -0.1426602154970169, -2.043262481689453, 1.5447187423706055], [-1.4419419765472412, -0.09357283264398575, -0.10124273598194122, -1.5953400135040283, 1.5447187423706055]],
                'gripper_states': [True, True, True]
            }

# dance list of dictionaries
dances = [l_arm_swing, r_arm_swing]
# sequence of dances
dance_sequence = [0, 0, 0, 1, 1, 1]

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


