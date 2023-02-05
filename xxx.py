# Code to create generic URDF from DH parameters

import numpy as np
from numpy import sin, cos, pi

from scipy.spatial.transform import Rotation as R

def DH_trans(DH, joint_val):

     d, theta, a, alpha = (0,0,0,0)

     if (DH[0] == 'r'):

         d, theta, a, alpha = (DH[1], joint_val, DH[2], DH[3])

     elif (DH[0] == 'p'):

         d, theta, a, alpha = (joint_val, DH[1], DH[2], DH[3])

     elif (DH[0] == 'f'):

         d, theta, a, alpha = (DH[1], DH[2], DH[3], DH[4])

     trans_mat = np.array([[cos(theta), -1*sin(theta)*cos(alpha), sin(theta)*sin(alpha),    a*cos(theta)],
                           [sin(theta), cos(theta)*cos(alpha),    -1*cos(theta)*sin(alpha), a*sin(theta)],
                           [0,          sin(alpha),               cos(alpha),               d           ],
                           [0,          0,                        0,                        1           ]])

     return trans_mat


def joint_transforms(DH_Params):

     transforms = []

     current_DOF = 0

     transforms.append(np.eye(4))

     for DH in DH_Params:
         
         if (DH[0] == 'r' or DH[0] == 'p'):
             transforms.append(DH_trans(DH, 0.0))
             current_DOF = current_DOF + 1

         else:
             transforms.append(DH_trans(DH, 0.0))

     return transforms

def joint_frames(transforms):
         
         joint_frames = [transforms[0]]
 
         for trans in transforms[1:]:
 
             joint_frames.append(joint_frames[-1] @ trans)
 
         return joint_frames

# DH Parameter Layout:
# ['r', d, a, alpha] for revolute joints
# ['p', theta, a, alpha] for prismatic joints
# ['f', d, theta, a, alpha] for fixed joints

def xml_string(DH_Params, limits, joint_names):

    outstring = ''

    transforms = joint_transforms(DH_Params)

    frames = joint_frames(transforms)

    outstring = outstring + '<robot name="robot">\n'

    outstring = outstring + '  <material name="blue">\n    <color rgba="0 0 0.8 1"/>\n  </material>\n'
    outstring = outstring + '  <material name="red">\n    <color rgba="0.8 0 0 1"/>\n  </material>\n'

    outstring = outstring + '  <link name="base_link"/>\n'
    outstring = outstring + '  <joint name="joint0" type="fixed">\n'
    outstring = outstring + '    <parent link="base_link"/>\n'
    outstring = outstring + '    <child  link="a0"/>\n'
    outstring = outstring + '  </joint>\n'

    for i in range(len(transforms) - 1):

        el = transforms[i]
        fr = frames[i]

        # We need to create a cylinder to represent the joint
        # If the index is not zero, connect it to the previous link
        # And a joint to connect it to the link
        # And a box to connect the joints

        rpy = R.from_matrix(fr[0:3,0:3]).as_euler("XYZ")
        
        outstring = outstring + '  <link name="a{}">\n'.format(i)
        outstring = outstring + '    <visual>\n'
        outstring = outstring + '      <origin rpy="{} {} {}" xyz="{} {} {}"/>\n'.format(rpy[0], rpy[1], rpy[2], el[0,3], el[1,3], el[2,3])
        outstring = outstring + '      <geometry>\n'
        outstring = outstring + '        <cylinder length="0.09" radius="0.009"/>\n'
        outstring = outstring + '      </geometry>\n'
        outstring = outstring + '      <material name="blue"/>\n'
        outstring = outstring + '    </visual>\n'
        outstring = outstring + '  </link>\n'

        # If not on the first transformation, fix the cylinder to the previous link
        if(i != 0):
            outstring = outstring + '  <joint name="fix_a{}_to_l{}" type="fixed">\n'.format(i, i-1)
            outstring = outstring + '    <parent link="l{}"/>\n'.format(i-1)
            outstring = outstring + '    <origin rpy="0 0 0" xyz="0 0 0"/>\n'
            outstring = outstring + '    <child link="a{}"/>\n'.format(i)
            outstring = outstring + '  </joint>\n'

        # Add a cylinder that goes from the current origin to the next one
        origins_vector = transforms[i + 1][0:3,3]

        origins_vector_norm = np.linalg.norm(origins_vector)

        cylinder_origin = origins_vector/2

        rpy = [0, 0, 0]

        if (origins_vector_norm != 0.0):

            origins_vector_unit = origins_vector/origins_vector_norm

            axis = np.cross(origins_vector, np.array([0, 0, -1]))

            axis_norm = np.linalg.norm(axis)
            if (axis_norm != 0.0):
                axis = axis/np.linalg.norm(axis)

            angle = np.arccos(origins_vector_unit @ np.array([0, 0, 1]))

            print('axis is {}'.format(axis))
            print('angle is {}'. format(angle))

            rpy = R.from_rotvec(angle * axis).as_euler("XYZ")

        outstring = outstring + '  <link name="l{}">\n'.format(i)
        outstring = outstring + '    <visual>\n'
        outstring = outstring + '      <origin rpy="{} {} {}" xyz="{} {} {}"/>\n'.format(rpy[0], rpy[1], rpy[2], cylinder_origin[0], cylinder_origin[1], cylinder_origin[2])
        outstring = outstring + '      <geometry>\n'
        outstring = outstring + '        <cylinder length="{}" radius="0.01"/>\n'.format(origins_vector_norm) 
        outstring = outstring + '      </geometry>\n'
        outstring = outstring + '      <material name="red"/>\n'
        outstring = outstring + '    </visual>\n'
        outstring = outstring + '  </link>\n'

        # Add the actual joint between the cylinder and link

        jointType = ''

        if (DH_Params[i][0] == "r"):
            # jointType = 'continuous'
            jointType = 'revolute'
        elif (DH_Params[i][0] == "p"):
            jointType = 'prismatic'
        else:
            jointType = 'fixed'

        # outstring = outstring + '  <joint name="move_l{}_from_a{}" type="{}">\n'.format(i, i, jointType)
        outstring = outstring + '  <joint name="{}" type="{}">\n'.format(joint_names[i], jointType)
        outstring = outstring + '    <parent link="a{}"/>\n'.format(i)
        outstring = outstring + '    <origin rpy="0 0 0" xyz="{} {} {}"/>\n'.format(el[0,3], el[1,3], el[2,3])   
        outstring = outstring + '    <axis xyz="{} {} {}"/>\n'.format(fr[0,2], fr[1,2], fr[2,2])
        outstring = outstring + '    <child link="l{}"/>\n'.format(i)
        outstring = outstring + '    <limit effort="0" lower="{}" upper="{}" velocity="0"/>\n'.format(limits[i][0], limits[i][1])
        outstring = outstring + '  </joint>\n' 

    outstring = outstring + '</robot>\n'

    return outstring

f = open("outfile.xml", "w")

DH_Params = []
# DH_Params.append(['r', 4.45, -1.50, -pi/2])
# DH_Params.append(['r', 0, 9.00, 0])
# DH_Params.append(['r', 0, 1.50, -pi/2])
# DH_Params.append(['r', 9.38, 0, pi/2])
# DH_Params.append(['r', 0, 0, pi/2])
# DH_Params.append(['r', 2.00, 0, 0])

x = 5.2e-3
y = 290.77e-3
th_s = np.arctan(x / y)
a_2 = np.sqrt(x * x + y * y)
DH_Params.append(['r', 41.45e-3, 67.5e-3, np.pi])
DH_Params.append(['r', -117.48e-3, 46.0e-3, -np.pi / 2])
DH_Params.append(['r', 0.0, a_2, 0.0])
DH_Params.append(['r', 0.0, 40.8e-3, -np.pi / 2])
DH_Params.append(['r', -160.26e-3, 40.8e-3, -np.pi / 2])
DH_Params.append(['r', 0.0, 40.8e-3, -np.pi / 2])
DH_Params.append(['p', -224.16e-3, 0.0, np.pi / 2])
DH_Params.append(['r', 0.0, 120.0e-3, 0.0])

limits = (
    (0,6.3),
    (0,3.2),
    (0,3.2),
    (0,6.2),
    (0,3.2),
    (-3.141592,3.141592),
    (0,0.04),
    (0,6.3),
)
# limits = (
#     (0,6.3), # g
#     (0,6.3), # f
#     (-3.2,3.2), # e
#     (0,6.3), # d
#     (0,6.3), # c
#     (0,6.3), # b
#     (0,6.3), # a
#     (0,6.3), # 0
# )
joint_names = (
    'bravo_axis_g',
    'bravo_axis_f',
    'bravo_axis_e',
    'bravo_axis_d',
    'bravo_axis_c',
    'bravo_axis_b',
    'bravo_axis_a',
    'move_l7_from_a7',
)
f.write(xml_string(DH_Params, limits, joint_names))

f.close()
