# quadro
ros2 quadruped

TODO:
Walking controller redesign and refactor - it should be possible to have walking on a plane and yaw happen at the same time,
also the controller should take in walking speed (in m/s) and generate corresponding gait patterns

Add this to robot.urdf

<link name="world"/>

<joint name="fixed" type="fixed">
    <parent link="world"/>
    <child link="motor"/>
</joint>

----------- LEG CONFIGURATION ---------------

Back Right (br) leg:

<!-- br_m1_s1 - Hip joint -->
<axis xyz="0 0 -1"/>

<!-- br_m2_s2 - Thigh joint -->
<axis xyz="0 0 -1"/>

<!-- br_m3_s3 - Shin joint -->
<axis xyz="0 0 -1"/>

<!-- br_l4_l3 - Mimic joint (coupled to shin) -->
<axis xyz="0 0 -1"/>

Back Left (bl) leg:

<!-- bl_m1_s1 - Hip joint -->
<axis xyz="0 0 -1"/>

<!-- bl_m2_s2 - Thigh joint -->
<axis xyz="0 0 1"/>

<!-- bl_m3_s3 - Shin joint -->
<axis xyz="0 0 1"/>

<!-- bl_l4_l3 - Mimic joint (coupled to shin) -->
<axis xyz="0 0 1"/>

Front Right (fr) leg:

<!-- fr_m1_s1 - Hip joint -->
<axis xyz="0 0 1"/>

<!-- fr_m2_s2 - Thigh joint -->
<axis xyz="0 0 -1"/>

<!-- fr_m3_s3 - Shin joint -->
<axis xyz="0 0 -1"/>

<!-- fr_l4_l3 - Mimic joint (coupled to shin) -->
<axis xyz="0 0 -1"/>

Front Left (fl) leg:

<!-- fl_m1_s1 - Hip joint -->
<axis xyz="0 0 1"/>

<!-- fl_m2_s2 - Thigh joint -->
<axis xyz="0 0 1"/>

<!-- fl_m3_s3 - Shin joint -->
<axis xyz="0 0 1"/>

<!-- fl_l4_l3 - Mimic joint (coupled to shin) -->
<axis xyz="0 0 1"/>

Summary:

- Reversed joints (z = -1): br_m1_s1, br_m2_s2, br_m3_s3, br_l4_l3, fr_m2_s2, fr_m3_s3, fr_l4_l3
- Non-reversed joints (z = 1): bl_m1_s1, bl_m2_s2, bl_m3_s3, bl_l4_l3, fr_m1_s1, fl_m1_s1, fl_m2_s2, fl_m3_s3,
fl_l4_l3


[fl_m1_s1, fl_m2_s2, fl_l4_l3, fr_m1_s1, fr_m2_s2, fr_l4_l3, bl_m1_s1, bl_m2_s2, bl_l4_l3, br_m1_s1, br_m2_s2, br_l4_l3]