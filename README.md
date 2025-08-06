# quadro
ros2 quadruped

Add this to robot.urdf

<link name="world"/>

<joint name="fixed" type="fixed">
    <parent link="world"/>
    <child link="motor"/>
</joint>

