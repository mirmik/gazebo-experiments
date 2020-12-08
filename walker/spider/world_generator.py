#!/usr/bin/env python3

file = open("world.xml", "w")

GRAVITY = -1
BODY_LENGTH = 5

inertia_st = """
  <mass> 1 </mass>
  <inertia>
    <ixx>0.05893</ixx>
    <iyy>0.33333</iyy>
    <izz>0.33333</izz>
    <ixy>0</ixy>
    <ixz>0</ixz>
    <iyz>0</iyz>
  </inertia>
"""

leg_template = """
	  <link name="shoulder_{number}">
        <collision name="collision_shoulder_{number}_0">
          <pose>{pose_shoulder}</pose>
          <geometry>
            <sphere>
            	<radius>0.3</radius>
            </sphere>    
          </geometry>
        </collision>
  
        <visual name="visual_shoulder_{number}_0">
          <pose>{pose_shoulder}</pose>
          <geometry>
            <sphere>
            	<radius>0.3</radius>
            </sphere>    
          </geometry>
        </visual>

        <inertial>
          <pose>{pose_shoulder}</pose>
        </inertial>
      </link>


	  <link name="high_leg_{number}">
        <collision name="collision_high_leg_{number}_0">
          <pose>{pose_high}</pose>
          <geometry>
            <box>
            	<size>{L} {W} {H}</size>
            </box>    
          </geometry>
        </collision>
  
        <visual name="visual_high_leg_{number}_0">
          <pose>{pose_high}</pose>
          <geometry>
            <box>
            	<size>{L} {W} {H}</size>
            </box>    
          </geometry>
        </visual>

        <inertial>
          <pose>{pose_high}</pose>
          {inertia_st}
        </inertial>
      </link>

      <link name="low_leg_{number}">
        <collision name="collision_low_leg_{number}_0">
          <pose>{pose_low}</pose>
          <geometry>
            <box>
            	<size>{L} {W} {H}</size>
            </box>    
          </geometry>
        </collision>
  
        <visual name="visual_low_leg_{number}_0">
          <pose>{pose_low}</pose>
          <geometry>
            <box>
            	<size>{L} {W} {H}</size>
            </box>    
          </geometry>
        </visual>

        <inertial>
          <pose>{pose_low}</pose>
          {inertia_st}
        </inertial>
      </link>	

      <link name="fin_leg_{number}">
        <collision name="collision_fin_leg_{number}_0">
          <pose>{pose_fin}</pose>
          <geometry>
            <sphere>
            	<radius>{R}</radius>
            </sphere>    
          </geometry>
        </collision>

          <inertial>
            <pose>{pose_fin}</pose>
            <mass>0.1</mass>
            <inertia>
              <ixx>0.1</ixx>
              <iyy>0.1</iyy>
              <izz>0.1</izz>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyz>0</iyz>
            </inertia>
          </inertial>
  
        <visual name="visual_fin_leg_{number}_0">
          <pose>{pose_fin}</pose>
          <geometry>
            <sphere>
            	<radius>{R}</radius>
            </sphere>    
          </geometry>
        </visual>

        <inertial>
          <pose>{pose_fin}</pose>
        </inertial>
      </link>

      <joint name="joint_shoulder_{number}" type="revolute">
      	 <pose> {pjoint_shoulder} </pose>
         <parent>body</parent>
         <child>shoulder_{number}</child>
         <axis>
         	<xyz>0 0 {axis}</xyz>
	        <use_parent_model_frame>true</use_parent_model_frame>
	     </axis>
      </joint>

      <joint name="joint_high_{number}" type="revolute">
      	 <pose> {pjoint_shoulder} </pose>
         <parent>shoulder_{number}</parent>
         <child>high_leg_{number}</child>
         <axis>
         	<xyz>0 {axis} 0</xyz>
	        <use_parent_model_frame>true</use_parent_model_frame>
	     </axis>
      </joint>

      <joint name="joint_low_{number}" type="revolute">
      	 <pose> {pjoint_low} </pose>
         <parent>high_leg_{number}</parent>
         <child>low_leg_{number}</child>
         <axis>
          <xyz>0 {axis} 0</xyz>
	      <use_parent_model_frame>true</use_parent_model_frame>
         </axis>
      </joint>

      <joint name="joint_fin_{number}" type="fixed">
      	 <pose> {pjoint_fin} </pose>
         <parent>low_leg_{number}</parent>
         <child>fin_leg_{number}</child>
      </joint>
"""

legs = [0] * 6

WW = 0.5
L = 2
H = 0.5
W = 0.5
R = 0.4

LHWR = { "L":L, "H":H, "W":W, "R":R }

legs[0] = leg_template.format(number=0, axis="-1", inertia_st=inertia_st, pose_shoulder=f"-{WW}  {BODY_LENGTH/2}  0  0  0  0", pose_high=f"-{WW+L/2}  {BODY_LENGTH/2}  0  0  0  0", pose_low=f"-{WW+L/2*3}  {BODY_LENGTH/2}  0  0  0  0", pose_fin=f"-{WW+L/2*4}  {BODY_LENGTH/2}  0  0  0  0", pjoint_shoulder=f"-{WW}  {BODY_LENGTH/2}  0  0  0  0   ", pjoint_low=f"-{WW+L}  {BODY_LENGTH/2}  0  0  0  0   ", pjoint_fin=f"-{WW+L*2}  {BODY_LENGTH/2}  0  0  0  0   ", **LHWR)
legs[1] = leg_template.format(number=1, axis="-1", inertia_st=inertia_st, pose_shoulder=f"-{WW}  0                0  0  0  0", pose_high=f"-{WW+L/2}  0                0  0  0  0", pose_low=f"-{WW+L/2*3}  0                0  0  0  0", pose_fin=f"-{WW+L/2*4}  0                0  0  0  0", pjoint_shoulder=f"-{WW}  0                0  0  0  0   ", pjoint_low=f"-{WW+L}  0                0  0  0  0   ", pjoint_fin=f"-{WW+L*2}  0                0  0  0  0   ", **LHWR)
legs[2] = leg_template.format(number=2, axis="-1", inertia_st=inertia_st, pose_shoulder=f"-{WW} -{BODY_LENGTH/2}  0  0  0  0", pose_high=f"-{WW+L/2} -{BODY_LENGTH/2}  0  0  0  0", pose_low=f"-{WW+L/2*3} -{BODY_LENGTH/2}  0  0  0  0", pose_fin=f"-{WW+L/2*4} -{BODY_LENGTH/2}  0  0  0  0", pjoint_shoulder=f"-{WW} -{BODY_LENGTH/2}  0  0  0  0   ", pjoint_low=f"-{WW+L} -{BODY_LENGTH/2}  0  0  0  0   ", pjoint_fin=f"-{WW+L*2} -{BODY_LENGTH/2}  0  0  0  0   ", **LHWR)
legs[3] = leg_template.format(number=3, axis=" 1", inertia_st=inertia_st, pose_shoulder=f" {WW}  {BODY_LENGTH/2}  0  0  0  0", pose_high=f" {WW+L/2}  {BODY_LENGTH/2}  0  0  0  0", pose_low=f" {WW+L/2*3}  {BODY_LENGTH/2}  0  0  0  0", pose_fin=f" {WW+L/2*4}  {BODY_LENGTH/2}  0  0  0  0", pjoint_shoulder=f" {WW}  {BODY_LENGTH/2}  0  0  0  0", pjoint_low=f" {WW+L}  {BODY_LENGTH/2}  0  0  0  0", pjoint_fin=f" {WW+L*2}  {BODY_LENGTH/2}  0  0  0  0", **LHWR)
legs[4] = leg_template.format(number=4, axis=" 1", inertia_st=inertia_st, pose_shoulder=f" {WW}  0                0  0  0  0", pose_high=f" {WW+L/2}  0                0  0  0  0", pose_low=f" {WW+L/2*3}  0                0  0  0  0", pose_fin=f" {WW+L/2*4}  0                0  0  0  0", pjoint_shoulder=f" {WW}  0                0  0  0  0", pjoint_low=f" {WW+L}  0                0  0  0  0", pjoint_fin=f" {WW+L*2}  0                0  0  0  0", **LHWR)
legs[5] = leg_template.format(number=5, axis=" 1", inertia_st=inertia_st, pose_shoulder=f" {WW} -{BODY_LENGTH/2}  0  0  0  0", pose_high=f" {WW+L/2} -{BODY_LENGTH/2}  0  0  0  0", pose_low=f" {WW+L/2*3} -{BODY_LENGTH/2}  0  0  0  0", pose_fin=f" {WW+L/2*4} -{BODY_LENGTH/2}  0  0  0  0", pjoint_shoulder=f" {WW} -{BODY_LENGTH/2}  0  0  0  0", pjoint_low=f" {WW+L} -{BODY_LENGTH/2}  0  0  0  0", pjoint_fin=f" {WW+L*2} -{BODY_LENGTH/2}  0  0  0  0", **LHWR)

spider = """
    <model name="spider">
      <plugin name="mirmik" filename="libmirmik.so"/>
      <pose>0 0 3 0 0 0</pose>

      <link name="body">
        <collision name="collision_body_0">
          <pose>0 0 0 0 0 0</pose>
          <geometry>
            <box>
            	<size>1 {body_length} 0.5</size>
            </box>    
          </geometry>
        </collision>

        <visual name="visual_body_0">
          <pose>0 0 0 0 0 0</pose>
          <geometry>
            <box>
            	<size>1 {body_length} 0.5</size>
            </box>    
          </geometry>
        </visual>
      </link>

      {0}
      {1}
      {2}
      {3}
      {4}
      {5}

      <!--<joint name="krep" type="fixed">
         <pose> 0 0 0 0 0 0 </pose>
         <parent>world</parent>
         <child>body</child>
      </joint>-->

    </model>
""".format(*legs, 
		body_length=BODY_LENGTH+1)

txt = """<?xml version="1.0"?>
<sdf version="1.4">
  <world name="default">
    <plugin name="world" filename="libworld.so"/>

    <gravity>0 0 {gravity}</gravity>

    <include>
      <uri>model://sun</uri>
    </include>

    <!--<model name="arr">
     <link name="arr0">
        <collision name="collision_arr_0">
          <pose>0 0 0 0 0 0</pose>
          <geometry>
            <box>
              <size>10 10 1</size>
            </box>    
          </geometry>
        </collision>

        <visual name="visual_arr_0">
          <pose>0 0 0 0 0 0</pose>
          <geometry>
            <box>
              <size>10 10 1</size>
            </box>    
          </geometry>
        </visual>
      </link>
    </model>

      <joint name="krep2" type="fixed">
         <pose> 0 0 0 0 0 0 </pose>
         <parent>world</parent>
         <child>arr</child>
      </joint>-->

    <include>
      <uri>model://ground_plane</uri>
    </include>

    {spider}

  </world>
</sdf>
""".format(spider=spider, gravity=GRAVITY)

file.write(txt)