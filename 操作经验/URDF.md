# URDF

## 1、组成

- ### 声明信息：

  - ```xml
    <?xml version="1.0"?>
    <robot name="fishbot">
        <link>		</link>
        <joint>		</joint>
    
    </robot>
    ```

- ### 关键组件：

  - **Link（部件）：**

    - ```xml
      <link name="base_link">							<!-- 属性name指定部件名字-->
        	<visual>
          <origin xyz="0 0 0.0" rpy="0 0 0 "/>			<!-- 坐标 欧拉角 -->
          <geometry>
              <cylinder length="0.12" radius="0.10"/>  	<!--圆柱 高 半径-->
           </geometry>
        </visual>
      </link>
      ```

    - **标签定义：**

      - ```xml
        <visual></visual>           <!--显示形状-->
        ```

        - ```xml
          <geometry></geometry>	<!--几何形状-->
          ```

          - ```xml
            <box></box>				<!--长方体-->
            <box size="1 1 1" />  	<!--长宽高-->
            ```

          - ```xml
            <cylinder></cylinder>				<!--圆柱体-->
            <cylinder radius="1" length="0.5"/>	<!--半径 高度-->
            ```

          - ```xml
            <sphere></sphere>			<!--球体-->
            <sphere radius="0.015"/>	<!--半径-->
            ```

          - ```xml
            <mesh></mesh>		<!--第三方导出的模型文件-->
            <mesh filename="package://robot_description/meshes/base_link.DAE"/>
            ```

        - ```xml
          <origin></origin> 	<!--可选，默认在几何中心-->
          <origin xyz="0 0 0 " rpy="0 0 0 "></origin>  
          <!--xyz默认为零矢量，rpy表示翻滚、俯仰、偏航-->
          ```

        - ```xml
          <material></material>	<!--材质-->
          <material name="white">
              <color rgba="1.0 1.0 1.0 0.5" />  <!--红 绿 蓝 透明度（%）-->
          </material>
          ```

        - ```xml
          <collision></collision>	<!--碰撞属性-->
          <collision>
              <origin xyz="0 0 0.0" rpy="0 0 0"/>			<!--碰撞体的中心位姿-->
              <geometry>
              	<cylinder length="0.12" radius="0.10"/>	<!--用于碰撞检测的几何形状-->
              </geometry>
              <material name="blue">
              	<color rgba="0.1 0.1 1.0 0.5"/>			<!--使碰撞包围体可见-->
              </material>
          </collision>
          ```

        - ```xml
          <inertial></inertial> <!--惯性参数-->
          <inertial>
              <mass value="0.2"/>
              <inertia ixx="0.0122666" ixy="0" ixz="0" iyy="0.0122666" iyz="0" izz="0.02"/>
          </inertial>
          ```

          - 惯性参数配置：https://mp.weixin.qq.com/s/3L8Lilesy2W_WY5qup0gmA

- **Joint（关节）：**

  - **主要写明父子关系**

    - 连接类型：是否固定的，可以旋转的……
    - 父部件名
    - 子部件名
    - 父子之间相对位置
    - 父子之间的旋转轴

    

  - **建立一个雷达部件 laser_link , 并将其固定到 base_link **

    - ```xml
      <?xml version="1.0"?>
      <robot name="fishbot">
          
          <!--base link-->
          <link name = "base_link">
              <visual>
              	<origin xyz="0 0 0 0" rpy="0 0 0" />
                  <geometry>
                      <cylinder length="0.12" radius="0.10" />
                  </geometry>
              </visual>
          </link>
          
          <!-- laser link -->
          <link name="laser_link">
          	<visual>
              	<origin xyz="0 0 0" rpy="0 0 0"/>
                  <geometry>
                      <cylinder length="0.12" radius="0.10"/>
                  </geometry>
              </visual>
          </link>
          
          <!-- laser joint -->
          <joint name="laser_joint" type="fixed">
          	<parent link="base_link" />
              <child link="laser_link" />
              <origin xyz="0 0 0.075" />
          </joint>
          
      </robot>
      ```

  - **joint 标签：**

    - name 关节的名称
    - type 关节的类型
      - **revolute** 		旋转关节，绕单轴旋转，角度有上下限，比如舵机 0-180
      - **continuous**     旋转关节，可以绕单轴无限旋转，比如自行车的前后轮
      - **fixed**               固定关节，无法运动
      - **prismatic**        滑动关节，沿某一轴线移动的关节，有位置极限
      - **planer**            平面关节，允许在xyz，rx ry rz六个方向运动
      - **floating**           浮动关节，允许进行平移、旋转运动

  - **joint的子标签：**

    - **parent** 父 link 名称

      ```xml
      <parent link = "base_link"/>
      ```

    - **child** 子 link 名称

      ```xml
      <child link = "laser_link"/>
      ```

    - **origin** 父子之间的关系 xyz rpy

      ```xml
      <origin xyz="0 0 0.014" />
      ```

    - **axis** 围绕旋转的关节轴

      ```xml
      <axis xyz="0 0 1" />
      ```

## 2、URDF可视化

- **建立功能包：**

  - ```bash
    ros2 pkg create fishbot_description --build--type ament_python
    ```

- **建立URDF文件：**

  - ```bash
    cd bot_description
    mkdir urdf
    touch fishbot_base.urdf
    ```

- **编辑urdf文件：**

  - ```xml
    <?xml version="1.0"?>
    <robot name="fishbot">
        
        <!--base link-->
        <link name = "base_link">
            <visual>
            	<origin xyz="0 0 0 0" rpy="0 0 0" />
                <geometry>
                    <cylinder length="0.12" radius="0.10" />
                </geometry>
            </visual>
        </link>
        
        <!-- laser link -->
        <link name="laser_link">
        	<visual>
            	<origin xyz="0 0 0" rpy="0 0 0"/>
                <geometry>
                    <cylinder length="0.12" radius="0.10"/>
                </geometry>
            </visual>
        </link>
        
        <!-- laser joint -->
        <joint name="laser_joint" type="fixed">
        	<parent link="base_link" />
            <child link="laser_link" />
            <origin xyz="0 0 0.075" />
        </joint>
        
    </robot>
    ```

- **建立launch文件：**

  - ```bash
    mkdir launch
    touch display_rviz2.launch.py
    ```

- **编辑launch文件：**

  - ```python
    #以下是rviz
    import os
    from launch import LaunchDescription
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node
    from launch_ros.substitutions import FindPackageShare
    
    
    def generate_launch_description():
        package_name = 'fishbot_description'
        urdf_name = "fishbot_base.urdf"
    
        ld = LaunchDescription()
        pkg_share = FindPackageShare(package=package_name).find(package_name) 
        urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_model_path]
            )
    
        joint_state_publisher_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            arguments=[urdf_model_path]
            )
    
        rviz2_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            )
    
        ld.add_action(robot_state_publisher_node)
        ld.add_action(joint_state_publisher_node)
        ld.add_action(rviz2_node)
    
        return ld
    
    #以下是gazebo
    import os
    from launch import LaunchDescription
    from launch.actions import ExecuteProcess
    from launch_ros.actions import Node
    from launch_ros.substitutions import FindPackageShare
    
    
    def generate_launch_description():
        robot_name_in_model = 'fishbot'
        package_name = 'fishbot_description'
        urdf_name = "fishbot_gazebo.urdf"
    
        ld = LaunchDescription()
        pkg_share = FindPackageShare(package=package_name).find(package_name) 
        urdf_model_path = os.path.join(pkg_share, f"urdf/{urdf_name} ")
    
        # Start Gazebo server
        start_gazebo_cmd = ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen')
    
        # Launch the robot
        spawn_entity_cmd = Node(
            package='gazebo_ros', 
            executable='spawn_entity.py',
            arguments=['-entity', robot_name_in_model,  '-file', urdf_model_path ], output='screen')
    
        ld.add_action(start_gazebo_cmd)
        ld.add_action(spawn_entity_cmd)
    
        return ld
    ```

- **修改setup.py：**

  - ```python
    #导入头文件
    from glob import glob
    import os
    #加入目录安装
     data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
            (os.path.join('share', package_name, 'urdf'), glob('urdf/**')),
        ],
    ```

- **编译、source、运行：**

  - ```bash
    #运行rviz
    ros2 launch fishbot_description display_rviz2.launch.py
    #运行gazebo
    ros2 launch fishbot_description gazebo.launch.py
    ```

  

## 3、Fishbot源码


```xml
<?xml version="1.0"?>
<robot name="fishbot">


  <!-- Robot Footprint -->
  <link name="base_footprint"/>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0.0 0.0 0.076" rpy="0 0 0"/>
  </joint>


  <!-- base link -->
  <link name="base_link">
  	<visual>
      <origin xyz="0 0 0.0" rpy="0 0 0"/>
      <geometry>
		<cylinder length="0.12" radius="0.10"/>
      </geometry>
      <material name="blue">
      	<color rgba="0.1 0.1 1.0 0.5" /> 
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.0" rpy="0 0 0"/>
      <geometry>
		<cylinder length="0.12" radius="0.10"/>
      </geometry>
      <material name="blue">
      	<color rgba="0.1 0.1 1.0 0.5" /> 
      </material>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0122666" ixy="0" ixz="0" iyy="0.0122666" iyz="0" izz="0.02"/>
    </inertial>
  </link>
    
  <!-- laser link -->
  <link name="laser_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.02" radius="0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0.0 0.0 0.0 0.5" /> 
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.02" radius="0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0.0 0.0 0.0 0.5" /> 
      </material>
    </collision>
    <inertial>
    <mass value="0.1"/>
      <inertia ixx="0.000190416666667" ixy="0" ixz="0" iyy="0.0001904" iyz="0" izz="0.00036"/>
    </inertial>
  </link>
  
  <!-- laser joint -->
  <joint name="laser_joint" type="fixed">
      <parent link="base_link" />
      <child link="laser_link" />
      <origin xyz="0 0 0.075" />
  </joint>

  <link name="imu_link">
  	<visual>
      <origin xyz="0 0 0.0" rpy="0 0 0"/>
      <geometry>
		    <box size="0.02 0.02 0.02"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.0" rpy="0 0 0"/>
      <geometry>
		    <box size="0.02 0.02 0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
        <inertia ixx="0.000190416666667" ixy="0" ixz="0" iyy="0.0001904" iyz="0" izz="0.00036"/>
      </inertial>
  </link>

  <!-- imu joint -->
  <joint name="imu_joint" type="fixed">
      <parent link="base_link" />
      <child link="imu_link" />
      <origin xyz="0 0 0.02" />
  </joint>


  <link name="left_wheel_link">
      <visual>
        <origin xyz="0 0 0" rpy="1.57079 0 0"/>
        <geometry>
          <cylinder length="0.04" radius="0.032"/>
        </geometry>
          <material name="black">
            <color rgba="0.0 0.0 0.0 0.5" /> 
          </material>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="1.57079 0 0"/>
        <geometry>
          <cylinder length="0.04" radius="0.032"/>
        </geometry>
          <material name="black">
            <color rgba="0.0 0.0 0.0 0.5" /> 
          </material>
      </collision>
      <inertial>
        <mass value="0.2"/>
          <inertia ixx="0.000190416666667" ixy="0" ixz="0" iyy="0.0001904" iyz="0" izz="0.00036"/>
        </inertial>
  </link>
    
  <link name="right_wheel_link">
      <visual>
        <origin xyz="0 0 0" rpy="1.57079 0 0"/>
        <geometry>
          <cylinder length="0.04" radius="0.032"/>
        </geometry>
          <material name="black">
            <color rgba="0.0 0.0 0.0 0.5" /> 
          </material>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="1.57079 0 0"/>
        <geometry>
          <cylinder length="0.04" radius="0.032"/>
        </geometry>
          <material name="black">
            <color rgba="0.0 0.0 0.0 0.5" /> 
          </material>
      </collision>
      <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.000190416666667" ixy="0" ixz="0" iyy="0.0001904" iyz="0" izz="0.00036"/>
    </inertial>
  </link>
    
  <joint name="left_wheel_joint" type="continuous">
      <parent link="base_link" />
      <child link="left_wheel_link" />
      <origin xyz="-0.02 0.10 -0.06" />
      <axis xyz="0 1 0" />
  </joint>

  <joint name="right_wheel_joint" type="continuous">
      <parent link="base_link" />
      <child link="right_wheel_link" />
      <origin xyz="-0.02 -0.10 -0.06" />
      <axis xyz="0 1 0" />
  </joint>

  <link name="caster_link">
      <visual>
        <origin xyz="0 0 0" rpy="1.57079 0 0"/>
        <geometry>
            <sphere radius="0.016"/>
        </geometry>
          <material name="black">
            <color rgba="0.0 0.0 0.0 0.5" /> 
          </material>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="1.57079 0 0"/>
        <geometry>
            <sphere radius="0.016"/>
        </geometry>
          <material name="black">
            <color rgba="0.0 0.0 0.0 0.5" /> 
          </material>
      </collision>
      <inertial>
      <mass value="0.02"/>
      <inertia ixx="0.000190416666667" ixy="0" ixz="0" iyy="0.0001904" iyz="0" izz="0.00036"/>
    </inertial>
  </link>
    
  <joint name="caster_joint" type="fixed">
      <parent link="base_link" />
      <child link="caster_link" />
      <origin xyz="0.06 0.0 -0.076" />
      <axis xyz="0 1 0" />
  </joint>



  <gazebo reference="caster_link">
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="caster_link">
    <mu1 value="0.0"/>
    <mu2 value="0.0"/>
    <kp value="1000000.0" />
    <kd value="10.0" />
    <!-- <fdir1 value="0 0 1"/> -->
  </gazebo>

<!--gazebo的两轮差速插件：gazebo_ros_diff_drive-->
<!--用于控制机器人轮子关节的位置变化，还可计算位姿（里程计）-->
<!--输入：轮距直径+控制指令    输出：里程计+里程计tf+轮子tf-->
  <gazebo>
    <plugin name='diff_drive' filename='libgazebo_ros_diff_drive.so'>
        <!--ros相关配置，包括命名空间和话题重映射等--> 
        <ros>
            <namespace>/</namespace>
            <remapping>cmd_vel:=cmd_vel</remapping>
            <remapping>odom:=odom</remapping>
          </ros>
      	  <!--数据更新速率-->
          <update_rate>30</update_rate>
          <!-- wheels -->
          <!--左右关节名称-->
          <left_joint>left_wheel_joint</left_joint>
          <right_joint>right_wheel_joint</right_joint>
          <!--轮距直径-->
          <wheel_separation>0.2</wheel_separation>
          <wheel_diameter>0.065</wheel_diameter>
          <!-- 轮子最大力矩、最大加速度-->
          <max_wheel_torque>20</max_wheel_torque>
          <max_wheel_acceleration>1.0</max_wheel_acceleration>
          <!-- 发布开关 -->
          <publish_odom>true</publish_odom>
          <publish_odom_tf>true</publish_odom_tf>
          <publish_wheel_tf>false</publish_wheel_tf>
          <odometry_frame>odom</odometry_frame>
          <robot_base_frame>base_footprint</robot_base_frame>
      </plugin>


      <plugin name="fishbot_joint_state" filename="libgazebo_ros_joint_state_publisher.so">
        <ros>
          <remapping>~/out:=joint_states</remapping>
        </ros>
        <update_rate>30</update_rate>
        <joint_name>right_wheel_joint</joint_name>
        <joint_name>left_wheel_joint</joint_name>
      </plugin>    
      </gazebo> 

      <gazebo reference="laser_link">
        <material>Gazebo/Black</material>
      </gazebo>

    <gazebo reference="imu_link">
      <sensor name="imu_sensor" type="imu">
      <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
          <ros>
            <namespace>/</namespace>
            <remapping>~/out:=imu</remapping>
          </ros>
          <initial_orientation_as_reference>false</initial_orientation_as_reference>
        </plugin>
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <visualize>true</visualize>
        <imu>
          <angular_velocity>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
                <bias_mean>0.0000075</bias_mean>
                <bias_stddev>0.0000008</bias_stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
                <bias_mean>0.0000075</bias_mean>
                <bias_stddev>0.0000008</bias_stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
                <bias_mean>0.0000075</bias_mean>
                <bias_stddev>0.0000008</bias_stddev>
              </noise>
            </z>
          </angular_velocity>
          <linear_acceleration>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
                <bias_mean>0.1</bias_mean>
                <bias_stddev>0.001</bias_stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
                <bias_mean>0.1</bias_mean>
                <bias_stddev>0.001</bias_stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
                <bias_mean>0.1</bias_mean>
                <bias_stddev>0.001</bias_stddev>
              </noise>
            </z>
          </linear_acceleration>
        </imu>
      </sensor>
    </gazebo>

    <gazebo reference="laser_link">
      <sensor name="laser_sensor" type="ray">
      <always_on>true</always_on>
      <visualize>true</visualize>
      <update_rate>5</update_rate>
      <pose>0 0 0.075 0 0 0</pose>
      <ray>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1.000000</resolution>
              <min_angle>0.000000</min_angle>
              <max_angle>6.280000</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.120000</min>
            <max>3.5</max>
            <resolution>0.015000</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
      </ray>

      <plugin name="laserscan" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <!-- <namespace>/tb3</namespace> -->
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>laser_link</frame_name>
      </plugin>
      </sensor>
    </gazebo>

</robot>
```

