1. sudo apt-get install libgps-dev
2. compile agx_sdk independently
mkdir build && cd build
cmake .. && sudo make install -j8

3. xacro贴图
    <link name="marker_link">
      <visual>
      <origin xyz="0 0 0.008" rpy="0 0 0" />
        <geometry>
          <box size="0.2 0.2 1e-5" />
        </geometry>
      </visual>
      <collision>
      <origin xyz="0 0 0.008" rpy="0 0 0" />
        <geometry>
          <box size="0.2 0.2 1e-5" />
        </geometry>
      </collision>
    </link>
这里visual和collision都不能有name !!!
否则贴图无法显示

4. fatal error: opencv2/aruco.hpp: No such file or directory
   42 | #include <opencv2/aruco.hpp>
      |          ^~~~~~~~~~~~~~~~~~~
sudo ln -s /usr/include/opencv4/opencv2 /usr/include/

5. sdf中定义parent和link
<joint name='laser_livox_joint' type='fixed'>
      <child>laser_livox</child>
      <parent>base_link</parent>
    </joint>
xacro中定义
<joint name="laser_livox_joint" type="fixed">
    <parent link="base_link"/>  <!-- 父链接是 base_link -->
    <child link="livox"/>       <!-- 子链接是 livox -->
</joint>

5. 测量aruco大小
sudo apt-get install screenruler
测定整个板子和码的相对大小，然后用aruco_board.xacro中的box size进行解算

6. 编译livox_laser_simulation error: ‘InternalMetadataWithArena’ in namespace ‘google::protobuf::internal’ does not name a type;
记得conda deactivate！！
