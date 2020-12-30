# Hướng dẫn collect data từ LGSVL Simulator sử dụng ROS-1/ROS-2
## Chuẩn bị phần cứng
- Hệ điều hành: Ubuntu 18.04
- RAM: 32GB
- GPU: NVIDIA GeForce RTX 2080 Ti

## Chuẩn bị phần mềm
- [Cài đặt Docker](https://docs.docker.com/engine/install/ubuntu/)
- [Cài đặt Nvidia driver](https://www.linuxbabe.com/ubuntu/install-nvidia-driver-ubuntu-18-04)
- [Cài đặt Nvidia Docker](https://github.com/NVIDIA/nvidia-docker)

##Cài đặt collect data sử dụng ROS-1

**B1.** Tải và chạy bản LGSVL 20.06

```
cd $HOME
wget https://github.com/lgsvl/simulator/releases/download/2020.06/lgsvlsimulator-linux64-2020.06.zip -O lgsvlsimulator-linux64-2020.06.zip
unzip lgsvlsimulator-linux64-2020.06.zip
rm lgsvlsimulator-linux64-2020.06.zip
cd lgsvlsimulator-linux64-2020.06
chmod u+x simulator
./simulator
```

**B2.** Mở Chrome truy cập vào địa chỉ: [http://localhost:8080/#/Vehicles](http://localhost:8080/#/Vehicles), click Add new để tạo mới 1 con xe với thông tin sau:

- Vehicle Name: Collect Data Vehicle
- Vehicle URL: [https://assets.lgsvlsimulator.com/21c5f6b4ac54695a0f514d1272b6314356168cf2/vehicle_Lincoln2017MKZ](https://assets.lgsvlsimulator.com/21c5f6b4ac54695a0f514d1272b6314356168cf2/vehicle_Lincoln2017MKZ)

Click **Submit**

Chọn tiếp phần cài đặt xe vừa tạo

Cài đặt với các thông tin sau:

- Bridge Type: ROS
- Sensors

```
[
  {
    "type": "2D Ground Truth",
    "name": "2D Ground Truth",
    "params": {
      "Frequency": 10,
      "Topic": "export_2d_ground_truth_data_topic"
    },
    "transform": {
      "x": 0,
      "y": 1.7,
      "z": -0.2,
      "pitch": 0,
      "yaw": 0,
      "roll": 0
    }
  }
]
```

Sau đó click **Submit**

**Note:** Để cho đơn giản, tutorial này chỉ hướng dẫn xuất thử 2d ground truth data, đối với các loại sensor data khác, xem chi tiết config tại đây

**B3.** Truy cập địa chỉ [http://localhost:8080/#/Simulations](http://localhost:8080/#/Simulations), Click Add new để tạo mới một Simulation với thông tin sau:

Tab General

- Simulation Name: Collect Data Simulation
- Select Cluster: Local Machine
- API Only: Không check
- Headless Mode:  Không check

Tab Map & Vehicle

- Interactive Mode: Check
- Select Map: BoregasAve
- Select Vehicles:
- Vehicle: Collect Data Vehicle
- ROS Bridge Address: localhost:9090

Tab Traffic

- Use Predefined Seed: Không Check
- Random Traffic: Check
- Random Pedestrians: Check

Tab Weather

- Time of day: 10:00
- Rain: 0
- Wetness: 0
- Fog: 0
- Cloudiness: 0

Click **Submit**

Chọn Simulation Collect Data Simulation và Start Play.

Tại giao diện của LGSVL Simulator

- Click vào nút Play (số 1)
- Mở Tab Visualize (số 2)
- Bật con mắt (số 3)
- Quan sát các box xanh bao quanh các phương tiện di chuyển xung quanh xe (số 4), mục tiêu là sẽ lấy ra được tọa độ của các box này.

Chọn tab **Bridge**, quan sát các giá trị hiện tại trên Tab Bridge

- Bridge Status: Disconnected

**B4.** Clone project lanefollowing, build docker container

```
cd $HOME
git clone https://github.com/hzung/lanefollowing.git
cd lanefollowing
git checkout r1.0.0
./docker/scripts/build.sh
```

Kết quả build được image: `lgsvl/lanefollowing:latest`

**B5.** Start container

```
cd lanefollowing
docker run -dt --rm --name lanefollowing -p 9090:9090 -v $(pwd):/lanefollowing lgsvl/lanefollowing:latest
docker exec -it lanefollowing /bin/bash
```

**B6.** Bên trong shell của container, build ROS

```
./scripts/build.sh
```

**B7.** Bên trong shell của container, build LGSVL ROS-1 message

```
source /opt/ros/${ROS1_DISTRO}/setup.bash
cd /lanefollowing/catkin_ws/src
git clone https://github.com/lgsvl/lgsvl_msgs.git
cd /lanefollowing/catkin_ws/
catkin_make
source /lanefollowing/catkin_ws/devel/setup.bash

# Xác nhận build
rosmsg list | grep 'lgsvl'

# Kết quả
lgsvl_msgs/BoundingBox2D
lgsvl_msgs/BoundingBox3D
...
```

**Note:** Trong các lần tiếp theo, mỗi lần chạy ros1, chúng ta cần chạy 2 lệnh này đầu tiên để setup các thứ liên quan để có thể gọi được các lệnh ROS

```
source /opt/ros/${ROS1_DISTRO}/setup.bash
source /lanefollowing/catkin_ws/devel/setup.bash
```

**B8.** Bên trong shell của container, chạy ROS-1 Bridge

```
roslaunch rosbridge_server rosbridge_websocket.launch
```

Kết quả là một ros bridge server chạy tại cổng 9090 trên container.

Lúc này, bên tab Bridge của LGSVL Simulator sẽ chuyển sang trạng thái connected

Và bắt đầu publish message vào bên trong container. Sử dụng thêm một số lệnh sau để xác thực việc docker container đã nhận được message từ LGSVL:

**B9.** Bên trong shell của container

```
# Setup ros1
source /opt/ros/${ROS1_DISTRO}/setup.bash
source /lanefollowing/catkin_ws/devel/setup.bash

# Liệt kê các topic
rostopic list

# Danh sách topic, trong đó có topic bắn theo topic cài đặt ở B2
...
/export_2d_ground_truth_data_topic
...

# Log các message được bắn ra trong topic 
rostopic echo /export_2d_ground_truth_data_topic

# Kết quả
```
Như vậy chúng ta đã collect được data từ LGSVL Simulator sử dụng ROS-1

## Cài đặt collect data sử dụng ROS-2
**B1.** Tải và chạy bản LGSVL 20.06

```
cd $HOME
wget https://github.com/lgsvl/simulator/releases/download/2020.06/lgsvlsimulator-linux64-2020.06.zip -O lgsvlsimulator-linux64-2020.06.zip
unzip lgsvlsimulator-linux64-2020.06.zip
rm lgsvlsimulator-linux64-2020.06.zip
cd lgsvlsimulator-linux64-2020.06
chmod u+x simulator
./simulator
```

**B2.** Mở Chrome truy cập vào địa chỉ: [http://localhost:8080/#/Vehicles](http://localhost:8080/#/Vehicles), click Add new để tạo mới 1 con xe với thông tin sau:

- Vehicle Name: Collect Data Vehicle
- Vehicle URL: [https://assets.lgsvlsimulator.com/21c5f6b4ac54695a0f514d1272b6314356168cf2/vehicle_Lincoln2017MKZ](https://assets.lgsvlsimulator.com/21c5f6b4ac54695a0f514d1272b6314356168cf2/vehicle_Lincoln2017MKZ)

Click **Submit**

Chọn tiếp phần cài đặt xe vừa tạo

Cài đặt với các thông tin sau:

- Bridge Type: ROS2
- Sensors

```
[
  {
    "type": "2D Ground Truth",
    "name": "2D Ground Truth",
    "params": {
      "Frequency": 10,
      "Topic": "export_2d_ground_truth_data_topic"
    },
    "transform": {
      "x": 0,
      "y": 1.7,
      "z": -0.2,
      "pitch": 0,
      "yaw": 0,
      "roll": 0
    }
  }
]
```

Sau đó click **Submit**

**Note:** Để cho đơn giản, tutorial này chỉ hướng dẫn xuất thử 2d ground truth data, đối với các loại sensor data khác, xem chi tiết config tại đây

**B3.** Truy cập địa chỉ [http://localhost:8080/#/Simulations](http://localhost:8080/#/Simulations), Click Add new để tạo mới một Simulation với thông tin sau:

Tab General

- Simulation Name: Collect Data Simulation
- Select Cluster: Local Machine
- API Only: Không check
- Headless Mode:  Không check

Tab Map & Vehicle

- Interactive Mode: Check
- Select Map: BoregasAve
- Select Vehicles:
- Vehicle: Collect Data Vehicle
- ROS Bridge Address: localhost:9090

Tab Traffic

- Use Predefined Seed: Không Check
- Random Traffic: Check
- Random Pedestrians: Check

Tab Weather

- Time of day: 10:00
- Rain: 0
- Wetness: 0
- Fog: 0
- Cloudiness: 0

Click **Submit**

Chọn Simulation Collect Data Simulation và Start Play.

Tại giao diện của LGSVL Simulator

- Click vào nút Play (số 1)
- Mở Tab Visualize (số 2)
- Bật con mắt (số 3)
- Quan sát các box xanh bao quanh các phương tiện di chuyển xung quanh xe (số 4), mục tiêu là sẽ lấy ra được tọa độ của các box này.

Chọn tab **Bridge**, quan sát các giá trị hiện tại trên Tab Bridge

Bridge Status: Disconnected

**B4.** Clone project lanefollowing, build docker container

```
cd $HOME
git clone https://github.com/hzung/lanefollowing.git
cd lanefollowing
git checkout r1.0.0
./docker/scripts/build.sh
```

Kết quả build được image: `lgsvl/lanefollowing:latest`
**B5.** Start container

```
cd lanefollowing
docker run -dt --rm --name lanefollowing -p 9090:9090 -v $(pwd):/lanefollowing lgsvl/lanefollowing:latest
docker exec -it lanefollowing /bin/bash
```

**B6.** Bên trong shell của container, build ROS

```
./scripts/build.sh
```

**B7.** Bên trong shell của container, build LGSVL ROS-1 message

```
source /opt/ros/${ROS2_DISTRO}/setup.bash
cd /lanefollowing/ros2_ws/src
git clone https://github.com/lgsvl/lgsvl_msgs.git
cd lgsvl_msgs
git checkout ${ROS2_DISTRO}-devel
colcon build
source install/setup.bash

# Xác nhận build
ros2 msg list | grep 'lgsvl'

# Kết quả
lgsvl_msgs/msg/BoundingBox2D
lgsvl_msgs/msg/BoundingBox3D
...
```

**B8.** Bên trong shell của container, cài đặt và chạy LGSVL ROS2 Bridge

```
# Cài đặt bridge
cd /lanefollowing/ros2_ws/src
git clone https://github.com/lgsvl/ros2-lgsvl-bridge.git
cd ros2-lgsvl-bridge
git checkout ${ROS2_DISTRO}-devel
colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Release'
source install/setup.bash

# Chạy lgsvl bridge
lgsvl_bridge
```

**Note:** Trong các lần tiếp theo, mỗi lần chạy ros2, chúng ta cần chạy lệnh này đầu tiên để setup các thứ liên quan để có thể gọi được các lệnh ROS2

```
source /opt/ros/${ROS2_DISTRO}/setup.bash
source /lanefollowing/ros2_ws/src/lgsvl_msgs/install/setup.bash
source /lanefollowing/ros2_ws/src/ros2-lgsvl-bridge/install/setup.bash
```

Kết quả là một lgsvl ros2 bridge server chạy tại cổng 9090 trên container.

Lúc này, bên tab Bridge của LGSVL Simulator sẽ chuyển sang trạng thái connected

Và bắt đầu publish message vào bên trong container. Sử dụng thêm một số lệnh sau để xác thực việc docker container đã nhận được message từ LGSVL:

**B9.** Bên trong shell của container

```
# Setup ros2
source /opt/ros/${ROS2_DISTRO}/setup.bash
source /lanefollowing/ros2_ws/src/lgsvl_msgs/install/setup.bash
source /lanefollowing/ros2_ws/src/ros2-lgsvl-bridge/install/setup.bash

# Liệt kê các topic
ros2 topic list

# Danh sách topic, trong đó có topic bắn theo topic cài đặt ở B2
...
/lgsvl/export_2d_ground_truth_data_topic
...

# Log các message được bắn ra trong topic 
ros2 topic echo /lgsvl/export_2d_ground_truth_data_topic

# Kết quả

Như vậy chúng ta đã collect được data từ LGSVL Simulator sử dụng ROS-2
```
