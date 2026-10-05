# Điều khiển UR3e bằng LLM

## 1. Cài phần mềm cần thiết

Máy cần **Ubuntu 22.04**, **ROS 2 Humble** và **Gemini API key**.
Nếu chưa có ROS, cài theo [hướng dẫn ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

Sau khi cài ROS, mở terminal và chạy:

```bash
sudo apt update
sudo apt install -y build-essential cmake git \
  python3-colcon-common-extensions python3-rosdep python3-pytest python3-yaml \
  ros-humble-moveit ros-humble-ur-moveit-config ros-humble-ur-controllers \
  ros-humble-ros-gz ros-humble-ign-ros2-control ros-humble-ros2-controllers

sudo rosdep init
rosdep update
```

## 2. Clone repo

Chỉ làm bước này nếu chưa có repo trong workspace:

```bash
mkdir -p ~/workspaces/ur_gz/src
cd ~/workspaces/ur_gz
git clone -b assignment_3 https://github.com/TNSang25/UniversalRobotDemo.git src
```

## 3. Build

Mở terminal mới và chạy:

```bash
cd ~/workspaces/ur_gz
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1

rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --allow-overriding ur_description --cmake-force-configure
source install/setup.bash
ros2 run ur_task_planner check_install.py
```

Thấy `Installation OK` là có thể chuyển sang bước chạy.
`PYTHONNOUSERSITE=1` giúp tránh lỗi xung đột Python/pytest.

## 4. Chạy chương trình

**Terminal 1 — mở mô phỏng và camera RGB-D:**

```bash
cd ~/workspaces/ur_gz
export PYTHONNOUSERSITE=1
source install/setup.bash
ros2 launch ur_simulation_gz ur3e_rg2_table_depth.launch.py launch_rviz:=false
```

Đợi ba controller báo `Configured and activated`, rồi mở terminal 2.
Mỗi lần chạy, vị trí và góc yaw của 5 cube được sinh ngẫu nhiên.
Muốn lặp lại một bố cục để debug, thêm `random_seed:=42` vào lệnh launch.

**Terminal 2 — mở Rviz, skill executor và LLM planner:**

Thêm GEMINI API KEY:

```bash
cd ~/workspaces/ur_gz
export PYTHONNOUSERSITE=1
source install/setup.bash
export GEMINI_API_KEY=''
ros2 launch ur_task_planner task_planner.launch.py \
  scene_file:=scene_depth.yaml launch_observer:=true
```

Đợi dòng `Skill Executor Node ready.` trước khi gửi lệnh.

**Terminal 3 — gửi lệnh:**

```bash
export PYTHONNOUSERSITE=1
source ~/workspaces/ur_gz/install/setup.bash
ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'Đưa cube đỏ vào zone B.'}"
```

Theo dõi `/task_status` ở terminal 4 bên dưới. Đợi lệnh đầu báo `SUCCESS`,
sau đó gửi lệnh tiếp theo trong terminal 3:

```bash
ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'Đưa cube xanh dương vào zone B.'}"
```

**Terminal 4 — theo dõi kết quả (mở trước khi gửi lệnh):**

```bash
export PYTHONNOUSERSITE=1
source ~/workspaces/ur_gz/install/setup.bash
ros2 topic echo /task_status --full-length
```

Muốn xem vị trí cube và cube đang chiếm từng zone, mở thêm terminal và chạy:

```bash
export PYTHONNOUSERSITE=1
source ~/workspaces/ur_gz/install/setup.bash
ros2 topic echo /scene_state --full-length
```

Nếu đã có pose xác nhận nhưng ảnh mới thiếu cube, executor ghi tên cube thiếu
và tự đưa tay về `home`
để camera nhìn rõ, giữ nguyên kẹp nếu đang cầm cube, rồi chờ ảnh mới để quan sát
lại một lần.

Dừng chương trình bằng **Ctrl+C ở terminal 2, rồi terminal 1**.
