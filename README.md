# Điều khiển UR3e bằng LLM

Ra lệnh bằng tiếng Việt hoặc tiếng Anh để robot UR3e gắp và đặt các khối trong Gazebo.
Ví dụ: **"Đưa khối màu đỏ vào vùng B."**

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

Nếu `rosdep init` báo đã tồn tại, bỏ qua lệnh đó. Chạy `rosdep update` như bình thường.

## 2. Clone repo

Chỉ làm bước này nếu chưa có repo trong workspace:

```bash
mkdir -p ~/workspaces/ur_gz/src
cd ~/workspaces/ur_gz
git clone -b assignment_2 https://github.com/TNSang25/UniversalRobotDemo.git src/UniversalRobotDemo
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

**Terminal 1 — mở mô phỏng:**

```bash
cd ~/workspaces/ur_gz
export PYTHONNOUSERSITE=1
source install/setup.bash
ros2 launch ur_simulation_gz ur3e_rg2_table.launch.py launch_rviz:=false
```

Đợi ba controller báo `Configured and activated`, rồi mở terminal 2.

**Terminal 2 — mở bộ điều khiển:**

Thay `YOUR_API_KEY` bằng Gemini API key của bạn:

```bash
cd ~/workspaces/ur_gz
export PYTHONNOUSERSITE=1
source install/setup.bash
export GEMINI_API_KEY='YOUR_API_KEY'
ros2 launch ur_task_planner task_planner.launch.py
```

Đợi dòng `Skill Executor Node ready.`

**Terminal 3 — gửi lệnh:**

```bash
export PYTHONNOUSERSITE=1
source ~/workspaces/ur_gz/install/setup.bash
ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'Đưa khối màu đỏ vào vùng B.'}"
```

Có thể đổi câu lệnh. Robot có khối **đỏ, xanh dương, vàng** và vùng **A, B, C**.
Gửi từng lệnh một và đợi robot làm xong trước khi gửi lệnh tiếp theo.

Muốn xem kết quả, mở thêm terminal và chạy:

```bash
export PYTHONNOUSERSITE=1
source ~/workspaces/ur_gz/install/setup.bash
ros2 topic echo /task_status --full-length
```

Dừng chương trình bằng **Ctrl+C ở terminal 2, rồi terminal 1**.

Hướng dẫn cập nhật repo, xử lý lỗi và thông tin kỹ thuật nằm trong
[tài liệu chi tiết](docs/SETUP_DETAILS.md).
