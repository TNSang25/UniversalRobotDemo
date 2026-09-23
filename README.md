# UniversalRobotDemo

Dự án này là không gian làm việc (workspace) dành cho mô phỏng tay máy Universal Robots (UR) trên Gazebo (Ignition) và sử dụng MoveIt 2 để lên kế hoạch quỹ đạo tại ROS 2. 

## 📦 Cấu trúc Package

Dự án bao gồm 3 package chính như sau:

### 1. `ur_description`
Đây là package chứa toàn bộ các file cấu hình về URDF, Xacro, lưới vật thể (meshes), và các thông số vật lý của các dòng tay máy UR khác nhau (ur3, ur3e, ur5, ur5e, ur10, ur10e...). 
- **Chức năng:** Cung cấp mô hình 3D, động học và động lực học của robot để MoveIt 2 và Gazebo có thể hiểu và mô phỏng chính xác hình dáng vật lý của tay máy.

### 2. `ur_simulation_gz`
Đây là package mô phỏng tay máy UR tích hợp với Gazebo (phiên bản mới) và MoveIt 2. 
- **Chức năng:** Bao gồm các file launch dùng để khởi động môi trường mô phỏng Gazebo, nạp mô hình robot vào thế giới giả lập, khởi chạy các controller (chẳng hạn như `joint_trajectory_controller`), và khởi động hệ thống MoveIt 2 để sẵn sàng nhận lệnh điều khiển.

### 3. `ur_trajectory_drawer`
Đây là package chứa các node (viết bằng C++) tương tác với `MoveGroupInterface` của MoveIt 2 để điều khiển đầu cuối của tay máy (end-effector: `tool0`) di chuyển theo một quỹ đạo đặc biệt trong không gian.
- **Chức năng:** Tính toán đường dẫn tọa độ Descartes (Cartesian path) và điều khiển robot di chuyển. Nó có thể yêu cầu tay máy vẽ các hình dạng nhất định (đường tròn, chữ S). Nó cũng sử dụng `visualization_msgs::msg::Marker` để hiển thị trước quỹ đạo sắp vẽ bằng đường màu đỏ trên RViz.

---

## 🚀 Hướng dẫn chạy

### Yêu cầu ban đầu
1. Mở một terminal, di chuyển vào workspace và build các package:
   ```bash
   cd ~/workspaces/ur_gz
   colcon build
   source install/setup.bash
   ```
2. Khởi động môi trường mô phỏng (Gazebo + RViz + MoveIt):
   ```bash
   ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e
   ```
   *Lưu ý: Bạn có thể đổi `ur_type` thành mẫu tay máy bạn cần (vd: `ur3e`, `ur10e`).*

### 1️⃣ Vẽ Hình Tròn (Circle)
Mở một terminal mới, source môi trường và chạy node vẽ hình tròn. Node này sẽ tính toán tọa độ Descartes theo hàm sin/cos để di chuyển đầu cuối tay máy theo dạng vòng tròn bán kính 10cm.

```bash
cd ~/workspaces/ur_gz
source install/setup.bash
ros2 launch ur_trajectory_drawer trajectory_drawer.launch.py
```
*Kết quả:* Bạn sẽ thấy RViz hiện lên một đường marker dạng hình tròn màu đỏ, sau đó robot bắt đầu thực thi chuyển động theo vòng tròn đó.

### 2️⃣ Vẽ Chữ S
Mở một terminal mới (hoặc dùng terminal sau khi chạy xong node hình tròn), chạy node vẽ chữ S. Node này sẽ điều khiển tay máy ghép 2 nửa đường tròn lại để tạo thành chữ S chiều cao tổng cộng khoảng 20cm.

```bash
cd ~/workspaces/ur_gz
source install/setup.bash
ros2 launch ur_trajectory_drawer s_drawer.launch.py
```
*Kết quả:* Bạn sẽ thấy đường viền Marker màu đỏ dạng chữ S xuất hiện trên RViz và robot sẽ mượt mà vẽ theo quỹ đạo hình chữ S đó.