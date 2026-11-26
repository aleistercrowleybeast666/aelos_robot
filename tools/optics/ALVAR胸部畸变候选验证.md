# 胸部 ALVAR 原始角点去畸变候选

候选沿用 640×480 胸部相机的 `plumb_bob` 标定。检测器照常解码，四个实测角点进入 OpenCV PnP 时传入相机消息的五个畸变系数。厂家代码的三个姿态解算重载最终都调用这一处求解。标定消息在启动时已完整保存，因此无需改变 `Camera` 类的内存布局。

只有 `/r3_optics_trial/ar_track_alvar` 或 `/chest/ar_track_alvar` 可以通过私参 `~use_raw_chest_distortion` 启用。默认关闭，运行中的求解器会保持首次读取的开关值。程序同时检查图像和标定话题、相机坐标系、图像尺寸及标定参数。头部节点和已经校正过的图像话题沿用原求解方式。若上游改变原图话题的像素处理方式，需重新验收后启用。对每个码先抽查从光轴至各角点的局部可逆性，并要求四角去畸变再投影回原图的误差均在 0.5 像素内。任何一个角失败时，本次码使用原算法并告警，后续码独立判断。这个自检可检出部分极角解算失败，判断标定是否贴合真实镜头仍需实拍。

## 生成并编译候选

在带 ROS Noetic 和 OpenCV 开发包的独立 catkin 工作区操作。将 `TOOLS` 指向本文件所在目录，`WS` 指向包含厂家源码副本及依赖的测试工作区。生成器检查厂家 `Camera.cpp` 和 `IndividualMarkersNoKinect.cpp` 的 SHA-256，可用来复核随附补丁。

```bash
source /opt/ros/noetic/setup.bash
ALVAR="$WS/src/ar_track_alvar/ar_track_alvar"
python3 "$TOOLS/build_alvar_chest_optics_patch.py" "$ALVAR" | cmp - "$TOOLS/r3_chest_raw_pose.patch"
(cd "$ALVAR" && patch --dry-run -p1 < "$TOOLS/r3_chest_raw_pose.patch" && patch -p1 < "$TOOLS/r3_chest_raw_pose.patch")
(cd "$WS" && catkin_make --pkg ar_track_alvar)
source "$WS/devel/setup.bash"
g++ -std=c++14 "$TOOLS/test_alvar_chest_optics.cpp" -o "$WS/test_alvar_chest_optics" \
  $(pkg-config --cflags --libs ar_track_alvar opencv4)
```

## 独立 ROS master 的原生测试

两个终端均设置 `ROS_MASTER_URI=http://127.0.0.1:11431` 并加载该工作区。第一个运行 `roscore -p 11431`。第二个执行以下 Bash 命令。每次都启动独立测试进程，直接调用 `Marker::UpdatePose` 使用的相机求解入口。测试使用含非零第五项的合成四角，并把结果与 OpenCV 对照。

```bash
export ROS_MASTER_URI=http://127.0.0.1:11431
test_pnp() {
  "$WS/test_alvar_chest_optics" "__ns:=$1" __name:=ar_track_alvar \
    "camera_image:=$2" camera_info:=/usb_cam_chest/camera_info \
    "_use_raw_chest_distortion:=$3" "_expect_distortion:=$4" "_edge_pixels:=${5:-false}"
}
test_pnp /r3_optics_trial /usb_cam_chest/image_raw true true
test_pnp /r3_optics_trial /usb_cam_chest/image_raw false false
test_pnp /r3_optics_trial /usb_cam_chest/image_rect true false
test_pnp /head /usb_cam_chest/image_raw true false
test_pnp /chest /usb_cam_chest/image_raw true true
test_pnp /chest /usb_cam_chest/image_raw true false true
```

## 现场画面的离线 A/B 回放

使用记录了胸部原图、CameraInfo 和同时间 TF 的 ROS bag。在上述独立 master 上分别运行原算法和候选算法，每轮从头播放相同的四个话题。两个求解器的 TF 广播共用 `ar_marker_*` 名称，应分轮运行。每轮使用新启动的 master，以清除上一轮 TF 和 ROS 时间。先将下列三个环境变量设为录包时的标记边长（cm）和两个检测门限。

```bash
export ROS_MASTER_URI=http://127.0.0.1:11431
rosparam set /use_sim_time true
rosrun ar_track_alvar individualMarkersNoKinect __ns:=/r3_optics_trial __name:=ar_track_alvar \
  camera_image:=/usb_cam_chest/image_raw camera_info:=/usb_cam_chest/camera_info \
  "_marker_size:=$MARKER_SIZE_CM" "_max_new_marker_error:=$NEW_MARKER_ERROR" "_max_track_error:=$TRACK_ERROR" \
  _output_frame:=/map _use_raw_chest_distortion:=true
```

在另一个连接同一测试 master 的终端运行回放。基线轮把上述私参设为 `false`。实际现场采集的 `marker_size` 和检测门限应与录包时配置一致。

```bash
rosbag play --clock "$BAG" --topics /usb_cam_chest/image_raw /usb_cam_chest/camera_info /tf /tf_static
```

逐帧对照 `/r3_optics_trial/ar_pose_marker` 中相同时间戳、相同 ID 的结果。用机器人已摆正的中央、左缘、右缘和不同距离观测比较横偏、航向和离散度。实机效果以这些对照为准。普通远程部署包不会安装该补丁。
