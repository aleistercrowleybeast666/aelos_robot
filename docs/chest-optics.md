# 胸部广角相机与扫码航向

三号机器人的胸部相机发布 640×480 原图。`CameraInfo.K` 给出焦距和主点，`CameraInfo.D` 保存厂家 `plumb_bob` 模型的五个系数，依次为 `k1, k2, p1, p2, k3`。其中三个径向系数描述镜头对离轴光线的弯曲，两个切向系数描述镜头元件偏心等因素产生的不对称畸变。

## 实际跑路线时的处理

在命令 0、1、2 的去程中，右跨结束后，机器人优先读取同一站姿的新 Tag 3，准备转入通往 Tag 4 的直行段。在机身坐标中，码越靠侧面，几帧一致的小角度估计也可能受镜头畸变影响。

当整窗的 Tag 3 都在机身前向左右 50° 以外、视觉偏角已经落在直行容差内，并且刚完成动作后的磁航向也在容差内，程序沿用本段已由 Tag 3 建立的航向参考，直接进入直行。这个判断要求磁样本与当前任务、负载和站姿对应。报告的 `tag3_straight_heading` 会标明沿用参考的原因。50° 由码相对机身的坐标估算，不能当作相机内参或镜头的校准角。偏差较大、码出现在较中央位置或磁样本无法佐证时，继续按当前路段的视觉纠偏流程处理。

### Tag 3 示教位置的航向零点

11 月 26 日 21:28 人工沿通道摆正机器人后，左侧 Tag 3 的三帧原始航向为 −2.628°、−2.640°、−2.603°。程序以中位数 −2.628° 为该视角的零点。相同位置读到 −2.628° 时，转向选择和磁航向参考都使用 0°。程序同时保留原始角度、补偿权重和修正结果。

该补偿仅用于去程 Tag 3 的右跨出发和右跨后转直行。程序比较相机指纹、检测器模式、码在机身坐标里的三维位置、码面法向及观测角度。观测偏离示教位置时，补偿平滑减至零。示教记录没有负载字段，单抱和双抱均须符合上述几何条件。转向判断仍需当前站姿的新帧。示教参数位于 [tag3_outbound_heading.json](../pi_deploy_bundle/app/config/tag3_outbound_heading.json)。

## 11 月 26 日场地粗标定

在五个独立站姿的黄白格里提取了 174 个角点，其中 172 个可用同一套模型稳定反解。按地面格线应当笔直来检验镜头，无需知道拍摄距离或格子边长。机器人脚底位置、相机安装俯仰和不同负载的机身倾角仍需各自标定。

| 格线位置 | 原图横格线误差（像素 RMS） | 实机参数校正后 |
| --- | ---: | ---: |
| 近处下沿 | 2.25 | 1.27 |
| 中距中央 | 0.73 | 0.55 |
| 画面左侧 | 0.59 | 0.56 |
| 画面右侧 | 0.36 | 0.29 |
| 近处上沿 | 1.26 | 0.96 |

另外固定内参，只让五个畸变系数同时乘以一个倍率。前四个站姿拟合出 1.425 倍。用第五个站姿的 30 个角点独立核对，按原始像素计算的格点到拟合曲线的 RMS 从原厂的 0.726 增至 0.843 像素。五姿态轮流留出时，调参后的平均误差也增加了 0.039 像素，因此部署包采用实机读取的原厂系数。光学对照未覆盖每个极角。

两份人工摆正的扫码记录也保留了实际偏差。Tag 3 在画面左侧时报告约 −2.63°。在另一处站姿，中央的 Tag 4 和左缘的 Tag 5 两次同帧航向相差 4.84° 和 5.18°。这些差异还含有码的铺设角和姿态解算误差，不能给所有码统一加减 5°。

[场地取角工具](../tools/optics/check_floor_grid.py) · [固定内参的留出验证工具](../tools/optics/fit_floor_distortion.py)

## 查看去畸变画面

在树莓派上加载与现有胸部相机相同的 ROS 环境，运行下面的命令。原图和去畸变图分别发布，便于同时查看。此节点只处理图像和标定，不调用运动服务。

```bash
python3 /home/lemon/aelos_auto_r3_python/chest_rectification.py ros
```

| 用途 | 图像话题 | 配对的 CameraInfo |
| --- | --- | --- |
| 厂家胸部原图，现有 ALVAR 输入 | `/usb_cam_chest/image_raw` | `/usb_cam_chest/camera_info` |
| 按厂家标定重映射的诊断图 | `/r3/chest_rect/image_rect` | `/r3/chest_rect/camera_info` |

可在另一终端运行 `rqt_image_view /r3/chest_rect/image_rect`。诊断节点配对相同时间戳的图像和标定，输出保留原时间戳和胸部相机坐标系。新图附带对应的内参和全零畸变系数。包内 `app/config/chest_optics.json` 保存这台机器人的厂家 K、D、R、P 和原相机指纹。需要指定这份配置时，可在 `ros` 参数后加 `--calibration /home/lemon/aelos_auto_r3_python/config/chest_optics.json`。相机指纹不匹配时使用实时 CameraInfo。画面边缘的黑区表示该输出位置超出原图范围，或厂家模型在该处不能支持可靠的映射。去畸变保留广角相机的透视效果，地面平行线在图上仍可趋向同一个消失点。

要比较校正前后同一条地面格线的弯曲程度，或在机器人摆正时比较同一码位于中央、左侧、右侧的航向读数，都无需测量相机到码的距离。使用原始分辨率，记录码的位置、当时的机身朝向和负载。仅凭一张直行照片无法独立求出全部镜头系数。

## 在同一段录像上验证实际扫码

有包含 `/usb_cam_chest/image_raw`、`/usb_cam_chest/camera_info` 和胸部相机到 `/map` 的 TF 的 ROS bag 时，可以直接运行厂家的 ALVAR 可执行程序。做两轮独立回放，分别将同一个检测器接到原图和去畸变图。这样可以对照是否解出了正确的 ID，以及解出后给出的位姿。

1. 在测试机上准备与录包时一致的 ROS 环境。每个终端先运行 `export ROS_MASTER_URI=http://127.0.0.1:11431`。每轮单独运行 `roscore -p 11431`，再在另一个终端运行 `rosparam set /use_sim_time true`。两轮之间退出测试进程并重启这个本地 master，以隔开同名的 `ar_marker_*` TF。
2. 第二轮先运行 `python3 /home/lemon/aelos_auto_r3_python/chest_rectification.py ros`。测试机上也可直接使用部署包 `app/` 下的同名脚本。将 `NEW_MARKER_ERROR` 和 `TRACK_ERROR` 设为录包时实际的检测及跟踪门限，两轮使用相同设置。若录包时改过检测频率、码的格数或边框格数，两轮也须传入相同私参。
3. 在一个终端启动检测器。第一轮取 `VIEW=raw`，第二轮取 `VIEW=rect`。码边长使用这台机器人已实测的 **4.4 cm**。

```bash
VIEW=raw
case "$VIEW" in
  raw)  IMAGE=/usb_cam_chest/image_raw; INFO=/usb_cam_chest/camera_info ;;
  rect) IMAGE=/r3/chest_rect/image_rect; INFO=/r3/chest_rect/camera_info ;;
esac
rosrun ar_track_alvar individualMarkersNoKinect __ns:=/r3_replay __name:=ar_track_alvar \
  "camera_image:=$IMAGE" "camera_info:=$INFO" _marker_size:=4.4 \
  "_max_new_marker_error:=${NEW_MARKER_ERROR:?请填写录制时的检测门限}" \
  "_max_track_error:=${TRACK_ERROR:?请填写录制时的跟踪门限}" _output_frame:=/map
```

4. 另开一个终端先运行 `rosbag record -O raw-result.bag /r3_replay/ar_pose_marker`，第二轮把输出名改成 `rect-result.bag`。随后在另一终端让两轮播放同一个 `$BAG`，均从同一起点开始并保留启动后的相机帧。

```bash
rosbag play --clock -r 0.5 "$BAG" --topics /usb_cam_chest/image_raw /usb_cam_chest/camera_info /tf /tf_static
```

5. 对照两份 `ar_pose_marker` 的相机帧时间戳和 ID，统计漏读的帧。对两种方式都读到的同一帧，对照已知摆正时的航向、位置和连续多帧波动。几何校正可能同时改变边缘四角的形状和可见区域，这两类结果都应记录。

## 可选的原生角点解算

当前厂家 ALVAR 在原图中检测码并取得四个角点，再用 PnP 计算三维位姿。在已核对的厂家版本里，这一步传入了内参和空畸变数组。启用上述诊断画面不会改变该节点输出给导航的位姿。

仓库另附原生补丁，供有 ROS Noetic 和 OpenCV 开发环境的队员离线验证。候选把原始角点、原图内参和完整五个畸变系数送入同一次 PnP。对每个码检查四角的局部可逆性和回投误差，任一角误差超过 0.5 像素或局部模型不可靠时，该次观测使用厂家解算。这个数值检查验证模型能否在原像素处往返。镜头在真实场地的精度还要用独立观测确认。

普通控制台部署更新树莓派应用，原生补丁单独构建，默认关闭。具体的源码校验、原生测试和独立 ROS master 上的同帧对照见[原生补丁验证步骤](../tools/optics/ALVAR胸部畸变候选验证.md)。
