# AELOS R3 控制台

用于在 Windows 上连接 AELOS 3 号机器人、预览胸部相机、执行自动任务和部署树莓派程序。仓库的 `pi_deploy_bundle/` 是当前发布的树莓派部署包。

## 下载和打开

1. 从 [GitHub Releases](https://github.com/aleistercrowleybeast666/aelos_robot/releases/latest) 下载 `AelosR3Control-windows-x64.zip`。
2. 解压整个压缩包，双击 `AelosR3Control.exe`。同一目录的 `robot.exe` 提供命令行，两个程序共用 `_internal` 目录。
3. 在「SSH配置与程序部署」填写树莓派的 SSH 地址、端口、用户名和密码。默认地址为 `192.168.1.100`，端口 `22`，用户名 `lemon`。保存的密码由当前 Windows 用户加密。

首次连接时，用系统 OpenSSH 对照实际树莓派的主机指纹建立本机记录。例如在终端运行 `ssh lemon@192.168.1.100`，核对指纹后接受，再返回控制台连接。

## 部署树莓派程序

进入「SSH配置与程序部署」并选择部署来源。两种方式都会核对 `manifest.json` 及清单中的文件 SHA-256，再通过 SSH 上传。部署会在树莓派上保留上一版备份，默认替换 `/home/lemon/aelos_auto_r3_python`。需要回退时使用同一页的「恢复备份」。

| 来源 | 用法 |
| --- | --- |
| 远程部署（默认） | 点击「部署程序」时下载仓库的当前部署包。默认地址为 `https://raw.githubusercontent.com/aleistercrowleybeast666/aelos_robot/main/pi_deploy_bundle`。可改为任意提供同一目录结构的 HTTPS 地址。 |
| 本地部署 | 选择含 `manifest.json`、`app/` 和 `owner/` 的目录。可提前保存仓库中的 `pi_deploy_bundle/`，用于现场离线部署。 |

远程地址可填部署包目录、`manifest.json` 的 HTTPS 下载地址，或对应的 GitHub 目录链接。使用默认地址时，控制台一次下载 GitHub 的 `main` 归档，再从中读取和校验部署包。Windows 控制台负责下载，树莓派只需可从本机通过 SSH 访问。

普通树莓派部署沿用机上现有 MCU 登记。已经更新 MCU 并拿到读回回执时，可在部署时登记回执。

如旧版 R3 镜像的控制节点一直等待串口，ROS 重载时可能出现 SSH 在线但控制服务套接字消失。对已识别的旧版串口程序，控制台会先备份并安装退出修复，再提示重启树莓派一次。看到该提示后，重连并再次部署。已修复的机器可正常热部署。

本版更新了 R3 的小转、持物小跨和空载连续侧跨。将 [R3 MCU 动作覆盖包](https://github.com/aleistercrowleybeast666/aelos_robot/releases/download/v1.0.7/AelosR3-MCU-return-and-strafe.zip) 解压，在 3 号机 U 盘模式下将包内的 `lua` 和 `dat` 合并到对应目录，再重启并部署树莓派程序。命令 2 回程会向当前 MCU 查询连续侧跨能力，旧 MCU 可按原有动作逐跨执行。控制台显示返程读到的码、实际纠偏动作和复测结果。

## 控制机器人

在「控制」页点击「连接机器人」。连接成功后会自动尝试打开胸部相机，也可手动开关。可以分别启动单程搬运原始物资、单程搬运转运物资和连续搬运转运物资。

控制台显示任务状态、回执和最近的扫码或纠偏诊断。停止按钮经独立连接取消后续动作。收到任务结果后，运行日志会显示本机 JSON 报告的实际保存路径。

比赛部署包在 `app/config/route_steps.json` 中启用了 `continue_estimated_route`。命令 0、1、2 会在当前路段尝试找码和纠偏。仍缺少可靠位姿时，沿该段剩余动作前进，到下一动作边界再看码。报告分别记录实际确认的码和估算进度。命令 1、2 在规定取物点确认只有一块时单抓，数量无法判断时默认双抱，实际件数和落点需现场确认。

Tag 3 右跨转直行时，程序优先用同一站姿新读到的 Tag 3 校准航向。远处 Tag 4 主要用于估计距离。配置项 `tag4_forward_trial.heading_use_within_m` 默认为 `0.35` 米，是试跑使用的分界，可根据场地实测调整。

胸部广角画面的查看方式和 Tag 3 边缘纠偏条件见[胸部相机与扫码航向](docs/chest-optics.md)。

如果任务中相机短时断流，程序按当前路段重新取帧。人工停止或控制连接中断后，已经下发且完成状态未知的动作保留在报告里。重新连接时，同一次抓取或行走请求只会分派一次。若控制台提示上一趟可能持物，取下方块、摆稳机器人后点击「恢复空载站姿」。

### 命令 2 的两趟搬运

首趟放物起立后，机器人按以下名义步数空载返回。左右均以当时的机身朝向为准。

| 区段 | 名义动作 |
| --- | --- |
| 大本营 → Tag 6 | 掉头 180°，扫 Tag 5 纠偏，前进 10 步，扫 Tag 6 |
| Tag 6 → Tag 7 | 左跨 6 步，接着前进 10 步，扫 Tag 7。新 MCU 在航向稳定时连续左跨 3＋3 步 |
| Tag 7 → 中转区 | 右跨 9 步，扫 Tag 1，前进 12 步，掉头 180°，从去程方向重扫 Tag 1 |
| 接驳取物 → 第二趟去程 | 左跨 4 步、前进 4 步，识别底排后单抓或双抱。抱稳后沿名义路线后退 4 步、右跨 4 步，重扫 Tag 1，沿去程通道送入大本营，放下后结束 |

回程 Tag 5、6、7、1 的目标航向相对各自码面依次为 180°、0°、0°、180°。中转区掉头后，再用新帧按 0° 校准 Tag 1。回程时，程序用同一码的多帧视觉航向更新磁航向参考。扣除帧间波动后，余差超过配置容差（默认 8°）且 MCU 已确认支持空载小转时，会先小转再扫码。超过 25.5° 时可选空载大转。回程中相机正常、漏码且有可能走过头时，程序先后退两步重扫。若新帧仍指向同一码且距离过近，可再退两步。读到远码或其他码时按该路段的观测和行进方向处理。

树莓派程序的 `checkpoint_scan.py` 提供共用的读帧和检查点扫码入口。`checkpoint_alignment.py` 负责视觉转向和磁航向参考，`checkpoint_errors.py` 区分相机故障、控制连接异常和漏码。各路段仍在自己的路线文件中决定是否借步、侧跨或继续行进。

返程反扫 Tag 5 时还会根据当前空载位姿校正横向位置。Tag 6、7 在开始侧跨前检查前后距离。试跑初值在 `command2_empty_departure` 中，门口横向容差为 5.5 cm，侧跨前的距码范围为 12–21 cm。每次位移后使用新帧检查效果；上位机日志中的正横偏表示机身位于目标右侧。

可在部署包的 `app/config/route_steps.json` 查看和调整路线。MCU 回执记录完成的动作，实际落点会随负载和地面变化。

## 查找和分享任务报告

| 运行日志按钮 | 用法 |
| --- | --- |
| 打开日志文件夹 | 在资源管理器中查看或清理报告。默认目录为 `%LOCALAPPDATA%\AelosR3Deploy\task_reports`。 |
| 更改日志目录 | 选择新文件夹后，下次收到的报告就保存在那里。以前的文件仍在原目录。所选目录暂时无法写入时，当次报告会保存到默认目录，下次继续尝试所选目录。 |
| 分享任务报告… | 默认方式为 GitHub。可以启用后续报告自动推送，也可以手选最近一份或多份 JSON 单次分享，包括旧目录中的报告。 |

### 推送到 GitHub

1. 在分享窗口填写日志仓库的 `owner/repo` 或 GitHub 仓库网址。默认是 `aleistercrowleybeast666/aelos_robot`。公开仓库中的日志分支也公开。如仅供队内查看，可新建私有仓库，创建时勾选添加 README，再把这里改为它的仓库名。
2. 用具有目标仓库写权限的账号创建短期个人访问令牌。仓库所有者或具备写权限的组织成员可使用细粒度令牌，选中目标仓库，授予 **Repository permissions → Contents → Read and write**。在分享窗口输入令牌，或在启动控制台前设置环境变量 `AELOS_GITHUB_TOKEN`。[GitHub 令牌设置说明](https://docs.github.com/zh/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens)
3. **自动推送**。将完整的 GitHub 访问令牌粘贴到令牌框，即会启用。手动逐字输入时按回车，或点击「启用自动推送」。控制台用当前 Windows 用户的 DPAPI 加密保存令牌，每次收到自动任务、手动扫码、停止、部署检查、部署或回滚的终结回执，就先写本机报告，再从后台发送。定时状态轮询及相机预览不生成报告。本次打开控制台以来若已有报告，启用时也会发送最近一份。网络中断后会在后台重试。关闭自动推送会清除控制台保存的令牌，赛后也可到 GitHub 撤销授权。
4. **单次推送**。要补传以前的报告，可以手选文件并点击「推送到 GitHub」。控制台将本批新增报告作为一次提交写入独立的 `robot-logs` 分支，队员在该分支的 `reports/年/月/日/` 下查看。相同报告会跳过。两人同时推送时，控制台会基于最新提交重试。若仍提示冲突，可以稍后重传同一批。

GitHub 对外部协作者使用细粒度令牌写入他人仓库有限制。遇到这种情况，可让队员通过下面的共享目录交报告，由仓库所有者或具备权限的组织成员统一推送。[GitHub 对协作者的令牌限制](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens#fine-grained-personal-access-tokens-limitations)

### 导出 ZIP 或写入共享文件夹

在分享窗口切换方式即可。**ZIP** 适合把一场试跑的报告打包带走。**共享文件夹** 支持有写入权限的 Windows 共享路径（如 `\\比赛电脑\机器人日志`），也支持同步盘客户端已设置同步的本地文件夹。共享目录写入中断后可重选同一批报告再发，已写入且内容相同的文件会跳过。

三种方式都从所选任务报告生成分享副本，保留命令、时间、结果和诊断，移除报告顶层的 SSH 目标以及可识别的凭据字段和令牌文本。本机原件保留在保存目录。场地、动作和故障细节仍可供队员排查，选择公开仓库时也会向读者公开。

[控制界面截图](https://raw.githubusercontent.com/aleistercrowleybeast666/aelos_robot/v1.0.3/docs/control-console.png) · [日志分享窗口截图](https://raw.githubusercontent.com/aleistercrowleybeast666/aelos_robot/v1.0.3/docs/report-sharing.png)

## 命令行

在解压目录执行；PowerShell 下使用 `./robot.exe`。源码目录也可运行 `python -m r3_deployer`。

```powershell
./robot.exe config --ask-password
./robot.exe check
./robot.exe update
./robot.exe update --source local --local-path 'D:\Robot\pi_deploy_bundle'
./robot.exe config --source remote --remote-url 'https://example.org/r3/pi_deploy_bundle'
./robot.exe status
./robot.exe run 0
./robot.exe stop
```

`config` 保存设置，`update` 的 `--source`、`--remote-url` 和 `--local-path` 仅覆盖本次部署。`update --run 0` 可在部署完成且控制服务通过检查后启动命令 0。

## 从源码运行和打包

在 Windows x64 上使用 Python 3.14 构建。

```powershell
python -m pip install -r requirements.txt
python -m r3_deployer
python -m pip install pyinstaller==6.16.0
python tools/build_windows.py
```

打包生成 `dist/AelosR3Control-windows-x64.zip` 和 SHA-256 校验文件。可通过 `Get-FileHash .\AelosR3Control-windows-x64.zip -Algorithm SHA256` 核对。更新树莓派程序时，将新包整体替换仓库中的 `pi_deploy_bundle/`，随包提交 `manifest.json`。已安装的控制台在下一次远程部署时从 `main` 获取更新。
