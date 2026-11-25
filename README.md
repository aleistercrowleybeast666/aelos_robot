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

远程地址可填部署包目录、`manifest.json` 的 HTTPS 下载地址，或对应的 GitHub 目录链接。Windows 控制台负责下载，树莓派只需可从本机通过 SSH 访问。

普通树莓派部署沿用机上现有 MCU 登记。已经更新 MCU 并拿到读回回执时，可在部署时登记回执。MCU 的 Lua 文件和历史动作包由本地维护。

## 控制机器人

在「控制」页点击「连接机器人」。连接成功后会自动尝试打开胸部相机，也可手动开关。可以分别启动单程搬运原始物资、单程搬运转运物资和连续搬运转运物资。

控制台显示任务状态、回执和最近的扫码或纠偏诊断。停止按钮经独立连接取消后续动作。任务报告保存在 `%LOCALAPPDATA%\AelosR3Deploy\task_reports`。

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
