"""R3 control CLI and double-click launcher entry point."""

import argparse
from getpass import getpass
import json
from pathlib import Path
import re
import sys

from . import settings


def _add_source_options(parser):
    parser.add_argument('--source', dest='deployment_source', choices=('remote', 'local'),
                        help='选择部署包来源，remote 为远程，local 为本地')
    parser.add_argument('--remote-url', metavar='HTTPS_URL', help='远程部署包目录，包含 manifest.json')
    parser.add_argument('--local-path', metavar='目录', help='本地部署包目录')


def main(argv=None):
    parser = argparse.ArgumentParser(prog='robot', description='AELOS 3 号机器人控制与部署')
    actions = parser.add_subparsers(dest='action')
    actions.add_parser('gui', help='图形界面（默认）')
    actions.add_parser('check', help='只读测试 SSH 和树莓派目录')
    update = actions.add_parser('update', aliases=['deploy'], help='上传并替换整目录，保留备份')
    _add_source_options(update)
    update.add_argument('--run', metavar='命令', choices=('0', '1', '2'),
                        help='部署成功并核对控制服务后启动指定命令')
    update.add_argument('--mcu-readback', type=Path, metavar='回执.json',
                        help='MCU 复位后登记 robot usb-readback 或 usb-deploy 的读回回执')
    actions.add_parser('rollback', help='还原上一份树莓派备份')
    run = actions.add_parser('run', help='启动指定自动机器人命令')
    run.add_argument('command', choices=('0', '1', '2'),
                     help='0 单程搬运原始物资，1 单程搬运转运物资，2 连续搬运转运物资')
    actions.add_parser('stop', help='取消机器人后续自动动作')
    actions.add_parser('status', help='读取当前机器人控制状态')
    actions.add_parser('scan', help='只读识别胸部相机视野中的标记')
    config = actions.add_parser('config', help='查看或修改部署设置')
    config.add_argument('--host')
    config.add_argument('--port', type=int)
    config.add_argument('--username')
    config.add_argument('--remote-path')
    _add_source_options(config)
    credentials = config.add_mutually_exclusive_group()
    credentials.add_argument('--ask-password', action='store_true', help='交互输入并加密保存 SSH 密码')
    credentials.add_argument('--manual', type=Path, nargs='?', const=settings.MANUAL, help='从本地 AELOS 手册读取并加密保存 SSH 密码')
    for name in ('usb-check', 'usb-deploy'):
        usb = actions.add_parser(name, help='检查或覆盖已连接电脑的 R3 U 盘')
        usb.add_argument('overlay', type=Path)
        usb.add_argument('manifest', type=Path)
        usb.add_argument('drive')
        if name == 'usb-deploy':
            usb.add_argument('--backup-root', type=Path, default=settings.settings_path().parent/'usb_backups')
    readback = actions.add_parser('usb-readback', help='只读校验已手动覆盖的 MCU U 盘并保存回执')
    readback.add_argument('drive')
    readback.add_argument('--overlay', type=Path, default=settings.DEFAULT_MCU_OVERLAY)
    restore = actions.add_parser('usb-restore', help='还原已连接电脑的 R3 U 盘')
    restore.add_argument('drive')
    restore.add_argument('backup', type=Path)
    args = parser.parse_args(argv)
    try:
        values = settings.load()
        if args.action in ('config', 'update', 'deploy'):
            overrides = {key: value for key in settings.DEFAULTS
                         if (value := getattr(args, key, None)) is not None}
            if overrides:
                values = settings.validate({**values, **overrides})
        if args.action == 'config':
            password = getpass('SSH 密码：') if args.ask_password else None
            if args.manual:
                password = settings.manual_password(args.manual)
            settings.save(values, password)
            print(json.dumps({**values, 'password_saved': settings.password_for(values) is not None},
                             ensure_ascii=False, indent=2))
        elif args.action in (None, 'gui'):
            from .control_window import run_control_window

            run_control_window(values)
        elif args.action.startswith('usb-'):
            from . import usb_mode

            if args.action == 'usb-check':
                result = usb_mode.inspect_usb(args.overlay, args.manifest, args.drive)
            elif args.action == 'usb-readback':
                result = usb_mode.McuReadback_VerifyAndSave(
                    args.drive, args.overlay, settings.settings_path().parent/'usb_receipts')
            elif args.action == 'usb-deploy':
                result = usb_mode.deploy_usb(args.overlay, args.manifest, args.drive, args.backup_root)
            else:
                result = usb_mode.restore_usb(args.drive, args.backup)
            print(json.dumps(result, ensure_ascii=False, indent=2))
        elif args.action in ('update', 'deploy') and args.mcu_readback is not None:
            from .deploy import operate

            secret = None if settings.password_for(values) else getpass('SSH 密码：')
            activated = operate('deploy', values, secret, lambda line: print(line, flush=True), mcu_readback=args.mcu_readback)
            if args.run is not None:
                if not (activated.get('status') == 'deployed' and activated.get('owner_reloaded') is True
                        and activated.get('path') == settings.DEFAULT_PI_PATH
                        and activated.get('mcu_readback_registered') is True
                        and re.fullmatch(r'[0-9a-f]{32}', activated.get('id') or '')):
                    raise RuntimeError('部署回执尚未确认正式路径、控制节点和 MCU 读回登记。')
                from .control_api import RobotApi

                api = RobotApi(values, secret, log=lambda line: print(line, file=sys.stderr, flush=True))
                result = api.start(args.run)
                print(json.dumps(result, ensure_ascii=False, indent=2))
                return 0 if result.get('ok') else 1
        elif args.action in ('run', 'stop', 'status', 'scan') or getattr(args, 'run', None):
            from .control_api import RobotApi

            secret = None if settings.password_for(values) else getpass('SSH 密码：')
            api = RobotApi(values, secret, log=lambda line: print(line, file=sys.stderr, flush=True))
            if args.action == 'run':
                result = api.start(args.command)
            elif args.action in ('update', 'deploy'):
                result = api.update_then_start(args.run)
            else:
                result = getattr(api, args.action)()
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 0 if result.get('ok') else 1
        else:
            from .deploy import operate

            secret = None if settings.password_for(values) else getpass('SSH 密码：')
            action = 'deploy' if args.action == 'update' else args.action
            operate(action, values, secret, lambda line: print(line, flush=True))
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        print(str(exc) or '操作已取消。', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
