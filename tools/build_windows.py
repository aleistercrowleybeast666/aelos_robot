"""Build the portable AELOS R3 controller archive for Windows x64."""

import argparse
import hashlib
from importlib.metadata import distribution
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
from tempfile import TemporaryDirectory


PROJECT = Path(__file__).resolve().parents[1]
ARCHIVE_NAME = 'AelosR3Control-windows-x64'
PACKAGES = ('PyInstaller', 'PySide6_Essentials', 'shiboken6', 'paramiko',
            'cryptography', 'bcrypt', 'PyNaCl', 'cffi', 'invoke', 'pypdf')


def package_licenses(destination):
    destination.mkdir()
    attribution = []
    for name in PACKAGES:
        package = distribution(name)
        attribution.append(f'{package.metadata["Name"]} {package.version}')
        license_dir = destination / name
        for item in package.files or ():
            if '.dist-info' not in str(item) or not any(
                    part.lower().startswith(('license', 'copying', 'notice')) for part in item.parts[1:]):
                continue
            source = package.locate_file(item)
            if source.is_file():
                target = license_dir.joinpath(*item.parts[1:])
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(source, target)
    (destination / 'versions.txt').write_text('\n'.join(attribution) + '\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=PROJECT / 'dist')
    parser.add_argument('--mcu-overlay', type=Path, metavar='目录',
                        help='随便携版附带已核验的 R3 MCU 读回对照文件')
    args = parser.parse_args()
    if os.name != 'nt' or struct.calcsize('P') != 8 or os.environ.get('PROCESSOR_ARCHITECTURE') != 'AMD64':
        parser.error('Use a Windows x64 Python environment.')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if args.mcu_overlay is not None:
        sys.path.insert(0, str(PROJECT))
        from r3_deployer.usb_mode import _overlay
        root, expected, _ = _overlay(args.mcu_overlay, args.mcu_overlay / 'MANIFEST.json')
    with TemporaryDirectory(prefix='aelos-r3-build-') as temp:
        scratch = Path(temp)
        subprocess.run([sys.executable, '-m', 'PyInstaller', '--noconfirm',
                        '--distpath', str(scratch / 'dist'), '--workpath', str(scratch / 'build'),
                        str(PROJECT / 'tools' / 'r3_windows.spec')], cwd=PROJECT, check=True)
        folder = scratch / 'dist' / 'AelosR3Control'
        if args.mcu_overlay is not None:
            for relative in (*expected, 'MANIFEST.json'):
                target = folder / 'mcu_overlay' / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(root / relative, target)
        shutil.copyfile(PROJECT / 'README.md', folder / 'README.md')
        package_licenses(folder / 'THIRD_PARTY_LICENSES')
        archive = Path(shutil.make_archive(str(scratch / ARCHIVE_NAME), 'zip', folder.parent, folder.name))
        artifact = output / archive.name
        shutil.copyfile(archive, artifact)
    with artifact.open('rb') as file:
        digest = hashlib.file_digest(file, 'sha256').hexdigest()
    artifact.with_suffix('.zip.sha256').write_text(f'{digest}  {artifact.name}\n', encoding='ascii')
    print(artifact)


if __name__ == '__main__':
    main()
