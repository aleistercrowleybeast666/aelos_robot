from pathlib import Path


project = Path(SPECPATH).resolve().parent
application = Analysis(
    [str(project / 'robot-control.pyw')],
    pathex=[str(project)],
    datas=[(str(project / 'r3_deployer' / 'remote.py'), 'r3_deployer')],
    # The controller uses Qt images and pypdf's text extractor.
    excludes=['tkinter', 'r3_deployer.gui', 'PIL', 'numpy', 'Crypto'],
)
modules = PYZ(application.pure)
window = EXE(modules, application.scripts, [], exclude_binaries=True,
             name='AelosR3Control', console=False)
console = EXE(modules, application.scripts, [], exclude_binaries=True,
              name='robot', console=True)
distribution = COLLECT(window, console, application.binaries, application.datas,
                       name='AelosR3Control')
