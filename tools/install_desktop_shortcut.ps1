$ErrorActionPreference = 'Stop'
$r3App = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).ProviderPath
$r3Gui = Join-Path $r3App 'robot-control.pyw'
$r3Python = & python -B -c 'import sys; print(sys.executable)'
if ($LASTEXITCODE -ne 0) { throw 'Python is unavailable.' }
$r3Pythonw = Join-Path (Split-Path -LiteralPath $r3Python) 'pythonw.exe'
if (-not (Test-Path -LiteralPath $r3Pythonw -PathType Leaf)) { throw 'pythonw.exe is unavailable.' }
$r3Desktop = [Environment]::GetFolderPath('DesktopDirectory')
$r3ShortcutPath = Join-Path $r3Desktop 'AELOS R3 Control.lnk'
$r3Shell = New-Object -ComObject WScript.Shell
$r3Shortcut = $r3Shell.CreateShortcut($r3ShortcutPath)
$r3ShortcutArguments = '-B "' + $r3Gui + '"'
if ((Test-Path -LiteralPath $r3ShortcutPath) -and $r3Shortcut.Arguments -ne $r3ShortcutArguments) {
    throw 'An unrelated shortcut is already present.'
}
$r3Shortcut.TargetPath = $r3Pythonw
$r3Shortcut.Arguments = $r3ShortcutArguments
$r3Shortcut.WorkingDirectory = $r3App
$r3Shortcut.Description = 'Control, preview and deploy AELOS R3'
$r3Shortcut.Save()
Write-Output $r3ShortcutPath
