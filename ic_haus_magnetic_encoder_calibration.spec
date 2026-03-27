# -*- mode: python ; coding: utf-8 -*-

from PyInstaller.utils.hooks import collect_data_files

# mu_3sl_interface loads its native DLLs via ctypes at runtime from
# <package>/bin/<platform>/<arch>/.  PyInstaller cannot detect these
# automatically, so we collect them explicitly.
mu_3sl_datas = collect_data_files('mu_3sl_interface', subdir='bin')

a = Analysis(
    ['__main__.py'],
    pathex=[],
    binaries=[],
    datas=mu_3sl_datas,
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='ic_haus_magnetic_encoder_calibration',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
