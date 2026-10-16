# Packaging

Installer / bundle inputs. The design, build commands, release workflow and the extension lookup contract are in
[docs/packaging.md](../docs/packaging.md); the CMake side is `cmake/WpiEmuInstall.cmake`.

| Path | Purpose |
|---|---|
| `icons/` | original app icon (`wpiemu.svg`) and the generated PNG / ICO / ICNS (`make_icons.py`) |
| `windows/wpiemu.rc.in` | icon + version resource attached to `wpiemu_app.exe` |
| `macos/` | `Info.plist.in`, `entitlements.plist`, `sign_bundle.cmake` (CPack pre-build signing hook), `notarize.sh` |
| `linux/` | `build-appimage.sh`, desktop file, AppStream metadata, AppRun hook |
| `THIRD-PARTY-NOTICES.txt` | installed with the product |
