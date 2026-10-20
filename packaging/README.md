# Packaging

Read this when locating installer and bundle inputs.
Prerequisites: [release preparation](../docs/releasing.md).

- [icons](icons): original artwork, generated PNG/ICO/ICNS assets and their generator.
- [windows](windows): executable icon/version resource template.
- [macos](macos): bundle metadata, entitlements, signing, notarization and minimum-OS checks.
- [linux](linux): AppImage script, desktop/AppStream metadata and AppRun hook.
- [THIRD-PARTY-NOTICES.txt](THIRD-PARTY-NOTICES.txt): installed attribution notices.
- [Install rules](../cmake/WpiEmuInstall.cmake): install layout and CPack configuration.

Next: [Releasing](../docs/releasing.md) for packaging commands, artifacts and signing.
