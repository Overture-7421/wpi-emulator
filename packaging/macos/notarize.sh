#!/usr/bin/env bash
# Signs (optional), notarizes and staples the dmg produced by `cpack -G DragNDrop`.
#
#   packaging/macos/notarize.sh build/macos/wpi-emulator-0.1.0-macos-arm64.dmg
#
# Credentials come ONLY from the environment (CI secrets / your shell), never from the repository:
#   WPIEMU_CODESIGN_IDENTITY   "Developer ID Application: Name (TEAMID)"; when set the dmg itself is signed too.
#   WPIEMU_CODESIGN_KEYCHAIN   optional keychain that holds the identity
# and one of the two notarytool credential sets:
#   App Store Connect API key: APPLE_API_KEY_PATH (the .p8 file), APPLE_API_KEY_ID, APPLE_API_ISSUER
#   Apple ID:                  APPLE_ID, APPLE_TEAM_ID, APPLE_APP_PASSWORD (app-specific password)
# Without credentials the script exits 0 after saying so (unsigned developer builds stay usable).
set -euo pipefail

DMG="${1:-}"
[[ -f "$DMG" ]] || { echo "usage: $0 <path/to/file.dmg>" >&2; exit 2; }
[[ "$(uname -s)" == "Darwin" ]] || { echo "error: notarization needs macOS" >&2; exit 1; }

if [[ -n "${WPIEMU_CODESIGN_IDENTITY:-}" ]]; then
    sign_args=(--force --timestamp --sign "$WPIEMU_CODESIGN_IDENTITY")
    [[ -z "${WPIEMU_CODESIGN_KEYCHAIN:-}" ]] || sign_args+=(--keychain "$WPIEMU_CODESIGN_KEYCHAIN")
    codesign "${sign_args[@]}" "$DMG"
fi

if [[ -n "${APPLE_API_KEY_PATH:-}" && -n "${APPLE_API_KEY_ID:-}" && -n "${APPLE_API_ISSUER:-}" ]]; then
    auth=(--key "$APPLE_API_KEY_PATH" --key-id "$APPLE_API_KEY_ID" --issuer "$APPLE_API_ISSUER")
elif [[ -n "${APPLE_ID:-}" && -n "${APPLE_TEAM_ID:-}" && -n "${APPLE_APP_PASSWORD:-}" ]]; then
    auth=(--apple-id "$APPLE_ID" --team-id "$APPLE_TEAM_ID" --password "$APPLE_APP_PASSWORD")
else
    echo "notarize: no Apple credentials in the environment; skipping notarization of $DMG"
    exit 0
fi
[[ -n "${WPIEMU_CODESIGN_IDENTITY:-}" ]] || { echo "error: notarization requires a Developer ID signed app (WPIEMU_CODESIGN_IDENTITY)" >&2; exit 1; }

echo "notarize: submitting $DMG"
xcrun notarytool submit "$DMG" "${auth[@]}" --wait
xcrun stapler staple "$DMG"
xcrun stapler validate "$DMG"
spctl --assess --type open --context context:primary-signature --verbose=2 "$DMG"
echo "notarize: done"
