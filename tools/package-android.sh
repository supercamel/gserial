#!/bin/sh
set -eu
gserial_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
ooblerg_root=$(CDPATH= cd -- "${1:?Usage: package-android.sh OOBLERG_CHECKOUT OUTPUT [RELEASE]}" && pwd)
mkdir -p -- "${2:?An output repository directory is required}"
package_output=$(CDPATH= cd -- "$2" && pwd)
package_release=${3:-gserial-usb-r1}
for package_abi in x86_64 arm64-v8a; do
    sqgipkg build --manifest "$gserial_root/examples/android/sqgipkg.json" \
        --platform android --arch "$package_abi"
    package_sdk="$gserial_root/examples/android/.sqgipkg/android/$package_abi/native-sdk"
    if test -e "$package_sdk/usr/share/sqgi-android/java/org/gserial/TestUsbBackend.java"; then
        echo 'Refusing to export the test backend' >&2
        exit 1
    fi
    python3 "$ooblerg_root/tools/android_modules.py" --sdk "$package_sdk" \
        --output "$package_output" --sdk-id noble-android-r4 --release "$package_release" \
        --prefix extras --version 0.3.0
done
