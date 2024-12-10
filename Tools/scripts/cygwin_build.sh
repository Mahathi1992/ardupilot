#!/usr/bin/env bash
set -x
set -e
TOOLCHAIN=x86_64-pc-cygwin
GPP_COMPILER="${TOOLCHAIN}-g++"
$GPP_COMPILER -print-sysroot
SYS_ROOT=$($GPP_COMPILER -print-sysroot)
echo "SYS_ROOT=$SYS_ROOT"
git config --global --add safe.directory /cygdrive/d/a/ardupilot/ardupilot
rm -rf artifacts
mkdir artifacts
(
    python ./waf --color yes --toolchain $TOOLCHAIN --board sitl configure 2>&1
    python ./waf plane 2>&1
    python ./waf copter 2>&1
    # python ./waf heli 2>&1
    # python ./waf rover 2>&1
    # python ./waf sub 2>&1
) | tee artifacts/build.txt
# copy both with exe and without to cope with differences
# between windows versions in CI
cp -v build/sitl/bin/arduplane artifacts/ArduPlane.elf.exe
cp -v build/sitl/bin/arducopter artifacts/ArduCopter.elf.exe

cp -v build/sitl/bin/arduplane artifacts/ArduPlane.elf
cp -v build/sitl/bin/arducopter artifacts/ArduCopter.elf

# Find all cyg*.dll files returned by cygcheck for each exe in artifacts
# and copy them over
for exe in artifacts/*.exe; do 
    echo $exe
    cygcheck $exe | grep -oP 'cyg[^\s\\/]+\.dll' | while read -r line; do
      cp -v /usr/bin/$line artifacts/
    done
done

git log -1 > artifacts/git.txt
ls -l artifacts/