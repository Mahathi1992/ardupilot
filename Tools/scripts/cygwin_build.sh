#!/bin/bash
git config --global --add safe.directory /cygdrive/d/a/ardupilot/ardupilot
rm -rf artifacts
mkdir artifacts
(python ./waf --color yes --toolchain i686-pc-cygwin --board sitl configure 2>&1
python ./waf plane 2>&1
python ./waf copter 2>&1) | tee artifacts/build.txt
i686-pc-cygwin-g++ -print-sysroot
cp -v build/sitl/bin/arduplane artifacts/ArduPlane.elf.exe
cp -v build/sitl/bin/arducopter artifacts/ArduCopter.elf.exe
cp -v build/sitl/bin/arduplane artifacts/ArduPlane.elf
cp -v build/sitl/bin/arducopter artifacts/ArduCopter.elf
cp -v /usr/i686-pc-cygwin/sys-root/usr/bin/*.dll artifacts/
git log -1 > artifacts/git.txt
ls -l artifacts/
