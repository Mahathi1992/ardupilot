echo "#pragma once"  > ArduPlane/date_version.h
YR=`date +"%Y"`
YR=$(($YR - 2000))
echo "#define BUILD_DATE_YEAR $YR" >> ArduPlane/date_version.h
date +"#define BUILD_DATE_MONTH  %-m"  >> ArduPlane/date_version.h
date +"#define BUILD_DATE_DAY  %-d"  >> ArduPlane/date_version.h
date +"#define BUILD_DATE_TM  %H%M"  >> ArduPlane/date_version.h
date +"#define TMSTR \"%Y-%m-%d_%H:%M\"" >> ArduPlane/date_version.h
echo "Date version updated"
CFLAGS=-Wno-maybe-uninitialized CXXFLAGS=-Wno-maybe-uninitialized ./waf plane
