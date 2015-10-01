#!/bin/sh

#export PATH=$PATH:/opt/android-ndk-r8e

# path to where the "roscpp_android_ndk" cross-compiled environment is located
export NDK_MODULE_PATH="/home/ecorbellini/Android/test"

debugging=0

for var in "$@"
do
    if [ ${var} = "--debug-symbols" ] ; then
        debugging=1
    else
  	export NDK_MODULE_PATH=${var}
    fi
done

DEF_NDK_MODULE_DESCRIPTOR="${NDK_MODULE_PATH}/roscpp_android_ndk/Android.mk"

if [ -z ${NDK_MODULE_PATH} ] ; then
    echo "NDK Module path not set!"
    echo "Usage $0 /path/to/your/workspace [--debug-symbols]"
    exit 1
fi

if [ ! -f ${DEF_NDK_MODULE_DESCRIPTOR} ] ; then
    echo "File >${DEF_NDK_MODULE_DESCRIPTOR}< doesn't exist. Something is not right!"
    exit 2
fi

echo "Using workspace at >${NDK_MODULE_PATH}<"

## Comment or uncomment the Android.mk debug flag which is inherited by all other NDK modules
if [ ${debugging} -eq 1 ] ; then
    echo "Doing debug build"
    sed -ri 's/^#+(LOCAL_EXPORT_CFLAGS\s+\+=.*)\s*$/\1/' ${DEF_NDK_MODULE_DESCRIPTOR} 
else
    echo "Doing release build"
    sed -ri 's/^(LOCAL_EXPORT_CFLAGS\s+\+=.*)$/#\1/' ${DEF_NDK_MODULE_DESCRIPTOR} 
fi

SCRIPT="native/Android.mk"
APPLICATION="native/Application.mk"
cd src/main

ndk-build APP_BUILD_SCRIPT=${SCRIPT} NDK_APPLICATION_MK=${APPLICATION} NDK_DEBUG=${debugging}

if [ $? -ne 0 ] ; then
    echo "ndk-build failed. Aborting."
    exit 3
fi

# Installing debug symbols on the APK on the phone is not necessary
# Instead, use: adb logcat | $NDK_ROOT/ndk-stack -sym /project/obj/local/armeabi
#if [  ${debugging} -eq 1 ] ; then
#    echo "Replacing stripped shared libraries from libs/armeabi/ with non-stripped ones from obj/local/armeabi/"
    # cp obj/local/armeabi/*.so libs/armeabi/

#    echo "We've uncommented the LOCAL_EXPORT_CFLAGS from ${DEF_NDK_MODULE_DESCRIPTOR} to enabled debugging"
#else
#    echo "We've commented the LOCAL_EXPORT_CFLAGS from ${DEF_NDK_MODULE_DESCRIPTOR} to disable debugging"
#fi
