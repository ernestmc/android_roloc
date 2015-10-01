LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE    := localization_jni
LOCAL_SRC_FILES := src/localization_jni.cpp
LOCAL_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_LDLIBS := -landroid -llog
LOCAL_STATIC_LIBRARIES := roscpp_android_ndk
include $(BUILD_SHARED_LIBRARY)

$(call import-module, roscpp_android_ndk)
