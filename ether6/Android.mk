LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES := ether6.c
LOCAL_MODULE := ether6

LOCAL_CFLAGS := -DHAVE_CONFIG_H
LOCAL_C_INCLUDES := android include src
#LOCAL_LDLIBS := -lhamlib -Lobj/local/armeabi

include $(BUILD_STATIC_LIBRARY)
