LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES := wr1000.c wr1500.c wr1550.c wr3100.c wr3150.c wr3500.c wr3700.c \
		g303.c g313-posix.c g305.c winradio.c linradio/wrg313api.c
LOCAL_MODULE := winradio

LOCAL_CFLAGS := -DHAVE_CONFIG_H
LOCAL_C_INCLUDES := android include src
#LOCAL_LDLIBS := -lhamlib -Lobj/local/armeabi

include $(BUILD_STATIC_LIBRARY)
