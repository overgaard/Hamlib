LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
        ../android/ltdl.c \
        rig.c \
        serial.c \
        misc.c \
        register.c \
        event.c \
        cal.c \
        conf.c \
        tones.c \
        rotator.c \
        locator.c \
        rot_reg.c \
        rot_conf.c \
        iofunc.c \
        ext.c \
        mem.c \
        settings.c \
        parallel.c \
        usb_port.c \
        debug.c \
        network.c \
        cm108.c \
	gpio.c \
	microham.c

LOCAL_MODULE := libhamlib
LOCAL_CFLAGS := -DHAVE_CONFIG_H
LOCAL_C_INCLUDES := android include
LOCAL_STATIC_LIBRARIES := adat alinco amsat aor ars celestron \
        drake dummy easycomm ether6 flexradio fodtrack \
        gs232a heathkit icom jrc kachina kenwood kit \
        lowe m2 pcr prm80 racal rft \
        rotorez rs sartek skanti spid tapr tentec ts7400 tuner \
        uniden winradio wj yaesu icmarine dorji barrett elad cnctrk \
	prosistel meade rot_ioptron

LOCAL_LDLIBS := -llog

include $(BUILD_SHARED_LIBRARY)
