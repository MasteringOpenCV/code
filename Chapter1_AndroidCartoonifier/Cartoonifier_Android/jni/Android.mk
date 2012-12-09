#/****************************************************************************
#*   Cartoonifier, for Android.
#*****************************************************************************
#*   by Shervin Emami, 5th Dec 2012 (shervin.emami@gmail.com)
#*   http://www.shervinemami.info/
#*****************************************************************************
#*   Ch1 of the book "Mastering OpenCV with Practical Computer Vision Projects"
#*   Copyright Packt Publishing 2012.
#*   http://www.packtpub.com/cool-projects-with-opencv/book
#****************************************************************************/


LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

OPENCV_LIB_TYPE:=STATIC
OPENCV_INSTALL_MODULES:=on

# Path to OpenCV.mk file, which is generated when you build OpenCV for Android.
# include C:\OpenCV\android\build\OpenCV.mk
# include ~/OpenCV/android/build/OpenCV.mk
include ../includeOpenCV.mk
ifeq ("$(wildcard $(OPENCV_MK_PATH))","")
    #try to load OpenCV.mk from default install location
    include $(TOOLCHAIN_PREBUILT_ROOT)/user/share/OpenCV/OpenCV.mk
else
    include $(OPENCV_MK_PATH)
endif

LOCAL_MODULE    := cartoonifier
LOCAL_LDLIBS +=  -llog -ldl

# Since we have source + headers files in an external folder, we need to show where they are.
LOCAL_SRC_FILES := jni_part.cpp
LOCAL_SRC_FILES += ../../Cartoonifier_Desktop/cartoon.cpp
LOCAL_SRC_FILES += ../../Cartoonifier_Desktop/ImageUtils_0.7.cpp
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../../Cartoonifier_Desktop


include $(BUILD_SHARED_LIBRARY)
