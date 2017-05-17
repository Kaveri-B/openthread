#
#  Copyright (c) 2017, The OpenThread Authors.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. Neither the name of the copyright holder nor the
#     names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#

include $(abs_top_nlbuild_autotools_dir)/automake/pre.am

lib_LIBRARIES                                 = libopenthread-aducm3029.a

# Do not enable -pedantic-errors for rail
override CFLAGS                              := $(filter-out -pedantic-errors,$(CFLAGS))
override CXXFLAGS                            := $(filter-out -pedantic-errors,$(CXXFLAGS))

ADUCM3029_SDK_SRCDIR                            = $(top_srcdir)/third_party/adi/ADuCM302x_IoT_DFP/1.0.0

libopenthread_aducm3029_a_CPPFLAGS                                                = \
    -D__ADUCM3029__                                                     \
    -D__START=main                                                     \
    -I$(top_srcdir)/include                                                     \
    -I$(top_srcdir)/examples/platforms                                          \
    -I$(top_srcdir)/src/core                                                    \
    -I$(top_srcdir)/examples/platforms/aducm3029/CMSIS                          \
    -I$(ADUCM3029_SDK_SRCDIR)/Include		                                \
    -I$(ADUCM3029_SDK_SRCDIR)/Include/sys	                                \
    -I$(ADUCM3029_SDK_SRCDIR)/Include/config	                                \
    -I$(ADUCM3029_SDK_SRCDIR)/Include/drivers/general                           \
    -Wno-unused-parameter                                                       \
    $(NULL)

libopenthread_aducm3029_a_SOURCES            = \
    alarm.c                                 \
    flash.c                                 \
    misc.c                                  \
    platform.c                              \
    radio.c                                 \
    random.c                                \
    uart.c                                  \
    startup-gcc.c                                  \
    $(NULL)

if OPENTHREAD_ENABLE_DIAG
libopenthread_aducm3029_a_SOURCES           += \
    diag.c                                  \
    $(NULL)
endif

nodist_libopenthread_aducm3029_a_SOURCES           =  \
    @top_builddir@/third_party/adi/ADuCM302x_IoT_DFP/1.0.0/Source/system_ADuCM3029.c                                         \
    @top_builddir@/third_party/adi/ADuCM302x_IoT_DFP/1.0.0/Source/GCC/startup_ADuCM3029.c                                         \
    $(NULL)

noinst_HEADERS                                                                = \
    platform-aducm3029.h                                                        \
    $(NULL)


Dash                                                                          = -
libopenthread_aducm3029_a_LIBADD                                                  = \
    $(shell find $(top_builddir)/examples/platforms/utils $(Dash)type f $(Dash)name "*.o")

include $(abs_top_nlbuild_autotools_dir)/automake/post.am
