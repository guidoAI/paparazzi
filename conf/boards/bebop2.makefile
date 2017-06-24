# Hey Emacs, this is a -*- makefile -*-
#
# bebop2.makefile
#
# http://wiki.paparazziuav.org/wiki/Bebop
#

BOARD=bebop
BOARD_CFG=\"boards/$(BOARD).h\"

ARCH=linux
$(TARGET).ARCHDIR = $(ARCH)
# include conf/Makefile.bebop (with specific upload rules) instead of only Makefile.linux:
ap.MAKEFILE = bebop

# -----------------------------------------------------------------------
USER=foobar
HOST?=192.168.42.1
SUB_DIR=internal_000/paparazzi
FTP_DIR=/data/ftp
TARGET_DIR=$(FTP_DIR)/$(SUB_DIR)
# -----------------------------------------------------------------------

# The datalink default uses UDP
MODEM_HOST         ?= 192.168.42.255

ifeq ($(VIEWVIDEO_HOST),)
  ifeq ($(OS),Windows_NT)
    $(warning Warning: Viewvideo host auto-ip support not enabled for windows os, please configure VIEWVIDEO_HOST in your airframe file)
  else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Linux)
      VIEWVIDEO_HOST     ?= $(shell $(PAPARAZZI_SRC)/sw/tools/get_ip_from_route.sh $(HOST))
    else
      ifeq ($(UNAME_S),Darwin)
        VIEWVIDEO_HOST     ?= $(shell $(PAPARAZZI_SRC)/sw/tools/get_ip_from_route_osx.sh $(HOST))
        ifeq ($(VIEWVIDEO_HOST), 127.0.0.1 )
        	$(warning Warning: Could not find route to $(HOST), defaulting back to 192.168.42.1)
            HOST           =192.168.42.1
            MODEM_HOST     = 192.168.42.255
            VIEWVIDEO_HOST = $(shell $(PAPARAZZI_SRC)/sw/tools/get_ip_from_route_osx.sh $(HOST))
        endif
      endif
    endif
    $(info Info: Viewvideo host auto-ip resolved to $(VIEWVIDEO_HOST))
  endif
endif

# The GPS sensor is connected internally
GPS_PORT           ?= UART1
GPS_BAUD           ?= B230400

# notify that it is the new bebop
$(TARGET).CFLAGS += -DBEBOP_VERSION2

# handle linux signals by hand
$(TARGET).CFLAGS += -DUSE_LINUX_SIGNAL -D_GNU_SOURCE

# board specific init function
$(TARGET).srcs +=  $(SRC_BOARD)/board.c

# Compile the video specific parts
$(TARGET).CFLAGS += -DI2C_BUF_LEN=56 -DUSE_I2C0
$(TARGET).srcs +=  $(SRC_BOARD)/mt9v117.c $(SRC_BOARD)/mt9f002.c modules/computer_vision/lib/isp/libisp.c modules/computer_vision/lib/isp/libisp_config.c

# Link static (Done for GLIBC)
$(TARGET).CFLAGS += -DLINUX_LINK_STATIC
$(TARGET).LDFLAGS += -static

# limit main loop to 1kHz so ap doesn't need 100% cpu
#$(TARGET).CFLAGS += -DLIMIT_EVENT_POLLING

# -----------------------------------------------------------------------

# default LED configuration
RADIO_CONTROL_LED				?= none
BARO_LED           			?= none
AHRS_ALIGNER_LED   			?= 1
GPS_LED            			?= none
SYS_TIME_LED       			?= 0
