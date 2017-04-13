ROOT_DIR := $(notdir $(CURDIR))
ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

#weilun@adv - begin
include ../adv.mk
#weilun@adv - end

SRC := $(CURDIR)/../src
BOARD_ROOT := $(SRC)/hardware/startup/boards
SAMPLEDIR := $(SRC)/hardware/support/appsmedia-sample
HOST_MKIFS := mkifs

HOST_MKIFS := mkifs

SUFFIXES := .build .ifs .raw

.PHONY: all clean appsmedia-sample

#weilun@adv - begin
#all: ifs-mx6q-sabresmart.raw
all: preinstall ifs-mx6q-sabresmart.raw
#weilun@adv - end

clean: 
	$(RM_HOST) ifs-mx6q-sabresmart.raw ipl-mx6q-sabresmart.bin 
	$(RM_HOST) ifs-appsmedia-sample-mx6q-sabresmart.bin
	$(RM_HOST) appsmedia-sample-mx6q-sabresmart.build

#ifs-mx6q-sabresmart.raw: smart-device.build
#	$(HOST_MKIFS) -vvv -r../install $(MKIFSFLAGS) $^ $@

#weilun@adv - begin
preinstall:
ifeq ($(CONFIG_ADV_BOARD),rom5420)
	cp -f $(SRC)/hardware/startup/boards/imx6x/smart-device/arm/le.v7/startup-imx6x-smart-device  $(CURDIR)/../prebuilt/armle-v7/boot/sys/startup-imx6x-smart-device.rom5420
endif
ifeq ($(CONFIG_ADV_BOARD),rsb4410)
	cp -f $(SRC)/hardware/startup/boards/imx6x/smart-device/arm/le.v7/startup-imx6x-smart-device  $(CURDIR)/../prebuilt/armle-v7/boot/sys/startup-imx6x-smart-device.rsb4410
endif
ifeq ($(CONFIG_ADV_BOARD),rom7420)
	cp -f $(SRC)/hardware/startup/boards/imx6x/smart-device/arm/le.v7/startup-imx6x-smart-device  $(CURDIR)/../prebuilt/armle-v7/boot/sys/startup-imx6x-smart-device.rom7420
endif
ifeq ($(CONFIG_ADV_BOARD),dmsba16)
	cp -f $(SRC)/hardware/startup/boards/imx6x/smart-device/arm/le.v7/startup-imx6x-smart-device  $(CURDIR)/../prebuilt/armle-v7/boot/sys/startup-imx6x-smart-device.dmsba16
endif

ifeq ($(CONFIG_ADV_BOARD),rom7420)
ifs-mx6q-sabresmart.raw: smart-device.rom7420.build
	$(HOST_MKIFS) -vvv -r../install $(MKIFSFLAGS) $^ $@
endif
ifeq ($(CONFIG_ADV_BOARD),rom5420)
ifs-mx6q-sabresmart.raw: smart-device.rom5420.build
	$(HOST_MKIFS) -vvv -r../install $(MKIFSFLAGS) $^ $@
endif
ifeq ($(CONFIG_ADV_BOARD),rsb4410)
ifs-mx6q-sabresmart.raw: smart-device.rsb4410.build
	$(HOST_MKIFS) -vvv -r../install $(MKIFSFLAGS) $^ $@
endif
ifeq ($(CONFIG_ADV_BOARD),dmsba16)
ifs-mx6q-sabresmart.raw: smart-device.dmsba16.build
	$(HOST_MKIFS) -vvv -r../install $(MKIFSFLAGS) $^ $@
endif
#weilun@adv - end

appsmedia-sample: ifs-appsmedia-sample-mx6q-sabresmart.bin

appsmedia-sample-mx6q-sabresmart.build:
	cat $(BOARD_ROOT)/imx6x/smart-device/build $(SAMPLEDIR)/appsmedia-sample-imx6.build $(SAMPLEDIR)/appsmedia-sample-common.build > $(CURDIR)/$@

ifs-appsmedia-sample-mx6q-sabresmart.bin: appsmedia-sample-mx6q-sabresmart.build
	$(HOST_MKIFS) -r../install -v $(MKIFSFLAGS) $^ $@
