#!/bin/bash

DRV_VERSION=0.0.1

DRV_IMX=arducam_64mp
KERNEL_VERSION=$(uname -r)

sudo dkms remove -m ${DRV_IMX} -v ${DRV_VERSION} --all
sudo /lib/modules/$KERNEL_VERSION/kernel/drivers/media/i2c/arducam_64mp.ko.xz.bak /lib/modules/$KERNEL_VERSION/kernel/drivers/media/i2c/arducam_64mp.ko.xz
