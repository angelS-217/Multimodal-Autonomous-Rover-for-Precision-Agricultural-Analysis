#!/usr/bin/bash

DRV_VERSION=0.0.1

DRV_IMX=arducam_64mp
KERNEL_VERSION=$(uname -r)

sudo apt install dkms -y

echo "Uninstalling any previous ${DRV_IMX} module"
#dkms status ${DRV_IMX} | awk -F', ' '{print $2}' | xargs -n1 sudo dkms remove -m ${DRV_IMX} -v 
sudo dkms remove -m ${DRV_IMX} -v ${DRV_VERSION} --all
sudo /lib/modules/$KERNEL_VERSION/kernel/drivers/media/i2c/arducam_64mp.ko.xz /lib/modules/$KERNEL_VERSION/kernel/drivers/media/i2c/arducam_64mp.ko.xz.bak

sudo mkdir -p /usr/src/${DRV_IMX}-${DRV_VERSION}

sudo cp -r $(pwd)/* /usr/src/${DRV_IMX}-${DRV_VERSION}

sudo dkms add -m ${DRV_IMX} -v ${DRV_VERSION}
sudo dkms build -m ${DRV_IMX} -v ${DRV_VERSION}
sudo dkms install -m ${DRV_IMX} -v ${DRV_VERSION}
