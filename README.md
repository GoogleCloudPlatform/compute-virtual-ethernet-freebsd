## Overview

Google Virtual NIC (gVNIC) is a virtual network interface designed specifically
for Compute Engine. It is required to support per VM Tier1 networking
performance, and for using certain VM shapes. gve is the driver for gVNIC.

### Supported Hardware

The driver here binds to a single PCI device id used by the virtual Ethernet
device found in some Compute Engine VMs.

Field         | Value    | Comments
------------- | -------- | --------
Vendor ID     | `0x1AE0` | Google
Device ID     | `0x0042` |
Sub-vendor ID | `0x1AE0` | Google
Sub-device ID | `0x0058` |
Revision ID   | `0x0`    |
Device Class  | `0x200`  | Ethernet

### Supported Kernels

FreeBSD 13.1 and upwards

## Features

gve supports the following features:  

* RX checksum offload
* TX chesksum offload
* TSO
* Software LRO
* Jumbo frames
* RSS

## Limitations

gve does not yet support the following features:

* Ability to change ring sizes
* Ability to change RSS config from userspace
* Hardware LRO
* Netmap (4) support
* Polling (4) support

## Driver diagnostics

* Per-queue stats can be viewed by running `sysctl -a | grep gve0`. Aggregated
stats can be viewed by running `netstat -I gve0`.  

* If bootverbose is on, the driver logs its version at load time and this can be learnt by running
`dmesg | grep gve0`.  

* The state of the driver taskqueues can be learnt by running `procstat -ta |
grep gve0`.  

## Installation

The following instructions are for installing the driver as an out-of-tree module.
They refer to the gcloud command, but many of them can also be performed on the web UI.

1. Create a FreeBSD 13 instance in your project, log into it, and become root.

2. Install bash: `pkg install bash`.

3. Install git: `pkg install git` and then clone this repository and cd into it.

4. Run `./build_src.sh`; this should create a `build/` directory.

5. Run `make -C build/` to compile the driver and verify that a `gve.ko` exists in
   `build/`.

6. To have the driver automatically load on boot, run:

```
cp build/gve.ko /boot/modules/gve.ko
echo 'gve_load="YES"' >> /boot/loader.conf
```

7. Stop the VM.

8. Using gVNIC on a custom image requires the use of a VM image that has been
   deliberately tagged with the gVNIC "guest OS feature". To create such an image
   based off of the VM stopped in the previous step, use the following gcloud command.

```
gcloud compute images create ${IMAGE_NAME?} \
    --source-disk ${DISK_NAME?} \
    --guest-os-features=GVNIC
```

   Here `DISK_NAME` refers to the boot disk of the instance you just stopped.
   This new image `IMAGE_NAME` has the gve driver installed and loading with every boot. 

9. Use the image you have created in the previous step, which now has the gve driver
   installed, to create and start a VM with a gVNIC network interface.

```
gcloud compute instances create ${INSTANCE_NAME?} \
    --source-image ${IMAGE_NAME?} \
    --zone ${PREFERRED_ZONE?} \
    --network-interface=nic-type=GVNIC
```
   Note that to use TIER_1 networking an additional flag needs to be supplied: 

```
--network-performance-configs=total-egress-bandwidth-tier=TIER_1
```
