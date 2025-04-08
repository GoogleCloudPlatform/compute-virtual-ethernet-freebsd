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
* Hardware LRO
* Jumbo frames
* RSS
* Changing queue count
* Changing ring size

## Limitations

gve does not yet support the following features:

* Ability to change RSS config from userspace
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

## Sysctl Variables

gve exposes the following sysctl(8) variables:

* **hw.gve.driver_version**  
The driver version. This is read-only.  

* **hw.gve.queue_format**  
The queue format in use. This is read-only.  

* **hw.gve.disable_hw_lro**  
Setting this boot-time tunable to 1 disables Large Receive Of-
fload (LRO) in the NIC. The default value is 0, which means
hardware LRO is enabled by default. The software LRO stack in
the kernel is always used. This sysctl variable needs to be
set before loading the driver, using loader.conf(5).

* **dev.gve.X.num_rx_queues and dev.gve.X.num_tx_queues**  
Run-time tunables that represent the number of currently used RX/TX queues.
The default value is the max number of RX/TX queues the device can support. This call turns down the interface while setting up the new queues, which may potentially cause any new packets to be dropped.
This call can fail if the system is not able to provide the driver with enough resources.
In that situation, the driver will revert to the previous number of RX/TX queues.
If this also fails, a device reset will be triggered.
*Note*: sysctl nodes for queue stats remain available even if a queue is removed.  

* **dev.gve.X.rx_ring_size and dev.gve.X.tx_ring_size**  
Run-time tunables that represent the current ring size for RX/TX queues.
The default value is set to device defaults for ring size.
This call turns down the interface while setting up the queues with the new ring size,
which may potentially cause any new packets to be dropped.
This call can fail if the system is not able to provide the driver with enough resources.
In that situation, the driver will try to revert to the previous ring size for RX/TX queues.
If this also fails, the device will be in an unhealthy state and will need to be reloaded.
This value must be a power of 2 and within the defined range.

## Examples
**Change the TX queue count to 4 for the gve0 interface**
```
sysctl dev.gve.0.num_tx_queues=4
```
**Change the RX queue count to 4 for the gve0 interface**
```
sysctl dev.gve.0.num_rx_queues=4
```
**Change the TX ring size to 512 for the gve0 interface**
```
sysctl dev.gve.0.tx_ring_size=512
```
**Change the RX ring size to 512 for the gve0 interface**
```
sysctl dev.gve.0.rx_ring_size=512
```