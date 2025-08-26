# Notes for Convoy - Aerospace
Ben Cometto's notes
More recent than 26 Aug 2025


## Hardware
Wheel is [Logitech G29 Driving Force Racing Wheel](https://www.logitechg.com/en-us/shop/p/driving-force-racing-wheel).

Has a 24V DC input (AC adapter) and a USB cable.

The wheel comes with clamps, and the pedals plug into the wheel.

## Interfacing the wheel with a VM
Linux VM info:

```bash
dbcometto@dbcometto-vm:~$ hostnamectl
 Static hostname: dbcometto-vm
       Icon name: computer-vm
         Chassis: vm
      Machine ID: defc2bbd5b2449b4b86e54fc4719e93f
         Boot ID: 09abb686523e47548b557cfe6ac7ffa4
  Virtualization: vmware
Operating System: Ubuntu 22.04.5 LTS              
          Kernel: Linux 6.8.0-65-generic
    Architecture: x86-64
 Hardware Vendor: VMware, Inc.
  Hardware Model: VMware Virtual Platform
```

First, need to ensure wheel is connected to machine.  On my VM hoster (VMWare Workstation Pro), I right clicked on the wheel device and clicked connect.

Second, need to ID how the wheel is connected:
```bash
dbcometto@dbcometto-vm:~$ cat /proc/bus/input/devices
I: Bus=0019 Vendor=0000 Product=0001 Version=0000
N: Name="Power Button"
P: Phys=LNXPWRBN/button/input0
S: Sysfs=/devices/LNXSYSTM:00/LNXPWRBN:00/input/input0
U: Uniq=
H: Handlers=kbd event0 
B: PROP=0
B: EV=3
B: KEY=8000 10000000000000 0

...

I: Bus=0003 Vendor=046d Product=c24f Version=0111
N: Name="Logitech G29 Driving Force Racing Wheel"
P: Phys=usb-0000:02:00.0-2.1/input0
S: Sysfs=/devices/pci0000:00/0000:00:11.0/0000:02:00.0/usb2/2-2/2-2.1/2-2.1:1.0/0003:046D:C24F.0002/input/input7
U: Uniq=
H: Handlers=event6 js1 
B: PROP=0
B: EV=20001b
B: KEY=1ff 0 0 0 0 0 0 ffff00000000 0 0 0 0
B: ABS=30027
B: MSC=10
B: FF=300040000 0
```
Notice the handlers are `event6` and `js1`.

Next, need a tool to view the event logs: `evtest`.  Install using

```bash
dbcometto@dbcometto-vm:~$ sudo apt-get install evtest
```

Then, running this tool on `event6` yields

```bash
dbcometto@dbcometto-vm:~$ sudo evtest /dev/input/event6
Input driver version is 1.0.1
Input device ID: bus 0x3 vendor 0x46d product 0xc24f version 0x111
Input device name: "Logitech G29 Driving Force Racing Wheel"
Supported events:
  Event type 0 (EV_SYN)
  Event type 1 (EV_KEY)
    Event code 288 (BTN_TRIGGER)
    Event code 289 (BTN_THUMB)
    Event code 290 (BTN_THUMB2)
    Event code 291 (BTN_TOP)
    Event code 292 (BTN_TOP2)
    Event code 293 (BTN_PINKIE)
    Event code 294 (BTN_BASE)
    Event code 295 (BTN_BASE2)
    Event code 296 (BTN_BASE3)
    Event code 297 (BTN_BASE4)
    Event code 298 (BTN_BASE5)
    Event code 299 (BTN_BASE6)
    Event code 300 (?)
    Event code 301 (?)
    Event code 302 (?)
    Event code 303 (BTN_DEAD)
    Event code 704 (BTN_TRIGGER_HAPPY1)
    Event code 705 (BTN_TRIGGER_HAPPY2)
    Event code 706 (BTN_TRIGGER_HAPPY3)
    Event code 707 (BTN_TRIGGER_HAPPY4)
    Event code 708 (BTN_TRIGGER_HAPPY5)
    Event code 709 (BTN_TRIGGER_HAPPY6)
    Event code 710 (BTN_TRIGGER_HAPPY7)
    Event code 711 (BTN_TRIGGER_HAPPY8)
    Event code 712 (BTN_TRIGGER_HAPPY9)
  Event type 3 (EV_ABS)
    Event code 0 (ABS_X)
      Value   6917
      Min        0
      Max    65535
    Event code 1 (ABS_Y)
      Value    255
      Min        0
      Max      255
    Event code 2 (ABS_Z)
      Value    255
      Min        0
      Max      255
    Event code 5 (ABS_RZ)
      Value    255
      Min        0
      Max      255
    Event code 16 (ABS_HAT0X)
      Value      0
      Min       -1
      Max        1
    Event code 17 (ABS_HAT0Y)
      Value      0
      Min       -1
      Max        1
  Event type 4 (EV_MSC)
    Event code 4 (MSC_SCAN)
  Event type 21 (EV_FF)
    Event code 82 (FF_CONSTANT)
    Event code 96 (FF_GAIN)
    Event code 97 (FF_AUTOCENTER)
Properties:
Testing ... (interrupt to exit)
```

It appears that every time the wheel state is updated, it publishes an updated to `/dev/input/event6`.  For example, when the wheel is turned:

```bash
Event: time 1756240038.248521, type 3 (EV_ABS), code 0 (ABS_X), value 7663
Event: time 1756240038.248521, -------------- SYN_REPORT ------------
Event: time 1756240038.679108, type 3 (EV_ABS), code 0 (ABS_X), value 7664
Event: time 1756240038.679108, -------------- SYN_REPORT ------------
Event: time 1756240044.272556, type 3 (EV_ABS), code 0 (ABS_X), value 7663
Event: time 1756240044.272556, -------------- SYN_REPORT ------------
Event: time 1756240044.448397, type 3 (EV_ABS), code 0 (ABS_X), value 7664
Event: time 1756240044.448397, -------------- SYN_REPORT ------------
Event: time 1756240065.457809, type 3 (EV_ABS), code 0 (ABS_X), value 7663
Event: time 1756240065.457809, -------------- SYN_REPORT ------------
Event: time 1756240066.140065, type 3 (EV_ABS), code 0 (ABS_X), value 7664
Event: time 1756240066.140065, -------------- SYN_REPORT ------------

```

Or when the square button is pressed:
```bash
Event: time 1756240143.123011, type 4 (EV_MSC), code 4 (MSC_SCAN), value 90002
Event: time 1756240143.123011, type 1 (EV_KEY), code 289 (BTN_THUMB), value 1
Event: time 1756240143.123011, -------------- SYN_REPORT ------------
Event: time 1756240143.185002, type 4 (EV_MSC), code 4 (MSC_SCAN), value 90002
Event: time 1756240143.185002, type 1 (EV_KEY), code 289 (BTN_THUMB), value 0
Event: time 1756240143.185002, -------------- SYN_REPORT ------------
```

Or when the gas pedal is pressed:
```bash
Event: time 1756240170.561856, type 3 (EV_ABS), code 2 (ABS_Z), value 143
Event: time 1756240170.561856, -------------- SYN_REPORT ------------
Event: time 1756240170.564404, type 3 (EV_ABS), code 2 (ABS_Z), value 152
Event: time 1756240170.564404, -------------- SYN_REPORT ------------
Event: time 1756240170.568135, type 3 (EV_ABS), code 2 (ABS_Z), value 161
Event: time 1756240170.568135, -------------- SYN_REPORT ------------
Event: time 1756240170.569855, type 3 (EV_ABS), code 2 (ABS_Z), value 182
Event: time 1756240170.569855, -------------- SYN_REPORT ------------
Event: time 1756240170.571786, type 3 (EV_ABS), code 2 (ABS_Z), value 193
Event: time 1756240170.571786, -------------- SYN_REPORT ------------
Event: time 1756240170.573970, type 3 (EV_ABS), code 2 (ABS_Z), value 205
Event: time 1756240170.573970, -------------- SYN_REPORT ------------
Event: time 1756240170.576024, type 3 (EV_ABS), code 2 (ABS_Z), value 217
Event: time 1756240170.576024, -------------- SYN_REPORT ------------
Event: time 1756240170.580360, type 3 (EV_ABS), code 2 (ABS_Z), value 228
Event: time 1756240170.580360, -------------- SYN_REPORT ------------
Event: time 1756240170.582094, type 3 (EV_ABS), code 2 (ABS_Z), value 251
Event: time 1756240170.582094, -------------- SYN_REPORT ------------
```

Alternatively, we can view the joystick data on `js1`.

First, get the tool:
```bash
dbcometto@dbcometto-vm:~$ sudo apt-get install joystick
```

Then, using the tool begins with
```bash
dbcometto@dbcometto-vm:~$ jstest /dev/input/js1
Driver version is 2.1.0.
Joystick (Logitech G29 Driving Force Racing Wheel) has 6 axes (X, Y, Z, Rz, Hat0X, Hat0Y)
and 25 buttons (Trigger, ThumbBtn, ThumbBtn2, TopBtn, TopBtn2, PinkieBtn, BaseBtn, BaseBtn2, BaseBtn3, BaseBtn4, BaseBtn5, BaseBtn6, ?, ?, ?, BtnDead, (null), (null), (null), (null), (null), (null), (null), (null), (null)).
Testing ... (interrupt to exit)
```
and outputs nearly unreadable data at first if the terminal is too small.  Increasing the size of the terminal makes it possible to read, and yields an update like

```bash
Axes:  0:-30297  1: 32767  2: 32767  3: 32767  4:     0  5:     0 Buttons:  0:off  1:off  2:off  3:off  4:off  5:off  6:off  7:off  8:off  9:off 10:off 11:off 12:off 13:off 14:off 15:off 16:off 17:off 18:off 19:off 20:off 21:off 22:off 23:off 24:off
```
every time the state of the wheel changes.  Note that each field corresponds to an input.  Axis 0 is the wheel, Axes 1, 2, and 3 are the pedals, and axis 4 and 5 are the keypad.  The rest of the buttons fall into the button category.




## Interfacing the wheel with ROS
