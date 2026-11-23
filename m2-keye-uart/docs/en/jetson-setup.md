# Jetson Orin Nano with JetPack 6.2: enabling the M.2 Key E UART

[M2-KeyE-UART](../../README.md) · [Українська](../uk/jetson-setup.md)

This guide turns on the UART of the M.2 Key E slot so the card appears as `/dev/ttyTHS3`. It is written for the **Jetson Orin Nano developer kit with JetPack 6.2** and applies to any board on the P3768 carrier (Orin Nano, Orin NX) with JetPack 6.

What happens: the slot's UART is `uartb`, device-tree node `serial@3110000`. NVIDIA's default device tree disables it. A small device-tree overlay enables it with the high-speed UART driver and gives it the alias `serial3`, which becomes `/dev/ttyTHS3`. The bootloader applies the overlay at every boot; nothing is flashed.

You need: the Jetson with JetPack 6.2 installed and booting, a keyboard and screen or an SSH session, and internet access for one package. About 10 minutes.

## 1. Check the release

```bash
cat /etc/nv_tegra_release
```

The line must start with `# R36`, which is JetPack 6. This guide has not been checked on R35 or older (JetPack 5).

## 2. Insert the card

1. Shut the Jetson down and unplug its power.
2. The M.2 Key E slot is under the Jetson module and holds the Wi-Fi/Bluetooth card. Take the module off, as for any change of the Wi-Fi card.
3. Undo the Wi-Fi card's screw, unplug the antenna leads from it and take it out. Tuck the antenna leads away so that they touch nothing.
4. Insert the M2-KeyE-UART card at an angle, press it down and fix it with the same screw. With wires on the pigtail, lead them out past the screw end. For an extra margin, put a strip of insulating tape between the card and the module.
5. Put the module back and power on. The red PWR LED on the card lights up as soon as the slot has 3.3 V.

The UART is not working yet: that comes with the overlay.

## 3. Build the overlay

```bash
sudo apt update
sudo apt install -y device-tree-compiler
wget https://raw.githubusercontent.com/vdoom/boards/main/m2-keye-uart/jetson/m2-keye-uartb-overlay.dts
dtc -@ -I dts -O dtb -o m2-keye-uartb.dtbo m2-keye-uartb-overlay.dts
sudo cp m2-keye-uartb.dtbo /boot/
```

Without internet on the Jetson, copy [`m2-keye-uartb-overlay.dts`](../../jetson/m2-keye-uartb-overlay.dts) over by USB stick or `scp`. `dtc` must finish without errors; it creates `m2-keye-uartb.dtbo`.

## 4. Add the overlay to the boot entry

Back up the boot configuration first:

```bash
sudo cp /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.orig
sudo nano /boot/extlinux/extlinux.conf
```

Find the line `DEFAULT primary` (the name after `DEFAULT` may differ). Then find the `LABEL` section with that name and add one line at the end of it, indented like the other lines:

```
LABEL primary
      MENU LABEL primary kernel
      LINUX /boot/Image
      ...                                   (leave the existing lines as they are)
      OVERLAYS /boot/m2-keye-uartb.dtbo
```

If that section already has an `OVERLAYS` line, for example one written by jetson-io for the 40-pin header, do not add a second one. Append the file to the existing line, separated by a space:

```
      OVERLAYS /boot/some-other-overlay.dtbo /boot/m2-keye-uartb.dtbo
```

Save with Ctrl+O, Enter, and leave with Ctrl+X.

Optional safety net: copy the whole `LABEL primary` section below itself, rename the copy to `LABEL backup` and `MENU LABEL backup kernel`, and remove the `OVERLAYS` line from the copy. If the Jetson ever fails to boot with the overlay, pick `backup` in the boot menu shown at start-up.

## 5. Reboot and check

```bash
sudo reboot
```

After the reboot:

```bash
tr -d '\0' < /proc/device-tree/bus@0/serial@3110000/status; echo   # okay
ls -l /dev/ttyTHS3
dmesg | grep -i 3110000
```

- `okay`: the UART is enabled. `ls` shows `/dev/ttyTHS3`, and `dmesg` shows the driver attached to `3110000.serial`.
- `disabled`: the overlay was not applied. Check the `OVERLAYS` line, the file name in `/boot` and that you edited the section named in `DEFAULT`. If everything looks right, use the alternative in step 8.

## 6. Use the port without sudo

`ls -l /dev/ttyTHS3` shows the group that owns the port, normally `dialout`. Add yourself to it once, then log out and back in:

```bash
sudo usermod -aG dialout $USER
```

## 7. Loopback test

Connect TX to RX on the pigtail pads, and RTS to CTS if those wires are fitted. Then:

```bash
stty -F /dev/ttyTHS3 115200 raw -echo -crtscts      # crtscts if RTS-CTS is looped too
cat /dev/ttyTHS3 &
echo hello > /dev/ttyTHS3                            # "hello" must come back
kill %1
```

That checks the whole chain: Jetson → card → wire → card → Jetson. Repeat at 921600 baud or higher if you plan to run fast, then remove the loop and wire the flight controller as described in the [README](../../README.md#flight-controller).

## 8. If the overlay is not applied

Merge the overlay into a copy of the device tree and boot from that copy instead.

1. Find the device tree in use. If the `LABEL` section has an `FDT` line, it names the file. Otherwise list `/boot/dtb/`: on the developer kit there is usually a single `kernel_tegra234-p3768-0000+p3767-....dtb` file.
2. Merge, using that name in place of `<your-kernel>`:

   ```bash
   sudo fdtoverlay -i /boot/dtb/<your-kernel>.dtb -o /boot/dtb/<your-kernel>-m2uart.dtb /boot/m2-keye-uartb.dtbo
   ```

3. In the `LABEL` section, remove the overlay from the `OVERLAYS` line and set the `FDT` line, adding one if there is none:

   ```
         FDT /boot/dtb/<your-kernel>-m2uart.dtb
   ```

4. Reboot and check as in step 5.

After a JetPack update, repeat this step: the update installs a fresh device tree, and the merged copy is not updated with it.

## Fused and secure-boot units

Units with secure boot enabled accept only signed overlays and device trees. On those, the overlay has to be built into the signed device tree of the image you flash; the steps above do not apply.

## JetPack 6.2.2

This release has a DMA problem on the Orin high-speed UART: corrupted received data and SMMU faults in `dmesg`. NVIDIA's fix for `serial@3100000` is an `iommus = <&smmu_niso0 TEGRA234_SID_GPCDMA>;` entry on the UART node, or running the UART without DMA. Nobody has confirmed it for `serial@3110000`, which uses the same DMA controller. If you see garbage on RX under this release, apply the same fix to this node; the comment at the top of the overlay file explains how. JetPack 6.2 itself does not need it: the card was tested on it.

## Undo

Put the original boot configuration back and reboot:

```bash
sudo cp /boot/extlinux/extlinux.conf.orig /boot/extlinux/extlinux.conf
sudo reboot
```

To use Wi-Fi again, shut down, take the card out and put the Wi-Fi card back with its antenna leads.
