**Notes**

-For JH7110 specifically this repository contains currently  clock gen drivers, GPIO and reset (not properly tested yet).<br />
-The repository also contains some drivers (mostly by M. Horne) for JH7100 (ethernet, MMC), they need further development.<br />
-Does not boot because fails to mount the root file system.<br />
-Some issues related to copyrights need to be solved.<br />
-Codes by J.S. are beginner's work. Criticism is welcome.<br />

**How to test**

-Download FreeBSD's generic SD image dated 2023-07-27 and copy it to your microSD card<br />
https://download.freebsd.org/snapshots/riscv/riscv64/ISO-IMAGES/14.0/FreeBSD-14.0-CURRENT-riscv-riscv64-GENERICSD-20230727-474708c334a7-264358.img.xz <br />
-Clone this repository and build the kernel<br />
-Copy the kernel to SD image's UFS partition's file: /boot/kernel/kernel<br />
-Copy a device tree file jh7110-visionfive-v2.dtb (placed to the root of this repository) to SD image's EFI partition's file: /dtb/jh7110-visionfive-v2.dtb<br />

-One of booting methods is described here (tested without updating firmware):<br />
1) QSPI boot mode (board's physical boot switches must point to the right)<br />
2) after switching power on, stop autoboot by pressing any key when a counter appears<br />
3) then type commands<br />
load mmc 1:1 0x48000000 dtb/jh7110-visionfive-v2.dtb<br />
load mmc 1:1 0x44000000 EFI/BOOT/bootriscv64.efi<br />
bootefi 0x44000000 0x48000000<br />
<br />
