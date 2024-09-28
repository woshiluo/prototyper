$ cd ./openwrt
$ git checkout 603a3c6
应用本项目目录下的 `docs/openwrt-patch.patch`。

```shell
$ curl https://raw.githubusercontent.com/rustsbi/prototyper/refs/heads/main/docs/openwrt-patch.patch --output openwrt-patch.patch
$ git apply openwrt-patch.patch
$ sed -i.bak 's/CONFIG_BOOTCOMMAND=*/CONFIG_BOOTCOMMAND="scsi scan; fatload scsi 0:3 84000000 Image; setenv bootargs root=\/dev\/sda4 rw earlycon console=\/dev\/ttyS0 rootwait; booti 0x84000000 - ${fdtcontroladdr};"/' .config
$ cd openwrt
$ ./scripts/feeds update -a
$ ./scripts/feeds install -a
```

修改配置：
```shell
$ make -j$(nproc) menuconfig
进入 `Target System`，选中 `$SiFive U-based RISC-V boards`。

$ make -j$(nproc) kernel_menuconfig
`Device Drivers` $\rightarrow$ `Serial ATA and Parallel ATA drivers (libata)` $\rightarrow$ `AHCI SATA support`  
`Device Drivers` $\rightarrow$ `Network device support` $\rightarrow$ `Ethernet driver support` $\rightarrow$ `Intel devices` $\rightarrow$ `Intel(R) PRO/1000 Gigabit Ethernet support`  
设为 `built-in`。
$ make -j$(nproc) defconfig download clean world
$ cd ..
$ cp ./openwrt/bin/targets/sifiveu/generic/openwrt-sifiveu-generic-sifive_unleashed-ext4-sdcard.img.gz ./
$ gzip -dk openwrt-sifiveu-generic-sifive_unleashed-ext4-sdcard.img.gz