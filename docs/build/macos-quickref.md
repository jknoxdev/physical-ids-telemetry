```
west build -b nrf52840_mdk_usb_dongle lima-node/firmware -- -DCONFIG_BUILD_OUTPUT_UF2=y
```
```
cp build/firmware/zephyr/zephyr.uf2 /Volumes/UF2BOOT      
```