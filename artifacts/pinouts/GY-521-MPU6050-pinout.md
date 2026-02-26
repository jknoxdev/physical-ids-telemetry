# GY-521 / MPU-6050 Pinout Reference

> **Module:** GY-521 breakout board (generic)  
> **Chip:** InvenSense MPU-6050  
> **Interface:** I2C  
> **Target:** nRF52840-MDK-USB-Dongle / Zephyr RTOS  

---

## Pin Connections

| GY-521 Pin | Connect To | Notes |
|---|---|---|
| VCC | 3.3V | **Do not use 5V** — connect direct to 3.3V, bypasses onboard regulator, cleaner signal |
| GND | GND | |
| SCL | nRF52840 SCL | Serial Clock Line — I2C clock |
| SDA | nRF52840 SDA | Serial Data Line — I2C bidirectional data |
| INT | nRF52840 GPIO | Interrupt output — pulls low on motion threshold breach, used for ARMED → EVENT_DETECTED transition |
| AD0 | GND | I2C address select — GND = 0x68 (default), 3.3V = 0x69 |
| XDA | Leave floating | Auxiliary Serial Data — secondary I2C bus for external magnetometer, unused in LIMA |
| XCL | Leave floating | Auxiliary Serial Clock — pairs with XDA, unused in LIMA |

---

## I2C Address

| AD0 | Address |
|---|---|
| GND (default) | `0x68` |
| 3.3V | `0x69` |

Allows two MPU-6050 modules on the same I2C bus if needed.

---

## Key Notes

- **Power:** The MPU-6050 sensor operates at 3.3V. The GY-521 module has an onboard 3.3V regulator allowing 5V on VCC, but connect to **3.3V directly** when used with the nRF52840 to avoid power supply noise causing false motion triggers.
- **INT pin:** Wire this up. The MPU-6050 motion detection interrupt (MOT_THR register) allows the nRF52840 to wake from sleep on threshold crossing — eliminates polling and is essential for low-power ARMED state operation.
- **AD0:** Tie to GND for default address 0x68. Only change if running two MPU-6050s on the same bus.
- **XDA/XCL:** Secondary I2C bus for auxiliary sensors (e.g. magnetometer via MotionFusion). Not used in LIMA — leave floating.

---

## Reference Documents

- [MPU-6050 Product Specification Rev 3.4](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [MPU-6050 Register Map and Descriptions](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)

---

*Part of the LIMA Node hardware reference — see `artifacts/specsheets/` for full datasheets.*
