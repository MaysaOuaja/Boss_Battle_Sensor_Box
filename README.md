# Survival Sensor Box Flash Package

This self-contained Windows package deploys the tested Survival Sensor Box firmware to an identical Infineon PSOC(TM) Edge AI Kit.

It flashes the CM55 Deepcraft image, the matching CM33 MicroPython image, and the MicroPython application. The package includes precompiled application bytecode, so neither Deepcraft nor `mpy-cross` is required on the deployment computer.

## Supported Hardware

- Infineon `KIT_PSE84_AI`
- Device: `PSE846GPS2DBZC4A`
- KitProg3 USB connection
- Windows and Python 3

Do not use this package on a different PSoC Edge board. Its firmware, pin mapping, PDM configuration, and flash loader are specific to the supported kit.

## Package Contents

| Path | Purpose |
| --- | --- |
| `firmware/cm55_deepcraft.hex` | Tested CM55 image with the trained Deepcraft siren-direction model |
| `firmware/cm33_micropython.hex` | Matching CM33 MicroPython image with PDM-clock preparation |
| `firmware/PSE84_SMIF.FLM` | Flash loader required by OpenOCD |
| `firmware/qspi_config.cfg` | SMIF flash-bank layout required to erase and program the CM55 image |
| `app/main.py` | Button-gated MicroPython startup file |
| `app/device_main.mpy` | Precompiled Survival Sensor Box application for the included CM33 image |
| `app/device_main.py` | Readable source matching the deployed application bytecode |
| `app/survival_config.py` | User-editable Wi-Fi, LED brightness, temperature, and direction settings |
| `app/bmi270_config.bin` | BMI270 configuration blob |
| `tools/openocd/` | Bundled OpenOCD and KitProg3/PSoC Edge scripts |

The CM55 image uses the currently tested PDM-only audio path. It runs the trained Deepcraft model using the two digital PDM microphones; the analog microphone capture path is not part of this package.

## Flash a New Board

1. Connect the `KIT_PSE84_AI` through its KitProg3 USB connector.
2. Close every serial monitor that uses the board's COM port, including Thonny and VS Code Serial Monitor.
3. Open PowerShell in this package directory.
4. Install the one Python dependency:

   ```powershell
   py -m pip install -r requirements.txt
   ```

5. Flash the complete stack, replacing `COM68` with the board's USB-UART port:

   ```powershell
   py .\deploy_survival_sensor_box.py --port COM68
   ```

6. Reset the board and press its button during the four-second startup window.

The command erases and overwrites the flash sectors occupied by both firmware images, verifies both images through OpenOCD, and verifies the byte size of every uploaded application file. Existing CM55 firmware at the same address is replaced. The KitProg3 USB-UART port can receive a new COM number after the CM33 image resets the board; the script follows the same KitProg3 by its USB serial number.

## Configure Wi-Fi and LEDs

Edit `app/survival_config.py` before an application-only update. Do not edit `app/device_main.mpy`; it is precompiled firmware.

```python
WIFI_TARGET_SSID = "Eric"
LED_BRIGHTNESS = 0.2
TEMPERATURE_OFFSET_C = -13.5
TARGET_WINDOW = 4
```

`WIFI_TARGET_SSID` must exactly match the Wi-Fi network name. `LED_BRIGHTNESS` accepts values from `0.0` (off) to `1.0` (full brightness); `0.2` is the current default. Save the configuration file, close any serial monitor, then upload only the app:

```powershell
py .\deploy_survival_sensor_box.py --port COM68 --skip-cm55 --skip-cm33
```

## Partial Updates

To upload only the MicroPython application after the board has already received this exact CM33 firmware:

```powershell
py .\deploy_survival_sensor_box.py --port COM68 --skip-cm55 --skip-cm33
```

To reflash only the CM55 image or CM33 image, add `--skip-app`. A serial port is not required when the application upload is skipped:

```powershell
py .\deploy_survival_sensor_box.py --skip-cm33 --skip-app
py .\deploy_survival_sensor_box.py --skip-cm55 --skip-app
```

## GitHub Notes

Keep the directory layout unchanged after uploading it to GitHub. The deployment script resolves every file relative to its own location and contains no user-specific absolute paths.

The bundled OpenOCD tooling remains third-party software. See `THIRD_PARTY_NOTICES.md` before redistributing the package.