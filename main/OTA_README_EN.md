# Zigbee OTA and sensor settings

This guide covers the device's temperature and humidity settings, Home Assistant temperature entities, and Zigbee OTA releases through Zigbee2MQTT (Z2M).

## Temperature units and calibration

The `Temperature unit` setting in Z2M defaults to `celsius`. It selects the unit shown on the display and used by the legacy MQTT field `temperature`. The device recalculates the display's temperature color thresholds for the selected unit.

The converter also publishes two explicit MQTT properties:

| Property | Unit behavior |
| --- | --- |
| `temperature_celsius` | Celsius value, kept in °C in Home Assistant |
| `temperature_fahrenheit` | Fahrenheit value, kept in °F in Home Assistant |

Use `sensor.co2_temperature_celsius` or `sensor.co2_temperature_fahrenheit` in a Home Assistant card when you want a specific unit. Neither entity has the Home Assistant `temperature` device class, so HA does not convert their values to the preferred unit system. The two values represent the same reading; for example, 26.5 °C is about 79.7 °F.

Temperature calibration is an absolute offset from −5 to +5 °C in 0.1 °C steps. Humidity calibration is an absolute offset from −30 to +30 percentage points in steps of 1. Both start at zero, are stored in NVS across reboots, and affect the displayed and Zigbee-reported measurements. The temperature correction remains in Celsius regardless of the selected display unit.

For a Fahrenheit temperature gauge, use Fahrenheit values for the gauge bounds and color thresholds as well. The display threshold equivalents are 64.4 °F (18 °C) and 78.8 °F (26 °C).

## OTA version fields

- `zcl_version` identifies the Zigbee Cluster Library version. Keep the SDK value; it is not the firmware release number.
- `FW_APPLICATION_VERSION` is the Basic-cluster `ApplicationVersion` attribute.
- `OTA_FILE_VERSION` is the 32-bit OTA firmware version. The current repository value is `0x00010003`.
- `OTA_MANUFACTURER_CODE` (`0x1234`) and `OTA_IMAGE_TYPE` (`0x5678`) must match in the firmware, `.ota` image, and `index.json`.

For each new OTA release, increase `OTA_FILE_VERSION`, build the firmware, and give the same version to the packager. It must be greater than the version installed on the device. For example, the next release after `0x00010003` would use `0x00010004` in both places. Do not reuse a version for different image contents.

## First installation and OTA limits

The partition table provides `otadata` and two 1344 KiB application slots (`ota_0` and `ota_1`). Flash the partition table and the first OTA-capable firmware over USB. OTA then updates the application image only; it does not update the bootloader, partition table, or `display_fs` assets. If display fonts or images change, flash the SPIFFS image separately.

The device validates the manufacturer, image type, version, and declared image size from the received Zigbee OTA file header before writing the application sub-element to the update slot.

## Build and package a Zigbee OTA image

Build the project with ESP-IDF or CLion. The example below uses the CLion output directory; adjust the binary path if your build uses another directory.

```powershell
idf.py build
```

After changing `OTA_FILE_VERSION` in `main/zigbee_CO2.c` to the next release number, package that exact build. This example packages version `0x00010004` into the repository's `OTA` folder and sets a GitHub Raw URL prefix:

```powershell
python .\main\make_zigbee_ota.py .\cmake-build-debug\zigbee_CO2.bin --version 0x00010004 --output-dir .\OTA --url-prefix "https://raw.githubusercontent.com/NickBorodulin/zigbee-co2-monitor/main/OTA"
```

The script creates `OTA/co2-00010004.ota` and updates `OTA/index.json`. Confirm that the firmware version, packager version, header fields, index version, file size, and URL all refer to the same release. Commit and push both the `.ota` file and `index.json` so GitHub Raw can serve them.

## Configure Zigbee2MQTT

There are two ways to make Z2M use the index:

1. **Local index:** put the OTA files in a directory the Z2M container can read, then set `zigbee_ota_override_index_location` inside the existing top-level `ota:` section in `configuration.yaml`:

   ```yaml
   ota:
     zigbee_ota_override_index_location: "OTA/index.json"
   ```

   The path is relative to the Z2M data directory, where `configuration.yaml` resides. If an `ota:` section already exists, add the setting inside it instead of creating a duplicate. Restart Z2M after changing the configuration.

2. **GitHub Raw index:** in the device's OTA controls, select the custom index source and enter:

   ```text
   https://raw.githubusercontent.com/NickBorodulin/zigbee-co2-monitor/main/OTA/index.json
   ```

After changing the source, request an OTA check again. An image will only be offered when its manufacturer, image type, and version match the device and installed firmware.

## Start an update

In Z2M, open the CO2 device, check for updates, then start the offered update. The same operations are available over MQTT. Replace `CO2` with the device's friendly name or IEEE address if needed.

Check for an update:

```text
Topic:   zigbee2mqtt/bridge/request/device/ota_update/check
Payload: {"id":"CO2","url":"https://raw.githubusercontent.com/NickBorodulin/zigbee-co2-monitor/main/OTA/index.json"}
```

Start the update after Z2M reports that a newer image is available:

```text
Topic:   zigbee2mqtt/bridge/request/device/ota_update/update
Payload: {"id":"CO2"}
```

Keep the monitor powered until Z2M reports completion and the device restarts. Verify the firmware version and sensor reports after reboot. Re-pairing is normally unnecessary because Zigbee data is stored in its own partition.

## Troubleshooting and limits

- **No image available:** verify that `OTA_FILE_VERSION`, the packager's `--version`, and `index.json` `fileVersion` match, and that the OTA version is higher than the installed version.
- **Index or download error:** open the index URL and the `.ota` URL from the index to confirm both are reachable. Check that the image file was pushed to the same branch and path.
- **Device rejects the image:** inspect the serial log for manufacturer, image type, version, and OTA-header errors. The identifiers must be `0x1234` and `0x5678`, and the image version must be newer than the running firmware.
- **Download timeout:** keep the device powered and close to the Zigbee network. A timeout means the transfer did not complete; use the serial log to identify whether the device rejected the image or stopped receiving blocks.

OTA transfer should be validated on the target device before relying on unattended updates. It cannot update the bootloader, partition table, or display filesystem.
