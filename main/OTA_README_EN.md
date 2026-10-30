# Zigbee OTA for the ESP32-H2 CO2 Monitor

This firmware includes a Zigbee OTA Upgrade client (ZCL cluster `0x0019`), writes firmware images using the ESP-IDF OTA API, and includes a packager for Zigbee2MQTT OTA files.

## Version fields

- `zcl_version` is the version of the ZCL specification. Keep the SDK value (`0x08`); it is not the firmware release number.
- `FW_APPLICATION_VERSION` is the one-byte `ApplicationVersion` attribute (`0x0001`) in the Basic cluster. Zigbee2MQTT may display it as the application version.
- `OTA_FILE_VERSION` is the 32-bit version stored in the OTA cluster and OTA image. Increase it for every release. The current base version in the OTA code is `0x00010000`.
- `OTA_MANUFACTURER_CODE` (`0x1234`) and `OTA_IMAGE_TYPE` (`0x5678`) must match in the firmware, OTA image, and Zigbee2MQTT index.

For the next release, build the firmware with `OTA_FILE_VERSION` set to `0x00010001`, then package that exact build with `--version 0x00010001`. Increment the value again for each subsequent release. Do not change `zcl_version` to indicate a firmware release.

## First installation

The previous partition table contained only a `factory` app partition. OTA requires an `otadata` partition and two app slots, `ota_0` and `ota_1`. Flash the OTA-capable firmware and the new partition table over USB for the first installation.

The Zigbee storage and SPIFFS partitions move to new addresses. After this migration, the device may need to be paired with the Zigbee network again. Reflash the SPIFFS image from the new build so the display fonts and other assets are present.

The partition table allocates 1280 KiB to each app slot. The previous firmware image was close to 1216 KiB, so check the size of every new build before flashing it.

## Build an OTA image

From the root of the ESP-IDF project, build the firmware:

```powershell
idf.py build
```

Package the resulting application binary. The version passed to `--version` must exactly match `OTA_FILE_VERSION` in the firmware source:

```powershell
python .\main\make_zigbee_ota.py .\build\zigbee_CO2.bin --version 0x00010001 --output-dir .\ota_release
```

The script creates `ota_release\co2-00010001.ota` and `ota_release\index.json`. Increment the version for the next release, for example to `0x00010002`.

## Configure Zigbee2MQTT

Copy the `ota_release` directory to the Zigbee2MQTT data directory, next to `configuration.yaml`. With Docker, place it inside the host directory mounted as the Zigbee2MQTT data directory so the container can read it.

Add this setting to the existing `ota:` section in `configuration.yaml`:

```yaml
ota:
  zigbee_ota_override_index_location: "ota_release/index.json"
```

If `configuration.yaml` already has an `ota:` section, add the setting under that section; do not create a second `ota:` key. The OTA image path in `index.json` (`ota_release/co2-<version>.ota`) is relative to the directory containing `configuration.yaml`.

Restart Zigbee2MQTT after changing the configuration. Then check whether an update is available for the device and start the update.

## Start an update

In the Zigbee2MQTT interface, open the CO2 device and check for an update. When Zigbee2MQTT reports that an update is available, start it.

You can also check and start an update over MQTT. Replace `CO2` with the device's friendly name or IEEE address if needed.

Check for an update:

Topic:

```text
zigbee2mqtt/bridge/request/device/ota_update/check
```

Payload:

```json
{"id":"CO2"}
```

Start the update:

Topic:

```text
zigbee2mqtt/bridge/request/device/ota_update/update
```

Payload:

```json
{"id":"CO2"}
```

## During and after the update

Keep the device powered and do not disconnect it until Zigbee2MQTT reports that the update has completed. The device state should show `update.state: "updating"` and progress. After the transfer, the ESP32-H2 reboots into the updated firmware. The process can take a while; its duration depends on the device and Zigbee network quality.

After reboot, check the serial log for application startup, confirm that the device is present in the Zigbee network, and verify that new readings appear in Zigbee2MQTT. Re-pairing is normally not required because Zigbee data is stored in a separate partition.

If Zigbee2MQTT does not find an update, check that:

1. `OTA_FILE_VERSION` in the firmware matches the version passed to the packager.
2. The new OTA version is greater than the version currently installed.
3. Zigbee2MQTT can read both `index.json` and the `.ota` file at the configured paths.

Zigbee2MQTT may not offer an image whose version is equal to or lower than the installed version.

## Scope and validation

OTA updates transfer the application image only. They do not update the bootloader, partition table, or `display_fs` filesystem. If fonts or other display assets change, their filesystem image must be updated separately.

The firmware source and OTA partition table are included in this repository. The OTA transfer itself should be tested on the device before treating the OTA process as verified.
