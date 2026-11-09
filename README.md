# Zigbee CO₂ Monitor

An open-source indoor air-quality monitor built around the ESP32-H2 and Sensirion SCD40. It reports CO₂ concentration, temperature, relative humidity, and estimated illuminance over Zigbee, and displays the readings on a round GC9A01 TFT.

## Features

- **CO₂, temperature, and humidity:** measured by the SCD40 over I²C.
- **Zigbee:** the firmware configures the ESP32-H2 as a Zigbee Router and exposes standard measurement clusters for Zigbee2MQTT.
- **Ambient light:** a photoresistor divider feeds the ESP32-H2 ADC. The firmware uses the reading both to estimate illuminance for Zigbee and to adjust display brightness.
- **Automatic backlight:** PWM drives an external N-channel MOSFET to control the GC9A01 backlight. Brightness changes smoothly and is limited to 10–100%.
- **Round display:** a 240 × 240 GC9A01 screen shows a CO₂ gauge, large numeric reading, temperature, humidity, and Zigbee connection state.
- **Selectable temperature units:** Celsius is the default. The Zigbee2MQTT `Temperature unit` setting changes the display. Corrected temperature is published as separate fixed-unit `temperature_celsius` and `temperature_fahrenheit` properties for Home Assistant; the display setting does not change them.
- **Sensor calibration:** temperature offset is adjustable from −5 to +5 °C in 0.1 °C steps; humidity offset is adjustable from −30 to +30 percentage points in steps of 1. Both default to zero, are saved across reboots, and affect the display and Zigbee readings. Temperature calibration is always expressed in Celsius, even when the display uses Fahrenheit.
- **Antialiased assets:** display glyphs and icons are stored as 4-bpp assets in a SPIFFS partition.
- **Zigbee OTA support:** the firmware contains an OTA Upgrade client that validates the received image header before writing the application, and the repository includes a Zigbee OTA image packager. See [OTA_README_EN.md](main/OTA_README_EN.md) for setup and release steps.
- **PCB fabrication files:** the Gerber archive is in [`hardware/gerber`](hardware/gerber/).

## Hardware

| Part | Purpose |
| --- | --- |
| ESP32-H2 Mini | Main controller and IEEE 802.15.4 Zigbee radio |
| Sensirion SCD40 | CO₂, temperature, and relative humidity sensor |
| GC9A01 TFT, 240 × 240 | Round color display |
| Photoresistor and resistor divider | Ambient-light sensing |
| N-channel MOSFET | GC9A01 backlight switching and PWM dimming |
| Push button | Zigbee factory reset when held for 3 seconds |

### 3D-printable enclosure

The enclosure files are available as STL models in [`hardware/3D Print`](hardware/3D%20Print/). Each preview shows the model from isometric, top, and side views.

**Main enclosure — `BOX_004.stl`**

![Main enclosure STL preview: isometric, top, and side views](hardware/3D%20Print/previews/BOX_004_preview.jpg)

**Top cover — `BOX_top_004.stl`**

![Top cover STL preview: isometric, top, and side views](hardware/3D%20Print/previews/BOX_top_004_preview.jpg)

### Pin connections

| Signal | ESP32-H2 GPIO | Notes |
| --- | ---: | --- |
| SCD40 SDA | GPIO10 | I²C data; sensor address `0x62` |
| SCD40 SCL | GPIO11 | I²C clock |
| GC9A01 SCLK | GPIO12 | SPI clock |
| GC9A01 MOSI / SDA | GPIO14 | SPI data; display does not use MISO |
| GC9A01 CS | GPIO2 | Chip select |
| GC9A01 DC | GPIO1 | Data / command |
| GC9A01 RES | GPIO3 | Display reset |
| Backlight MOSFET gate | GPIO0 | 20 kHz PWM; MOSFET pulls BLK low to turn the backlight off |
| Photoresistor divider midpoint | GPIO5 | ADC1 channel 4 |
| Push button | GPIO4 | Active-low; a 3-second hold erases Zigbee network data and restarts the device |

The photoresistor divider is connected between 3.3 V and GND, with its midpoint connected to GPIO5. The backlight MOSFET is an external low-side switch; do not connect the display's BLK pin directly to GPIO0.

## Display behavior

The CO₂ gauge spans **400–2000 ppm** and uses these colors:

| CO₂ level | Color |
| --- | --- |
| Below 800 ppm | Green |
| 800–999 ppm | Yellow |
| 1000–1499 ppm | Orange |
| 1500 ppm and above | Red |

Temperature and humidity text and icons change color with the reading:

| Measurement | Color thresholds |
| --- | --- |
| Temperature | Celsius mode: below 18 °C (64.4 °F): blue; 18–26 °C (64.4–78.8 °F): green; above 26 °C (78.8 °F): red. The thresholds are converted when Fahrenheit is selected. |
| Relative humidity | Below 30%: red; 30–39%: yellow; 40–60%: green; above 60–70%: yellow; above 70%: red |

The ambient-light sensor is calibrated using two reference readings, so illuminance is an estimate rather than a precision lux-meter measurement. Brightness is filtered and eased toward the target to avoid abrupt changes.

## Zigbee and Zigbee2MQTT

The firmware creates endpoint 1 with these standard measurement clusters:

| Measurement | Zigbee cluster |
| --- | --- |
| Illuminance | `0x0400` |
| Temperature | `0x0402` |
| Relative humidity | `0x0405` |
| Carbon dioxide | `0x040D` |
| Temperature display mode | `0x0204` |
| Device calibration (manufacturer-specific) | `0xFC00` |
| OTA Upgrade client | `0x0019` |

The device advertises manufacturer `Custom` and model `ESP32H2_CO2`. Zigbee2MQTT can expose the standard readings as device properties. Link quality is provided by the Zigbee network, not measured by the SCD40.

The device is configured with the Zigbee Router role. Whether it is selected as a parent or route for another device depends on the coordinator, network topology, link quality, and Zigbee routing decisions.

Hold the button on GPIO4 for at least 3 seconds to erase the device's Zigbee network data and restart it. After reboot, permit joining in Zigbee2MQTT to pair it again.

### Temperature and humidity settings

Zigbee2MQTT exposes `Temperature unit`, `Temperature calibration`, and `Humidity calibration` as writable device settings. The temperature unit defaults to Celsius and controls the display. Calibration defaults to zero and is applied before values are sent to Zigbee and rendered on the display. Temperature correction is an absolute Celsius offset (−5 to +5 °C, step 0.1 °C); humidity correction is an absolute percentage-point offset (−30 to +30, step 1).

For Home Assistant dashboards, use `sensor.co2_temperature_celsius` or `sensor.co2_temperature_fahrenheit` for a specific unit. These fixed-unit entities have no `temperature` device class, so Home Assistant does not convert their values to the installation's preferred unit. The device's display-unit selector and these entities are independent.

## Partition layout and OTA

The custom partition table in [`partitions.csv`](partitions.csv) reserves two 1344 KiB app slots for OTA, plus NVS, Zigbee storage, and a 64 KiB SPIFFS partition for display assets. The first installation of this layout must be flashed over USB. Later OTA updates transfer the application image only; they do not update the bootloader, partition table, or `display_fs` assets.

The OTA identifiers currently used by the firmware and packager are:

| Setting | Value |
| --- | --- |
| Current `OTA_FILE_VERSION` | `0x00010005` |
| `OTA_MANUFACTURER_CODE` | `0x1234` |
| `OTA_IMAGE_TYPE` | `0x5678` |

Increase `OTA_FILE_VERSION` for every new OTA image and pass the same value to `make_zigbee_ota.py`; it must be greater than the version installed on the device. The detailed Zigbee2MQTT configuration, image packaging, update, and troubleshooting instructions are in [main/OTA_README_EN.md](main/OTA_README_EN.md).

## Build and flash

The project is developed with **ESP-IDF v6.0.2** for the `esp32h2` target. Dependencies are declared in [`main/idf_component.yml`](main/idf_component.yml) and are resolved by the ESP-IDF Component Manager.

1. Install ESP-IDF and open its configured command prompt or terminal.
2. From the repository root, select the target:

   ```powershell
   idf.py set-target esp32h2
   ```

3. Open the project configuration:

   ```powershell
   idf.py menuconfig
   ```

   Under **Partition Table**, select a custom CSV partition table and set the filename to `partitions.csv`. Save the configuration and exit. The generated `sdkconfig` is intentionally not tracked in Git.

4. Build the application and display filesystem image:

   ```powershell
   idf.py build
   ```

5. Flash over USB and open the serial monitor, replacing `COM7` with the port for your board:

   ```powershell
   idf.py -p COM7 flash monitor
   ```

The component CMake file creates the SPIFFS image from `main/display_fs` and includes it in the flash operation.

## Repository structure

```text
.
├── hardware/gerber/       PCB manufacturing archive
├── OTA/                   Zigbee OTA image and index
├── main/
│   ├── display_fs/         4-bpp display glyph and icon assets
│   ├── zigbee_CO2.c        Firmware application
│   ├── make_zigbee_ota.py  Zigbee OTA image and index packager
│   └── OTA_README_EN.md    OTA instructions
├── zigbee2mqtt/
│   └── ESP32H2_CO2.mjs    External Zigbee2MQTT converter
├── partitions.csv         Custom OTA/SPIFFS partition table
└── CMakeLists.txt         ESP-IDF project entry point
```

## Licenses

The project source is licensed under the MIT License. Third-party fonts and icon assets retain their own licenses; see [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).

## Project status

This is a personal hardware and firmware project. The firmware has been built and run on the device. The Zigbee OTA transfer should be validated on the target device before relying on it for unattended updates.
