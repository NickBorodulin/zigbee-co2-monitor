#!/usr/bin/env python3
"""Wrap an ESP-IDF app .bin as a Zigbee OTA image and write a Z2M index."""

import argparse
import hashlib
import json
import struct
from pathlib import Path

OTA_FILE_ID = 0x0BEEF11E
OTA_HEADER_VERSION = 0x0100
OTA_HEADER_LENGTH = 56
OTA_STACK_VERSION = 0x0002
OTA_MANUFACTURER_CODE = 0x1234
OTA_IMAGE_TYPE = 0x5678


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("app_bin", type=Path, help="ESP-IDF application .bin")
    parser.add_argument("--version", type=lambda s: int(s, 0), default=0x00010001,
                        help="monotonically increasing OTA file version (default: 0x00010001)")
    parser.add_argument("--output-dir", type=Path, default=Path("ota_release"))
    parser.add_argument("--url-prefix", default=None,
                        help="file path relative to Zigbee2MQTT configuration directory")
    args = parser.parse_args()

    app_path = args.app_bin.resolve()
    app = app_path.read_bytes()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    ota_name = f"co2-{args.version:08x}.ota"
    ota_path = args.output_dir / ota_name
    url_prefix = args.url_prefix if args.url_prefix is not None else args.output_dir.name

    element = struct.pack("<HI", 0x0000, len(app)) + app
    total_size = OTA_HEADER_LENGTH + len(element)
    header = (
        struct.pack(
            "<IHHHHHIH",
            OTA_FILE_ID,
            OTA_HEADER_VERSION,
            OTA_HEADER_LENGTH,
            0,
            OTA_MANUFACTURER_CODE,
            OTA_IMAGE_TYPE,
            args.version,
            OTA_STACK_VERSION,
        )
        + b"ESP32-H2 CO2 monitor".ljust(32, b"\0")
        + struct.pack("<I", total_size)
    )
    if len(header) != OTA_HEADER_LENGTH:
        raise RuntimeError(f"Invalid OTA header length: {len(header)}")

    image = header + element
    ota_path.write_bytes(image)
    index = [{
        "fileVersion": args.version,
        "fileSize": len(image),
        "manufacturerCode": OTA_MANUFACTURER_CODE,
        "imageType": OTA_IMAGE_TYPE,
        "sha512": hashlib.sha512(image).hexdigest(),
        "url": f"{url_prefix.rstrip('/')}/{ota_name}" if url_prefix else ota_name,
    }]
    (args.output_dir / "index.json").write_text(
        json.dumps(index, indent=2) + "\n", encoding="utf-8"
    )
    print(f"OTA image: {ota_path.resolve()} ({len(image)} bytes)")
    print(f"Zigbee2MQTT index: {(args.output_dir / 'index.json').resolve()}")
    print(f"manufacturer=0x{OTA_MANUFACTURER_CODE:04x}, image_type=0x{OTA_IMAGE_TYPE:04x}, "
          f"version=0x{args.version:08x}")


if __name__ == "__main__":
    main()
