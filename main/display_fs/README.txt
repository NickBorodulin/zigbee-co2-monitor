Display assets, D4 version 1

Every *.bin file begins with 7 bytes:
  'D', '4', version 1, width as little-endian uint16, height as little-endian uint16
The remaining bytes are 4-bits-per-pixel coverage, high nibble first.

Copy this entire directory to main/display_fs in the ESP-IDF project.
The main component CMakeLists.txt must call:
  spiffs_create_partition_image(display_fs display_fs FLASH_IN_PROJECT)

The Poetsen One-derived glyph files retain the font's SIL Open Font License 1.1
and attribution. See the font source and license: https://openfontlicense.org/.
