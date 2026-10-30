"""Extract the current packed 4-bpp display masks into the SPIFFS image folder."""
from pathlib import Path
import re
import math
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "display_fs"
OUT.mkdir(exist_ok=True)

def array_from(text: str, name: str) -> bytes:
    match = re.search(rf"{re.escape(name)}\[[^]]+\]\s*=\s*\{{(.*?)\}};", text, re.S)
    if not match:
        raise ValueError(f"Missing asset array: {name}")
    return bytes(int(value, 16) for value in re.findall(r"0x([0-9A-Fa-f]{2})", match.group(1)))

def save(name: str, width: int, height: int, data: bytes) -> None:
    pixels = width * height
    expected = (pixels + 1) // 2
    if len(data) != expected:
        raise ValueError(f"{name}: expected {expected} bytes, got {len(data)}")
    # Common 4-byte header: ASCII D4, version, width, height (little-endian u16s).
    (OUT / f"{name}.bin").write_bytes(b"D4\x01" + width.to_bytes(2, "little") +
                                       height.to_bytes(2, "little") + data)

large = (ROOT / "poetsen_digits.h").read_text(encoding="utf-8")
small = (ROOT / "poetsen_small_assets.h").read_text(encoding="utf-8")
source = (ROOT / "zigbee_CO2.c").read_text(encoding="utf-8")
large_widths = [54, 29, 49, 47, 50, 48, 50, 44, 49, 50]
for digit, width in enumerate(large_widths):
    save(f"large_{digit}", width, 68, array_from(large, f"poetsen_digit_{digit}"))
save("ppm", 31, 12, array_from(small, "poetsen_ppm_small_coverage"))
def icon_source(name: str, width: int, height: int) -> bytes:
    # These two source masks were removed from the C file when moved to SPIFFS.
    # Reuse the checked-in packed files on later regeneration runs.
    try:
        return array_from(source, name)
    except ValueError:
        asset_name = "co2_icon" if name == "mdi_molecule_co2_coverage" else "wifi_icon"
        blob = (OUT / f"{asset_name}.bin").read_bytes()
        if blob[:3] != b"D4\x01":
            raise ValueError(f"Invalid saved asset: {asset_name}")
        w = int.from_bytes(blob[3:5], "little")
        h = int.from_bytes(blob[5:7], "little")
        if (w, h) != (width, height):
            raise ValueError(f"Unexpected saved asset dimensions: {asset_name}")
        return blob[7:]

save("co2_icon", 32, 32, icon_source("mdi_molecule_co2_coverage", 32, 32))
save("wifi_icon", 16, 16, icon_source("mdi_wifi_coverage", 16, 16))

def pack_levels(levels: list[int]) -> bytes:
    return bytes((levels[i] << 4) | (levels[i + 1] if i + 1 < len(levels) else 0)
                 for i in range(0, len(levels), 2))

def raster(width: int, height: int, inside) -> bytes:
    levels = []
    for y in range(height):
        for x in range(width):
            hits = sum(inside(x + (sx + .5) / 4, y + (sy + .5) / 4)
                       for sy in range(4) for sx in range(4))
            levels.append(round(hits * 15 / 16))
    return pack_levels(levels)

def font_glyph(character: str, size: int = 17, line_height: int = 14) -> tuple[int, bytes]:
    """Rasterize a glyph from the same Poetsen One TTF used for the digits."""
    font_path = ROOT / "PoetsenOne-Regular.ttf"
    font = ImageFont.truetype(str(font_path), size * 4)
    bbox = font.getbbox(character)
    high_width = max(1, bbox[2] - bbox[0])
    high_height = max(1, bbox[3] - bbox[1])
    high_res = Image.new("L", (high_width, high_height), 0)
    ImageDraw.Draw(high_res).text((-bbox[0], -bbox[1]), character, font=font, fill=255)
    ink_bounds = high_res.getbbox()
    if ink_bounds is None:
        raise ValueError(f"Poetsen One glyph {character!r} has no visible pixels")
    high_res = high_res.crop(ink_bounds)
    glyph_width = max(1, (high_res.width + 3) // 4)
    glyph_height = max(1, (high_res.height + 3) // 4)
    antialiased = high_res.resize((glyph_width, glyph_height), Image.Resampling.LANCZOS)

    canvas = Image.new("L", (glyph_width, line_height), 0)
    canvas.paste(antialiased, (0, max(0, (line_height - glyph_height) // 2)))
    levels = [round(value * 15 / 255) for value in canvas.tobytes()]
    return glyph_width, pack_levels(levels)

# Re-render the small digits from the same face at 4x resolution. The source
# masks had very few fractional edge pixels, so they looked stair-stepped.
for digit in range(10):
    glyph_width, glyph_data = font_glyph(str(digit))
    save(f"small_{digit}", glyph_width, 14, glyph_data)

def circle(x, y, cx, cy, radius):
    return (x-cx)**2 + (y-cy)**2 <= radius**2

# Native-size icons, sampled 4x4 per destination pixel and stored as 4-bpp masks.
thermo = raster(24, 24, lambda x, y:
    (9 <= x <= 15 and 5 <= y <= 15) or circle(x, y, 12, 5, 3) or circle(x, y, 12, 15, 5))
save("thermometer", 24, 24, thermo)

# Material-style water drop, matching the existing 24x24 control-point silhouette.
points = [(12.0, 3.25)]
def cubic(p0, p1, p2, p3, n=32):
    for i in range(1, n + 1):
        t = i / n; u = 1 - t
        points.append((u**3*p0[0] + 3*u*u*t*p1[0] + 3*u*t*t*p2[0] + t**3*p3[0],
                       u**3*p0[1] + 3*u*u*t*p1[1] + 3*u*t*t*p2[1] + t**3*p3[1]))
cubic((12,3.25),(12,3.25),(18,10),(18,14))
for i in range(1, 49):
    a = math.pi * i / 48
    points.append((12 + 6*math.cos(a), 14 + 6*math.sin(a)))
cubic((6,14),(6,10),(12,3.25),(12,3.25))
def in_poly(x, y):
    inside = False
    j = len(points) - 1
    for i, (xi, yi) in enumerate(points):
        xj, yj = points[j]
        if (yi > y) != (yj > y) and x < (xj-xi)*(y-yi)/(yj-yi)+xi:
            inside = not inside
        j = i
    return inside
save("water", 24, 24, raster(24, 24, in_poly))

# Small unit/punctuation glyphs. C and % come directly from the supplied TTF.
save("glyph_o", 12, 14, raster(12, 14, lambda x,y: circle(x,y,4.5,3.5,3.0) and not circle(x,y,4.5,3.5,1.5)))
for character, name in (("C", "glyph_C"), ("%", "glyph_percent")):
    glyph_width, glyph_data = font_glyph(character)
    save(name, glyph_width, 14, glyph_data)
save("glyph_dot", 12, 14, raster(12, 14, lambda x,y: 4 <= x <= 6 and 12 <= y <= 13))
save("glyph_minus", 12, 14, raster(12, 14, lambda x,y: 2 <= x <= 8 and 6 <= y <= 7))
