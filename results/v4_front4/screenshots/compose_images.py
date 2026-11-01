import math, numpy as np
from PIL import Image, ImageDraw, ImageFont
S = "/private/tmp/claude-501/-Users-stefanosfragkoulis-Documents-UtopiaLabs-new-AIRFRAME-DESIGNER/cf6f152b-9caa-463d-9617-54abca8ce590/scratchpad"
F = lambda w, s: ImageFont.truetype(f"{S}/fonts/Montserrat-{w}.ttf", s)
GREEN = (30, 146, 85); WHITE = (255, 255, 255)

def proj(T, d, up, H, ext, W):
    d = np.array(d, float); v = -d / np.linalg.norm(d); up = np.array(up, float)
    upo = up - np.dot(up, v) * v; upo /= np.linalg.norm(upo); right = np.cross(v, upo); s = H / ext
    return lambda p: (W / 2 + s * np.dot(np.array(p) - T, right), H / 2 - s * np.dot(np.array(p) - T, upo))

def pill(d, xy, text, anchor_pt=None, size=30):
    f = F("SemiBold", size); x, y = xy
    w = d.textlength(text, font=f); h = size * 1.25
    if anchor_pt:
        d.line([anchor_pt, (x + w / 2 + 14, y + h / 2)], fill=WHITE, width=3)
        d.ellipse([anchor_pt[0] - 7, anchor_pt[1] - 7, anchor_pt[0] + 7, anchor_pt[1] + 7], fill=GREEN, outline=WHITE, width=2)
    d.rounded_rectangle([x, y, x + w + 28, y + h + 6], radius=(h + 6) / 2, fill=(0, 0, 0))
    d.text((x + 14, y + 3), text, font=f, fill=WHITE)

FRONT = np.array([0.0, 49.0, -30.5])
piv = {"M7": (-5.92, 51.24, -25.5), "M8": (5.92, 51.24, -25.5), "M9": (-5.2, 47.95, -35.5), "M10": (5.2, 47.95, -35.5)}

# hero: three-quarter view after
im = Image.open(f"{S}/shots/iso_front_left.png").convert("RGB")
P = proj(FRONT, (-0.8, 0.75, -1.0), (0, 1, 0), 1100, 60, 1600)
d = ImageDraw.Draw(im)
offs = {"M7": (150, -60), "M8": (-250, -110), "M9": (40, 110), "M10": (-250, -30)}
for m, p in piv.items():
    x, y = P(p); ox, oy = offs[m]; pill(d, (x + ox, y + oy), m, (x, y))
im.crop((240, 120, 1560, 1010)).save(f"{S}/pdfimg/hero_iso.png")

# top view after, labelled
im = Image.open(f"{S}/shots/top.png").convert("RGB")
P = proj(FRONT, (0, 1, 0), (0, 0, -1), 1100, 55, 1600); d = ImageDraw.Draw(im)
offs = {"M7": (-330, 60), "M8": (230, 60), "M9": (-360, -150), "M10": (250, -150)}
for m, p in piv.items():
    x, y = P(p); ox, oy = offs[m]; pill(d, (x + ox, y + oy), m, (x, y))
im.crop((330, 230, 1270, 800)).save(f"{S}/pdfimg/top_after.png")

# before / after pairs, same physical frame
def pair(after, today, box_after, name):
    a = Image.open(f"{S}/shots/{after}.png").convert("RGB")
    t = Image.open(f"{S}/shots/{today}.png").convert("RGB")
    k = 1200 / 55 / (1100 / 55)
    cx, cy = 800, 550; l, tp, r, b = box_after
    ca = a.crop((cx + l, cy + tp, cx + r, cy + b))
    ct = t.crop((int(900 + l * k), int(600 + tp * k), int(900 + r * k), int(600 + b * k))).resize(ca.size, Image.LANCZOS)
    ca.save(f"{S}/pdfimg/{name}_after.png"); ct.save(f"{S}/pdfimg/{name}_today.png")
pair("nose_on", "today_nose_on", (-430, -400, 430, 260), "nose")
pair("side_left", "today_side_left", (-560, -400, 560, 230), "side")

# battery bay, top, with dimensions
im = Image.open(f"{S}/shots/battery_top.png").convert("RGB"); d = ImageDraw.Draw(im)
s = 1200 / 42; X = lambda xcm: 900 + s * xcm; Y = lambda zcm: 600 + s * (zcm - 14.5)
def dim(x0, x1, y, text, col):
    d.line([(x0, y), (x1, y)], fill=col, width=3)
    for x in (x0, x1): d.line([(x, y - 14), (x, y + 14)], fill=col, width=3)
    f = F("SemiBold", 30); w = d.textlength(text, font=f)
    d.rounded_rectangle([(x0 + x1) / 2 - w / 2 - 14, y - 26, (x0 + x1) / 2 + w / 2 + 14, y + 22], radius=22, fill=(0, 0, 0))
    d.text(((x0 + x1) / 2 - w / 2, y - 22), text, font=f, fill=col)
dim(X(-10.75), X(10.75), Y(14.5 - 8.0 - 2.2), "215 mm: four packs at 55 mm pitch", (90, 210, 140))
dim(X(-12.11), X(12.11), Y(14.5 + 8.0 + 2.6), "242 mm between the rail clamps", WHITE)
pill(d, (X(-10.75) - 420, Y(14.5) - 30), "centred at x = −145 mm", (X(-10.75), Y(14.5)))
im.crop((150, 250, 1650, 950)).save(f"{S}/pdfimg/battery_top.png")
im = Image.open(f"{S}/shots/battery_iso.png").convert("RGB")
im.crop((280, 180, 1500, 1020)).save(f"{S}/pdfimg/battery_iso.png")
print("ok")
