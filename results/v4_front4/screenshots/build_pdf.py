import json
from PIL import Image
from reportlab.lib.pagesizes import A4, landscape
from reportlab.pdfgen import canvas
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import Paragraph, Table, TableStyle
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.colors import HexColor, white

S = "/private/tmp/claude-501/-Users-stefanosfragkoulis-Documents-UtopiaLabs-new-AIRFRAME-DESIGNER/cf6f152b-9caa-463d-9617-54abca8ce590/scratchpad"
R = "/Users/stefanosfragkoulis/Documents/UtopiaLabs/new/AIRFRAME_DESIGNER/results/v4_front4"
LOGO = "/Users/stefanosfragkoulis/Library/Application Support/Claude/local-agent-mode-sessions/skills-plugin/1ee34dae-a177-4a03-8db6-118150cedb0b/6fe7f920-2077-44b1-812f-bae04978fec7/skills/airy-design/assets/airy-logo-black.png"
OUT = f"{R}/ATLAS_V4_option_C_front_motors.pdf"
for w in ("Regular", "Medium", "SemiBold", "Bold"):
    pdfmetrics.registerFont(TTFont(f"M-{w}", f"{S}/fonts/Montserrat-{w}.ttf"))
from reportlab.lib.fonts import addMapping
for f in ("M-Regular", "M-Medium", "M-SemiBold", "M-Bold"):
    addMapping(f, 0, 0, f); addMapping(f, 1, 0, "M-SemiBold" if f != "M-Bold" else "M-Bold"); addMapping(f, 0, 1, f); addMapping(f, 1, 1, "M-SemiBold")

BLACK, GREY, LGREY, GREEN, RULE, CALL = HexColor("#000000"), HexColor("#6B6B6B"), HexColor("#A4A4A4"), HexColor("#1E9255"), HexColor("#DDDDDD"), HexColor("#F5F5F5")
ORANGE, YELLOW, AXGREY = HexColor("#FF5A00"), HexColor("#FFC800"), HexColor("#A0A0AA")
W, H = landscape(A4)
M = 36
steps = json.load(open(f"{R}/cad_steps.json"))
opts = {o["id"]: o for o in json.load(open(f"{R}/dashboard/options.json"))}
bat = json.load(open(f"{R}/battery_study.json"))
mp = json.load(open(f"{R}/dashboard/mass_props.json"))

body = ParagraphStyle("b", fontName="M-Regular", fontSize=8, leading=11.2, textColor=BLACK)
small = ParagraphStyle("s", parent=body, fontSize=7, leading=9.6, textColor=GREY)
step = ParagraphStyle("st", parent=body, fontSize=7.8, leading=10.6, leftIndent=14, firstLineIndent=-14)
cell = ParagraphStyle("c", parent=body, fontSize=7, leading=8.8)
cellh = ParagraphStyle("ch", parent=cell, fontName="M-SemiBold", textColor=GREY, fontSize=6.3, leading=8)


def header(c, bold, rest, sub, n):
    c.setFillColor(BLACK); c.setFont("M-Bold", 19)
    c.drawString(M, H - M - 18, bold)
    x = M + pdfmetrics.stringWidth(bold + " ", "M-Bold", 19)
    c.setFont("M-Regular", 19); c.setFillColor(GREY); c.drawString(x, H - M - 18, rest)
    c.setFont("M-Regular", 8); c.setFillColor(GREY); c.drawString(M, H - M - 33, sub)
    lw, lh = Image.open(LOGO).size; h = 18
    c.drawImage(LOGO, W - M - h * lw / lh, H - M - 17, h * lw / lh, h, mask="auto")
    c.setStrokeColor(GREEN); c.setLineWidth(0.75); c.line(M, H - M - 42, W - M, H - M - 42)
    c.setFont("M-Regular", 6.5); c.setFillColor(LGREY)
    c.drawString(M, 20, "ATLAS V4 · four front motors · option C · static authority model in AIRFRAME_DESIGNER (results/v4_front4), not yet flown in SITL")
    c.drawRightString(W - M, 20, f"{n} / 3")


def image(c, path, x, y, w, caption=None, r=8):
    iw, ih = Image.open(path).size; h = w * ih / iw
    c.saveState(); p = c.beginPath(); p.roundRect(x, y, w, h, r); c.clipPath(p, stroke=0, fill=0)
    c.drawImage(path, x, y, w, h); c.restoreState()
    if caption:
        c.setFont("M-Regular", 6.8); c.setFillColor(GREY); c.drawString(x, y - 10, caption)
    return h


def label(c, x, y, text, size=7, font="M-SemiBold", color=GREY):
    c.setFont(font, size); c.setFillColor(color); c.drawString(x, y, text.upper())


def para(c, text, style, x, y_top, w):
    p = Paragraph(text, style); _, h = p.wrap(w, 1000); p.drawOn(c, x, y_top - h); return h


def callout(c, x, y_top, w, title, text):
    p = Paragraph(text, small); _, h = p.wrap(w - 22, 1000); h += 24
    c.setFillColor(CALL); c.roundRect(x, y_top - h, w, h, 6, stroke=0, fill=1)
    c.setFillColor(GREEN); c.rect(x, y_top - h, 2, h, stroke=0, fill=1)
    label(c, x + 12, y_top - 13, title, 6.8, "M-Bold", BLACK)
    p.drawOn(c, x + 12, y_top - h + 6)
    return h


def table(c, data, x, y_top, widths, align_right_from=1):
    t = Table(data, colWidths=widths)
    t.setStyle(TableStyle([("LINEBELOW", (0, 0), (-1, -1), 0.5, RULE), ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                           ("TOPPADDING", (0, 0), (-1, -1), 3), ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
                           ("LEFTPADDING", (0, 0), (-1, -1), 3), ("RIGHTPADDING", (0, 0), (-1, -1), 3)]))
    _, h = t.wrap(sum(widths), 1000); t.drawOn(c, x, y_top - h); return h


def P(t, st=cell): return Paragraph(t, st)


c = canvas.Canvas(OUT, pagesize=(W, H))
c.setTitle("ATLAS V4 option C front motors"); c.setAuthor("Airy · AIRFRAME_DESIGNER")

# ------------------------------------------------------------------ page 1
header(c, "OPTION C", "FRONT MOTOR SETUP", "ATLAS V4 with four front motors · Fusion: SMALL_SCALE_V4.2, Four front motors, Side by side (Side ESCs) · 23 October 2026", 1)
top = H - M - 56
hw = 505
hh = image(c, f"{S}/pdfimg/hero_iso.png", M, top - hw * 890 / 1320, hw)
# legend under the hero
ly = top - hh - 14
for i, (col, kind, txt) in enumerate(((AXGREY, "line", "thrust axis today"), (ORANGE, "line", "thrust axis, option C"), (YELLOW, "box", "fan turned to option C about its own centre"))):
    x = M + [0, 112, 240][i]
    c.setStrokeColor(col); c.setFillColor(col); c.setLineWidth(2.2)
    if kind == "line": c.line(x, ly + 2.5, x + 16, ly + 2.5)
    else: c.rect(x + 3, ly - 1, 9, 7, stroke=0, fill=1)
    c.setFont("M-Regular", 7); c.setFillColor(GREY); c.drawString(x + 22, ly, txt)

# stats
cx = M + hw + 28; cw = W - M - cx
label(c, cx, top - 6, "Authority at the hover, option C")
stats = [("%.2f" % opts["C_buildable"]["yaw"], "Yaw, rad/s²", "from %.2f as drawn" % opts["as_drawn"]["yaw"], GREEN),
         ("%.2f" % opts["C_buildable"]["roll"], "Roll, rad/s²", "from %.2f as drawn" % opts["as_drawn"]["roll"], BLACK),
         ("%.2f" % opts["C_buildable"]["pitch"], "Pitch, rad/s²", "from %.2f as drawn" % opts["as_drawn"]["pitch"], BLACK),
         ("%.2f" % mp[1]["mass_kg"], "Mass, kg", "+%.2f on V4 today" % (mp[1]["mass_kg"] - mp[0]["mass_kg"]), BLACK)]
for i, (v, lab, sub, col) in enumerate(stats):
    x = cx + (i % 2) * (cw / 2); y = top - 50 - (i // 2) * 66
    c.setFont("M-Bold", 28); c.setFillColor(col); c.drawString(x, y, v)
    c.setFont("M-Medium", 7.5); c.setFillColor(BLACK); c.drawString(x, y - 13, lab)
    c.setFont("M-Regular", 7); c.setFillColor(GREY); c.drawString(x, y - 23, sub)
ty = top - 168
label(c, cx, ty, "Front motor angles")
rows = [[P("Motor", cellh), P("Today", cellh), P("Option C", cellh)],
        [P("<b>M7, M8</b> (outer pair)"), P("upright, 30° in"), P("<font name='M-SemiBold'>22.6° forward, 45° in</font>")],
        [P("<b>M9, M10</b> (side by side)"), P("vertical"), P("<font name='M-SemiBold'>11.1° forward, 30° out</font>")]]
th = table(c, rows, cx, ty - 6, [cw * 0.40, cw * 0.24, cw * 0.36])
nh = para(c, "Tilt is forward from vertical; the lean (cant) is sideways, <i>in</i> toward the centreline or <i>out</i> away from it. "
        "The outer pair leans in harder and tips forward, the vertical pair leans out. Hover pitch becomes 23.8° (as drawn 17.5°).", small, cx, ty - 12 - th, cw)
callout(c, cx, ty - 12 - th - nh - 14, cw, "Why option C", "Yaw is the weakest axis on every V4 layout. Canting the front jets turns their thrust into a yaw "
        "lever 0.43 to 0.53 m ahead of the CG. Option C is the best weakest-axis result of a Monte Carlo over all four front angles "
        "(18,000 random designs, then constrained refinements) with pitch kept at 17 rad/s² or more, every angle within 45° and the side-by-side pair within 30°. "
        "With ±2° mounting error, CG ±10 mm and thrust ±5 %, the worst 5 % still give roll 10.4, pitch 16.4, yaw 7.8.")
callout(c, M, ly - 14, hw, "Clash check in Fusion", "The fans turned to option C were intersected with the model as temporary bodies. "
        "The M7 and M8 inlets touch (0.05 cm³) and clear once each moves 2 mm outboard. No fan reaches the ESCs, the foot or the other "
        "solid parts. Each fan cuts 5.5 to 7 cm³ into today's cradle, so its seats are redrawn (page 2). Mesh parts such as the rods were not checked.")
c.showPage()

# ------------------------------------------------------------------ page 2
header(c, "CAD CHANGES", "FROM TODAY'S DRAWING", "Fusion, direct modelling: Modify › Move/Copy, Rotate. Axes: X to the aircraft's right, Y up, Z aft (the nose is toward −Z). Millimetres.", 2)
top = H - M - 58
lw2 = 400; gw = (lw2 - 10) / 2
label(c, M, top, "Today"); label(c, M + gw + 10, top, "Option C")
y = top - 8
h1 = image(c, f"{S}/pdfimg/nose_today.png", M, y - gw * 660 / 860, gw)
image(c, f"{S}/pdfimg/nose_after.png", M + gw + 10, y - gw * 660 / 860, gw)
c.setFont("M-Regular", 6.8); c.setFillColor(GREY); c.drawString(M, y - h1 - 10, "Nose-on, looking aft: the leans. Orange arrows and yellow fans are option C.")
y = y - h1 - 22
h2 = image(c, f"{S}/pdfimg/side_today.png", M, y - gw * 630 / 1120, gw)
image(c, f"{S}/pdfimg/side_after.png", M + gw + 10, y - gw * 630 / 1120, gw)
c.drawString(M, y - h2 - 10, "From the left, nose to the left: the forward tips.")

rx = M + lw2 + 24; rw = W - M - rx
items = [
    "Work in the visible variant, <b>Four front motors - Side by side - Side ESCs</b>. Select only the fan occurrence named in the table, "
    "not its M4 lock bolts: they move with the new cradle.",
    "<b>Rotation 1, the lean.</b> Axis through the fan centre, parallel to Z. Angle from the table; positive turns X toward Y "
    "(right-hand about +Z). If the axis you pick points to −Z, flip the sign.",
    "<b>Rotation 2, the forward tip.</b> Axis through the same centre, in a plane parallel to X-Y, square to the fan's axis after step 2 "
    "(direction in the table). Positive tips the inlet toward the nose.",
    "<b>Shift M7 2 mm toward −X and M8 2 mm toward +X.</b> Without it their inlets touch.",
    "<b>Check every fan</b> against the last three columns before moving on. ±2° is enough: with CG and thrust scatter added, the worst 5 % of builds lose about 12 %.",
    "<b>Redraw the cradle seats</b> (FOUR_FRONT_MOTOR_CRADLE) and the lock-bolt positions around the moved fans; today's seats overlap each turned fan by 5.5 to 7 cm³.",
    "<b>In AIRFRAME_DESIGNER</b>, fly results/v4_front4/airframe_front4_C_buildable.json. Its hover pitch is 23.8°, so the board's "
    "pitch offset (SENS_BOARD_Y_OFF) and the H-flow lens tilt follow it.",
]
yy = top + 2
for i, t in enumerate(items):
    yy -= para(c, f"<font name='M-Bold' color='#1E9255'>{i + 1}</font>&nbsp;&nbsp;&nbsp;{t}", step, rx, yy, rw) + 5

# table across the bottom
def ax3(v): return ", ".join("0" if abs(x) < 5e-4 else ("%.3f" % x).replace("-", "−") for x in v)
def mm(v): return ", ".join(("%.1f" % x).replace("-", "−") for x in v)
def sg(v): return (("+%.1f" % v) if v > 0 else ("%.1f" % v)).replace("-", "−") + "°"
hdr = ["Motor", "Fusion occurrence", "Fan centre X, Y, Z", "Rot. 1 about +Z", "Rot. 2 axis X, Y, Z", "Rot. 2", "Shift X", "From vertical", "Nose-on lean", "Side view"]
rows = [[P(h, cellh) for h in hdr]]
for s in steps:
    inout = "in" if s["motor"] in ("M7", "M8") else "out"
    rows.append([P("<b>%s</b>" % s["motor"]), P(s["occ"]), P(mm(s["pivot_mm"])), P(sg(s["rot1_deg"])), P(ax3(s["rot2_axis"])),
                 P(sg(s["rot2_deg"])), P(("%+d mm" % s["shift_x_mm"]).replace("-", "−") if s["shift_x_mm"] else "none"),
                 P("%.1f° <font color='#6B6B6B'>(today %.0f°)</font>" % (s["from_vertical"], s["today_from_vertical"])),
                 P("%.1f° %s" % (s["nose_on_lean"], inout)), P("%.1f° forward" % s["side_lean_fwd"])])
tw = W - 2 * M
table(c, rows, M, 132, [tw * f for f in (0.05, 0.19, 0.12, 0.08, 0.13, 0.06, 0.06, 0.12, 0.09, 0.10)])
c.showPage()

# ------------------------------------------------------------------ page 3
header(c, "4TH BATTERY", "PLACEMENT", "Same 700 g pack as the three (160 × 50 × 50 mm). Positions in the structural frame: x forward, y right, z down, from the reference point.", 3)
top = H - M - 58
lw3 = 470
label(c, M, top, "Proposed: four packs side by side, in line with the three (green), seen from above")
hb = image(c, f"{S}/pdfimg/battery_top.png", M, top - 8 - lw3 * 700 / 1500, lw3)
y = top - 8 - hb - 16
label(c, M, y, "Authority against where the 4th pack sits on the centreline (front motors at option C)")
image(c, f"{S}/pdfimg/battery_chart.png", M, y - 6 - lw3 * 660 / 1980, lw3)

rx = M + lw3 + 26; rw = W - M - rx
c.setFont("M-Bold", 13); c.setFillColor(BLACK); c.drawString(rx, top - 8, "In the tray, beside the three")
yy = top - 18
yy -= para(c, "The authority peaks with the fourth pack at the tray (x −145 mm). Each 10 cm forward costs 0.3 to 0.4 rad/s² of yaw and 0.6 to 0.9 of pitch; "
              "on the PDB tray yaw drops to 7.9 and pitch to 16.4.", body, rx, yy, rw) + 8
spec = [("Layout", "four packs, 55 mm pitch as today, centred on the centreline at x −145 mm, z −580 mm"),
        ("Tray", "a four-pack tray: 215 mm of packs against 160 mm today; the rail clamps leave 242 mm, so about 13 mm a side for walls"),
        ("Lid", "move the hinge (today y −92 mm) and the latch (y +85 to +108 mm) to the front and rear ends"),
        ("Result", ("%.2f kg, CG x %.1f, y %.1f, z %.1f mm" % (mp[1]["mass_kg"], mp[1]["cg_x"] * 1000, mp[1]["cg_y"] * 1000, mp[1]["cg_z"] * 1000)).replace("-", "−")),
        ("Not counted", "the wider tray and lid, the 4th pack's lead")]
rows = [[P("<font name='M-SemiBold'>%s</font>" % k), P(v)] for k, v in spec]
yy -= table(c, rows, rx, yy, [rw * 0.22, rw * 0.78]) + 12
label(c, rx, yy, "Other places, front motors at option C"); yy -= 6
cand = bat["candidates"]
names = {"four-pack tray (55 mm pitch, centred)": "Four-pack tray (recommended)", "stacked on the lid, centred": "Stacked on the lid",
         "forward, on the centreline at x 0": "Forward, x 0", "on the PDB tray, x 0.15": "On the PDB tray, x 150",
         "aft of the tray, x -0.31": "Aft of the tray, x −310", "beside the three, one side (y +0.110)": "Beside the three, one side"}
rows = [[P(h, cellh) for h in ("Placement", "CG x, y, z mm", "Roll", "Pitch", "Yaw")]]
for r in cand:
    o = r["option C"]; g = o["cg"]
    rows.append([P(("<b>%s</b>" if "recommended" in names[r["name"]] else "%s") % names[r["name"]]),
                 P(("%.0f, %.1f, %.0f" % (g[0] * 1000, g[1] * 1000, g[2] * 1000)).replace("-", "−")),
                 P("%.2f" % o["roll"]), P("%.2f" % o["pitch"]), P("%.2f" % o["yaw"])])
yy -= table(c, rows, rx, yy, [rw * 0.40, rw * 0.27, rw * 0.11, rw * 0.11, rw * 0.11]) + 12
callout(c, rx, yy, rw, "If the tray cannot grow", "Stacked on the lid gives the same authority with the CG 3 mm higher, but the lid "
        "then cannot open without removing the pack. One pack beside the three on one side shows no loss in this static model, but it moves "
        "the CG 6 mm to the right: the aircraft hovers on uneven motors and rests unevenly on its legs. Keep it as a last resort.")
c.showPage()
c.save()
print("wrote", OUT)
