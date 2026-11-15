"""Generate the kipida_test_4l validation board, schematic, plugin config and spec.

Run with KiCad 10's bundled Python (it provides the pcbnew module):

    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" tools\\generate_kipida_test_4l.py

Writes (overwriting):
    pcb/kipida_test_4l/kipida_test_4l.kicad_pcb
    pcb/kipida_test_4l/kipida_test_4l.kicad_sch
    pcb/kipida_test_4l/kipida_test_4l.kipida.json   (Ki-PIDA power tree)
    VALIDATION.md                                                   (spec + expected results)

With --spec-only, only VALIDATION.md is rewritten (e.g. after updating KIPIDA_RESULTS).

Every structure is defined once below; geometry, schematic parts, plugin
config and the expected (hand-calculated) results all come from it, so they
cannot drift apart. Edit the structures or the stackup here and re-run.
"""
import json
import math
import re
import sys
import uuid
from pathlib import Path

import pcbnew

REPO = Path(__file__).resolve().parent.parent
PROJECT = "kipida_test_4l"
PROJ_DIR = REPO / "pcb" / PROJECT
PCB_FILE = PROJ_DIR / f"{PROJECT}.kicad_pcb"
SCH_FILE = PROJ_DIR / f"{PROJECT}.kicad_sch"
CFG_FILE = PROJ_DIR / f"{PROJECT}.kipida.json"
SPEC_FILE = REPO / "VALIDATION.md"
FP_LIB = "kipida_test"

# --------------------------------------------------------------------------
# Physics constants and stackup (common 4-layer 1.6 mm, 1 oz outer / 0.5 oz inner)
# --------------------------------------------------------------------------
RHO = 1.68e-5            # copper resistivity, ohm*mm (20 C), same value as Ki-PIDA
T_OUT = 0.035            # outer copper, mm (1 oz)
T_IN = 0.0152            # inner copper, mm (0.5 oz)
PREPREG = 0.2104         # mm, F.Cu-In1 and In2-B.Cu
CORE = 1.065             # mm, In1-In2
MASK = 0.01
PLATING = 0.025          # via barrel plating, mm
VIA_DRILL, VIA_PAD = 0.3, 0.6
RS_OUT = RHO / T_OUT     # ohm per square
RS_IN = RHO / T_IN
ZONE_CLEARANCE = 0.3

STACKUP = [  # (name, type, thickness, material, epsilon_r)
    ("F.SilkS", "Top Silk Screen", None, None, None),
    ("F.Paste", "Top Solder Paste", None, None, None),
    ("F.Mask", "Top Solder Mask", MASK, None, None),
    ("F.Cu", "copper", T_OUT, None, None),
    ("dielectric 1", "prepreg", PREPREG, "FR4", 4.4),
    ("In1.Cu", "copper", T_IN, None, None),
    ("dielectric 2", "core", CORE, "FR4", 4.6),
    ("In2.Cu", "copper", T_IN, None, None),
    ("dielectric 3", "prepreg", PREPREG, "FR4", 4.4),
    ("B.Cu", "copper", T_OUT, None, None),
    ("B.Mask", "Bottom Solder Mask", MASK, None, None),
    ("B.Paste", "Bottom Solder Paste", None, None, None),
    ("B.SilkS", "Bottom Silk Screen", None, None, None),
]
BOARD_THICKNESS = sum(t for _n, _t, t, _m, _e in STACKUP if t)

# Plated barrel area and resistance (barrel length = copper mid-plane to mid-plane)
BARREL_AREA = math.pi * (VIA_DRILL * PLATING - PLATING ** 2)
H_F_TO_B = T_OUT / 2 + PREPREG + T_IN + CORE + T_IN + PREPREG + T_OUT / 2
H_F_TO_IN1 = T_OUT / 2 + PREPREG + T_IN / 2
R_VIA_FB = RHO * H_F_TO_B / BARREL_AREA
R_VIA_F_IN1 = RHO * H_F_TO_IN1 / BARREL_AREA
CORNER_SQUARES = 0.56    # a 90 degree bend counts as ~0.56 squares instead of 1.0 along the centre line

# Ki-PIDA results on this board (sense pad to sense pad, volts), mesh 0.1 / 0.025 mm.
# Measured 2026-11-15 after the track (face-weighted mesh) and via (whole drill circle) fixes.
KIPIDA_DATE = "2026-11-15"
KIPIDA_RESULTS = {
    "A1": (47.808e-3, 47.819e-3), "A2": (47.895e-3, 47.904e-3), "A3": (47.947e-3, 47.951e-3),
    "B1": (27.687e-3, 27.726e-3), "C1": (55.155e-3, 55.132e-3), "F1": (47.895e-3, 47.904e-3),
    "F2": (19.148e-3, 19.152e-3), "D1": (7.1985e-3, 7.1996e-3),
    "E1": (21.098e-3, 20.941e-3), "E2": (9.893e-3, 9.547e-3), "E3": (6.200e-3, 6.095e-3),
    "H1": (2.44158, 2.44527),
    "G1": (71.269e-3, 71.309e-3), "G1 L1": (35.634e-3, 35.651e-3), "G1 L2": (59.391e-3, 59.424e-3),
    "I1": (47.940e-3, 47.944e-3), "I1 12V": (22.194e-3, 22.197e-3),
    "K1": (1.7189e-3, 1.7135e-3),
}


def cap_overlap_squares(wide, neck):
    """Squares removed from a neck by the round end cap of a wider track reaching into it (1-D).

    The cap (radius wide/2) is wider than the neck for d < d_max; there the copper width is
    2*sqrt(r^2 - d^2) instead of the neck width: integral of (1/neck - 1/(2 sqrt(r^2 - d^2))).
    """
    r = wide / 2
    d_max = math.sqrt(r * r - (neck / 2) ** 2)
    return d_max / neck - 0.5 * math.asin(d_max / r)

LAYERS = {"F": pcbnew.F_Cu, "In1": pcbnew.In1_Cu, "In2": pcbnew.In2_Cu, "B": pcbnew.B_Cu}
mm = pcbnew.FromMM


def V(x, y):
    return pcbnew.VECTOR2I(mm(x), mm(y))


def uid(name):
    return str(uuid.uuid5(uuid.NAMESPACE_URL, f"kipida_test_4l/{name}"))


# --------------------------------------------------------------------------
# Board / schematic / config builder
# --------------------------------------------------------------------------
class Builder:
    def __init__(self):
        self.board = pcbnew.BOARD()
        self.board.SetCopperLayerCount(4)
        self.nets = {}
        self.parts = []      # dicts used for the schematic
        self.rails = {}      # net -> rail dict for the plugin config
        self.regulators = []
        self.spec = []       # structure descriptions for VALIDATION.md
        self._ref = {"J": 0, "U": 0}

    def net(self, name):
        if name not in self.nets:
            n = pcbnew.NETINFO_ITEM(self.board, name)
            self.board.Add(n)
            self.nets[name] = n
        return self.nets[name]

    def rail(self, net, voltage):
        return self.rails.setdefault(net, {"net_name": net, "nominal_voltage": voltage,
                                           "sources": [], "loads": [], "child_regulators": []})

    # ---- copper ----------------------------------------------------------
    def track(self, net, layer, points, width):
        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            t = pcbnew.PCB_TRACK(self.board)
            t.SetStart(V(x0, y0))
            t.SetEnd(V(x1, y1))
            t.SetWidth(mm(width))
            t.SetLayer(LAYERS[layer])
            t.SetNet(self.net(net))
            self.board.Add(t)

    def via(self, net, x, y):
        v = pcbnew.PCB_VIA(self.board)
        v.SetPosition(V(x, y))
        v.SetDrill(mm(VIA_DRILL))
        v.SetWidth(mm(VIA_PAD))
        v.SetViaType(pcbnew.VIATYPE_THROUGH)
        v.SetNet(self.net(net))
        self.board.Add(v)

    def via_field(self, net, cx, cy, n, pitch=0.9):
        """n vias centred on (cx, cy): 1, a 2x2 square, or 2 rows of n/2."""
        if n == 1:
            pts = [(cx, cy)]
        elif n == 4:
            pts = [(cx + dx, cy + dy) for dx in (-pitch / 2, pitch / 2) for dy in (-pitch / 2, pitch / 2)]
        else:
            cols = n // 2
            pts = [(cx + (i - (cols - 1) / 2) * pitch, cy + dy) for i in range(cols) for dy in (-pitch / 2, pitch / 2)]
        for x, y in pts:
            self.via(net, x, y)

    def zone(self, net, layer, x0, y0, x1, y1):
        z = pcbnew.ZONE(self.board)
        z.SetLayer(LAYERS[layer])
        z.SetNet(self.net(net))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)  # solid: pads fully joined to the copper
        z.SetLocalClearance(mm(ZONE_CLEARANCE))
        o = z.Outline()
        o.NewOutline()
        for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
            o.Append(mm(x), mm(y))
        self.board.Add(z)

    def silk(self, text, x, y):
        t = pcbnew.PCB_TEXT(self.board)
        t.SetText(text)
        t.SetPosition(V(x, y))
        t.SetLayer(pcbnew.F_SilkS)
        t.SetTextSize(V(1.0, 1.0))
        t.SetTextThickness(mm(0.15))
        t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_LEFT)
        self.board.Add(t)

    # ---- parts -----------------------------------------------------------
    def _footprint(self, fpname, ref, value, x, y, side, pads, imax, description):
        """pads: (number, dx, dy, w, h, net[, kind]); kind "SMD" (default, rectangle on the
        footprint's side) or ("PTH", drill_mm) for a round plated through-hole pad (w = diameter)."""
        fp = pcbnew.FOOTPRINT(self.board)
        fp.SetFPIDAsString(f"{FP_LIB}:{fpname}")
        fp.SetReference(ref)
        fp.SetValue(value)
        fp.SetPosition(V(x, y))
        self.board.Add(fp)
        for num, dx, dy, w, h, net, *kind in pads:
            kind = kind[0] if kind else "SMD"
            p = pcbnew.PAD(fp)
            p.SetNumber(num)
            if kind == "SMD":
                p.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
                p.SetShape(pcbnew.PAD_SHAPE_RECTANGLE)
                p.SetSize(V(w, h))
                ls = pcbnew.LSET()
                for layer in (pcbnew.F_Cu, pcbnew.F_Mask):
                    ls.AddLayer(layer)
                p.SetLayerSet(ls)
            else:
                _pth, drill = kind
                p.SetAttribute(pcbnew.PAD_ATTRIB_PTH)
                p.SetShape(pcbnew.PAD_SHAPE_CIRCLE)
                p.SetSize(V(w, w))
                p.SetDrillShape(pcbnew.PAD_DRILL_SHAPE_CIRCLE)
                p.SetDrillSize(V(drill, drill))
                p.SetLayerSet(pcbnew.PAD.PTHMask())   # all copper layers + both masks
            fp.Add(p)
            # Bottom parts are built on top and flipped top-bottom, which mirrors y
            p.SetFPRelativePosition(V(dx, -dy if side == "B" else dy))
            p.SetNet(self.net(net))
        fp.Reference().SetFPRelativePosition(V(0, -(max(pad[4] for pad in pads) / 2 + 1.2)))
        fp.Reference().SetTextSize(V(0.8, 0.8))
        fp.Value().SetVisible(False)
        fp.SetField("Imax", imax)
        fp.GetField("Imax").SetVisible(False)
        fp.SetField("Description", description)
        sym = uid(f"sym/{ref}")
        fp.SetPath(pcbnew.KIID_PATH(f"/{sym}"))
        fp.SetSheetname("/")
        fp.SetSheetfile(SCH_FILE.name)
        if side == "B":
            fp.Flip(V(x, y), pcbnew.FLIP_DIRECTION_TOP_BOTTOM)
        return sym

    def terminal(self, net, role, imax, x, y, sense_dx, sense_dy, side="F", force=(3.0, 4.0),
                 group="", description="", force_drill=None):
        """Two-pad terminal: pad 1 = force (current), pad 2 = small Kelvin sense pad.

        force_drill: pad 1 becomes a round plated through-hole pad (diameter force[0]) with this drill.
        """
        self._ref["J"] += 1
        ref = f"J{self._ref['J']}"
        fpname = f"Terminal_{group}_{role}"
        force_pad = ("1", 0, 0, force[0], force[1], net) + ((("PTH", force_drill),) if force_drill else ())
        pads = [force_pad, ("2", sense_dx, sense_dy, 0.8, 0.8, net)]
        description = description or f"{group} {role} terminal (pad 1 force, pad 2 sense)"
        sym = self._footprint(fpname, ref, role.upper(), x, y, side, pads, imax, description)
        self.parts.append({"kind": "terminal", "ref": ref, "value": role.upper(), "fp": fpname,
                           "uuid": sym, "nets": {"1": net, "2": net}, "imax": imax, "group": group,
                           "description": description})
        return ref

    def regulator(self, net_in, net_out, imax, x, y, group):
        self._ref["U"] += 1
        ref = f"U{self._ref['U']}"
        fpname = f"Regulator_{group}"
        pads = [("1", -3, 0, 2, 2, net_in), ("2", 3, 0, 2, 2, net_out)]
        description = "Regulator stand-in: pad 1 IN, pad 2 OUT (behaviour set in the Ki-PIDA config)"
        sym = self._footprint(fpname, ref, "REG", x, y, "F", pads, imax, description)
        self.parts.append({"kind": "regulator", "ref": ref, "value": "REG", "fp": fpname, "uuid": sym,
                           "nets": {"1": net_in, "2": net_out}, "imax": imax, "group": group,
                           "description": description})
        return ref


# --------------------------------------------------------------------------
# Structures
# --------------------------------------------------------------------------
MARGIN = 5.0     # straight track between a force pad and the nearest sense point (current spreads evenly)
FORCE_HALF = 1.5  # half length of a 3 x 4 mm force pad along the track
TEST_V = 5.0     # nominal voltage of the single-net test rails (absolute value is irrelevant)


def sense_offset(width):
    """Sense pad touching the side of a track of this width (0.1 mm overlap)."""
    return width / 2 + 0.4 - 0.1


def straight(b, sid, net, layer, y, x0, segments, current, title, validates, issue="", cap_correction=False):
    """Source pad, 5 mm lead, measured segments [(length, width)], 5 mm lead, load pad.

    cap_correction: subtract the round end caps of wider sections that reach into a narrower one.
    """
    w0 = segments[0][1]
    side = "B" if layer == "B" else "F"
    xs = x0 + FORCE_HALF + MARGIN                       # first sense point
    lengths = [l for l, _w in segments]
    xe = xs + sum(lengths)                              # second sense point
    x_load = xe + MARGIN + FORCE_HALF
    b.silk(f"{sid}  {title}", x0 - 1.5, y - 5.4)
    b.terminal(net, "src", f"{current:g}A", x0, y, xs - x0, -sense_offset(w0), side, group=sid)
    b.terminal(net, "load", f"{current:g}A", x_load, y, xe - x_load, -sense_offset(segments[-1][1]), side, group=sid)
    pts, x = [(x0, y), (xs, y)], xs
    b.track(net, layer, [(x0, y), (xs, y)], w0)
    for length, width in segments:
        b.track(net, layer, [(x, y), (x + length, y)], width)
        x += length
    b.track(net, layer, [(xe, y), (x_load, y)], segments[-1][1])
    rail = b.rail(net, TEST_V)
    rail["sources"].append({"ref_des": f"J{b._ref['J'] - 1}", "pad_names": ["1"]})
    rail["loads"].append({"ref_des": f"J{b._ref['J']}", "total_current": current, "pad_names": ["1"],
                          "distribution_mode": "UNIFORM"})
    rs = RS_IN if layer.startswith("In") else RS_OUT
    squares = sum(l / w for l, w in segments)
    formula = f"{rs * 1e3:.3f} mOhm/sq x {squares:.1f} sq"
    approx = False
    if cap_correction:
        caps = sum(cap_overlap_squares(wa, wb) for (_la, wa), (_lb, wb) in zip(segments, segments[1:]) if wa > wb) \
            + sum(cap_overlap_squares(wb, wa) for (_la, wa), (_lb, wb) in zip(segments, segments[1:]) if wb > wa)
        formula += (f" = {fmt_mv(rs * squares * current)}, minus the round end caps of the wide sections "
                    f"reaching into the neck ({caps:.2f} sq, 1-D: -{fmt_mv(rs * caps * current)}). "
                    "Current crowding at the steps (not in the formula) adds a little back")
        squares -= caps
        approx = True
    b.spec.append(dict(id=sid, net=net, layer=layer, title=title, current=current,
                       geometry=" + ".join(f"{l:g} mm x {w:g} mm" for l, w in segments),
                       formula=formula, r=rs * squares, drop=rs * squares * current, approx=approx,
                       validates=validates, issue=issue))


def build(b):
    LX = 32.0  # left column: source pad centres
    # A: track widths, F.Cu
    for i, (w, cur) in enumerate(((0.5, 1.0), (1.0, 2.0), (2.0, 4.0))):
        straight(b, f"A{i + 1}", f"VAL_A{i + 1}_W{int(w * 100):03d}", "F", 30 + 10 * i, LX, [(50.0, w)], cur,
                 f"F.Cu track {w:g} mm", "Sheet resistance and track width",
                 "The sense pads (0.1 mm onto the track side) add a little copper at the sense points: "
                 "the drop is ~0.2 mm of track short (A1 -0.4 %, A3 -0.1 %), the same at every mesh size. "
                 "Fixed 2026-11-15: tracks were widened by 0.05 mm per side and every covered grid row "
                 "counted as a full grid step (A1 was -20 to -29 %)")

    # B: neck-down
    straight(b, "B1", "VAL_B1_NECK", "F", 60, LX, [(10.0, 1.0), (10.0, 0.25), (10.0, 1.0)], 1.0,
             "F.Cu 1 mm with 0.25 mm neck", "Series sections, current crowding at a neck",
             "The 1 mm tracks end in round caps (radius 0.5 mm) inside the neck, as KiCad draws them: "
             "real copper, so the expected value includes them", cap_correction=True)

    # C: inner-layer track reached through via fields at both ends
    y, net, w, cur = 72.0, "VAL_C1_IN1", 1.0, 1.0
    xs, xe = LX + FORCE_HALF + MARGIN + 3.0, LX + FORCE_HALF + MARGIN + 3.0 + 50.0
    xv1, xv2 = LX + 4.0, xe + MARGIN - 1.0
    x_load = xv2 + 4.0
    b.silk("C1  In1.Cu track 1 mm (via fields at both ends)", LX - 1.5, y - 5.4)
    b.terminal(net, "src", "1A", LX, y, xs - LX, -2.6, "F", group="C1",
               description="C1 source; sense pad joins the In1 track through a via")
    b.terminal(net, "load", "1A", x_load, y, xe - x_load, -2.6, "F", group="C1",
               description="C1 load; sense pad joins the In1 track through a via")
    b.track(net, "F", [(LX, y), (xv1 + 1.2, y)], 3.0)
    b.via_field(net, xv1, y, 6, pitch=0.9)
    b.track(net, "In1", [(xv1 - 1.2, y), (xv2 + 1.2, y)], w)   # covers both via fields completely
    b.via_field(net, xv2, y, 6, pitch=0.9)
    b.track(net, "F", [(xv2 - 1.2, y), (x_load, y)], 3.0)
    for xsense in (xs, xe):                     # sense: F.Cu pad -> stub -> via onto the In1 track
        b.track(net, "F", [(xsense, y - 2.6), (xsense, y)], 0.3)
        b.via(net, xsense, y)
    rail = b.rail(net, TEST_V)
    rail["sources"].append({"ref_des": f"J{b._ref['J'] - 1}", "pad_names": ["1"]})
    rail["loads"].append({"ref_des": f"J{b._ref['J']}", "total_current": cur, "pad_names": ["1"],
                          "distribution_mode": "UNIFORM"})
    sq = 50.0 / w
    b.spec.append(dict(id="C1", net=net, layer="In1", title="In1.Cu track 1 mm", current=cur,
                       geometry="50 mm x 1 mm on In1.Cu", formula=f"{RS_IN * 1e3:.3f} mOhm/sq x {sq:.1f} sq",
                       r=RS_IN * sq, drop=RS_IN * sq * cur,
                       validates="Inner-layer copper thickness (0.5 oz)",
                       issue="Sense points are vias: read the In1.Cu map. Each sense via barrel shorts the "
                             "track across its 0.3 mm drill (~0.07 mm of track each), hence ~-0.2 %"))

    # F1: like A2 but entirely on B.Cu (bottom pads)
    straight(b, "F1", "VAL_F1_BOTTOM", "B", 84, LX, [(50.0, 1.0)], 2.0, "B.Cu track 1 mm, bottom pads",
             "Pads and tracks on B.Cu (must equal A2)",
             "Fixed 2026-11-15: Ki-PIDA searched layers 0-31 only; B.Cu is 34, so bottom pads were not found")

    # F2: pad over an inner plane of the same net, plane joined only at the source end
    y, net, cur = 98.0, "VAL_F2_PLANE_UNDER", 1.0
    straight(b, "F2", net, "F", y, LX, [(40.0, 1.0)], cur, "F.Cu 1 mm over an In1 plane joined only at the source",
             "Current must stay in the F.Cu track: the plane is a dead end",
             "Fixed 2026-11-15: load current was injected into the plane under the pad (drop far too low)")
    b.zone(net, "In1", LX - 2.0, y - 5.0, LX + 2 * FORCE_HALF + 2 * MARGIN + 40.0 + 2.0, y + 5.0)
    b.via_field(net, LX + FORCE_HALF + 2.0, y, 4, pitch=0.9)  # plane joined between pad and first sense

    # G: bus with three loads
    y, net = 114.0, "VAL_G1_BUS"
    xs = LX + FORCE_HALF + MARGIN
    b.silk("G1  F.Cu 1 mm bus, three 1 A loads at 25/50/75 mm", LX - 1.5, y - 5.4)
    b.terminal(net, "src", "3A", LX, y, xs - LX, -sense_offset(1.0), "F", group="G1")
    rail = b.rail(net, TEST_V)
    rail["sources"].append({"ref_des": f"J{b._ref['J']}", "pad_names": ["1"]})
    b.track(net, "F", [(LX, y), (xs + 75.0 + 0.5, y)], 1.0)
    for k, d in enumerate((25.0, 50.0, 75.0)):
        xj = xs + d
        b.terminal(net, "load", "1A", xj, y + 5.0, 0.0, -5.0 - sense_offset(1.0), "F", group=f"G1L{k + 1}",
                   description=f"G1 load {k + 1} at {d:g} mm; sense pad on the bus at the junction")
        b.track(net, "F", [(xj, y), (xj, y + 5.0)], 1.0)
        rail["loads"].append({"ref_des": f"J{b._ref['J']}", "total_current": 1.0, "pad_names": ["1"],
                              "distribution_mode": "UNIFORM"})
    seg_r = RS_OUT * 25.0 / 1.0
    v = [-(3 * seg_r), -(3 * seg_r + 2 * seg_r), -(3 * seg_r + 2 * seg_r + 1 * seg_r)]
    b.spec.append(dict(id="G1", net=net, layer="F", title="Bus with three 1 A loads", current=3.0,
                       geometry="1 mm bus, loads at 25 / 50 / 75 mm from the source sense point",
                       formula=f"25 mm segments of {seg_r * 1e3:.1f} mOhm carrying 3 / 2 / 1 A",
                       r=None, drop=-v[2], loads_v=v,
                       validates="Several loads on one net, superposition",
                       issue="~-1 %: copper of the source sense pad and of the load branch junctions "
                             "(same at every mesh size)"))

    RX = 115.0  # right column
    # D: plane with full-edge contacts
    net, x0, y0, L, W, cur = "VAL_D1_PLANE", RX, 25.0, 40.0, 20.0, 10.0
    b.silk("D1  F.Cu plane 40 x 20 mm, full-edge contacts", x0, y0 - 2.5)
    b.zone(net, "F", x0, y0, x0 + L, y0 + W)
    b.terminal(net, "src", "10A", x0 + 1.0, y0 + W / 2, 4.0, 0.0, "F", force=(2.0, W), group="D1")
    b.terminal(net, "load", "10A", x0 + L - 1.0, y0 + W / 2, -4.0, 0.0, "F", force=(2.0, W), group="D1")
    rail = b.rail(net, TEST_V)
    rail["sources"].append({"ref_des": f"J{b._ref['J'] - 1}", "pad_names": ["1"]})
    rail["loads"].append({"ref_des": f"J{b._ref['J']}", "total_current": cur, "pad_names": ["1"],
                          "distribution_mode": "UNIFORM"})
    sq = (L - 2 * 2.0 - 2 * 3.0) / W   # sense points 3 mm inside the 2 mm force pads
    b.spec.append(dict(id="D1", net=net, layer="F", title="Plane, full-edge contacts", current=cur,
                       geometry="zone 40 x 20 mm, force pads 2 x 20 mm at both ends, sense 30 mm apart",
                       formula=f"{RS_OUT * 1e3:.3f} mOhm/sq x {sq:.2f} sq", r=RS_OUT * sq,
                       drop=RS_OUT * sq * cur,
                       validates="2D plane conduction (uniform current: exact)", issue=""))

    # E: via arrays F.Cu -> B.Cu -> F.Cu
    for k, n in enumerate((1, 4, 10)):
        net, y, cur = f"VAL_E{k + 1}_VIA{n:02d}", 58.0 + 9.0 * k, 5.0
        xv1, xv2 = RX + 8.0, RX + 14.0
        x_load = RX + 22.0
        b.silk(f"E{k + 1}  {n} via(s) down and {n} up", RX - 1.5, y - 5.0)
        b.terminal(net, "src", "5A", RX, y, xv1 - 2.5 - RX, -2.3, "F", group=f"E{k + 1}")
        b.terminal(net, "load", "5A", x_load, y, xv2 + 2.5 - x_load, -2.3, "F", group=f"E{k + 1}")
        # Strips as rectangular zones: square ends, so F.Cu stays split between the two
        # via fields (4 mm wide tracks have 2 mm round end caps that would bridge the gap).
        half = 2.2  # covers the largest via field (10 vias: +-1.8 mm + via pad)
        b.zone(net, "F", RX - FORCE_HALF, y - 2.0, xv1 + half, y + 2.0)
        b.via_field(net, xv1, y, n, pitch=0.9)
        b.zone(net, "B", xv1 - half, y - 2.0, xv2 + half, y + 2.0)
        b.via_field(net, xv2, y, n, pitch=0.9)
        b.zone(net, "F", xv2 - half, y - 2.0, x_load + FORCE_HALF, y + 2.0)
        rail = b.rail(net, TEST_V)
        rail["sources"].append({"ref_des": f"J{b._ref['J'] - 1}", "pad_names": ["1"]})
        rail["loads"].append({"ref_des": f"J{b._ref['J']}", "total_current": cur, "pad_names": ["1"],
                              "distribution_mode": "UNIFORM"})
        r = 2 * R_VIA_FB / n + RS_OUT * 6.0 / 4.0 + 2 * RS_OUT * 2.5 / 4.0
        b.spec.append(dict(id=f"E{k + 1}", net=net, layer="F/B", title=f"{n} via(s) down + {n} up", current=cur,
                           geometry=f"4 mm zone strips (F.Cu split by a 1.6 mm gap); {n} x 0.3 mm vias F->B, 6 mm on B.Cu, {n} vias B->F",
                           formula=f"no closed form. 1-D estimate: 2 x {R_VIA_FB * 1e3:.3f}/{n} mOhm (vias) + "
                                   f"{(RS_OUT * 11.0 / 4.0) * 1e3:.2f} mOhm (11 mm of 4 mm strip) = {r * 1e3:.2f} mOhm "
                                   f"-> {fmt_mv(r * cur)}",
                           r=r, drop=None, estimate=r * cur,
                           validates="Via barrel resistance, vias in parallel, current crowding into vias",
                           issue="The 1-D estimate is not a reference. It ignores (a) current crowding into the vias, "
                                 "which raises the drop (strongest for E1: 5 A through one via), and (b) the 4.4 mm "
                                 "where the F.Cu and B.Cu zones overlap at each via field and carry current side by "
                                 "side, which lowers it (strongest for E3: vias spread over 3.6 mm). Check instead "
                                 "that Ki-PIDA converges with the mesh (it does since 2026-11-15; each via used to be "
                                 "joined at one grid point and E1 rose from +25 % to +37 % as the mesh was refined) "
                                 "and, if possible, against a real-board measurement"))

    # H: PCB heater (meander)
    net, w, legs, pitch, leg_len, cur = "VAL_H1_HEATER", 0.2, 40, 0.6, 25.0, 1.0
    xa, y0 = RX + 6.0, 84.0
    xb = xa + leg_len
    b.silk("H1  heater: 0.2 mm meander, 40 x 25 mm legs", RX - 1.5, y0 - 4.6)
    pts = [(xa - 4.5, y0), (xa, y0)]
    for i in range(legs):
        y = y0 + i * pitch
        x_from, x_to = (xa, xb) if i % 2 == 0 else (xb, xa)
        if i > 0:
            pts.append((x_from, y))
        pts.append((x_to, y))
    y_end = y0 + (legs - 1) * pitch
    pts.append((xa - 4.5, y_end))
    b.track(net, "F", pts, w)
    b.terminal(net, "src", "1A", xa - 4.5, y0, 3.0, -sense_offset(w), "F", force=(3.0, 3.0), group="H1")
    b.terminal(net, "load", "1A", xa - 4.5, y_end, 3.0, sense_offset(w), "F", force=(3.0, 3.0), group="H1")
    rail = b.rail(net, TEST_V)
    rail["sources"].append({"ref_des": f"J{b._ref['J'] - 1}", "pad_names": ["1"]})
    rail["loads"].append({"ref_des": f"J{b._ref['J']}", "total_current": cur, "pad_names": ["1"],
                          "distribution_mode": "UNIFORM"})
    lead = 1.5                         # sense pads sit 1.5 mm before the meander starts / after it ends
    centre = 2 * lead + legs * leg_len + (legs - 1) * pitch
    corners = 2 * (legs - 1)
    sq = centre / w - (1.0 - CORNER_SQUARES) * corners
    r = RS_OUT * sq
    b.spec.append(dict(id="H1", net=net, layer="F", title="PCB heater (meander)", current=cur,
                       geometry=f"0.2 mm track, {legs} legs x {leg_len:g} mm, pitch {pitch:g} mm, centre line {centre:.1f} mm, {corners} corners",
                       formula=f"{RS_OUT * 1e3:.3f} mOhm/sq x ({centre:.1f}/0.2 - 0.44 x {corners}) sq",
                       r=r, drop=r * cur, power=r * cur ** 2,
                       validates="Long thin track with many corners; heat source for future thermal tests",
                       issue="The 0.4 mm gap between legs must stay larger than the mesh step, or neighbouring "
                             "legs merge (use 0.1 mm or finer)"))

    # I: regulator chain 12 V -> 5 V
    y = 135.0
    net12, net5, cur = "VAL_I1_12V", "VAL_I1_5V", 2.0
    xs = LX + FORCE_HALF + MARGIN
    xe = xs + 50.0
    xu = xe + MARGIN + 1.0 + 3.0                   # regulator centre (IN pad at -3)
    xs2 = xu + 3.0 + 1.0 + MARGIN
    xe2 = xs2 + 50.0
    x_load = xe2 + MARGIN + FORCE_HALF
    b.silk("I1  12 V -> regulator -> 5 V, 2 A load", LX - 1.5, y - 5.4)
    b.terminal(net12, "src", f"{5 * 2 / (0.9 * 12):.2f}A", LX, y, xs - LX, -sense_offset(1.0), "F", group="I1")
    src_ref = f"J{b._ref['J']}"
    b.regulator(net12, net5, "2A", xu, y, "I1")
    load_ref = b.terminal(net5, "load", "2A", x_load, y, xe2 - x_load, -sense_offset(1.0), "F", group="I1")
    b.track(net12, "F", [(LX, y), (xu - 3.0, y)], 1.0)
    b.track(net5, "F", [(xu + 3.0, y), (x_load, y)], 1.0)
    # 12 V sense at the regulator end: a sense pad of the source terminal can't reach, add a sense-only terminal
    b.terminal(net12, "sense", "0A", xe, y + 4.0, 0.0, -4.0 + sense_offset(1.0) - 0.8, "F", force=(1.5, 1.5),
               group="I1", description="I1 12 V sense point at the regulator input (no current)")
    b.track(net12, "F", [(xe, y), (xe, y + 4.0)], 0.3)
    # Same for the 5 V side: sense point 5 mm after the regulator OUT pad
    b.terminal(net5, "sense", "0A", xs2, y + 4.0, 0.0, -4.0 + sense_offset(1.0) - 0.8, "F", force=(1.5, 1.5),
               group="I1", description="I1 5 V sense point at the regulator output (no current)")
    b.track(net5, "F", [(xs2, y), (xs2, y + 4.0)], 0.3)
    r12 = b.rail(net12, 12.0)
    r12["sources"].append({"ref_des": src_ref, "pad_names": ["1"]})
    r12["child_regulators"].append({"name": "U1 (VAL_I1_5V)", "input_rail_name": net12, "input_ref_des": "U1",
                                    "input_pad_names": ["1"], "output_rail_name": net5, "output_ref_des": "U1",
                                    "output_pad_names": ["2"], "reg_type": "SWITCHING", "efficiency": 0.9})
    r5 = b.rail(net5, 5.0)
    r5["loads"].append({"ref_des": load_ref, "total_current": cur, "pad_names": ["1"],
                        "distribution_mode": "UNIFORM"})
    sq = 50.0
    i_in_sw = 5.0 * cur / (0.9 * 12.0)
    b.spec.append(dict(id="I1", net=f"{net12} / {net5}", layer="F", title="Regulator chain 12 V -> 5 V", current=cur,
                       geometry="1 mm tracks, 50 mm between sense points on each side",
                       formula=f"5 V: {cur:g} A x {RS_OUT * sq * 1e3:.1f} mOhm; 12 V: I_in x {RS_OUT * sq * 1e3:.1f} mOhm",
                       r=RS_OUT * sq, drop=RS_OUT * sq * cur,
                       reg=dict(i_lin=cur, i_sw=i_in_sw, d12_lin=RS_OUT * sq * cur, d12_sw=RS_OUT * sq * i_in_sw),
                       validates="Power-tree current propagation (linear and switching)",
                       issue="Sense pads as in A"))

    # K: plated through-hole pad, F.Cu -> barrel -> B.Cu, sensed at both ends of the barrel
    net, y, cur, w = "VAL_K1_PTH", 120.0, 5.0, 2.0
    pth_pad, pth_drill = 2.0, 1.0
    x_src = RX + 6.0                    # clear of the last G1 load pad (x <= 115)
    x_dut, x_load = x_src + 10.0, x_src + 20.0
    tap = 2.5                           # sense stubs: from the pad centre to the far side, away from the current
    b.silk("K1  through-hole pad F.Cu -> B.Cu, 5 A", x_src - 1.5, y - 5.0)
    src_ref = b.terminal(net, "src", "5A", x_src, y, 3.0, -sense_offset(w), "F", group="K1")
    dut_ref = b.terminal(net, "dut", "5A", x_dut, y, tap, 0.0, "F", force=(pth_pad, pth_pad), force_drill=pth_drill,
                         group="K1", description="K1 pad under test: pad 1 through-hole (1.0 mm drill, 2.0 mm pad); "
                                                 "pad 2 F.Cu sense on the far side of the pad (no current)")
    b.terminal(net, "sense", "0A", x_dut - tap, y + 3.5, 0.0, -3.5, "B", force=(1.5, 1.5), group="K1",
               description="K1 B.Cu sense on the far side of the pad under test (no current)")
    load_ref = b.terminal(net, "load", "5A", x_load, y, -3.0, sense_offset(w), "B", force=(2.5, 2.5),
                          force_drill=pth_drill, group="K1",
                          description="K1 load on a through-hole pad (current drawn from every layer of the pad)")
    b.track(net, "F", [(x_src, y), (x_dut, y)], w)              # current in on F.Cu ...
    b.track(net, "B", [(x_dut, y), (x_load, y)], w)             # ... out on B.Cu
    b.track(net, "F", [(x_dut, y), (x_dut + tap, y)], 0.3)      # F.Cu sense stub (+x, current comes from -x)
    b.track(net, "B", [(x_dut, y), (x_dut - tap, y)], 0.3)      # B.Cu sense stub (-x, current leaves to +x)
    b.track(net, "B", [(x_dut - tap, y), (x_dut - tap, y + 3.5)], 0.3)  # to the sense terminal's dummy pad
    rail = b.rail(net, TEST_V)
    rail["sources"].append({"ref_des": src_ref, "pad_names": ["1"]})
    rail["loads"].append({"ref_des": load_ref, "total_current": cur, "pad_names": ["1"],
                          "distribution_mode": "UNIFORM"})
    area = math.pi * (pth_drill * PLATING - PLATING ** 2)
    r = RHO * H_F_TO_B / area
    b.spec.append(dict(id="K1", net=net, layer="F/B", title="Through-hole pad, F.Cu -> B.Cu", current=cur,
                       geometry=f"{pth_pad:g} mm pad, {pth_drill:g} mm drill; sense stubs on F.Cu and B.Cu at the far "
                                f"side of the pad ({dut_ref} pad 2 -> sense terminal pad 2)",
                       formula=f"barrel: rho x {H_F_TO_B:.4f} mm / (pi x ({pth_drill:g} x {PLATING:g} - "
                               f"{PLATING:g}^2)) mm^2 = {r * 1e3:.3f} mOhm",
                       r=r, drop=r * cur,
                       validates="Through-hole pad barrel (pad drill, all layers), source/load on a through-hole pad",
                       issue="The sense stubs sit on the far side of the pad, where no current flows in the pad "
                             "copper, so they read the two ends of the barrel. Real barrels: plating thickness "
                             "varies (typ. 20-35 um), so expect +-25 % on a real board. The load sits on a "
                             "through-hole pad too: if Ki-PIDA did not find it, the drop would be 0"))

    # Board outline
    edge = pcbnew.PCB_SHAPE(b.board)
    edge.SetShape(pcbnew.SHAPE_T_RECTANGLE)
    edge.SetStart(V(20.0, 18.0))
    edge.SetEnd(V(195.0, 145.0))
    edge.SetLayer(pcbnew.Edge_Cuts)
    edge.SetWidth(mm(0.1))
    b.board.Add(edge)


# --------------------------------------------------------------------------
# Writers
# --------------------------------------------------------------------------
def stackup_block():
    out = ["\t\t(stackup"]
    for name, typ, t, mat, er in STACKUP:
        out.append(f'\t\t\t(layer "{name}"')
        out.append(f'\t\t\t\t(type "{typ}")')
        if t is not None:
            out.append(f"\t\t\t\t(thickness {t:g})")
        if mat:
            out.append(f'\t\t\t\t(material "{mat}")')
            out.append(f"\t\t\t\t(epsilon_r {er:g})")
            out.append("\t\t\t\t(loss_tangent 0.02)")
        out.append("\t\t\t)")
    out += ['\t\t\t(copper_finish "None")', "\t\t\t(dielectric_constraints no)", "\t\t)"]
    return "\n".join(out)


def write_pcb(b):
    b.board.GetDesignSettings().SetBoardThickness(mm(BOARD_THICKNESS))
    pcbnew.ZONE_FILLER(b.board).Fill(b.board.Zones())
    pcbnew.SaveBoard(str(PCB_FILE), b.board)
    text = PCB_FILE.read_text(encoding="utf-8")
    text = re.sub(r"\(setup\n", "(setup\n" + stackup_block() + "\n", text, count=1)
    PCB_FILE.write_text(text, encoding="utf-8")


def _prop(name, value, x, y, hide=False, indent="\t\t", angle=0):
    h = f"{indent}\t(hide yes)\n" if hide else ""
    return (f'{indent}(property "{name}" "{value}"\n{indent}\t(at {x:g} {y:g} {angle})\n{h}'
            f"{indent}\t(show_name no)\n{indent}\t(do_not_autoplace no)\n"
            f"{indent}\t(effects\n{indent}\t\t(font\n{indent}\t\t\t(size 1.27 1.27)\n{indent}\t\t)\n{indent}\t)\n{indent})\n")


def _pin(num, name, x, y, angle, length, indent="\t\t\t\t"):
    eff = lambda: f"{indent}\t\t(effects\n{indent}\t\t\t(font\n{indent}\t\t\t\t(size 1.27 1.27)\n{indent}\t\t\t)\n{indent}\t\t)\n"
    return (f"{indent}(pin passive line\n{indent}\t(at {x:g} {y:g} {angle})\n{indent}\t(length {length:g})\n"
            f'{indent}\t(name "{name}"\n{eff()}{indent}\t)\n{indent}\t(number "{num}"\n{eff()}{indent}\t)\n{indent})\n')


def _lib_symbol(name, ref, description, rect, pins):
    s = (f'\t\t(symbol "{FP_LIB}:{name}"\n\t\t\t(pin_names\n\t\t\t\t(offset 1.016)\n\t\t\t)\n'
         "\t\t\t(exclude_from_sim no)\n\t\t\t(in_bom yes)\n\t\t\t(on_board yes)\n\t\t\t(in_pos_files yes)\n"
         "\t\t\t(duplicate_pin_numbers_are_jumpers no)\n")
    s += _prop("Reference", ref, 0, 5.08, indent="\t\t\t")
    s += _prop("Value", name, 0, -6.35, indent="\t\t\t")
    s += _prop("Footprint", "", 0, 0, hide=True, indent="\t\t\t")
    s += _prop("Datasheet", "", 0, 0, hide=True, indent="\t\t\t")
    s += _prop("Description", description, 0, 0, hide=True, indent="\t\t\t")
    (x0, y0), (x1, y1) = rect
    s += (f'\t\t\t(symbol "{name}_0_1"\n\t\t\t\t(rectangle\n\t\t\t\t\t(start {x0:g} {y0:g})\n\t\t\t\t\t(end {x1:g} {y1:g})\n'
          "\t\t\t\t\t(stroke\n\t\t\t\t\t\t(width 0.254)\n\t\t\t\t\t\t(type default)\n\t\t\t\t\t)\n"
          "\t\t\t\t\t(fill\n\t\t\t\t\t\t(type background)\n\t\t\t\t\t)\n\t\t\t\t)\n\t\t\t)\n")
    s += f'\t\t\t(symbol "{name}_1_1"\n' + "".join(_pin(*p) for p in pins) + "\t\t\t)\n"
    s += "\t\t\t(embedded_fonts no)\n\t\t)\n"
    return s


# Pin connection points relative to the symbol origin (schematic y grows downwards)
TERMINAL_PINS = [("1", "F", -7.62, 0, 0, 2.54), ("2", "S", -7.62, -2.54, 0, 2.54)]
REGULATOR_PINS = [("1", "IN", -10.16, 0, 0, 2.54), ("2", "OUT", 10.16, 0, 180, 2.54)]


def _global_label(net, x, y, angle):
    justify = "right" if angle == 180 else "left"
    u = uid(f"label/{net}/{x:g}/{y:g}")
    return (f'\t(global_label "{net}"\n\t\t(shape passive)\n\t\t(at {x:g} {y:g} {angle})\n\t\t(fields_autoplaced yes)\n'
            f"\t\t(effects\n\t\t\t(font\n\t\t\t\t(size 1.27 1.27)\n\t\t\t)\n\t\t\t(justify {justify})\n\t\t)\n"
            f'\t\t(uuid "{u}")\n'
            + _prop("Intersheetrefs", "${INTERSHEET_REFS}", x, y, hide=True, indent="\t\t") + "\t)\n")


def _text(text, x, y):
    return (f'\t(text "{text}"\n\t\t(exclude_from_sim no)\n\t\t(at {x:g} {y:g} 0)\n'
            f"\t\t(effects\n\t\t\t(font\n\t\t\t\t(size 1.524 1.524)\n\t\t\t\t(bold yes)\n\t\t\t)\n\t\t\t(justify left bottom)\n\t\t)\n"
            f'\t\t(uuid "{uid("text/" + text)}")\n\t)\n')


def write_sch(b, root_uuid):
    out = ["(kicad_sch", "\t(version 20260306)", '\t(generator "eeschema")', '\t(generator_version "10.0")',
           f'\t(uuid "{root_uuid}")', '\t(paper "A3")',
           "\t(title_block", '\t\t(title "Ki-PIDA validation board, 4 layers")', '\t\t(rev "1")',
           '\t\t(comment 1 "Generated by tools/generate_kipida_test_4l.py - see VALIDATION.md")', "\t)",
           "\t(lib_symbols"]
    out.append(_lib_symbol("Terminal", "J", "Test terminal: pin 1 force (current), pin 2 Kelvin sense",
                           ((-5.08, 1.27), (5.08, -3.81)), [(n, nm, x, y, a, l) for n, nm, x, y, a, l in
                                                           [(p[0], p[1], p[2], p[3], p[4], p[5]) for p in TERMINAL_PINS]]))
    out.append(_lib_symbol("Regulator", "U", "Regulator stand-in: pin 1 IN, pin 2 OUT",
                           ((-7.62, 2.54), (7.62, -2.54)), REGULATOR_PINS))
    out.append("\t)")

    groups = []
    for p in b.parts:
        g = re.match(r"[A-Z]\d+", p["group"]).group(0)
        if g not in groups:
            groups.append(g)
    titles = {s["id"]: s["title"] for s in b.spec}
    grid = 1.27  # KiCad connection grid: pins and labels must sit on it
    row_y = {g: grid * (28 + 13 * i) for i, g in enumerate(groups)}
    col = {g: 0 for g in groups}
    for g in groups:
        out.append(_text(f"{g}: {titles.get(g, '')}", grid * 16, row_y[g] - grid * 5))
    for p in b.parts:
        g = re.match(r"[A-Z]\d+", p["group"]).group(0)
        x, y = grid * (48 + 56 * col[g]), row_y[g]
        col[g] += 1
        pins = TERMINAL_PINS if p["kind"] == "terminal" else REGULATOR_PINS
        lib = "Terminal" if p["kind"] == "terminal" else "Regulator"
        s = (f'\t(symbol\n\t\t(lib_id "{FP_LIB}:{lib}")\n\t\t(at {x:g} {y:g} 0)\n\t\t(unit 1)\n\t\t(body_style 1)\n'
             "\t\t(exclude_from_sim no)\n\t\t(in_bom yes)\n\t\t(on_board yes)\n\t\t(in_pos_files yes)\n\t\t(dnp no)\n"
             f'\t\t(uuid "{p["uuid"]}")\n')
        s += _prop("Reference", p["ref"], x, y - 5.08)
        s += _prop("Value", p["value"], x, y + 6.35)
        s += _prop("Footprint", f"{FP_LIB}:{p['fp']}", x, y, hide=True)
        s += _prop("Datasheet", "", x, y, hide=True)
        s += _prop("Description", p["description"], x, y, hide=True)
        s += _prop("Imax", p["imax"], x + 6.35, y + 6.35)
        for pin in pins:
            pin_uuid = uid("pin/" + p["ref"] + "/" + pin[0])
            s += f'\t\t(pin "{pin[0]}"\n\t\t\t(uuid "{pin_uuid}")\n\t\t)\n'
        s += (f'\t\t(instances\n\t\t\t(project "{PROJECT}"\n\t\t\t\t(path "/{root_uuid}"\n'
              f'\t\t\t\t\t(reference "{p["ref"]}")\n\t\t\t\t\t(unit 1)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n\t)\n')
        out.append(s)
        for num, _nm, px, py, angle, _l in pins:
            # library y points up, schematic y points down
            out.append(_global_label(p["nets"][num], x + px, y - py, 180 if angle == 0 else 0))
    out += ["\t(sheet_instances", '\t\t(path "/"', '\t\t\t(page "1")', "\t\t)", "\t)", "\t(embedded_fonts no)", ")"]
    SCH_FILE.write_text("\n".join(out) + "\n", encoding="utf-8")


def write_cfg(b):
    CFG_FILE.write_text(json.dumps({"version": "1.0", "rails": list(b.rails.values())}, indent=2) + "\n",
                        encoding="utf-8")


def fmt_mv(volts):
    return f"{volts * 1e3:.2f} mV" if abs(volts) < 1 else f"{volts:.3f} V"


def write_spec(b):
    L = []
    w = L.append
    w("# Ki-PIDA validation board: `kipida_test_4l`\n")
    w("Test structures with **hand-calculable** DC results, to validate the Ki-PIDA power-integrity "
      "simulation (and, later, thermal simulation with the heater).\n")
    w("> Generated by `tools/generate_kipida_test_4l.py` together with the board, the schematic and the "
      "Ki-PIDA config. Change the structures or the stackup there and re-run; do not edit this file by hand.\n")
    w("## Assumptions\n")
    w(f"* Copper resistivity rho = {RHO:g} Ohm*mm (20 C), the value Ki-PIDA uses.")
    w(f"* Sheet resistance: outer layers ({T_OUT * 1e3:g} um) **{RS_OUT * 1e3:.3f} mOhm/sq**; "
      f"inner layers ({T_IN * 1e3:g} um) **{RS_IN * 1e3:.3f} mOhm/sq**.")
    w(f"* Vias: {VIA_DRILL:g} mm drill, {VIA_PAD:g} mm pad, {PLATING * 1e3:g} um plating -> barrel copper "
      f"{BARREL_AREA:.4f} mm^2. Barrel length from copper mid-plane to mid-plane: F->B {H_F_TO_B:.4f} mm "
      f"(**{R_VIA_FB * 1e3:.3f} mOhm/via**).")
    w("* Resistance = sheet resistance x squares (length / width). Expected drops are between the two "
      "**Kelvin sense pads** (pad 2 of each terminal), not across the whole net: force pads and leads are "
      "excluded, so the formula is exact where the current is uniform.")
    w("* Temperature effects are ignored (copper +0.39 %/K). Keep real measurements short or at low current.\n")
    w("## Stackup (written into the board)\n")
    w("| Layer | Type | Thickness (mm) | Material | Er |")
    w("|---|---|---|---|---|")
    for name, typ, t, mat, er in STACKUP:
        if t is not None:
            w(f"| {name} | {typ} | {t:g} | {mat or ''} | {er or ''} |")
    w(f"\nTotal ~{BOARD_THICKNESS:.3f} mm.\n")
    w("## Structures and expected results\n")
    w("Each structure is its own net. Ki-PIDA config: `pcb/kipida_test_4l/kipida_test_4l.kipida.json` "
      "(sources on the force pad of the `SRC` terminal, loads on the force pad of the `LOAD` terminals).\n")
    w(f"Ki-PIDA columns: simulated drop between the same sense pads at mesh 0.1 mm and 0.025 mm "
      f"({KIPIDA_DATE}); deviation of the 0.025 mm result from the expected drop.\n")
    w("| ID | Net | Layer | Geometry (between sense pads) | I | Expected R | **Expected drop** | "
      "Ki-PIDA 0.1 mm | Ki-PIDA 0.025 mm | Deviation |")
    w("|---|---|---|---|---|---|---|---|---|---|")

    def deviation(sim, ref):
        return f"{(sim / ref - 1) * 100:+.1f} %" if sim is not None and ref else "-"

    for s in b.spec:
        approx = " (approx.)" if s.get("approx") else ""
        if s.get("drop") is None:
            r, drop = "-", f"no closed form (1-D est. {fmt_mv(s['estimate'])})"
        else:
            r = f"{s['r'] * 1e3:.2f} mOhm" if s.get("r") is not None and s["r"] < 1 else (f"{s['r']:.3f} Ohm" if s.get("r") else "-")
            r += approx
            drop = f"**{fmt_mv(s['drop'])}**{approx}"
        k1, k2 = KIPIDA_RESULTS.get(s["id"], (None, None))
        sim = lambda v: fmt_mv(v) if v is not None else "-"
        w(f"| {s['id']} | `{s['net']}` | {s['layer']} | {s['geometry']} | {s['current']:g} A | {r} | "
          f"{drop} | {sim(k1)} | {sim(k2)} | {deviation(k2, s.get('drop'))} |")
    w("")
    for s in b.spec:
        w(f"### {s['id']}: {s['title']}\n")
        w(f"* **Validates:** {s['validates']}")
        w(f"* **Formula:** {s['formula']}")
        if s.get("drop") is None and s["id"] in KIPIDA_RESULTS:
            k1, k2 = KIPIDA_RESULTS[s["id"]]
            w(f"* **Reference:** converged Ki-PIDA value **{fmt_mv(k2)}** (0.1 mm: {fmt_mv(k1)}, "
              f"{(k1 / k2 - 1) * 100:+.1f} %)")
        if "loads_v" in s:
            v = s["loads_v"]
            w(f"* **Expected voltage at the load sense pads** (relative to the source sense pad): "
              f"L1 {fmt_mv(v[0])}, L2 {fmt_mv(v[1])}, L3 {fmt_mv(v[2])}")
            res = [KIPIDA_RESULTS.get(f"G1 L{i}", (None, None))[1] for i in (1, 2)] + [KIPIDA_RESULTS["G1"][1]]
            w("* **Ki-PIDA (0.025 mm):** " + ", ".join(
                f"L{i + 1} -{fmt_mv(x)} ({deviation(x, -v[i])})" for i, x in enumerate(res)))
        if "reg" in s:
            g = s["reg"]
            w(f"* **5 V side:** {s['current']:g} A -> **{fmt_mv(s['drop'])}** between its sense pads.")
            w(f"* **12 V side (regulator input):** switching, 90 % (as configured): I_in = 5 V x 2 A / (0.9 x 12 V) = "
              f"{g['i_sw']:.3f} A -> **{fmt_mv(g['d12_sw'])}**; if set to LINEAR: I_in = 2 A -> {fmt_mv(g['d12_lin'])}.")
            k12 = KIPIDA_RESULTS["I1 12V"][1]
            w(f"* **Ki-PIDA (0.025 mm):** 5 V {fmt_mv(KIPIDA_RESULTS['I1'][1])} "
              f"({deviation(KIPIDA_RESULTS['I1'][1], s['drop'])}), 12 V {fmt_mv(k12)} ({deviation(k12, g['d12_sw'])}).")
        if "power" in s:
            w(f"* **Heater:** R = {s['r']:.3f} Ohm, at {s['current']:g} A -> {fmt_mv(s['drop'])} and "
              f"**{s['power']:.2f} W** (future thermal validation).")
        if s.get("issue"):
            w(f"* **Notes:** {s['issue']}")
        w("")
    w("## How to run the validation\n")
    w("1. Open `pcb/kipida_test_4l/kipida_test_4l.kicad_pcb` in the KiCad PCB Editor.")
    w("2. Start Ki-PIDA: the saved config loads the rails, sources and loads above.")
    w("3. Mesh resolution: 0.1 mm is enough for every structure (all within 0.2 % of the 0.025 mm result "
      "except the vias, E1-E3: up to 3.6 %).")
    w("4. Run, then hover over each **sense pad** on the 2D map (F.Cu; In1.Cu for C1) and compare the voltage "
      "difference with **Expected drop**.")
    w("")
    w("## Real-board measurements (optional)\n")
    w("* Drive current through the force pads (pad 1) with a bench supply or electronic load; measure voltage "
      "between the sense pads (pad 2) with a multimeter (4-wire / Kelvin).")
    w("* Expect +-10-15 % from copper-thickness tolerance and plating on the outer layers; heating raises resistance "
      "0.39 %/K, so keep high-current measurements short.")
    w("* Parts carry an **`Imax`** field (current through the terminal at the test condition) for future "
      "Ki-PIDA load import.")
    SPEC_FILE.write_text("\n".join(L) + "\n", encoding="utf-8")


def main():
    b = Builder()
    build(b)
    if "--spec-only" in sys.argv:          # rewrite VALIDATION.md only; board, schematic and config untouched
        write_spec(b)
        print(f"{len(b.spec)} structures; wrote {SPEC_FILE.name} only")
        return
    root_uuid = re.search(r'\(uuid "?([0-9a-f-]{36})"?\)', SCH_FILE.read_text(encoding="utf-8")).group(1) \
        if SCH_FILE.exists() else uid("root")
    write_pcb(b)
    write_sch(b, root_uuid)
    write_cfg(b)
    write_spec(b)
    print(f"{len(b.parts)} parts, {len(b.nets)} nets, {len(b.spec)} structures")
    print(f"wrote {PCB_FILE.name}, {SCH_FILE.name}, {CFG_FILE.name}, {SPEC_FILE.name}")


if __name__ == "__main__":
    main()
