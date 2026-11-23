"""INS10-A3 calibration jig for FreeCAD 1.x.

A 26-face block (a rhombicuboctahedron: a cube with every edge and every corner
cut flat) that holds the INS10-A3 board and the Raspberry Pi Pico it is wired to,
so that board, Pico and wires move as one rigid piece. Every face the block stands
on is a different, stable orientation for the accelerometer and gyroscope
calibration; the magnetometer is calibrated by turning the block in the hand.

Two printed parts, joined by four M3 screws, no supports, no other metal:
  base  the lower half. The Pico lies on a rail in a cavity open at the top; the
        USB plug lies in a channel that leaves through the +X face.
  lid   the upper half. The board is taped by its bare back into a shallow recess
        in the lid's underside, so that its sensors face down at the Pico. That is
        how the wires are soldered, and it keeps the wires short: the board sits
        right above the end of the Pico where most of its wires land.

Build with ./build.ps1, which also checks the result, or from this folder (the
name must be relative: freecadcmd garbles non-ASCII characters in arguments):
    "C:\\Program Files\\FreeCAD 1.1\\bin\\freecadcmd.exe" calibration_jig.py
or open it in FreeCAD (Macro > Macros... > Execute). The outputs are written next
to this file. Change the parameters below and run it again.

Frame: origin at the centre of the block; base and lid meet at z = 0.
  +Z  the lid.   +X  the face the USB cable leaves through.
  The Pico runs along X with its USB end at +X, components up.
"""

import math
import os

import FreeCAD as App
import Part

V = App.Vector

# --- parameters, mm ----------------------------------------------------------

SIZE = 85.0             # across two opposite square faces
WALL = 2.5              # least wall around the cavity

# INS10-A3 module (hardware/INS10-A.kicad_pcb): 22.5 x 13.2 x 1.6, no mounting holes,
# every part on one side. J1 is a row of 8 holes 1.25 mm from a long edge.
BOARD_L, BOARD_W, BOARD_T = 22.5, 13.2, 1.6
BOARD_PARTS_H = 1.3     # tallest part (0805 capacitor, crystal)
BOARD_CLEAR = 0.25      # recess clearance per side
BOARD_RECESS = 1.0      # depth of the recess in the lid's underside
J1_EDGE = 1.25
J1_PITCH = 2.54
J1_ROW_Y = 0.0          # J1 above the Pico's centre line; the board extends to +Y

# Raspberry Pi Pico: 51 x 21 x 1.0, micro-USB on the top side at one end.
PICO_L, PICO_W, PICO_T = 51.0, 21.0, 1.0
PICO_PARTS_H = 3.0      # micro-USB receptacle above the PCB
USB_OVERHANG = 1.3      # receptacle past the PCB edge
USB_Z = 1.4             # plug centre above the PCB top
PLUG_LEN = 25.0         # rigid part of the cable's plug; a longer one sticks out of +X
PLUG_W, PLUG_H = 12.0, 8.0
FRONT_GAP = 1.0         # plug end to the +X face
PICO_Z = -10.0          # underside of the Pico PCB
BELOW = 9.0             # cavity floor to Pico underside: room for pin headers (8.5 mm)
CAVITY_W = 26.0         # 2.5 mm beside the Pico for wires that leave it from below
RAIL_W = 8.0            # between the two pin rows
LIP_LEN = 1.2           # stop at the USB end of the rail
PLUG_SLOT_W = 13.0

# Four M3 x 16 screws, heads sunk in the lid, threads cut into the base.
SCREW_XY = [(10.0, 35.0), (-10.0, 35.0), (10.0, -35.0), (-10.0, -35.0)]
SCREW_LEN = 16.0
SCREW_IN_BASE = 10.0
SCREW_CLEAR_D = 3.4
SCREW_PILOT_D = 2.7     # M3 cuts its own thread in PLA or PETG
SCREW_HEAD_D = 6.5      # counterbore; an M3 cap head is 5.5
PILOT_DEPTH = 12.0

# Face labels, engraved. The number on a face is the position "lying on this face".
# Any first N numbers already give orientations spread in all directions:
# 1-5 the square faces, then edges and corners; 21-25 are next to the cable face,
# where the cable lies on the table, 25 lowest of all.
FACE_NUMBERS = {
    (0, 0, -1): 1, (0, 0, 1): 2, (0, -1, 0): 3, (0, 1, 0): 4, (-1, 0, 0): 5,
    (1, 0, 1): 6, (0, 1, -1): 7, (0, -1, 1): 8, (-1, 1, 0): 9, (-1, -1, 0): 10,
    (1, 1, 1): 11, (-1, -1, -1): 12, (0, 1, 1): 13, (0, -1, -1): 14, (-1, 0, 1): 15,
    (-1, 0, -1): 16, (1, -1, 1): 17, (-1, 1, -1): 18, (-1, 1, 1): 19, (-1, -1, 1): 20,
    (1, 1, 0): 21, (1, -1, 0): 22, (1, 1, -1): 23, (1, -1, -1): 24, (1, 0, -1): 25,
}
CABLE_FACE = (1, 0, 0)          # labelled USB: never lay the block on it
FONT = "C:/Windows/Fonts/arialbd.ttf"
TEXT_H_FACE = 12.0              # glyph height on square and edge faces
TEXT_H_CORNER = 7.5             # on the triangles
TEXT_DEPTH = 0.6
TEXT_MARGIN = 1.5               # least distance from a label to the face's edge
SIDE_TEXT_UP = 9.0              # on faces split by the seam, the label sits on the lid half

# Wires between the board and the Pico are soldered and stay as they are.
WIRE_MIN = 50.0         # the shortest one
# Raspberry Pi Pico pin of each J1 contact (wiring table in ../docs/en/user-guide.md).
J1_TO_PICO = {1: 36, 2: 23, 3: 14, 4: 22, 5: 15, 6: 24, 7: 25, 8: 21}

# --- derived -----------------------------------------------------------------

H = SIZE / 2.0
EDGE = SIZE / (1.0 + math.sqrt(2.0))                    # edge length of every face
CORNER_RATIO = (3.0 + math.sqrt(2.0)) / (math.sqrt(3.0) * (1.0 + math.sqrt(2.0)))
PCB_TOP = PICO_Z + PICO_T
FLOOR_Z = PICO_Z - BELOW
PICO_X0 = H - FRONT_GAP - PLUG_LEN - USB_OVERHANG - PICO_L   # rear end of the Pico
PICO_X1 = PICO_X0 + PICO_L                                    # USB end
USB_C = PCB_TOP + USB_Z
PIN_Y = PICO_W / 2.0 - 1.61
SEAT_Z = SCREW_LEN - SCREW_IN_BASE                            # under the screw head
BOARD_FACE_Z = BOARD_RECESS - BOARD_T                         # the board's parts side


def pico_pin(pin):
    """Pin centre on the Pico's top. Pins 1..20 run from the USB end along +Y,
    21..40 from the rear end along -Y (the Pico seen from above, USB at +X)."""
    if pin <= 20:
        return V(PICO_X1 - 1.37 - (pin - 1) * 2.54, PIN_Y, PCB_TOP)
    return V(PICO_X0 + 1.37 + (pin - 21) * 2.54, -PIN_Y, PCB_TOP)


# J1 runs along X with J1.1 towards the USB end. Its position along X makes the
# longest horizontal run of any wire as short as possible.
_need = [pico_pin(p).x + (k - 1) * J1_PITCH for k, p in J1_TO_PICO.items()]
J1_X1 = (max(_need) + min(_need)) / 2.0
BOARD_X = J1_X1 - 3.5 * J1_PITCH                              # board centre
BOARD_Y0 = J1_ROW_Y - J1_EDGE                                 # J1 edge of the board
BOARD_Y1 = BOARD_Y0 + BOARD_W

SQUARES = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
EDGES = ([(sx, sy, 0) for sx in (1, -1) for sy in (1, -1)] +
         [(sx, 0, sz) for sx in (1, -1) for sz in (1, -1)] +
         [(0, sy, sz) for sy in (1, -1) for sz in (1, -1)])
CORNERS = [(sx, sy, sz) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]


def unit(d):
    v = V(*d)
    return v.multiply(1.0 / v.Length)


def box(x0, x1, y0, y1, z0, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cylinder(x, y, d, z0, z1):
    return Part.makeCylinder(d / 2.0, z1 - z0, V(x, y, z0), V(0, 0, 1))


def half_space(direction, dist, size=1000.0):
    """A solid filling n . p >= dist."""
    n = unit(direction)
    b = Part.makeBox(size, size, size, V(-size / 2.0, -size / 2.0, 0.0))
    b.Placement = App.Placement(V(n.x * dist, n.y * dist, n.z * dist), App.Rotation(V(0, 0, 1), n))
    return b


def plane_distance(direction):
    return H * (CORNER_RATIO if all(direction) else 1.0)


_CACHE = {}


def cached(key, make):
    if key not in _CACHE:
        _CACHE[key] = make()
    return _CACHE[key]


def block(inset=0.0):
    def make():
        s = box(-(H - inset), H - inset, -(H - inset), H - inset, -(H - inset), H - inset)
        for d in EDGES + CORNERS:
            s = s.cut(half_space(d, plane_distance(d) - inset))
        return s.removeSplitter()
    return cached(("block", inset), make)


def one_solid(shape, name):
    shape = shape.removeSplitter()
    if len(shape.Solids) != 1:
        raise ValueError("%s came out as %d solids" % (name, len(shape.Solids)))
    return shape.Solids[0]


def cavity():
    """Everything taken out of the base, as one solid (for the checks too)."""
    room = box(PICO_X0 - 2.0, PICO_X1 + 2.0, -CAVITY_W / 2.0, CAVITY_W / 2.0, FLOOR_Z, 1.0)
    room = room.common(block(WALL))
    plug = box(PICO_X1, H + 1.0, -PLUG_SLOT_W / 2.0, PLUG_SLOT_W / 2.0, USB_C - PLUG_H / 2.0 - 0.5, 1.0)
    return room.fuse(plug).removeSplitter()


def holes():
    """Every hole that opens on an outer face, for the label checks."""
    s = [cavity()]
    for x, y in SCREW_XY:
        s.append(cylinder(x, y, SCREW_HEAD_D, SEAT_Z, H + 1.0))
    return Part.makeCompound(s)


def face_frame(d):
    """Outward normal, reading directions and centre of a face. Labels read
    upright with +Z up; on the top and bottom faces +X is up."""
    n = unit(d)
    if abs(n.z) > 0.999:
        up = V(1, 0, 0)
    else:
        up = V(0, 0, 1) - V(n.x, n.y, n.z).multiply(n.z)
        up.multiply(1.0 / up.Length)
    right = up.cross(n)
    centre = V(n.x, n.y, n.z).multiply(plane_distance(d))
    return n, right, up, centre


def text_shape(text, height):
    """Flat glyphs in the XY plane, centred on the origin. 6 and 9 are
    underlined, as on dice."""
    faces = [Part.makeFace(c, "Part::FaceMakerBullseye") for c in Part.makeWireString(text, FONT, height, 0.0)]
    bb = Part.makeCompound(faces).BoundBox
    if text in ("6", "9"):
        bar = 0.12 * height
        faces.append(Part.makePlane(bb.XLength, bar, V(bb.XMin, bb.YMin - 1.4 * bar, 0)))
    shape = Part.makeCompound(faces)
    bb = shape.BoundBox
    shape.translate(V(-bb.Center.x, -bb.Center.y, 0))
    return shape


def labels():
    """(face, text, local 2D bounding box, cutter) for every face."""
    return cached("labels", make_labels)


def make_labels():
    out = []
    for d in SQUARES + EDGES + CORNERS:
        text = "USB" if d == CABLE_FACE else str(FACE_NUMBERS[d])
        n, right, up, centre = face_frame(d)
        flat = text_shape(text, TEXT_H_CORNER if all(d) else TEXT_H_FACE)
        lift = SIDE_TEXT_UP if abs(n.z) < 1e-9 else 0.0
        bb = flat.BoundBox
        box2d = (bb.XMin, bb.XMax, bb.YMin + lift, bb.YMax + lift)
        o = centre + V(up.x, up.y, up.z).multiply(lift) + V(n.x, n.y, n.z).multiply(0.1)
        m = App.Matrix(right.x, up.x, n.x, o.x,
                       right.y, up.y, n.y, o.y,
                       right.z, up.z, n.z, o.z,
                       0, 0, 0, 1)
        flat = flat.copy()
        flat.transformShape(m)
        cutter = flat.extrude(V(n.x, n.y, n.z).multiply(-(TEXT_DEPTH + 0.1)))
        out.append((d, text, box2d, cutter))
    return out


def label_fit(d, box2d):
    """Least distance from the label's bounding box to the edge of its face (mm)."""
    n, right, up, centre = face_frame(d)
    dist = plane_distance(d)
    face = next(f for f in block().Faces
                if abs(f.CenterOfMass.dot(n) - dist) < 1e-6 and abs(abs(f.normalAt(0, 0).dot(n)) - 1) < 1e-9)
    pts = [((v.Point - centre).dot(right), (v.Point - centre).dot(up)) for v in face.Vertexes]
    pts.sort(key=lambda p: math.atan2(p[1], p[0]))                         # convex, counter-clockwise
    x0, x1, y0, y1 = box2d
    worst = float("inf")
    for i in range(len(pts)):
        (ax, ay), (bx, by) = pts[i], pts[(i + 1) % len(pts)]
        ex, ey = bx - ax, by - ay
        length = math.hypot(ex, ey)
        for px, py in ((x0, y0), (x0, y1), (x1, y0), (x1, y1)):
            worst = min(worst, (ex * (py - ay) - ey * (px - ax)) / length)   # > 0 inside
    return worst


def build_parts():
    if PICO_X0 - 2.0 < -(H - WALL - 1.0):
        raise ValueError("the Pico does not fit: lower PLUG_LEN or raise SIZE")
    if sorted(FACE_NUMBERS.values()) != list(range(1, 26)) or CABLE_FACE in FACE_NUMBERS:
        raise ValueError("FACE_NUMBERS must give 1..25 once each to the 25 faces other than the cable face")
    whole = block()
    base = whole.common(box(-H - 1, H + 1, -H - 1, H + 1, -H - 1, 0.0)).cut(cavity())
    rail = box(PICO_X0 - 2.0, PICO_X1, -RAIL_W / 2.0, RAIL_W / 2.0, FLOOR_Z - 0.01, PICO_Z)
    lip = box(PICO_X1, PICO_X1 + LIP_LEN, -RAIL_W / 2.0, RAIL_W / 2.0, FLOOR_Z - 0.01, PCB_TOP)
    base = base.fuse([rail, lip])
    for x, y in SCREW_XY:
        base = base.cut(cylinder(x, y, SCREW_PILOT_D, -PILOT_DEPTH, 1.0))
    base = one_solid(base, "the base")

    lid = whole.common(box(-H - 1, H + 1, -H - 1, H + 1, 0.0, H + 1))
    c = BOARD_CLEAR
    lid = lid.cut(box(BOARD_X - BOARD_L / 2.0 - c, BOARD_X + BOARD_L / 2.0 + c,
                      BOARD_Y0 - c, BOARD_Y1 + c, -1.0, BOARD_RECESS))
    for x, y in SCREW_XY:
        lid = lid.cut(cylinder(x, y, SCREW_CLEAR_D, -1.0, SEAT_Z + 0.01))
        lid = lid.cut(cylinder(x, y, SCREW_HEAD_D, SEAT_Z, H + 1.0))
    engraving = Part.makeCompound([c for _, _, _, c in labels()])
    base = one_solid(base.cut(engraving), "the base")
    lid = one_solid(lid.cut(engraving), "the lid")
    return base, lid


def reference_parts():
    """Rough solids of what goes into the jig, for the fit checks and the FCStd."""
    parts = {
        "Board": box(BOARD_X - BOARD_L / 2.0, BOARD_X + BOARD_L / 2.0, BOARD_Y0, BOARD_Y1,
                     BOARD_FACE_Z, BOARD_RECESS),
        "BoardParts": box(BOARD_X - 9.5, BOARD_X + 9.5, BOARD_Y0 + 2.0, BOARD_Y1 - 2.0,
                          BOARD_FACE_Z - BOARD_PARTS_H, BOARD_FACE_Z),
        "Pico": box(PICO_X0, PICO_X1, -PICO_W / 2.0, PICO_W / 2.0, PICO_Z, PCB_TOP),
        "PicoParts": box(PICO_X0 + 0.5, PICO_X1 + USB_OVERHANG, -8.0, 8.0, PCB_TOP, PCB_TOP + PICO_PARTS_H),
        "Plug": box(PICO_X1 + USB_OVERHANG, PICO_X1 + USB_OVERHANG + PLUG_LEN, -PLUG_W / 2.0, PLUG_W / 2.0,
                    USB_C - PLUG_H / 2.0, USB_C + PLUG_H / 2.0),
        # Pin headers, if the Pico has them: plastic strip and pins under the PCB.
        "Headers": box(PICO_X0 + 0.1, PICO_X1 - 0.1, PIN_Y - 1.27, PIN_Y + 1.27, PICO_Z - 2.5, PICO_Z).fuse([
            box(PICO_X0 + 0.1, PICO_X1 - 0.1, -PIN_Y - 1.27, -PIN_Y + 1.27, PICO_Z - 2.5, PICO_Z),
            box(PICO_X0 + 0.5, PICO_X1 - 0.5, PIN_Y - 0.32, PIN_Y + 0.32, PICO_Z - 8.5, PICO_Z),
            box(PICO_X0 + 0.5, PICO_X1 - 0.5, -PIN_Y - 0.32, -PIN_Y + 0.32, PICO_Z - 8.5, PICO_Z)]),
    }
    screws = [cylinder(x, y, 3.0, SEAT_Z - SCREW_LEN, SEAT_Z).fuse(cylinder(x, y, 5.5, SEAT_Z, SEAT_Z + 3.0))
              for x, y in SCREW_XY]
    parts["Screws"] = Part.makeCompound(screws)
    return parts


def outward_normal(solid, face):
    """Outward normal of a planar face, taken at a point that is surely on it."""
    pts, tris = face.tessellate(0.1)
    a, b, c = (pts[i] for i in tris[0])
    p = (a + b + c).multiply(1.0 / 3.0)
    n = face.normalAt(*face.Surface.parameter(p))
    probe = p + V(n.x, n.y, n.z).multiply(0.01)
    return n.multiply(-1.0) if solid.isInside(probe, 1e-7, False) else n


def overhang_area(solid, bed_z):
    """Area of faces looking down at less than 45 degrees to the bed (they need a
    bridge or support), not counting the face on the bed."""
    area = 0.0
    for f in solid.Faces:
        if not isinstance(f.Surface, Part.Plane):
            continue
        n = outward_normal(solid, f)
        if n.z < -math.cos(math.radians(45.0)) - 1e-6 and f.BoundBox.ZMin > bed_z + 1e-3:
            area += f.Area
    return area


def check(base, lid, parts):
    ok = True

    def line(name, good, note):
        nonlocal ok
        ok &= good
        print("  %-30s %s (%s)" % (name, "ok" if good else "PROBLEM", note))

    print("checks:")
    for name, solid in parts.items():
        overlap = base.common(solid).Volume + lid.common(solid).Volume
        if name == "Screws":   # the screw cuts its own thread in the pilot hole
            overlap = lid.common(solid).Volume + cavity().common(solid).Volume
        line(name, overlap < 1e-3, "overlap %.4f mm3" % overlap)
    line("base and lid apart", base.common(lid).Volume < 1e-3, "overlap %.4f mm3" % base.common(lid).Volume)
    heads = [cylinder(x, y, 5.5, SEAT_Z, SEAT_Z + 3.0) for x, y in SCREW_XY]
    sunk = all(abs(h.common(block()).Volume - h.Volume) < 1e-3 for h in heads)
    line("screw heads sunk in the lid", sunk, "M3 x %g, %g mm into the base" % (SCREW_LEN, SCREW_IN_BASE))
    plug_end = PICO_X1 + USB_OVERHANG + PLUG_LEN
    line("plug inside the block", plug_end <= H, "%.2f mm to the +X face" % (H - plug_end))
    line("parts of the board above the Pico", BOARD_FACE_Z - BOARD_PARTS_H > PCB_TOP + PICO_PARTS_H,
         "%.1f mm between them" % ((BOARD_FACE_Z - BOARD_PARTS_H) - (PCB_TOP + PICO_PARTS_H)))
    for name, solid in (("base", base), ("lid", lid)):
        line(name + " is one valid solid", solid.isValid() and len(solid.Solids) == 1, "")
    for name, solid, bed in (("base", base, -H), ("lid", lid, 0.0)):
        a = overhang_area(solid, bed)
        print("  %-30s %.0f mm2 (the board-recess bridge and the %.1f mm deep ceilings of the engraving)"
              % (name + " overhangs", a, TEXT_DEPTH))
    hole_set = holes()
    fits = [(text, label_fit(d, b), c.common(hole_set).Volume) for d, text, b, c in labels()]
    tight = min(fits, key=lambda f: f[1])
    line("labels inside their faces", tight[1] >= TEXT_MARGIN, "closest: '%s', %.1f mm from an edge" % tight[:2])
    hit = [t for t, _, v in fits if v > 1e-3]
    line("labels clear of holes", not hit, ", ".join(hit) or "none touches one")
    return ok


def face_report(whole):
    """Area of each of the 26 outer faces, to see that they are whole enough to stand on."""
    full_sq = EDGE * EDGE
    full_tri = math.sqrt(3.0) / 4.0 * EDGE * EDGE
    print("outer faces of the assembled block (normal, area mm2, share of a whole face):")
    for d in SQUARES + EDGES + CORNERS:
        n, dist, area = unit(d), plane_distance(d), 0.0
        for f in whole.Faces:
            if not isinstance(f.Surface, Part.Plane) or abs(f.CenterOfMass.dot(n) - dist) > 1e-3:
                continue
            u0, u1, v0, v1 = f.ParameterRange
            if abs(abs(f.normalAt((u0 + u1) / 2.0, (v0 + v1) / 2.0).dot(n)) - 1.0) < 1e-6:
                area += f.Area
        full = full_tri if all(d) else full_sq
        print("  %-12s %7.0f  %3.0f %%" % ("(%d,%d,%d)" % d, area, 100.0 * area / full))


def wire_lengths():
    """Straight distance from each J1 hole to its Pico pin, plus 5 mm at each
    soldered end for the bend."""
    print("J1 wire to Pico: straight distance + 10 mm for the bends (shortest wire %.0f mm):" % WIRE_MIN)
    worst = 0.0
    for k, pin in sorted(J1_TO_PICO.items()):
        j = V(J1_X1 - (k - 1) * J1_PITCH, J1_ROW_Y, BOARD_FACE_Z)
        length = (pico_pin(pin) - j).Length + 10.0
        worst = max(worst, length)
        print("  J1.%d -> Pico pin %2d   %3.0f mm" % (k, pin, length))
    return worst


def render_view(shapes, elev, azim, size=520):
    """Orthographic picture with a depth buffer (matplotlib's 3D axes sort whole
    triangles and draw hidden holes over the faces in front of them)."""
    import numpy as np
    e, a = math.radians(elev), math.radians(azim)
    d = np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])   # towards the eye
    r = np.cross([0.0, 0.0, 1.0], d)
    r /= np.linalg.norm(r)
    u = np.cross(d, r)
    light = 0.55 * d + 0.45 * u - 0.35 * r
    light /= np.linalg.norm(light)
    scale = size / (2.0 * H * 1.25)
    img = np.ones((size, size, 3))
    depth = np.full((size, size), -np.inf)
    for shape, rgb in shapes:
        pts, facets = shape.tessellate(0.05)
        if not facets:
            continue
        p = np.array([[q.x, q.y, q.z] for q in pts])[np.array(facets)]          # (n, 3, 3)
        n = np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0])
        n /= np.maximum(np.linalg.norm(n, axis=1)[:, None], 1e-12)
        n[(n @ d) < 0] *= -1.0
        tone = 0.32 + 0.68 * np.clip(n @ light, 0.0, 1.0)
        sx = p @ r * scale + size / 2.0
        sy = size / 2.0 - p @ u * scale
        sz = p @ d
        for k in range(len(p)):
            x, y, z = sx[k], sy[k], sz[k]
            x0, x1 = max(int(math.floor(x.min())), 0), min(int(math.ceil(x.max())), size - 1)
            y0, y1 = max(int(math.floor(y.min())), 0), min(int(math.ceil(y.max())), size - 1)
            if x0 > x1 or y0 > y1:
                continue
            gx, gy = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
            den = (y[1] - y[2]) * (x[0] - x[2]) + (x[2] - x[1]) * (y[0] - y[2])
            if abs(den) < 1e-12:
                continue
            w0 = ((y[1] - y[2]) * (gx - x[2]) + (x[2] - x[1]) * (gy - y[2])) / den
            w1 = ((y[2] - y[0]) * (gx - x[2]) + (x[0] - x[2]) * (gy - y[2])) / den
            w2 = 1.0 - w0 - w1
            inside = (w0 >= -1e-6) & (w1 >= -1e-6) & (w2 >= -1e-6)
            zz = w0 * z[0] + w1 * z[1] + w2 * z[2]
            sub = depth[y0:y1 + 1, x0:x1 + 1]
            win = inside & (zz > sub)
            sub[win] = zz[win]
            img[y0:y1 + 1, x0:x1 + 1][win] = np.array(rgb) * tone[k]
    # Dark outlines where depth jumps, so that faces and holes read clearly.
    dz = np.zeros_like(depth)
    finite = np.where(np.isfinite(depth), depth, -1e3)
    dz[1:, :] = np.maximum(dz[1:, :], np.abs(np.diff(finite, axis=0)))
    dz[:, 1:] = np.maximum(dz[:, 1:], np.abs(np.diff(finite, axis=1)))
    img[dz > 1.5] *= 0.35
    return img


def render_preview(base, lid, parts, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    blue, grey, orange, green = (0.55, 0.72, 0.95), (0.78, 0.8, 0.84), (0.95, 0.55, 0.25), (0.35, 0.75, 0.4)
    ink = (0.12, 0.16, 0.3)
    pico = Part.makeCompound([parts["Pico"], parts["PicoParts"], parts["Plug"]])
    board = Part.makeCompound([parts["Board"], parts["BoardParts"]])
    # The engraved floors have the same tone as the face around them; draw the
    # removed material in ink so that the labels show.
    text = Part.makeCompound([c for _, _, _, c in labels()]).common(block())
    whole = [(base.fuse(lid), blue), (text, ink)]
    views = [
        ("from +X +Y, above", whole, 28, 40),
        ("from +X -Y, above", whole, 28, -40),
        ("from -X, above", whole, 28, 180),
        ("from -X -Y, below", whole, -30, -130),
        ("from +X +Y, below", whole, -30, 50),
        ("base open: Pico and plug (orange), board (green)", [(base, grey), (pico, orange), (board, green)], 58, -60),
    ]
    fig, axes = plt.subplots(2, 3, figsize=(15, 10.4), dpi=110)
    for ax, (title, shapes, elev, azim) in zip(axes.flat, views):
        ax.imshow(render_view(shapes, elev, azim), interpolation="lanczos")
        ax.set_title(title, fontsize=10)
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def main():
    out = os.path.dirname(os.path.abspath(__file__))
    base, lid = build_parts()
    parts = reference_parts()
    ok = check(base, lid, parts)
    whole = base.fuse(lid).removeSplitter()
    face_report(whole)
    worst = wire_lengths()
    ok &= worst <= WIRE_MIN
    print("size %.1f mm, face edge %.2f mm, base %.0f cm3, lid %.0f cm3"
          % (SIZE, EDGE, base.Volume / 1000.0, lid.Volume / 1000.0))
    print("longest J1 wire needed: about %.0f mm of %.0f" % (worst, WIRE_MIN))
    if not ok:
        raise SystemExit("CHECKS FAILED: nothing written")

    import MeshPart
    meshes = {}
    for name, solid in (("base", base), ("lid", lid)):
        # Finer than this, the curves of the digits give slivers that cross each other.
        for lin, ang in ((0.02, 10.0), (0.05, 15.0)):
            mesh = MeshPart.meshFromShape(Shape=solid, LinearDeflection=lin, AngularDeflection=math.radians(ang),
                                          Relative=False)
            if mesh.isSolid() and not mesh.hasNonManifolds() and not mesh.hasSelfIntersections():
                break
        else:
            raise SystemExit("CHECKS FAILED: the %s STL is not a clean closed mesh" % name)
        print("  %-30s ok (%d triangles, closed, no self-intersections)" % (name + " STL", mesh.CountFacets))
        meshes[name] = mesh
    for name, mesh in meshes.items():
        mesh.write(os.path.join(out, "calibration-jig-%s.stl" % name))
    Part.makeCompound([base, lid]).exportStep(os.path.join(out, "calibration-jig.step"))

    doc = App.newDocument("CalibrationJig")
    objs = []
    for name, solid in (("Base", base), ("Lid", lid)):
        o = doc.addObject("Part::Feature", name)
        o.Shape = solid
        objs.append(o)
    group = doc.addObject("App::DocumentObjectGroup", "Reference_not_printed")
    refs = []
    for name, solid in parts.items():
        r = doc.addObject("Part::Feature", name)
        r.Shape = solid
        group.addObject(r)
        refs.append(r)
    doc.recompute()
    if App.GuiUp:
        import FreeCADGui as Gui
        for o in objs:
            o.ViewObject.ShapeColor = (0.55, 0.72, 0.95)
            o.ViewObject.Transparency = 50
        for r in refs:
            r.ViewObject.ShapeColor = (0.95, 0.55, 0.25)
            r.ViewObject.Visibility = r.Name in ("Board", "Pico", "Plug", "Screws")
        Gui.SendMsgToActiveView("ViewFit")
        Gui.activeDocument().activeView().viewIsometric()
    fcstd = os.path.join(out, "calibration-jig.FCStd")
    if os.path.exists(fcstd):
        os.remove(fcstd)    # otherwise FreeCAD leaves a .FCBak copy of the old one
    doc.saveAs(fcstd)
    if not App.GuiUp:
        App.closeDocument(doc.Name)

    if os.environ.get("JIG_PREVIEW", "1") != "0":
        render_preview(base, lid, parts, os.path.join(out, "preview.png"))
    print("written: calibration-jig-base.stl, calibration-jig-lid.stl, calibration-jig.step, .FCStd, preview.png")


main()
