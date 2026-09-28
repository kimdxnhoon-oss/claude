# V3 helical gripper generator for Autodesk Fusion.
#
# Builds the four printed parts from the V3 BOM draft as separate components
# and places them as an assembly (1 sun adapter, 3 planet stacks):
#   01 helical_arm                 x3
#   02 planet_drive_gear_45t       x3
#   03 arm_drive_spacer_3p2mm      x3
#   04 xl330_sun_gear_adapter_15t  x1
#
# Run: Utilities > ADD-INS > Scripts and Add-Ins > (+) > pick this folder > Run.
# All dimensions below are in mm / degrees. Fusion's API works in cm, so every
# value goes through cm() before it reaches the API.

import math
import os
import traceback

import adsk.core
import adsk.fusion

# ---------------------------------------------------------------- parameters
MODULE = 1.0               # sun 15T + planet 45T -> centre distance 30 mm
PRESSURE_ANGLE = 20.0
BACKLASH = 0.10            # total circumferential backlash per gear
SUN_TEETH = 15
PLANET_TEETH = 45
PLANET_ANGLES = (90.0, 210.0, 330.0)

PLANET_THICK = 3.0
SHAFT_HOLE = 3.3           # M3 axle, raise to 3.4-3.5 if the test print is tight
BOLT_HOLE = 3.3            # M3 arm bolts
BOLT_SPACING = 12.0

PLATE_LEN = 18.0           # spacer and arm flange footprint
PLATE_WID = 12.0
CORNER_R = 2.0
SPACER_THICK = 3.2
NUT_CAVITY = 7.2

ARM_FLANGE_THICK = 3.0
ARM_WIDTH = 6.0            # ribbon width (along flange length)
ARM_THICK = 3.0            # ribbon thickness (faces the gripper axis)
ARM_ROOT_Y = 4.5           # ribbon root centre on the flange, towards the axis
ARM_RISE = 3.0             # straight section before the helix starts
ARM_LENGTH_Z = 35.0        # axial reach of the finger
ARM_SWEEP_DEG = 90.0       # how far the finger wraps around the gripper axis
ARM_END_RADIUS = 10.0      # tip distance from the gripper axis
ARM_PATH_POINTS = 10

SUN_FACE = 4.0             # wider than the planet so it always covers the mesh
SUN_NECK_LEN = 5.0         # flange sits 5 mm above the gear face
SUN_NECK_DIA = 8.0
SUN_FLANGE_DIA = 18.0
SUN_FLANGE_THICK = 2.5
SUN_CENTER_HOLE = 3.0
XL330_PCD = 12.0
XL330_HOLE = 2.2           # M2 tapping screw clearance

PLA_DENSITY = 1.24         # g/cm^3
RULE_LIMIT_G = 120.0

# Set to a folder (e.g. os.path.expanduser('~/Desktop/v3_gripper')) to also
# write per-part STL + STEP files in print orientation.
EXPORT_DIR = ''

# ---------------------------------------------------------------- helpers
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation


def cm(v):
    return v / 10.0


def P(x, y, z=0.0):
    return adsk.core.Point3D.create(cm(x), cm(y), cm(z))


def val(v):
    return adsk.core.ValueInput.createByReal(v)


def collection(items):
    coll = adsk.core.ObjectCollection.create()
    for item in items:
        coll.add(item)
    return coll


def frame(origin, xaxis, yaxis, zaxis):
    m = adsk.core.Matrix3D.create()
    m.setWithCoordinateSystem(P(*origin),
                              adsk.core.Vector3D.create(*xaxis),
                              adsk.core.Vector3D.create(*yaxis),
                              adsk.core.Vector3D.create(*zaxis))
    return m


def extrude(comp, profiles, z0, height, op, body=None):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profiles, op)
    inp.setOneSideExtent(
        adsk.fusion.DistanceExtentDefinition.create(val(cm(height))),
        adsk.fusion.ExtentDirections.PositiveExtentDirection)
    if abs(z0) > 1e-9:
        inp.startExtent = adsk.fusion.OffsetStartDefinition.create(val(cm(z0)))
    if body is not None:
        inp.participantBodies = [body]
    return ext.add(inp)


def disk(comp, dia, z0, height, op, body=None):
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0), cm(dia / 2))
    return extrude(comp, sk.profiles.item(0), z0, height, op, body)


def cut_holes(comp, body, holes, z0=None, height=None):
    """holes: [(x, y, dia)]. Through-all unless z0/height are given."""
    sk = comp.sketches.add(comp.xYConstructionPlane)
    for x, y, d in holes:
        sk.sketchCurves.sketchCircles.addByCenterRadius(P(x, y), cm(d / 2))
    profiles = collection(sk.profiles)
    if z0 is not None:
        return extrude(comp, profiles, z0, height, CUT, body)
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profiles, CUT)
    inp.setAllExtent(adsk.fusion.ExtentDirections.SymmetricExtentDirection)
    inp.participantBodies = [body]
    return ext.add(inp)


def fillet(comp, edges, r):
    fl = comp.features.filletFeatures
    inp = fl.createInput()
    try:
        inp.edgeSetInputs.addConstantRadiusEdgeSet(edges, val(cm(r)), True)
    except AttributeError:  # older Fusion API
        inp.addConstantRadiusEdgeSet(edges, val(cm(r)), True)
    fl.add(inp)


def rounded_plate(comp, length, width, thick, r):
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.sketchCurves.sketchLines.addCenterPointRectangle(
        P(0, 0), P(length / 2, width / 2))
    body = extrude(comp, sk.profiles.item(0), 0.0, thick, NEW).bodies.item(0)
    if r > 0:
        vertical = []
        for e in body.edges:
            a, b = e.startVertex.geometry, e.endVertex.geometry
            if abs(a.x - b.x) < 1e-6 and abs(a.y - b.y) < 1e-6:
                vertical.append(e)
        fillet(comp, collection(vertical), r)
    return body


def bolt_pattern(dia):
    return [(-BOLT_SPACING / 2, 0, dia), (BOLT_SPACING / 2, 0, dia)]


# ---------------------------------------------------------------- gears
def involute_flank(n_teeth, samples=10):
    """Returns (addendum radius, root radius, [(radius, half_angle)])."""
    pa = math.radians(PRESSURE_ANGLE)
    r = MODULE * n_teeth / 2
    rb = r * math.cos(pa)
    ra = r + MODULE
    rf = r - 1.25 * MODULE

    def inv(a):
        return math.tan(a) - a

    half_pitch = math.pi / (2 * n_teeth) - BACKLASH / (2 * r)

    def half(rho):
        return half_pitch + inv(pa) - inv(math.acos(min(1.0, rb / rho)))

    r0 = max(rb, rf)
    radii = [r0 + (ra - r0) * i / (samples - 1) for i in range(samples)]
    flank = [(rho, half(rho)) for rho in radii]
    if flank[-1][1] <= 0:
        raise ValueError('%dT tooth comes to a point, lower BACKLASH' % n_teeth)
    return ra, rf, flank


def build_gear(comp, n_teeth, face, phase_deg):
    """Spur gear on the component XY plane, z = 0..face.

    phase_deg rotates the tooth pattern so tooth 0 is centred on that angle.
    """
    ra, rf, flank = involute_flank(n_teeth)
    phase = math.radians(phase_deg)

    def polar(rho, ang):
        a = ang + phase
        return P(rho * math.cos(a), rho * math.sin(a))

    base = disk(comp, 2 * rf, 0.0, face, NEW).bodies.item(0)
    base.name = '__gear_base__'

    sk = comp.sketches.add(comp.xYConstructionPlane)
    curves = sk.sketchCurves
    right = curves.sketchFittedSplines.add(
        collection([polar(rho, h) for rho, h in flank]))
    left = curves.sketchFittedSplines.add(
        collection([polar(rho, -h) for rho, h in flank]))
    curves.sketchArcs.addByThreePoints(
        right.endSketchPoint, polar(ra, 0.0), left.endSketchPoint)
    r_in = rf - 0.3 * MODULE  # sink the tooth root into the base disk
    h0 = flank[0][1]
    root_r = curves.sketchLines.addByTwoPoints(right.startSketchPoint, polar(r_in, h0))
    root_l = curves.sketchLines.addByTwoPoints(left.startSketchPoint, polar(r_in, -h0))
    curves.sketchLines.addByTwoPoints(root_r.endSketchPoint, root_l.endSketchPoint)
    tooth = extrude(comp, sk.profiles.item(0), 0.0, face, NEW).bodies.item(0)

    pats = comp.features.circularPatternFeatures
    pin = pats.createInput(collection([tooth]), comp.zConstructionAxis)
    pin.quantity = val(n_teeth)
    pin.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    pin.isSymmetric = False
    pats.add(pin)

    tools = [b for b in comp.bRepBodies if b.name != '__gear_base__']
    cf = comp.features.combineFeatures
    cin = cf.createInput(base, collection(tools))
    cin.operation = JOIN
    cf.add(cin)
    return base


def tooth_centred_phase(n_teeth, angle_deg):
    """Phase that puts a tooth centre on angle_deg."""
    return angle_deg % (360.0 / n_teeth)


def gap_centred_phase(n_teeth, angle_deg):
    """Phase that puts a tooth gap centre on angle_deg."""
    return (angle_deg - 180.0 / n_teeth) % (360.0 / n_teeth)


# ---------------------------------------------------------------- parts
def build_planet_gear(comp):
    # In the planet frame the sun is always at local -Y (270 deg).
    body = build_gear(comp, PLANET_TEETH, PLANET_THICK,
                      gap_centred_phase(PLANET_TEETH, 270.0))
    cut_holes(comp, body, [(0, 0, SHAFT_HOLE)] + bolt_pattern(BOLT_HOLE))
    return body


def build_spacer(comp):
    body = rounded_plate(comp, PLATE_LEN, PLATE_WID, SPACER_THICK, CORNER_R)
    cut_holes(comp, body, [(0, 0, NUT_CAVITY)] + bolt_pattern(BOLT_HOLE))
    return body


def arm_path_points():
    """Finger centre line in the arm frame (flange on z = 0, +Y towards the
    gripper axis, which sits at (0, centre distance)). The finger rises out of
    the flange, then spirals in towards the axis while wrapping around it."""
    axis_y = MODULE * (SUN_TEETH + PLANET_TEETH) / 2
    d0 = axis_y - ARM_ROOT_Y
    pts = [(0.0, ARM_ROOT_Y, 0.0), (0.0, ARM_ROOT_Y, ARM_RISE)]
    for i in range(1, ARM_PATH_POINTS + 1):
        t = i / ARM_PATH_POINTS
        phi = math.radians(-90.0 + ARM_SWEEP_DEG * t)
        d = d0 + (ARM_END_RADIUS - d0) * t
        z = ARM_RISE + ARM_LENGTH_Z * t
        pts.append((d * math.cos(phi), axis_y + d * math.sin(phi), z))
    return pts


def build_arm(comp):
    flange = rounded_plate(comp, PLATE_LEN, PLATE_WID, ARM_FLANGE_THICK, CORNER_R)

    path_sk = comp.sketches.add(comp.xYConstructionPlane)
    path_sk.name = 'finger path'
    spline = path_sk.sketchCurves.sketchFittedSplines.add(
        collection([P(*p) for p in arm_path_points()]))

    prof_sk = comp.sketches.add(comp.xYConstructionPlane)
    prof_sk.name = 'finger section'
    prof_sk.sketchCurves.sketchLines.addCenterPointRectangle(
        P(0, ARM_ROOT_Y), P(ARM_WIDTH / 2, ARM_ROOT_Y + ARM_THICK / 2))

    sweeps = comp.features.sweepFeatures
    sin_ = sweeps.createInput(prof_sk.profiles.item(0),
                              comp.features.createPath(spline, False), JOIN)
    sin_.orientation = adsk.fusion.SweepOrientationTypes.PerpendicularOrientationType
    sin_.participantBodies = [flange]
    sweeps.add(sin_)

    # The finger never comes back over y = 0, so through-all only hits the flange.
    cut_holes(comp, flange, bolt_pattern(BOLT_HOLE))
    return flange


def build_sun_adapter(comp):
    body = build_gear(comp, SUN_TEETH, SUN_FACE,
                      tooth_centred_phase(SUN_TEETH, PLANET_ANGLES[0]))
    disk(comp, SUN_NECK_DIA, SUN_FACE, SUN_NECK_LEN, JOIN, body)
    flange_z = SUN_FACE + SUN_NECK_LEN
    disk(comp, SUN_FLANGE_DIA, flange_z, SUN_FLANGE_THICK, JOIN, body)
    r = XL330_PCD / 2
    holes = [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), XL330_HOLE)
             for a in (45, 135, 225, 315)]
    # Start in the air gap beside the neck so only the flange is drilled.
    cut_holes(comp, body, holes, SUN_FACE + 0.5, SUN_NECK_LEN + SUN_FLANGE_THICK)
    cut_holes(comp, body, [(0, 0, SUN_CENTER_HOLE)])
    return body


# ---------------------------------------------------------------- assembly
def target_design(app):
    design = adsk.fusion.Design.cast(app.activeProduct)
    if design is None:
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
    return design


def new_part(parent, name, transform):
    occ = parent.occurrences.addNewComponent(transform)
    occ.component.name = name
    return occ.component


def planet_frames(angle_deg):
    """(gear/spacer frame, arm frame) for the planet at angle_deg.
    Local X runs along the bolt pair, local -Y (gear) / +Y (arm) faces the
    gripper axis. Fingers hang below the gears, the motor sits above."""
    a = math.radians(angle_deg)
    c, s = math.cos(a), math.sin(a)
    cd = MODULE * (SUN_TEETH + PLANET_TEETH) / 2
    u = (s, -c, 0.0)
    gear = frame((cd * c, cd * s, 0.0), u, (c, s, 0.0), (0.0, 0.0, 1.0))
    spacer = frame((cd * c, cd * s, -SPACER_THICK), u, (c, s, 0.0), (0.0, 0.0, 1.0))
    arm = frame((cd * c, cd * s, -SPACER_THICK), u, (-c, -s, 0.0), (0.0, 0.0, -1.0))
    return gear, spacer, arm


def export_parts(design, parts):
    os.makedirs(EXPORT_DIR, exist_ok=True)
    em = design.exportManager
    for name, comp, _ in parts:
        base = os.path.join(EXPORT_DIR, name)
        stl = em.createSTLExportOptions(comp.bRepBodies.item(0), base + '.stl')
        stl.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        em.execute(stl)
        em.execute(em.createSTEPExportOptions(base + '.step', comp))


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        design = target_design(app)
        top = new_part(design.rootComponent, 'V3_Helical_Gripper',
                       adsk.core.Matrix3D.create())

        # Sun gear face is 1 mm wider than the planets, centred on them.
        sun_tf = frame((0, 0, -(SUN_FACE - PLANET_THICK) / 2),
                       (1, 0, 0), (0, 1, 0), (0, 0, 1))
        gear_tf, spacer_tf, arm_tf = planet_frames(PLANET_ANGLES[0])

        specs = [
            ('01_helical_arm', 3, arm_tf, build_arm),
            ('02_planet_drive_gear_45t', 3, gear_tf, build_planet_gear),
            ('03_arm_drive_spacer_3p2mm', 3, spacer_tf, build_spacer),
            ('04_xl330_sun_gear_adapter_15t', 1, sun_tf, build_sun_adapter),
        ]
        parts, failed = [], []
        for name, qty, tf, builder in specs:
            comp = new_part(top, name, tf)
            try:
                body = builder(comp)
                body.name = name
                parts.append((name, comp, qty))
            except Exception:
                failed.append('%s\n%s' % (name, traceback.format_exc()))

        # Instances for the other two planets.
        by_name = {name: comp for name, comp, _ in parts}
        for angle in PLANET_ANGLES[1:]:
            gear_tf, spacer_tf, arm_tf = planet_frames(angle)
            for name, tf in (('01_helical_arm', arm_tf),
                             ('02_planet_drive_gear_45t', gear_tf),
                             ('03_arm_drive_spacer_3p2mm', spacer_tf)):
                if name in by_name:
                    top.occurrences.addExistingComponent(by_name[name], tf)

        lines = ['part | qty | cm3 each | g each | g total']
        total = 0.0
        for name, comp, qty in parts:
            vol = sum(b.physicalProperties.volume for b in comp.bRepBodies)
            g = vol * PLA_DENSITY
            total += g * qty
            lines.append('%s | %d | %.3f | %.2f | %.2f' % (name, qty, vol, g, g * qty))
        lines.append('')
        lines.append('100%% solid PLA: %.2f g (margin to %.0f g: %.2f g)'
                     % (total, RULE_LIMIT_G, RULE_LIMIT_G - total))
        lines.append('+40%% support/brim/purge: %.2f g' % (total * 1.4))

        if EXPORT_DIR and not failed:
            export_parts(design, parts)
            lines.append('Exported STL/STEP to ' + EXPORT_DIR)
        if failed:
            lines.append('')
            lines.append('FAILED:')
            lines.extend(failed)

        app.activeViewport.fit()
        ui.messageBox('\n'.join(lines), 'V3 helical gripper')
    except Exception:
        ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
