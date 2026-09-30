import adsk.core, adsk.fusion, math
V3 = adsk.core.Vector3D.create; P3 = adsk.core.Point3D.create
tbm = adsk.fusion.TemporaryBRepManager.get(); U = adsk.fusion.BooleanTypes
AX, AZ = -181.8, 32.35            # J1 軸 [E: CAD.md ASSUMED]
def cyl_y(x, z, y0, y1, r): return tbm.createCylinderOrCone(P3(x/10, y0/10, z/10), r/10, P3(x/10, y1/10, z/10), r/10)
def box(x0, x1, y0, y1, z0, z1):
    return tbm.createBox(adsk.core.OrientedBoundingBox3D.create(P3((x0+x1)/20, (y0+y1)/20, (z0+z1)/20), V3(1,0,0), V3(0,1,0), (x1-x0)/10, (y1-y0)/10, (z1-z0)/10))
def union(a, b): tbm.booleanOperation(a, b, U.UnionBooleanType); return a
def diff(a, b): tbm.booleanOperation(a, b, U.DifferenceBooleanType); return a
def inter(a, b): tbm.booleanOperation(a, b, U.IntersectionBooleanType); return a
def halfspace(phi_deg, side, ylo, yhi):
    # 軸から角 phi の線の「左（side=+1）/ 右（side=-1）」の半空間（軸からの距離 L = 200）
    a = math.radians(phi_deg); u = (math.cos(a), math.sin(a)); n = (-math.sin(a) * side, math.cos(a) * side)
    L = 200.0
    cx = AX + n[0] * L / 2; cz = AZ + n[1] * L / 2
    return tbm.createBox(adsk.core.OrientedBoundingBox3D.create(P3(cx/10, (ylo+yhi)/20, cz/10), V3(u[0], 0, u[1]), V3(n[0], 0, n[1]), L/10*1.0, L/10, (yhi-ylo)/10))
def sector(r0, r1, p0, p1, ylo, yhi):
    s = diff(cyl_y(AX, AZ, ylo, yhi, r1), cyl_y(AX, AZ, ylo - 1, yhi + 1, r0))
    w = inter(halfspace(p0, +1, ylo - 1, yhi + 1), halfspace(p1, -1, ylo - 1, yhi + 1))
    return inter(s, w)
def radial_pin(phi, r0, r1, y, rad):
    a = math.radians(phi)
    return tbm.createCylinderOrCone(P3((AX + r0*math.cos(a))/10, y/10, (AZ + r0*math.sin(a))/10), rad/10, P3((AX + r1*math.cos(a))/10, y/10, (AZ + r1*math.sin(a))/10), rad/10)
def sphere(x, y, z, r): return tbm.createSphere(P3(x/10, y/10, z/10), r/10)
def tube_y(phi, r, y0, y1, rad):
    a = math.radians(phi); return cyl_y(AX + r*math.cos(a), AZ + r*math.sin(a), y0, y1, rad)
def run(_context: str):
    d = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = d.rootComponent
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()); c = occ.component
    c.name = 'J1 STOPPER v2 + COVER VARIANTS STUDY 02 (R-003) - NOT PARTS - Eng theta_E -4..+3 = CAD +4..-3 [E-0011], steel dowel d3 + TPU pad t3 [E-0011], axis (-181.8, z32.35) [E ASSUMED]'
    PHI_C = 16.0; PHI_P = 40.0; RP = 47.0
    a_pin = math.degrees(1.5 / RP); a_pad = math.degrees(3.0 / RP)          # ピン半角 1.83°、TPU パッド厚 3 mm の角
    lo = PHI_P - 4.0 - a_pin - a_pad; hi = PHI_P + 3.0 + a_pin + a_pad        # 窓の端（パッドの裏面）
    specs = []
    for sg in (1, -1):
        y = 18.0 * sg
        specs.append(('STEEL DOWEL d3 x10.5 radial r38-48.5 phi%d y%+d (press-fit r38-45 into tongue core, exposed 3.5) [E-0011 ASSUMED steel]' % (PHI_P, y), radial_pin(PHI_P, 38.0, 48.5, y, 1.5)))
        specs.append(('DOWEL HOLE cutter d3.0 r38-45 phi%d y%+d (for tongue core, not cut) [C]' % (PHI_P, y), radial_pin(PHI_P, 38.0, 45.0, y, 1.5)))
        specs.append(('STOP WINDOW v2 cutter (hood) phi%.1f-%.1f r45.5-48.5 y%+d w3.6, TPU t3 [C]' % (lo, hi, y), sector(45.5, 48.5, lo, hi, y - 1.8, y + 1.8)))
        specs.append(('TPU PAD low-phi end t3 arc x w3.6 x r46-48 phi%.1f-%.1f y%+d [E-0011 3-6 mm]' % (lo, lo + a_pad, y), sector(46.0, 48.0, lo, lo + a_pad, y - 1.8, y + 1.8)))
        specs.append(('TPU PAD high-phi end t3 phi%.1f-%.1f y%+d' % (hi - a_pad, hi, y), sector(46.0, 48.0, hi - a_pad, hi, y - 1.8, y + 1.8)))
    # 覆いの案（首側に固定、襟の前縁 φ16 に重ねる）
    specs.append(('COVER A shell r49.5-51.1 phi14-24 |y|<=30 (8 deg, 870 mm3) [C]', sector(49.5, 51.1, PHI_C - 2, PHI_C + 8, -30, 30)))
    specs.append(('COVER B short shell r49.5-51.1 phi14-21 |y|<=30 (5 deg) [C]', sector(49.5, 51.1, PHI_C - 2, PHI_C + 5, -30, 30)))
    cc = sector(49.5, 51.1, PHI_C - 2, PHI_C + 1, -30, 30)
    union(cc, tube_y(19.0, 51.5, -30, 30, 2.0)); union(cc, sphere(AX + 51.5*math.cos(math.radians(19)), 30, AZ + 51.5*math.sin(math.radians(19)), 2.0)); union(cc, sphere(AX + 51.5*math.cos(math.radians(19)), -30, AZ + 51.5*math.sin(math.radians(19)), 2.0))
    specs.append(('COVER C rolled scarf: shell phi14-17 + tube r2 at phi19 r51.5 y+-30 + round caps [C]', cc))
    bf = c.features.baseFeatures.add(); bf.startEdit()
    for n, tb in specs: c.bRepBodies.add(tb, bf)
    bf.finishEdit()
    for b, (n, _) in zip(c.bRepBodies, specs):
        b.name = n
        x = b.boundingBox
        print(n[:34], "V=%.0f" % (b.volume*1000), "x %.1f..%.1f z %.1f..%.1f y %.1f..%.1f" % (x.minPoint.x*10, x.maxPoint.x*10, x.minPoint.z*10, x.maxPoint.z*10, x.minPoint.y*10, x.maxPoint.y*10))
    print('window phi', lo, hi)
