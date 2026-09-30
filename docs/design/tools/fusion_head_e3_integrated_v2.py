import adsk.core, adsk.fusion, math
V3 = adsk.core.Vector3D.create; P3 = adsk.core.Point3D.create
tbm = adsk.fusion.TemporaryBRepManager.get(); U = adsk.fusion.BooleanTypes
MOUTH, FUNNEL, CH_L, CH_W = 60.0, 30.0, 30.0, 30.0
WALL, FOOT = 1.6, 3.6
XM = -236.0
def area(P): return 0.5 * sum(P[i][0] * P[(i + 1) % len(P)][1] - P[(i + 1) % len(P)][0] * P[i][1] for i in range(len(P)))
def line_off(p, q, d, cw):
    dx, dy = q[0] - p[0], q[1] - p[1]; L = math.hypot(dx, dy); dx /= L; dy /= L
    nx, ny = (-dy, dx) if cw else (dy, -dx)
    return ((p[0] + nx * d, p[1] + ny * d), (dx, dy))
def isect(l1, l2):
    (p, d), (q, e) = l1, l2
    det = d[0] * (-e[1]) - d[1] * (-e[0]); t = ((q[0] - p[0]) * (-e[1]) - (q[1] - p[1]) * (-e[0])) / det
    return (p[0] + d[0] * t, p[1] + d[1] * t)
def inner_poly():
    s = (MOUTH / 2 - CH_W / 2) / FUNNEL; x0 = -3.0
    return [(x0, MOUTH / 2 + s * 3.0), (FUNNEL, CH_W / 2), (FUNNEL + CH_L, CH_W / 2), (FUNNEL + CH_L, -CH_W / 2), (FUNNEL, -CH_W / 2), (x0, -(MOUTH / 2 + s * 3.0))]
def outer_poly(d, front_x=0.0):
    P = inner_poly(); cw = area(P) < 0
    edges = [(P[i], P[i + 1]) for i in range(0, 5)]; L = [line_off(p, q, d, cw) for p, q in edges]; pts = []
    p, dd = L[0]; t = (front_x - p[0]) / dd[0]; pts.append((front_x, p[1] + dd[1] * t))
    for i in range(4): pts.append(isect(L[i], L[i + 1]))
    p, dd = L[4]; t = (front_x - p[0]) / dd[0]; pts.append((front_x, p[1] + dd[1] * t))
    return pts
class Ctx:
    def __init__(self, root):
        self.occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()); self.comp = self.occ.component; self.comp.name = 'scratch'
    def prism(self, poly, z0, z1, dx=0.0):
        sk = self.comp.sketches.add(self.comp.xYConstructionPlane)
        pts = [P3((x + XM + dx) / 10, y / 10, 0) for x, y in poly]
        for i in range(len(pts)): sk.sketchCurves.sketchLines.addByTwoPoints(pts[i], pts[(i + 1) % len(pts)])
        ei = self.comp.features.extrudeFeatures.createInput(sk.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ei.setDistanceExtent(False, adsk.core.ValueInput.createByReal((z1 - z0) / 10))
        b = self.comp.features.extrudeFeatures.add(ei).bodies.item(0); t = tbm.copy(b)
        if abs(z0) > 1e-9:
            m = adsk.core.Matrix3D.create(); m.translation = V3(0, 0, z0 / 10); tbm.transform(t, m)
        return t
    def done(self): self.occ.deleteMe()
def box(x0, x1, y0, y1, z0, z1):
    return tbm.createBox(adsk.core.OrientedBoundingBox3D.create(P3((x0+x1)/20, (y0+y1)/20, (z0+z1)/20), V3(1,0,0), V3(0,1,0), (x1-x0)/10, (y1-y0)/10, (z1-z0)/10))
def union(a, b): tbm.booleanOperation(a, b, U.UnionBooleanType); return a
def diff(a, b): tbm.booleanOperation(a, b, U.DifferenceBooleanType); return a

def cyl_z(x, y, z0, z1, r): return tbm.createCylinderOrCone(P3(x/10, y/10, z0/10), r/10, P3(x/10, y/10, z1/10), r/10)
def cyl_x(y, z, x0, x1, r): return tbm.createCylinderOrCone(P3(x0/10, y/10, z/10), r/10, P3(x1/10, y/10, z/10), r/10)
def inter(a, b): tbm.booleanOperation(a, b, U.IntersectionBooleanType); return a
EXCL = ('JAW dug', 'SIDE SKID', 'ToF DOWN window', 'CHEEK LED LENS', 'LINK', 'HOOD PIN', 'JAW PIVOT', 'SG90', 'CRANK', 'HOOD EAR')
def paw_skid(sg):
    y0, y1 = (36.0, 51.6) if sg > 0 else (-51.6, -36.0); yc = sg * 43.8
    paw = box(-222, -210, y0, y1, 0, 10); union(paw, cyl_z(-222, yc, 0, 10, 7.8)); union(paw, cyl_z(-210, yc, 0, 10, 7.8))
    n0, n1 = (36.0, 44.0) if sg > 0 else (-44.0, -36.0)
    sk = box(-206, -182, n0, n1, 0, 10); union(sk, cyl_z(-182, sg * 40.0, 0, 10, 4.0)); union(paw, sk)
    pocket = box(-224.0, -207.0, min(sg*37.6, sg*50.0), max(sg*37.6, sg*50.0), 2.9, 6.3)
    window = box(-221.0, -213.0, min(sg*40.8, sg*46.8), max(sg*40.8, sg*46.8), -0.5, 3.0)
    return diff(diff(paw, pocket), window)
def run(_context: str):
    d = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = d.rootComponent
    for o in list(root.occurrences):
        if o.name.startswith('HEAD E3 INTEGRATED v2'): o.deleteMe()
    def occ_by(prefix): return [o for o in root.occurrences if o.name.startswith(prefix)][0]
    ctx = Ctx(root)
    env = ctx.prism(outer_poly(WALL), 0, 20.6); union(env, ctx.prism(outer_poly(WALL), 0, 20.6, 0.4))
    union(env, ctx.prism(outer_poly(FOOT), 0, 8.6)); union(env, ctx.prism(outer_poly(FOOT), 0, 8.6, 0.4))
    ctx.done()
    cutters = [env, box(-194.8, -171.3, 21.2, 34.2, 9.8, 33.1), box(-206.0, -193.0, 16.4, 30.0, 12.5, 24.0),
               box(-207.0, -163.0, 16.8, 20.8, 4.0, 16.0), box(-207.0, -163.0, -20.8, -16.8, 4.0, 16.0)]
    v1 = occ_by('HEAD E3 INTEGRATED v1').component
    jaw_src = [b for b in occ_by('HEAD E2-V VISOR').component.bRepBodies if b.name.startswith('E2-V HEAD E2 lower jaw')][0]
    hl = occ_by('HOOD LIFT STUDY 02').component
    A = d.appearances.itemByName
    o = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()); comp = o.component
    comp.name = 'HEAD E3 INTEGRATED v2 - cute deformed - NOT FOR PRINT - v1 + ToF boards in side-skid paws (E-0008), hood lift s=4 TPU leaf ear, jaw re-dug [C]; v1 and originals kept'
    bf = comp.features.baseFeatures.add(); bf.startEdit(); added = []
    def add(tb, name, app): comp.bRepBodies.add(tb, bf); added.append((name, app))
    for b in v1.bRepBodies:
        if not b.name.startswith(EXCL): add(tbm.copy(b), b.name, b.appearance)
    t = tbm.copy(jaw_src)
    for cu in cutters: tbm.booleanOperation(t, tbm.copy(cu), U.DifferenceBooleanType)
    add(t, 'JAW dug v2 (hood s=4 envelope, SG90, leaf/crank zone, link slots; no rear ToF window)', jaw_src.appearance)
    for sg, tg in ((1, 'L (+y)'), (-1, 'R (-y)')):
        add(paw_skid(sg), 'SIDE SKID+PAW %s: paw round r7.8 |y|36-51.6 x-229.8..-202.2 + skid to x-178, pocket+window for ToF board [A]' % tg, A('SD01v2 sage'))
        add(box(-223.5, -207.5, min(sg*37.8, sg*49.8), max(sg*37.8, sg*49.8), 3.0, 6.2), 'ToF BOARD %s 12x18x3.2 in paw (window down) [ASSUMED length]' % tg, A('SD01v2 ref blue'))
        add(cyl_x(40.0 * sg, 4.5, -230.4, -227.5, 2.0), 'CHEEK LED LENS %s d4 at paw front y%+d z4.5 pink [B]' % (tg, 40 * sg), A('SD01v2 blush pink'))
    for b in hl.bRepBodies:
        add(tbm.copy(b), b.name, A('SD01v2 blush pink') if b.name.startswith('TPU LEAF') else A('SD01v2 ref blue'))
    bf.finishEdit()
    for b, (n, a) in zip(comp.bRepBodies, added):
        b.name = n
        if a: b.appearance = a
    print('bodies', comp.bRepBodies.count)
    for b in comp.bRepBodies:
        if b.name.startswith('JAW dug'): print(b.name[:20], 'V=%.0f' % (b.volume * 1000))
