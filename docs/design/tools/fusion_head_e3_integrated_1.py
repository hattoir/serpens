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

def run(_context: str):
    d = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = d.rootComponent
    occs = {o.name[:14]: o for o in root.occurrences}
    def occ_by(prefix): return [o for o in root.occurrences if o.name.startswith(prefix)][0]
    ctx = Ctx(root)
    # フード + 5 mm 昇降の包絡（壁 + 屋根 z 0..21.6、足の帯 z 0..9.6、後ろへ 0.7 mm）
    env = ctx.prism(outer_poly(WALL), 0, 21.6)
    union(env, ctx.prism(outer_poly(WALL), 0, 21.6, 0.7))
    union(env, ctx.prism(outer_poly(FOOT), 0, 9.6)); union(env, ctx.prism(outer_poly(FOOT), 0, 9.6, 0.7))
    ctx.done()
    cutters = [env,
        box(-214.2, -190.7, 21.2, 34.2, 9.8, 33.1),          # SG90 級の穴（+y 側）0.4 mm の余裕
        box(-219.0, -213.5, 19.0, 30.0, 12.0, 26.0),         # クランクと耳の動く範囲
        box(-207.0, -163.0, 16.8, 20.8, 4.0, 16.0), box(-207.0, -163.0, -20.8, -16.8, 4.0, 16.0),   # リンクの溝
        box(-172.5, -164.5, -7.5, 7.5, 0.0, 9.5)]           # ToF 下向きの窓
    c = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()); comp = c.component
    comp.name = 'HEAD E3 INTEGRATED v1 - cute deformed - NOT FOR PRINT - E2-V head + mouth line + hood B2 + drape gate; jaw dug for hood(5 mm lift), SG90, links, ToF window [C]; originals kept'
    bf = comp.features.baseFeatures.add(); bf.startEdit()
    added = []
    def add(tb, name, app):
        b = comp.bRepBodies.add(tb, bf); added.append((name, app))
    for o in (occ_by('HEAD E2-V VISOR'),):
        for b in o.component.bRepBodies:
            t = tbm.copy(b)
            if b.name.startswith('E2-V HEAD E2 lower jaw'):
                for cu in cutters: tbm.booleanOperation(t, tbm.copy(cu), U.DifferenceBooleanType)
                add(t, 'JAW dug: ' + b.name, b.appearance)
            else:
                add(t, b.name, b.appearance)
    for b in occ_by('MOUTH LINE option').component.bRepBodies: add(tbm.copy(b), b.name, b.appearance)
    for b in occ_by('LIGHT+SKID option').component.bRepBodies:
        if b.name.startswith('NORMAL light slit'): add(tbm.copy(b), b.name, b.appearance)
    for i, b in enumerate(occ_by('HEAD INTAKE on E3').component.bRepBodies):
        if i in (0, 3, 4, 5): add(tbm.copy(b), 'HOOD B2 form/gate part %d' % (i + 1), b.appearance)
    for b in occ_by('HOOD LIFT STUDY 01').component.bRepBodies: add(tbm.copy(b), b.name, b.appearance)
    bf.finishEdit()
    for b, (n, a) in zip(comp.bRepBodies, added):
        b.name = n
        if a: b.appearance = a
    print('bodies', comp.bRepBodies.count, 'timeline', d.timeline.count)
    for b in comp.bRepBodies:
        if b.name.startswith('JAW'): print(b.name, 'V=%.0f' % (b.volume * 1000))
