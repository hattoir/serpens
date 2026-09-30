import adsk.core, adsk.fusion, math
V3 = adsk.core.Vector3D.create; P3 = adsk.core.Point3D.create
tbm = adsk.fusion.TemporaryBRepManager.get(); U = adsk.fusion.BooleanTypes
def box(x0, x1, y0, y1, z0, z1):
    return tbm.createBox(adsk.core.OrientedBoundingBox3D.create(P3((x0+x1)/20, (y0+y1)/20, (z0+z1)/20), V3(1,0,0), V3(0,1,0), (x1-x0)/10, (y1-y0)/10, (z1-z0)/10))
def cyl_y(x, z, y0, y1, r): return tbm.createCylinderOrCone(P3(x/10, y0/10, z/10), r/10, P3(x/10, y1/10, z/10), r/10)
def cyl_x(y, z, x0, x1, r): return tbm.createCylinderOrCone(P3(x0/10, y/10, z/10), r/10, P3(x1/10, y/10, z/10), r/10)
def cyl_z(x, y, z0, z1, r): return tbm.createCylinderOrCone(P3(x/10, y/10, z0/10), r/10, P3(x/10, y/10, z1/10), r/10)
def union(a, b): tbm.booleanOperation(a, b, U.UnionBooleanType); return a
def diff(a, b): tbm.booleanOperation(a, b, U.DifferenceBooleanType); return a
def inter(a, b): tbm.booleanOperation(a, b, U.IntersectionBooleanType); return a
def sphere(x, y, z, r): return tbm.createSphere(P3(x/10, y/10, z/10), r/10)
def cute_skid(sg):
    y0, y1 = (36.0, 44.0) if sg > 0 else (-44.0, -36.0); yc = sg * 40.0
    s = box(-220, -178, y0, y1, 0, 10); union(s, box(-226, -220, y0, y1, 6, 10))
    union(s, inter(cyl_y(-220, 6, y0, y1, 6.0), box(-226, -220, y0, y1, 0, 6)))       # 前端は R6 で反らす
    st = box(-222, -182, y0, y1, -1, 11); union(st, cyl_z(-222, yc, -1, 11, 4.0)); union(st, cyl_z(-182, yc, -1, 11, 4.0))   # 上から見て丸い（スタジアム形）
    s = inter(s, st)
    return s
def run(_context: str):
    d = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    if d.appearances.itemByName('SD01v2 blush pink') is None:
        a = d.appearances.addByCopy(d.appearances.itemByName('SD01v2 ivory'), 'SD01v2 blush pink')
        for p in a.appearanceProperties:
            if p.name == 'Color': p.value = adsk.core.Color.create(242, 160, 170, 255)
    pink = d.appearances.itemByName('SD01v2 blush pink'); ivory = d.appearances.itemByName('SD01v2 ivory'); tpu = d.appearances.itemByName('SD01v2 skid TPU')
    comp = [o for o in d.rootComponent.occurrences if o.name.startswith('HEAD E3 INTEGRATED v1')][0].component
    specs = []
    for sg, tag in ((1, 'L (+y)'), (-1, 'R (-y)')):
        sk = cute_skid(sg)
        specs.append(('SIDE SKID %s cute pill x-226..-178 |y|36..44 z0..10, toe curl R6, round ends r4 [A]' % tag, sk, tpu))
        # 頬の丸いレンズ（かわいい頬の点）: 横スキッドの前端 X-226、z 4.5、直径 4
        specs.append(('CHEEK LED LENS %s d4 at skid nose X-226 y%+d z4.5 (Z<=6) pink [B]' % (tag, 40*sg), cyl_x(40.0*sg, 4.5, -227.6, -224.0, 2.0), pink))
        # 後ろのスキッド: 丸いかかと（半球）
        h = cyl_z(-168, 18.0*sg, 0, 3, 4.0); union(h, sphere(-168, 18.0*sg, 3, 4.0))
        specs.append(('REAR SKID heel %s dome r4 at x-168 y%+d z0..7 [C]' % (tag, 18*sg), h, tpu))
    specs.append(('ToF DOWN window x-172..-165 |y|<=7 z7..9 outside cavity [C]', box(-172, -165, -7, 7, 7, 9), d.appearances.itemByName('SD01v2 ref blue')))
    n0 = comp.bRepBodies.count
    bf = comp.features.baseFeatures.add(); bf.startEdit()
    for n, tb, a in specs: comp.bRepBodies.add(tb, bf)
    bf.finishEdit()
    for k, (n, tb, a) in enumerate(specs):
        b = comp.bRepBodies.item(n0 + k); b.name = n; b.appearance = a
    print(comp.bRepBodies.count)
