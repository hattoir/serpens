import adsk.core, adsk.fusion, math
V3 = adsk.core.Vector3D.create; P3 = adsk.core.Point3D.create
tbm = adsk.fusion.TemporaryBRepManager.get()
def box(x0, x1, y0, y1, z0, z1):
    return tbm.createBox(adsk.core.OrientedBoundingBox3D.create(P3((x0+x1)/20, (y0+y1)/20, (z0+z1)/20), V3(1,0,0), V3(0,1,0), (x1-x0)/10, (y1-y0)/10, (z1-z0)/10))
def cyl_y(x, z, y0, y1, r): return tbm.createCylinderOrCone(P3(x/10, y0/10, z/10), r/10, P3(x/10, y1/10, z/10), r/10)
def cyl_x(y, z, x0, x1, r): return tbm.createCylinderOrCone(P3(x0/10, y/10, z/10), r/10, P3(x1/10, y/10, z/10), r/10)
def side(sg):
    # sg=+1 は +y 側。y は sg 倍
    def yy(a, b): return (min(sg*a, sg*b), max(sg*a, sg*b))
    out = []
    # 平行リンク（水平、L=20 [A]）: フードの壁のピン x -202/-186、あごのピボット x -182/-166、z 8
    for tag, xp, yl in (('A', -202, (17.0, 18.6)), ('B', -186, (18.8, 20.4))):
        y0, y1 = yy(*yl)
        out.append(('LINK %s %s pin x%d -> pivot x%d, L=20 t1.6 [A]' % (tag, '+y' if sg > 0 else '-y', xp, xp + 20), box(xp - 2.5, xp + 22.5, y0, y1, 5.5, 10.5)))
    for xp in (-202, -186):
        y0, y1 = yy(16.6, 20.6)
        out.append(('HOOD PIN d2 x%d z8 %s [A]' % (xp, '+y' if sg > 0 else '-y'), cyl_y(xp, 8, y0, y1, 1.0)))
    for xp in (-182, -166):
        y0, y1 = yy(17.0, 20.6)
        out.append(('JAW PIVOT d2 x%d z8 %s [A]' % (xp, '+y' if sg > 0 else '-y'), cyl_y(xp, 8, y0, y1, 1.0)))
    return out
def run(_context: str):
    d = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = d.rootComponent
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()); c = occ.component
    c.name = 'HOOD LIFT STUDY 01 - NOT PARTS - cloche drop stroke 5 [B] one SG90-class drive [B] parallel link L20 [A], SG90 22.7x12.2x22.5 fit at |y|21.6-33.8 [C: hood_lift_check], hood/head unchanged'
    specs = []
    specs += side(1) + side(-1)
    # SG90 級（+y 側のみ駆動）: 軸は x 方向、前面 x -213.8、軸は y 27.7, z 15.2
    specs.append(('SG90-class body 22.7(x)x12.2(y)x22.5(z) at x-213.8..-191.1 y21.6..33.8 z10.2..32.7 [ASSUMED datasheet size]', box(-213.8, -191.1, 21.6, 33.8, 10.2, 32.7)))
    specs.append(('SG90 mounting tab 32.5 long t2.5 [ASSUMED]', box(-218.8, -186.1, 21.6, 33.8, 25.0, 27.5)))
    specs.append(('SG90 horn disc d7 t1.5 at shaft y27.7 z15.2 [ASSUMED]', cyl_x(27.7, 15.2, -215.3, -213.8, 3.5)))
    specs.append(('CRANK PIN d2 r=2.5 (stroke 5) at y27.7 z17.7 (up-only push) [A]', cyl_x(27.7, 17.7, -218.0, -215.3, 1.0)))
    specs.append(('HOOD EAR POST on roof x-218..-214.5 y19.5..21.5 z16.6..20.5 [A]', box(-218.0, -214.5, 19.5, 21.5, 16.6, 20.5)))
    specs.append(('HOOD EAR PLATE (crank pin pushes up) x-218..-214.5 y19.5..29.5 z19..20.5 [A]', box(-218.0, -214.5, 19.5, 29.5, 19.0, 20.5)))
    bf = c.features.baseFeatures.add(); bf.startEdit()
    for n, tb in specs: c.bRepBodies.add(tb, bf)
    bf.finishEdit()
    for b, (n, _) in zip(c.bRepBodies, specs): b.name = n
    print('bodies', c.bRepBodies.count, 'timeline', d.timeline.count)
