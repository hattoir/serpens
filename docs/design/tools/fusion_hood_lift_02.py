import adsk.core, adsk.fusion, math
V3 = adsk.core.Vector3D.create; P3 = adsk.core.Point3D.create
tbm = adsk.fusion.TemporaryBRepManager.get()
def box(x0, x1, y0, y1, z0, z1):
    return tbm.createBox(adsk.core.OrientedBoundingBox3D.create(P3((x0+x1)/20, (y0+y1)/20, (z0+z1)/20), V3(1,0,0), V3(0,1,0), (x1-x0)/10, (y1-y0)/10, (z1-z0)/10))
def cyl_y(x, z, y0, y1, r): return tbm.createCylinderOrCone(P3(x/10, y0/10, z/10), r/10, P3(x/10, y1/10, z/10), r/10)
def cyl_x(y, z, x0, x1, r): return tbm.createCylinderOrCone(P3(x0/10, y/10, z/10), r/10, P3(x1/10, y/10, z/10), r/10)
def side(sg):
    def yy(a, b): return (min(sg*a, sg*b), max(sg*a, sg*b))
    out = []; tg = '+y' if sg > 0 else '-y'
    for tag, xp, yl in (('A', -202, (17.0, 18.6)), ('B', -186, (18.8, 20.4))):
        y0, y1 = yy(*yl); out.append(('LINK %s %s pin x%d -> pivot x%d, L=20 t1.6 [A]' % (tag, tg, xp, xp + 20), box(xp - 2.5, xp + 22.5, y0, y1, 5.5, 10.5)))
    for xp in (-202, -186):
        y0, y1 = yy(16.6, 20.6); out.append(('HOOD PIN d2 x%d z8 %s [A]' % (xp, tg), cyl_y(xp, 8, y0, y1, 1.0)))
    for xp in (-182, -166):
        y0, y1 = yy(17.0, 20.6); out.append(('JAW PIVOT d2 x%d z8 %s [A]' % (xp, tg), cyl_y(xp, 8, y0, y1, 1.0)))
    return out
def run(_context: str):
    d = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = d.rootComponent
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()); c = occ.component
    c.name = 'HOOD LIFT STUDY 02 - NOT PARTS - stroke 4 [E-0008 gap<=4], SG90-class crank r2 [A], TPU leaf ear 11x1.4x9 (k 0.29 N/mm, ~1.2 N at 4 mm, PRIOR E26MPa) [C], parallel link L20 [A]; gravity descent only'
    specs = side(1) + side(-1)
    specs.append(('SG90-class body 22.7(x)x12.2(y)x22.5(z) at x-194.4..-171.7 y21.6..33.8 z10.2..32.7 [ASSUMED size]', box(-194.4, -171.7, 21.6, 33.8, 10.2, 32.7)))
    specs.append(('SG90 mounting tab 32.5 long t2.5 [ASSUMED]', box(-199.4, -166.9, 21.6, 33.8, 25.0, 27.5)))
    specs.append(('SG90 horn disc d7 t1.5 shaft y27.7 z15.8 [ASSUMED]', cyl_x(27.7, 15.8, -195.9, -194.4, 3.5)))
    specs.append(('CRANK PIN d2 r=2 (stroke 4) pin z13.8(down)..17.8(up) at y27.7, pushes leaf up only [A]', cyl_x(27.7, 13.8, -199.4, -195.9, 1.0)))
    specs.append(('HOOD EAR LUG on straight wall x-205..-194 y16.6..18.6 z11..17 [A]', box(-205.0, -194.0, 16.6, 18.6, 11.0, 17.0)))
    specs.append(('TPU LEAF EAR 11(x) x 9(y, L) x 1.4(z): x-205..-194 y18.6..27.6 z14.8..16.2, E26MPa PRIOR, ~1.2 N at 4 mm [C]', box(-205.0, -194.0, 18.6, 27.6, 14.8, 16.2)))
    bf = c.features.baseFeatures.add(); bf.startEdit()
    for n, tb in specs: c.bRepBodies.add(tb, bf)
    bf.finishEdit()
    for b, (n, _) in zip(c.bRepBodies, specs): b.name = n
    print(c.bRepBodies.count)
