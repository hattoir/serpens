import adsk.core, adsk.fusion
V3 = adsk.core.Vector3D.create; P3 = adsk.core.Point3D.create
tbm = adsk.fusion.TemporaryBRepManager.get(); U = adsk.fusion.BooleanTypes
def box(x0, x1, y0, y1, z0, z1):
    return tbm.createBox(adsk.core.OrientedBoundingBox3D.create(P3((x0+x1)/20, (y0+y1)/20, (z0+z1)/20), V3(1,0,0), V3(0,1,0), (x1-x0)/10, (y1-y0)/10, (z1-z0)/10))
def run(_context: str):
    d = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct); root = d.rootComponent
    for o in list(root.occurrences):
        if o.name.startswith('NECK-V v4 with COVER A'): o.deleteMe()
    neck = [o for o in root.occurrences if o.name.startswith('NECK-V VISOR')][0].component
    cover = [b for o in root.occurrences if o.name.startswith('J1 COVER A ADOPTED') for b in o.component.bRepBodies][0]
    o = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()); c = o.component
    c.name = 'NECK-V v4 with COVER A (integral, one print) - NOT FOR PRINT - NECK-V v3 dorsal bodies unioned with cover A over the collar edge (phi14-16 overlap) [C]; v3 kept'
    bf = c.features.baseFeatures.add(); bf.startEdit(); names = []
    for b in neck.bRepBodies:
        t = tbm.copy(b)
        bb = b.boundingBox
        if (not b.name.startswith(('NECK-V', 'KNUCKLE'))) and bb.minPoint.z * 10 >= 39.0 and bb.maxPoint.z * 10 <= 76.0 and bb.minPoint.x * 10 < -140:   # 首の背側 3 体（襟を持つ）
            y0, y1 = bb.minPoint.y * 10, bb.maxPoint.y * 10
            cv = tbm.copy(cover); tbm.booleanOperation(cv, box(-140, -130, y0 - 0.0, y1 + 0.0, 40, 60), U.IntersectionBooleanType)
            tbm.booleanOperation(t, cv, U.UnionBooleanType); names.append('NECK-V dorsal + cover A slice (y %.0f..%.0f)' % (y0, y1))
        else: names.append(b.name)
        c.bRepBodies.add(t, bf)
    bf.finishEdit()
    for b, n in zip(c.bRepBodies, names):
        b.name = n; src = neck.bRepBodies.item(list(c.bRepBodies).index(b)); b.appearance = src.appearance if src.appearance else None
        x = b.boundingBox
        if 'cover' in n: print(n, 'V=%.0f' % (b.volume * 1000), 'x %.1f..%.1f' % (x.minPoint.x * 10, x.maxPoint.x * 10))
    print('bodies', c.bRepBodies.count)
