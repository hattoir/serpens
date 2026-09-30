import adsk.core, adsk.fusion, math
V3 = adsk.core.Vector3D.create; P3 = adsk.core.Point3D.create
tbm = adsk.fusion.TemporaryBRepManager.get(); U = adsk.fusion.BooleanTypes
def box(x0, x1, y0, y1, z0, z1):
    return tbm.createBox(adsk.core.OrientedBoundingBox3D.create(P3((x0+x1)/20, (y0+y1)/20, (z0+z1)/20), V3(1,0,0), V3(0,1,0), (x1-x0)/10, (y1-y0)/10, (z1-z0)/10))
def cyl_y(x, z, y0, y1, r): return tbm.createCylinderOrCone(P3(x/10, y0/10, z/10), r/10, P3(x/10, y1/10, z/10), r/10)
def cyl_z(x, y, z0, z1, r): return tbm.createCylinderOrCone(P3(x/10, y/10, z0/10), r/10, P3(x/10, y/10, z1/10), r/10)
def union(a, b): tbm.booleanOperation(a, b, U.UnionBooleanType); return a
def diff(a, b): tbm.booleanOperation(a, b, U.DifferenceBooleanType); return a
def inter(a, b): tbm.booleanOperation(a, b, U.IntersectionBooleanType); return a
def sphere(x, y, z, r): return tbm.createSphere(P3(x/10, y/10, z/10), r/10)
def run(_context: str):
    d = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = d.rootComponent
    for o in list(root.occurrences):
        if o.name.startswith('TOF BOARD PLACEMENT STUDY 01'): o.deleteMe()
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()); c = occ.component
    c.name = 'TOF BOARD PLACEMENT STUDY 01 (R-007) - NOT PARTS - VL53L1X small board 12(y) x 18(x, ASSUMED length) x 3.2(z) [B], at side-skid nose X-226..-208, cliff 2-state only [E-0008]'
    specs = []
    # 案 S1: スキッドの外側に付ける（+y 側のみ図示。-y は鏡）。ブラケット = 3 mm 板でスキッド外面 |y|44 から出す
    for sg in (1, -1):
        y0, y1 = (44.0, 56.0) if sg > 0 else (-56.0, -44.0)
        specs.append(('S1 outboard: bracket plate t2 on skid outer face |y|44-46 x-226..-208 z2..8 %s' % ('L' if sg > 0 else 'R'), box(-226, -208, min(sg*44, sg*46), max(sg*44, sg*46), 2, 8)))
        specs.append(('S1 outboard: ToF board 12x18x3.2 |y|46-58 x-226..-208 z2.8..6 (window down) %s [ASSUMED length]' % ('L' if sg > 0 else 'R'), box(-226, -208, min(sg*46, sg*58), max(sg*46, sg*58), 2.8, 6.0)))
    # 案 S2: スキッドの前端を「手のひら（肉球）」状に広げる: x -226..-206、|y| 36..56、丸い。基板は穴の中、下に窓
    for sg in (1, -1):
        y0, y1 = (36.0, 51.6) if sg > 0 else (-51.6, -36.0); yc = sg * 43.8
        paw = box(-222, -210, y0, y1, 0, 10); union(paw, cyl_z(-222, yc, 0, 10, 7.8)); union(paw, cyl_z(-210, yc, 0, 10, 7.8))
        # つま先の反り R6 は前端で、上から見て円（半径 10）にする。ポケットと窓を抜く
        pocket = box(-224.0, -207.0, min(sg*37.6, sg*50.0), max(sg*37.6, sg*50.0), 2.9, 6.3)   # 基板 12.4 x 17 の余裕つき（y 40〜52.4）
        window = box(-221.0, -213.0, min(sg*40.8, sg*46.8), max(sg*40.8, sg*46.8), -0.5, 3.0)
        paw = diff(diff(paw, pocket), window)
        specs.append(('S2 paw: widened skid nose (round r7.8) |y|36-51.6 x-229.8..-202.2 z0..10 with board pocket + window %s [A]' % ('L' if sg > 0 else 'R'), paw))
        specs.append(('S2 paw: ToF board 12x18x3.2 in pocket y37.8-49.8 z3.0-6.2 %s [ASSUMED length]' % ('L' if sg > 0 else 'R'), box(-223.5, -207.5, min(sg*37.8, sg*49.8), max(sg*37.8, sg*49.8), 3.0, 6.2)))
    bf = c.features.baseFeatures.add(); bf.startEdit()
    for n, tb in specs: c.bRepBodies.add(tb, bf)
    bf.finishEdit()
    for b, (n, _) in zip(c.bRepBodies, specs):
        b.name = n
        x = b.boundingBox
        print(n[:40], "V=%.0f" % (b.volume*1000), "x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f" % (x.minPoint.x*10, x.maxPoint.x*10, x.minPoint.y*10, x.maxPoint.y*10, x.minPoint.z*10, x.maxPoint.z*10))
