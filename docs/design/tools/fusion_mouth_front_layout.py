import adsk.core, adsk.fusion, math
V3 = adsk.core.Vector3D.create; P3 = adsk.core.Point3D.create
tbm = adsk.fusion.TemporaryBRepManager.get(); U = adsk.fusion.BooleanTypes
def box(x0, x1, y0, y1, z0, z1):
    return tbm.createBox(adsk.core.OrientedBoundingBox3D.create(P3((x0+x1)/20, (y0+y1)/20, (z0+z1)/20), V3(1,0,0), V3(0,1,0), (x1-x0)/10, (y1-y0)/10, (z1-z0)/10))
def cyl_y(x, z, y0, y1, r): return tbm.createCylinderOrCone(P3(x/10, y0/10, z/10), r/10, P3(x/10, y1/10, z/10), r/10)
def union(a, b): tbm.booleanOperation(a, b, U.UnionBooleanType); return a
def diff(a, b): tbm.booleanOperation(a, b, U.DifferenceBooleanType); return a
def inter(a, b): tbm.booleanOperation(a, b, U.IntersectionBooleanType); return a

def side_skid(y0, y1):
    # x -226..-178, z 0..10, 前端は R6 で反らす（側面から見て、底が (-220,0) から (-226,6) へ上がる）[A: mouth_front_layout §2]
    s = box(-220, -178, y0, y1, 0, 10)
    union(s, box(-226, -220, y0, y1, 6, 10))
    q = inter(cyl_y(-220, 6, y0, y1, 6.0), box(-226, -220, y0, y1, 0, 6))
    union(s, q)
    return s

def run(_context: str):
    d = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = d.rootComponent
    for o in list(root.occurrences):
        if o.name.startswith('MOUTH FRONT LAYOUT STUDY 01') and o.component.bRepBodies.count == 0:
            o.deleteMe()
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    c = occ.component
    c.name = 'MOUTH FRONT LAYOUT STUDY 01 - NOT PARTS - side skids |y|36-44 [C: mouth_front_layout §2], rear skids [C], ToF out of cavity [C], oblique LED at side-skid nose X-226 |y|40 Z<=6 [B]'
    specs = [
        ('SIDE SKID L (+y) x-226..-178 |y|36..44 z0..10 nose R6 [A]', side_skid(36, 44)),
        ('SIDE SKID R (-y) x-226..-178 |y|36..44 z0..10 nose R6 [A]', side_skid(-44, -36)),
        ('REAR SKID L (+y) x-172..-164 |y|14..22 z0..7 [C]', box(-172, -164, 14, 22, 0, 7)),
        ('REAR SKID R (-y) x-172..-164 |y|14..22 z0..7 [C]', box(-172, -164, -22, -14, 0, 7)),
        ('ToF DOWN outside cavity x-172..-165 |y|<=7 z7..9 [C] (VL53L1X min 40 mm => not for c,t; floor-lost 2-state only)', box(-172, -165, -7, 7, 7, 9)),
        ('OBLIQUE LED L (+y) 3x3x3 at side-skid nose X-226 y40 z3..6 [B: Z<=6]', box(-226, -223, 38.5, 41.5, 3, 6)),
        ('OBLIQUE LED R (-y) 3x3x3 at side-skid nose X-226 y-40 z3..6 [B: Z<=6]', box(-226, -223, -41.5, -38.5, 3, 6)),
    ]
    bf = c.features.baseFeatures.add()
    bf.startEdit()
    for name, tb in specs:
        c.bRepBodies.add(tb, bf)
    bf.finishEdit()
    print('timeline', d.timeline.count, [b.name[:20] for b in c.bRepBodies])
