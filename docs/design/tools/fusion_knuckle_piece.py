import adsk.core, adsk.fusion, math
V3 = adsk.core.Vector3D.create; P3 = adsk.core.Point3D.create
tbm = adsk.fusion.TemporaryBRepManager.get()
U = adsk.fusion.BooleanTypes

def box(cx, cy, cz, lx, ly, lz, ang_deg=0.0):
    a = math.radians(ang_deg)
    ob = adsk.core.OrientedBoundingBox3D.create(P3(cx/10, cy/10, cz/10), V3(math.cos(a), math.sin(a), 0), V3(-math.sin(a), math.cos(a), 0), lx/10, ly/10, lz/10)
    return tbm.createBox(ob)

def cyl(x, y, z0, z1, r):
    return tbm.createCylinderOrCone(P3(x/10, y/10, z0/10), r/10, P3(x/10, y/10, z1/10), r/10)

def union(a, b): tbm.booleanOperation(a, b, U.UnionBooleanType); return a
def diff(a, b): tbm.booleanOperation(a, b, U.DifferenceBooleanType); return a
def inter(a, b): tbm.booleanOperation(a, b, U.IntersectionBooleanType); return a

def union_component(name_prefix, root):
    occ = [o for o in root.allOccurrences if o.component.name.startswith(name_prefix)][0]
    acc = None
    for b in occ.component.bRepBodies:
        bb = b.boundingBox
        if bb.maxPoint.z * 10 < 8.0 or bb.minPoint.z * 10 > 76.0:
            continue
        t = tbm.copy(b)
        if acc is None: acc = t
        else: union(acc, t)
    return acc

def wedge(theta_deg, R_big=200.0):
    """+x を中心に ±theta の扇（原点）。半空間 2 つの共通部分。"""
    t = math.radians(theta_deg)
    n1 = (math.sin(t), math.cos(t)); n2 = (math.sin(t), -math.cos(t))
    parts = []
    for n in (n1, n2):
        ang = math.degrees(math.atan2(n[1], n[0]))
        parts.append(box(n[0] * R_big / 2, n[1] * R_big / 2, 0, R_big, 2 * R_big, 100.0, ang))
    return inter(parts[0], parts[1])

def build(ax, prefA, prefB, xa0, xa1, xb0, xb1, Rlabel, locpos, hole_d=8.6, peg_r=28.0):
    des = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = des.rootComponent
    shift = adsk.core.Matrix3D.create(); shift.translation = V3(-ax / 10, 0, -0.8)
    # ---- A（玉のリンク）----
    A = union_component(prefA, root)
    inter(A, box((xa0 + xa1) / 2, 0, 42, xa1 - xa0, 100, 68))            # x 範囲, z 8..76
    tbm.transform(A, shift)                                               # 軸を原点、z 0..68
    diff(A, cyl(0, 0, -1, 12, hole_d / 2))                                # 中心の穴 D8.6、深さ 12
    union(A, cyl(peg_r, 0, -2.2, 0.5, 1.8))                               # 案内の突起 D3.6、下へ 2.2
    xr = xa0 - ax
    union(A, box(xr - 10 + 1, 0, 1.25, 22, 1.5, 2.5))                     # 後ろの指針（幅 1.5、高さ 2.5、後ろへ 20）
    # ---- B（椀のリンク）----
    B = union_component(prefB, root)
    inter(B, box((xb0 + xb1) / 2, 0, 42, xb1 - xb0, 100, 68))
    tbm.transform(B, shift)
    for (px, py) in locpos:
        diff(B, cyl(px, py, -1, 6, 3.2))                                  # 位置決めの穴 D6.4、深さ 6
    # ---- 板 ----
    pl = box(6, 0, -2, 172, 150, 4)                                       # x −80..92, y ±75, z −4..0
    union(pl, cyl(0, 0, -0.5, 12, 4.0))                                   # 中心の軸 D8、高さ 12
    for (px, py) in locpos:
        union(pl, cyl(px, py, -0.5, 5, 3.0))                              # 位置決めの突起 D6、高さ 5
    ring = diff(cyl(0, 0, -2.5, 0.5, peg_r + 2.2), cyl(0, 0, -3, 1, peg_r - 2.2))
    diff(pl, inter(ring, wedge(55.7)))                                    # 案内の溝: 幅 4.4、深さ 2.5、±52.5°
    for phi in (-50, -25, 0, 25, 50):
        a = math.radians(phi); ux, uy = -math.cos(a), -math.sin(a)
        L = 6.0 if phi == 0 else 4.0
        union(pl, box(70 * ux, 70 * uy, 0.3, L, 1.2, 0.6, math.degrees(math.atan2(uy, ux))))
    return A, B, pl

def add_bodies(comp, temps, names, appearance=None):
    base = comp.features.baseFeatures.add(); base.startEdit()
    out = []
    for t, n in zip(temps, names):
        b = comp.bRepBodies.add(t, base); b.name = n; out.append(b)
        if appearance is not None: b.appearance = appearance
    base.finishEdit()
    return out


def make(tag, title, ax, pa, pb, xa0, xa1, xb0, xb1, loc, srcname):
    import os
    app = adsk.core.Application.get(); des = adsk.fusion.Design.cast(app.activeProduct); root = des.rootComponent
    for o in list(root.occurrences):
        if o.component.name.startswith('KNUCKLE TEST ' + tag):
            o.deleteMe()
    A, B, pl = build(ax, pa, pb, xa0, xa1, xb0, xb1, tag, loc)
    flip = adsk.core.Matrix3D.create(); flip.setWithArray([1,0,0,0, 0,-1,0,0, 0,0,-1,6.8, 0,0,0,1])
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()); comp = occ.component
    comp.name = 'KNUCKLE TEST %s A1 print - %s [measured CAD], holes/pegs [ASSUMED], z0-68 [C]' % (tag, title)
    nm = ['BALL A (print upside down) - drum %s [CAD measured], center hole d8.6 [ASSUMED rod8+0.6], guide peg d3.6 r28 [ASSUMED], pointer [ASSUMED]' % title,
          'SOCKET B (print upside down) - socket from CAD [%s], locating holes d6.4 [ASSUMED]' % ('measured R49.5 lip 39deg' if tag == 'R46' else 'ASSUMED geometry, LINK1 socket at J2 not measured'),
          'PLATE - 172x150x4 [ASSUMED], axle d8 [ASSUMED], guide groove stop +-52deg [ASSUMED = mech stop 52], ticks 0/25/50 [ASSUMED]']
    srcb = [b for o in root.allOccurrences if o.component.name.startswith(srcname) for b in o.component.bRepBodies if 'belly' in b.name][0]
    bodies = add_bodies(comp, [A, B, pl], nm, srcb.appearance)
    res = []
    for b in bodies: res.append((b.name[:10], round(b.volume, 1), round(b.area, 1)))
    occ2 = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()); c2 = occ2.component
    c2.name = 'KNUCKLE TEST %s PRINT ORIENTATION (A,B flipped, no supports) [C]' % tag
    A2 = tbm.copy(bodies[0]); B2 = tbm.copy(bodies[1]); P2 = tbm.copy(bodies[2])
    tbm.transform(A2, flip); tbm.transform(B2, flip)
    pb_ = add_bodies(c2, [A2, B2, P2], ['%s_BALL_A_PLA' % tag, '%s_SOCKET_B_PLA' % tag, '%s_PLATE_PLA' % tag], srcb.appearance)
    out = r'C:\Users\Public\serpens_gapcheck\knuckle_out'
    os.makedirs(out, exist_ok=True)
    for b in pb_:
        o = des.exportManager.createSTLExportOptions(b, out + '\KNUCKLE_' + b.name + '.stl'); o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        des.exportManager.execute(o)
        bb = b.boundingBox; res.append((b.name, [round(v*10,1) for v in (bb.minPoint.x, bb.maxPoint.x, bb.minPoint.y, bb.maxPoint.y, bb.minPoint.z, bb.maxPoint.z)]))
    occ2.isLightBulbOn = False
    return res
