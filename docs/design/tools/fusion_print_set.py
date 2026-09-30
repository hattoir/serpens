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
def cyl_z(x, y, z0, z1, r): return tbm.createCylinderOrCone(P3(x/10, y/10, z0/10), r/10, P3(x/10, y/10, z1/10), r/10)
OUT = 'C:/Users/Public/serpens_gapcheck/print_out/'
def run(_context: str):
    d = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct); root = d.rootComponent
    for o in list(root.occurrences):
        if o.name.startswith('PRINT SET 2026-10-01'): o.deleteMe()
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()); c = occ.component
    c.name = 'PRINT SET 2026-10-01 - test coupons and mocks for HT-008/009/010/011 (print orientation, each body = one STL) - NOT ROBOT PARTS'
    specs = []
    # --- HT-008 TPU: 板ばね耳のクーポン 6 個（クランプ用の台 12x10x3 + 板 12 x L x t、平置き）+ パッド 3.0 / 6.0 各 4 個 ---
    t = box(0, 0.1, 0, 0.1, 0, 0.1)
    plate = None
    coupons = [(1.2, 9), (1.4, 9), (1.6, 9), (1.4, 8), (1.4, 10), (1.4, 9)]
    for i, (tt, L) in enumerate(coupons):
        x0 = i * 16.0
        b = box(x0, x0 + 12, 0, 10, 0, 3.0); union(b, box(x0, x0 + 12, 10, 10 + L, 0, tt))
        plate = b if plate is None else union(plate, b)
    for i in range(4):
        union(plate, box(i * 10.0, i * 10.0 + 3.0, 40, 43.6, 0, 2.0))
        union(plate, box(i * 10.0, i * 10.0 + 6.0, 50, 53.6, 0, 2.0))
    specs.append(('HT008_TPU_COUPONS_leaf x6 (t1.2/1.4/1.6 L9; t1.4 L8/L10/L9) + pad blocks 3.0 and 6.0 x4, TPU flat', plate))
    # --- HT-009 PLA: 首の芯の見本（穴 x3）2 種 + 窓の帯 2 種 ---
    blk = box(0, 24, 0, 14, 0, 14)
    for k in range(3): diff(blk, cyl_z(4 + 8 * k, 7, 7, 14.5, 1.3))                 # 下穴 d2.6 深さ 7（リーマで 3.0〜3.1 に仕上げる）
    specs.append(('HT009_CORE_SAMPLE_A pilot d2.6 x3 depth7 (ream to 3.0) PLA', blk))
    blk2 = box(0, 24, 0, 14, 0, 14)
    for k, dia in enumerate((2.95, 3.0, 3.05)): diff(blk2, cyl_z(4 + 8 * k, 7, 7, 14.5, dia / 2))
    specs.append(('HT009_CORE_SAMPLE_B as-printed holes d2.95/3.00/3.05 depth7 PLA', blk2))
    s1 = box(0, 40, 0, 12, 0, 2.0); diff(s1, box(12.6, 27.4, 4.2, 7.8, -0.5, 2.5))
    specs.append(('HT009_WINDOW_STRIP_W14p8 (window 14.8 x 3.6, wall 2.0) PLA - print 1 flat + 1 on edge', s1))
    s2 = box(0, 46, 0, 12, 0, 2.0); diff(s2, box(12.6, 33.4, 4.2, 7.8, -0.5, 2.5))
    specs.append(('HT009_WINDOW_STRIP_W20p8 (window 20.8 x 3.6, wall 2.0) PLA - print 1 flat + 1 on edge', s2))
    # --- HT-010 PLA: 覆い付きの襟の切れ端 + visor の切れ端 ---
    col = sector(49.5, 52.0, 0, 16, -34, 34); union(col, sector(49.5, 51.1, 14, 24, -30, 30))
    specs.append(('HT010_COLLAR_SLICE_with_COVER_A integral (collar r49.5-52 phi0-16 + cover A phi14-24, y+-34) PLA', col))
    specs.append(('HT010_VISOR_MOCK r46-48 phi16-80 y+-31 PLA (head visor arc, 2 mm)', sector(46.0, 48.0, 16, 80, -31, 31)))
    # --- HT-011 PLA: 肉球の実寸合わせ（ポケット 上が開いている）18.4 と 20.4 ---
    for pl, tag in ((18.4, '18p4'), (20.4, '20p4')):
        paw = box(-222, -210, 36, 51.6, 0, 10); union(paw, cyl_z(-222, 43.8, 0, 10, 7.8)); union(paw, cyl_z(-210, 43.8, 0, 10, 7.8))
        diff(paw, box(-225.0, -225.0 + pl, 37.6, 50.0, 2.9, 10.5)); diff(paw, box(-221.0, -213.0, 40.8, 46.8, -0.5, 3.0))
        specs.append(('HT011_PAW_MOCK_pocket%s (open top, window down; fit check for the real ToF board) PLA' % tag, paw))
    bf = c.features.baseFeatures.add(); bf.startEdit()
    for n, tb in specs: c.bRepBodies.add(tb, bf)
    bf.finishEdit()
    em = d.exportManager
    for b, (n, tb) in zip(c.bRepBodies, specs):
        b.name = n
        opt = em.createSTLExportOptions(b, OUT + n.split(' ')[0] + '.stl'); opt.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh; em.execute(opt)
        x = b.boundingBox; print(n.split(' ')[0], 'V=%.0f mm3' % (b.volume * 1000), 'size %.1f x %.1f x %.1f' % ((x.maxPoint.x - x.minPoint.x) * 10, (x.maxPoint.y - x.minPoint.y) * 10, (x.maxPoint.z - x.minPoint.z) * 10))
