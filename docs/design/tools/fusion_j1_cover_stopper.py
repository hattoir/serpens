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
def run(_context: str):
    d = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = d.rootComponent
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()); c = occ.component
    c.name = 'J1 COVER + STOPPER STUDY 01 - NOT PARTS - axis (-181.8, z32.35) [E ASSUMED], angles in CAD sign (+ = head DOWN, see j1_sign_correction), cover r49.5-51.1 [C], range Eng -5..+10 = CAD +5..-10 [E]'
    PHI_COLLAR = 16.0    # 首の襟の前縁の角（p0 で測定 [C]）
    specs = []
    # 上の輪の覆い（頭が下がる = CAD + 側。Engineering の -5° 端 = CAD +5°、+3° の余裕）: 襟の前縁を同心の殻で前へ延ばす。y ±30 [B]
    specs.append(('COVER UPPER shell r49.5-51.1 phi14-24 |y|<=30, extends collar front edge (phi16) over hood by 8 deg = 7.1 mm at r51 [C]', sector(49.5, 51.1, PHI_COLLAR - 2.0, PHI_COLLAR + 8.0, -30.0, 30.0)))
    # ストッパー: 首の芯（r45）の側面から出る半径方向のピン（|y|=18、φ 3）が、フード（r46-48）の弧状の窓に入る
    PHI_P = 40.0
    for sg in (1, -1):
        specs.append(('STOP PIN radial d3 r44-48.5 phi%d y%+d on tongue core [A]' % (PHI_P, 18*sg), radial_pin(PHI_P, 44.0, 48.5, 18.0*sg, 1.5)))
        # 窓（フード側に切る形の工具体）: 頭の座標の角 = 世界の角 - θ_CAD。CAD +5〜-10 → 窓は φp-5 ... φp+10 に、ピンの半角 (1.5/47 rad = 1.8°) を足す
        specs.append(('STOP SLOT WINDOW (cutter for hood) phi%.1f-%.1f r45.5-48.5 y%+d w3.6 [C]' % (PHI_P - 5 - 1.8, PHI_P + 10 + 1.8, 18*sg), sector(45.5, 48.5, PHI_P - 5 - 1.8, PHI_P + 10 + 1.8, 18.0*sg - 1.8, 18.0*sg + 1.8)))
    bf = c.features.baseFeatures.add(); bf.startEdit()
    for n, tb in specs: c.bRepBodies.add(tb, bf)
    bf.finishEdit()
    for b, (n, _) in zip(c.bRepBodies, specs):
        b.name = n
        x = b.boundingBox
        print(n[:40], "x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f V=%.0f" % (x.minPoint.x*10, x.maxPoint.x*10, x.minPoint.y*10, x.maxPoint.y*10, x.minPoint.z*10, x.maxPoint.z*10, b.volume*1000))
