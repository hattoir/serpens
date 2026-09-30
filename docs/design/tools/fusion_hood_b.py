import adsk.core, adsk.fusion, math, os
V3 = adsk.core.Vector3D.create; P3 = adsk.core.Point3D.create
tbm = adsk.fusion.TemporaryBRepManager.get(); U = adsk.fusion.BooleanTypes

def area(poly): return 0.5 * sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1] for i in range(len(poly)))

def line_off(p, q, d, cw):
    dx, dy = q[0] - p[0], q[1] - p[1]; L = math.hypot(dx, dy); dx /= L; dy /= L
    nx, ny = (-dy, dx) if cw else (dy, -dx)      # 外向きの法線: 時計回りなら左
    return ((p[0] + nx * d, p[1] + ny * d), (dx, dy))

def isect(l1, l2):
    (p, d), (q, e) = l1, l2
    det = d[0] * (-e[1]) - d[1] * (-e[0])
    t = ((q[0] - p[0]) * (-e[1]) - (q[1] - p[1]) * (-e[0])) / det
    return (p[0] + d[0] * t, p[1] + d[1] * t)

class Ctx:
    def __init__(self, root):
        self.root = root; self.occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()); self.comp = self.occ.component; self.comp.name = 'scratch'
    def prism(self, poly, z0, z1):
        sk = self.comp.sketches.add(self.comp.xYConstructionPlane)
        pts = [P3(x / 10, y / 10, 0) for x, y in poly]
        for i in range(len(pts)): sk.sketchCurves.sketchLines.addByTwoPoints(pts[i], pts[(i + 1) % len(pts)])
        prof = sk.profiles.item(0)
        ei = self.comp.features.extrudeFeatures.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ei.setDistanceExtent(False, adsk.core.ValueInput.createByReal((z1 - z0) / 10))
        f = self.comp.features.extrudeFeatures.add(ei); b = f.bodies.item(0)
        t = tbm.copy(b)
        if abs(z0) > 1e-9:
            m = adsk.core.Matrix3D.create(); m.translation = V3(0, 0, z0 / 10); tbm.transform(t, m)
        return t
    def done(self): self.occ.deleteMe()

def box(cx, cy, cz, lx, ly, lz, ang=0.0):
    a = math.radians(ang)
    return tbm.createBox(adsk.core.OrientedBoundingBox3D.create(P3(cx / 10, cy / 10, cz / 10), V3(math.cos(a), math.sin(a), 0), V3(-math.sin(a), math.cos(a), 0), lx / 10, ly / 10, lz / 10))
def cyl_z(x, y, z0, z1, r): return tbm.createCylinderOrCone(P3(x / 10, y / 10, z0 / 10), r / 10, P3(x / 10, y / 10, z1 / 10), r / 10)
def cyl_x(y, z, x0, x1, r): return tbm.createCylinderOrCone(P3(x0 / 10, y / 10, z / 10), r / 10, P3(x1 / 10, y / 10, z / 10), r / 10)
def union(a, b): tbm.booleanOperation(a, b, U.UnionBooleanType); return a
def diff(a, b): tbm.booleanOperation(a, b, U.DifferenceBooleanType); return a
def inter(a, b): tbm.booleanOperation(a, b, U.IntersectionBooleanType); return a

# ---- 寸法 [B]=ブリーフ [A]=ASSUMED [C]=計算 ----
MOUTH = 60.0; FUNNEL = 30.0; CH_L = 30.0; CH_W = 30.0; CH_H = 15.0     # [B]
WALL = 1.6; FOOT = 3.6; FOOT_H = 4.0                                     # [A] 壁 1.6（[B] 最小 1.2 以上）、足の帯 3.6×4
GRV_W = 1.2; GRV_D = 2.5                                                  # [A] スカートの溝
ROOF = WALL; H_OUT = CH_H + ROOF

def inner_poly():
    s = (MOUTH / 2 - CH_W / 2) / FUNNEL          # 漏斗の傾き（半幅の減り / 奥行き）[C] 0.5
    x0 = -3.0
    return [(x0, MOUTH / 2 + s * 3.0), (FUNNEL, CH_W / 2), (FUNNEL + CH_L, CH_W / 2), (FUNNEL + CH_L, -CH_W / 2), (FUNNEL, -CH_W / 2), (x0, -(MOUTH / 2 + s * 3.0))]

def outer_poly(d, front_x=0.0):
    P = inner_poly(); cw = area(P) < 0
    edges = [(P[i], P[i + 1]) for i in range(0, 5)]                        # 前の辺（最後の閉じ辺）を除く 5 辺
    L = [line_off(p, q, d, cw) for p, q in edges]
    pts = []
    # 前の縁（x = front_x）と最初の辺の交点
    p, dd = L[0]; t = (front_x - p[0]) / dd[0]; pts.append((front_x, p[1] + dd[1] * t))
    for i in range(4): pts.append(isect(L[i], L[i + 1]))
    p, dd = L[4]; t = (front_x - p[0]) / dd[0]; pts.append((front_x, p[1] + dd[1] * t))
    return pts

def groove_ring():
    a = outer_poly_ring(1.2); b = outer_poly_ring(2.4)
    return a, b

def outer_poly_ring(d):
    # 溝の帯用: 前を x = -1（通り抜け）まで延ばす
    return outer_poly(d, front_x=-1.0)


def build_hood(ctx):
    inn = inner_poly()
    # 外形: 壁 (WALL)、足の帯 (FOOT)
    body = ctx.prism(outer_poly(WALL), 0, H_OUT)
    union(body, ctx.prism(outer_poly(FOOT), 0, FOOT_H))
    # 耳（リップの取り付け）: x 1..7, |y| 30..40, z 2.2..4.0
    for s in (1, -1):
        union(body, box(4.0, s * 35.0, 3.1, 6.0, 10.0, 1.8))
    # 垂れ布のクランプ座: roof 上の前縁、x 0..8, |y| ≤ 27, z H_OUT .. H_OUT+3
    union(body, box(4.0, 0, H_OUT + 1.5, 8.0, 54.0, 3.0))
    # ロッドの受け（後ろ）: x 61.6..83.6, |y| ≤ 11, z 0..H_OUT
    xr0 = FUNNEL + CH_L + WALL
    union(body, box(xr0 + 11.0, 0, H_OUT / 2, 22.0, 22.0, H_OUT))
    # 空間（前後・下に貫通）
    diff(body, ctx.prism(inn, -1.0, CH_H))
    # スカートの溝（足の帯の中央、幅 1.2、深さ 2.5、下から）
    ring = ctx.prism(outer_poly(2.4, -1.0), -1.0, GRV_D); diff(ring, ctx.prism(outer_poly(1.2, -1.0), -2.0, GRV_D + 1))
    diff(body, ring)
    # 耳の穴（ペグ用 d2.4、貫通）
    for s in (1, -1):
        diff(body, cyl_z(4.0, s * 37.0, 1.0, 5.0, 1.2))
    # クランプ座の下穴（M2、d1.6、深さ 3.2、座の上から）
    for s in (1, -1):
        diff(body, cyl_z(4.0, s * 22.0, H_OUT + 3.0 - 3.2, H_OUT + 3.1, 0.8))
    # ロッドの穴 d8.4、深さ 20（後ろの端から）
    xe = xr0 + 22.0
    diff(body, cyl_x(0, H_OUT / 2, xe - 20.0, xe + 0.5, 4.2))
    # リップ用の先端: ここには無し
    return body

def lip_poly():
    return [(-14, -40), (7, -40), (7, -34.2), (0, -34.2), (0, 34.2), (7, 34.2), (7, 40), (-14, 40)]

def build_lip(ctx, h=0.0):
    b = ctx.prism(lip_poly(), h, h + 1.2)
    a = math.radians(12.0)
    d = (math.cos(a), 0.0, math.sin(a)); n = (-math.sin(a), 0.0, math.cos(a))
    st = (-14.0, 0.0, h + 0.6); L = 10.0; T = 2.0
    c = (st[0] + d[0] * L / 2 + n[0] * T / 2, 0.0, st[2] + d[2] * L / 2 + n[2] * T / 2)
    rb = tbm.createBox(adsk.core.OrientedBoundingBox3D.create(P3(c[0] / 10, 0, c[2] / 10), V3(d[0], 0, d[2]), V3(0, 1, 0), L / 10, 90.0 / 10, T / 10))
    diff(b, rb)
    for s in (1, -1):
        union(b, cyl_z(4.0, s * 37.0, h + 1.0, h + 1.2 + 3.0, 1.15))
    return b

def washer(ctx, t):
    body = cyl_z(0, 0, 0, t, 3.5)
    diff(body, cyl_z(0, 0, -1, t + 1, 1.3)); return body
