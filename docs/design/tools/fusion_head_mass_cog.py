import adsk.core, adsk.fusion, json
def dens(n):
    if n.startswith(('SG90', 'CRANK')): return 1.44
    if n.startswith('ToF BOARD'): return 1.45
    if n.startswith(('TPU', 'HOOD B2 form/gate part 4', 'HOOD B2 form/gate part 5', 'HOOD B2 form/gate part 6')): return 1.2
    if n.startswith(('SIDE SKID', 'REAR SKID')): return 1.2      # TPU/PLA 相当（ASSUMED）
    return 1.24                                                   # PLA
def run(_context: str):
    d = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    out = {}
    for o in d.rootComponent.occurrences:
        if not o.name.startswith(('HEAD E3 INTEGRATED v1', 'HEAD E3 INTEGRATED v2')): continue
        tag = 'v1' if 'v1' in o.name[:24] else 'v2'
        rows = []
        for b in o.component.bRepBodies:
            p = b.physicalProperties; c = p.centerOfMass; v = b.volume * 1000   # cm3 -> mm3 ではなく cm^3 → *1000 で mm3/1000... 
            rows.append((b.name, b.volume, [c.x * 10, c.y * 10, c.z * 10], dens(b.name)))
        out[tag] = rows
    for tag, rows in out.items():
        for f in (1.0, 0.3):      # 頭の殻の中実の割合（あご・上の頭: 1.0 = 中実の上限、0.3 = 殻の仮定）
            M = 0; mx = my = mz = 0
            for n, vol, c, rho in rows:
                jaw_like = n.startswith(('JAW dug', 'E2-V HEAD E2 lower jaw', 'ボディ11', 'ボディ12')) or 'HEAD E2' in n
                m = vol * rho * (f if jaw_like else 1.0)            # vol は cm^3
                M += m; mx += m * c[0]; my += m * c[1]; mz += m * c[2]
            print(tag, 'jaw/upper solid fraction', f, 'mass %.1f g' % M, 'COG x %.1f y %.2f z %.1f' % (mx / M, my / M, mz / M))
        sg = [r for r in rows if r[0].startswith('SG90-class body')]
        print(tag, 'SG90 present', len(sg))
