# 子どもの身体寸法と接触安全: 資料調査（2026-09-29）

> **出どころ**: Engineering Agent が委任した調査エージェントの報告（モデルが書いたもの）。
> **検証**: Snyder 1977 の百分位（首囲・手首囲・手首幅・指の直径・頭幅）は、私が生データ（3,901 件）から**独立に計算し直して一致を確認した**。
> それ以外の数値と出典（CPSC の引く力、ISO/TS 15066、Walker 2010、Nikolajsen 2011、EN 71 / ASTM 等）は**まだ一次資料で再確認していない**（`SAFETY_UNVERIFIED`）。
> 標準・規格は本文の写しではなく二次的な要約を含む。数値を設計に使うときは、この文書の「未確認」欄を必ず見ること。
> 3 歳未満（Primary target）、特に 2 歳未満の首・手首・指の寸法は**信頼できる出典が見つかっていない**（Snyder 1977 に 2 歳未満は 3 件だけ）。

---

# Serpens safety design: source research (child anthropometry and contact thresholds)

Status: v1 written 2026-09-29. Labels: **[M]** direct measurement, **[S]** standard requirement, **[D]** derived (by me or by the source's own scaling). Anything I could not verify is stated as such. No number below is estimated from memory.

## PART A. Child anthropometry (ages ~0-6)

### A0. Data sources actually used
- **Snyder RG, Schneider LW, Owings CL, Reynolds HM, Golomb DH, Schork MA (1977)**, *Anthropometry of Infants, Children, and Youths to Age 18 for Product Safety Design*, UM-HSRI-77-17, Univ. of Michigan HSRI for CPSC. I used the **raw individual-level data file** distributed by UMTRI (3,901 records, mm): http://mreed.umtri.umich.edu/mreed/downloads/anthro/child/child_anthro_1977.txt (report PDF, same folder, `Snyder_1977_Child.pdf`; catalog https://deepblue.lib.umich.edu/handle/2027.42/684). Percentiles below are **[D]**: I computed them from the raw records, unweighted, by whole-year age bin. They are not the report's printed tables.
  - Limits of this file: essentially **no children under 2 y** (only 3 records at age <1 y). Neck/breadth items were taken on a subset, so n is small (37-96 per bin) and the P5/P95 tails are soft. Finger "diameter" items are the smallest hole through which the first joint cannot pass (plastic hole template, 1975 report method), i.e. a probe-relevant value, not tissue thickness. Units for those columns were inferred from the report ("IN CMS"); one-decimal mm.
- Snyder RG et al. (1975), *Physical Characteristics of Children as Related to Death and Injury for Consumer Product Safety Design*, UM-HSRI-BI-75-5 (newborn to 12 y, includes infants and a little-finger diameter, neck circumference for children): http://mreed.umtri.umich.edu/mreed/downloads/anthro/child/Snyder_1975_Child.pdf. Its tables are scanned graphs; OCR was unusable, so I did **not** extract infant values from it.

### A1. Snyder 1977 raw data, my percentiles (mm), P5 / P50 / P95 **[D from M]**

| Measure (mm) | 2-<3 y (n) | 4-<5 y (n) | 6-<7 y (n) |
|---|---|---|---|
| Neck circumference | 216/238/254 (37) | 227/248/271 (85) | 234/259/278 (69) |
| Neck breadth (lateral) | 61/70/77 | 67/77/87 | 67/78/89 |
| Wrist circumference | 100/112/125 (37) | 104/116/133 (85) | 110/122/136 (69) |
| Wrist breadth | 24/30/35 | 26/30/34 | 26/31/36 |
| Upper-arm circumference | 136/157/174 (109) | 145/164/188 (253) | 153/173/209 (215) |
| Forearm circumference | 140/154/173 | 149/162/183 | 153/171/194 |
| Chest circumference | 461/497/533 (108) | 495/537/590 (253) | 526/580/663 (215) |
| Chest breadth at axilla | 140/160/172 (37) | 157/171/190 (84) | 163/183/203 (68) |
| Head breadth | 125/133/142 (108) | 129/137/147 (250) | 132/139/149 (213) |
| Head length | 163/173/182 (38) | 167/179/191 (83) | 171/181/195 (69) |
| Hand breadth | 45/50/56 | 50/55/63 | 54/60/67 |
| Hand length | 91/102/111 | 106/118/129 | 119/129/145 |
| Index-finger length | 35/40/46 (45) | 40/45/52 (89) | 44/51/58 (67) |
| Index-finger diameter (1st joint cannot pass) | 8.7/9.5/10.3 (46) | 9.5/10.3/11.9 (88) | 10.3/11.1/12.7 (66) |
| Middle-finger diameter (same method) | 8.7/9.5/11.1 (47) | 9.5/10.3/11.9 | 10.3/11.9/12.7 |
| Min. hand clearance (smallest round hole the hand passes) | 38.1/41.3/47.6 (47) | 42.3/47.6/54.1 (87) | 44.5/50.8/57.2 (66) |

Absolute minima observed (any child 2-<7 y, raw): neck circumference 200 mm; wrist circumference 92 mm; wrist breadth 21 mm; head breadth 119 mm.
Derived diameters **[D]** (circumference/pi): neck P5 216 mm gives 68.8 mm; observed minimum 200 mm gives 63.7 mm. Wrist P5 100 mm gives 31.8 mm; minimum 92 mm gives 29.3 mm.

### A2. Under-2-year gap (the primary target group)
- **Neck circumference <12 months: not found in any reliable source.** A retail baby-clothing size chart shows 8.5-10 cm; that is a garment size, not a measurement, and is inconsistent with the data below. Do not use it.
- 13-24 months: Santos D et al. 2015, *Crit Care* 19(Suppl 2):P31 (conference abstract, 279 children): neck mean 23.4 cm (SD 1.3) boys, 22.9 cm (SD 1.3) girls at 13-15 months, https://pmc.ncbi.nlm.nih.gov/articles/PMC4601619/ **[M, low confidence: abstract only, measurement site not stated]**. **[D]** mean minus 1.645 SD gives about 21.3 cm (boys) and 20.8 cm (girls) as an approximate P5 (assumes normality).
- **Rejected:** a Turkish preschool neck paper (Kondolot 2017, *J Clin Res Pediatr Endocrinol* 9:17) returned by a web summarizer with 14-21 cm percentiles. Those values are far below Snyder's measured 2-3 y minimum (200 mm) and the table was not present in the page I could open; I could not verify them and treat them as unreliable.
- Newborn: chest circumference mean 33.87 cm (SD 1.43), n=120 term newborns; head 34.8 cm boys / 34.2 cm girls. Azevedo et al. 2019, *BMC Pediatrics* 19:341 **[M]**, https://pmc.ncbi.nlm.nih.gov/articles/PMC6761712/.
- Wrist/finger data for <2 y: not found. (Indian reference, 3 y: wrist mean 10.4 cm boys / 10.0 cm girls, *J Pediatr Endocrinol Metab* 2018, https://doi.org/10.1515/jpem-2017-0161, a cross-check that agrees with Snyder's 2-3 y median 112 mm.)
- Japan's MHLW 2010 infant growth survey (chest/head percentiles, 0-6 y) exists at https://www.mhlw.go.jp/stf/shingi/2r9852000001tmct-att/2r9852000001tmea.pdf but the PDF text was not machine-readable; **not extracted**. Childata (Norris & Wilson, DTI 1995) is the standard 0-18 y handbook; paywalled, not obtained.

### A3. Arm circumference by age (WHO reference, cm, P5 / P95) **[S-type growth standard, direct from table]**
WHO Child Growth Standards, *Arm circumference-for-age 3 months to 5 years (percentiles)*, WHO 2007. Boys: https://cdn.who.int/media/docs/default-source/child-growth/child-growth-standards/indicators/arm-circumference-for-age/acfa_boys_3_5_percentiles.pdf ; girls: `.../acfa_girls_3_5_percentiles.pdf`.
- Boys: 12 mo 13.0 / 16.8; 36 mo 13.9 / 18.2; 60 mo 14.4 / 19.1.
- Girls: 12 mo 12.4 / 16.6; 36 mo 13.7 / 18.3; 60 mo 14.5 / 19.7. (Girls 24 mo P5 13.0.)
- WHO measures the mid-upper arm; Snyder's 2-<3 y upper-arm P5 of 13.6 cm agrees. Other months were not read reliably from the OCR layout and are omitted. The NCHS NHANES 2015-2018 (National Health Statistics Report 46, Table 21, https://www.cdc.gov/nchs/data/series/sr_03/sr03-046-508.pdf) gives mid-arm circumference from 2 months; table text was misaligned in extraction, so I do not quote it.
- WHO head circumference (cm, P5 / P95): boys 12 mo 44.0 / 50.1, 36 mo 47.1 / 52.4; girls 12 mo 42.7 / 49.2, 36 mo 46.2 / 51.5. Source: WHO *Head circumference-for-age, birth to 5 years (percentiles)*, https://cdn.who.int/media/docs/default-source/child-growth/child-growth-standards/indicators/head-circumference-for-age/hcfa_girls_0_5_percentiles.pdf (girls; boys file in the same folder).

### A4. Child strength
- Grip (squeeze) force, bilateral mean, analog Takei dynamometer, n=3,218 Swedish preschoolers aged 3.0-5.5 y: Ramirez-Osuna A et al. 2026, *Sports Med Open* 12:19, https://pmc.ncbi.nlm.nih.gov/articles/PMC12949204/ **[M]**. P95 (kgf, converted to N at 9.807 **[D]**): boys 3.0 y 6.75 kgf (66 N), 4.0 y 8.88 (87 N), 5.0 y 8.80 (86 N); girls 3.0 y 4.55 (45 N), 4.0 y 7.93 (78 N), 5.0 y 8.55 (84 N). This is squeeze grip, **not** pulling force on an object.
- **Pull and grip force, direct measurement [M]** (Newtons, peak of 1-s moving average, max of 3 trials per child, both sexes; Q.95 = 95th percentile, max = largest single child). Source: CPSC, *Child Strength Measures: Children 24 through 71 Months Old* (400 children, Appendix D tables; publication year not captured in the extracted text), https://www.cpsc.gov/s3fs-public/Child-Strength-Measures-for-Children-24-through-71-Months-Old.pdf ; and CPSC, *Child Strength Measures: Children 6 through 23 Months Old*, https://www.cpsc.gov/s3fs-public/Child-Strength-Measures-for-Children-6-through-23-Months-Old.pdf .
  - 1-hand seated pull (bar, chest height, 1H_Seated_Pull): 24-29 mo Q.95 47.8 N (max 54.7); 30-35 mo 72.7 (max 80.5); 36-47 mo 110.1 (max 116.4); 48-59 mo 111.8 (max 116.0); 60-71 mo 209.5 (max 238.5).
  - 2-hand elbow-height standing pull (2H_Elbow_Pull): 24-29 mo Q.95 63.9 N (max 67.6); 30-35 mo 71.7 (max 78.3); 36-47 mo 97.4 (max 117.7). 2-hand seated pull 30-35 mo Q.95 149.9 N (max 165.4).
  - 24 mm-dynamometer grip (Dynamometer_24mm_Elbow_Grip): 24-29 mo Q.95 36.9 N (max 46.2); 30-35 mo 53.3 (max 67.9); 36-47 mo 65.6 (max 69.2); 48-59 mo 90.6 (max 103.5); 60-71 mo 104.2 (max 123.5).
  - Under 2 y (6-23 mo report; small n, 6-8 mo n=1-16 per task): pulling a bar or toy at 9-11 mo Q.95 about 22-29 N (max 23-32 N, e.g. BarHandle_Pull 22.3, Squishy tug-of-war 29.4); cart pull at 12-17 mo Q.95 30.4 N (max 36.0); dynamic bar pull 12-17 mo Q.95 34.7 N (max 37.6); 24 mm grip 9-11 mo Q.95 21.7 N (max 25.4). Older 12-23 mo rows partly unparsed.
  - Conclusion **[D]**: a toddler (2-<3 y) can pull about 50-80 N one-handed and up to about 165 N with two hands seated; a 3-5 y child can apply 100-120 N, and a 5-y-old up to about 240 N. Earlier work: Brown et al. 1973/74 (ages 2-6, n=50 per age) and Owings et al. 1975 (ages 2-10) are described in the CPSC report; their values were not extracted.
- Secondary, unverified: average maximum pinch about 5.7 lb (about 25 N) for ages 3.5-4.5 (Owings 1977, quoted in a search snippet only).

## PART B. Contact safety thresholds

### B1. ISO/TS 15066:2016, Annex A, Table A.2 (quasi-static contact) **[S, ADULT-BASED]**
Read from the licensed copy at https://www.diag.uniroma1.it/deluca/pHRI_elective/ISO_TS_15066_2016_en.pdf (Table A.2 and footnotes a-d; Table A.3).
- **Provenance (footnotes a, b):** pressure limits come from a single Univ. of Mainz pain-onset study on **100 healthy adults**, a **flat 1.4 x 1.4 cm metal probe with 2 mm edge radius**; the limit is the **75th percentile** of the recorded onset values, so about 25% of adults felt pain below it. Force limits are from a literature review (188 sources) at the lowest energy giving AIS 1 (bruise). The standard itself states the values come from one study and may change. Transient limits are simply **2x** the quasi-static values (footnote c); none is given for skull/face. Clause A.3.2: contact with face, skull, forehead is **not permissible**.
- Pressure (N/cm2) / force (N):
  - Neck: neck muscle 140, seventh neck muscle 210 / 150 N.
  - Chest: sternum 120, pectoral muscle 170 / 140 N.
  - Face: masticatory muscle 110 / 65 N. Skull: forehead 130 / 130 N, temple 110.
  - Upper arm: deltoid 190, humerus 220, radial bone 190 / 150 N. Lower arm: forearm muscle 180, arm nerve 180 / 160 N.
  - Hand and fingers: forefinger pad 300 (dominant) / 270 (non-dom.), forefinger end joint 280 / 220, thenar eminence 200, palm 260, back of hand 200 / 190 / force 140 N.
- Effective mass / spring constant (Table A.3): neck 1.2 kg, 50 N/mm; hand and fingers 0.6 kg, 75 N/mm; lower arm 2 kg, 40 N/mm.
- **Not child-validated. No child scaling exists in the standard.** Cross-check against children (B4): measured child thenar pain thresholds average only about 18 N/cm2, an order of magnitude under 200 (different method, see B4).

### B2. Toy and product-safety standards **[S]**
- **EN 71-1:2011+A3** (https://law.resource.org/pub/eu/toys/en.71.1.2014.html; clause figures from a summariser, verify against the standard): hinges 4.10.3, if a 5 mm rod enters the hinge-line gap then a 12 mm rod must also; winder keys 4.10.2; scissor-like action clearance at least 12 mm 4.10.1; wheel/body gaps 4.15.1.6 same 5/12 mm rule; wheel slots or holes wider than 5 mm prohibited 4.15.1.6; projectile energy 0.08 J (rigid) and 0.5 J (resilient) 4.17.3, resilient impact surface up to 0.16 J/cm2 (arrows); toy bags: drawstring rule above 380 mm opening perimeter 4.4.
- **ASTM F963-11** (https://law.resource.org/pub/us/cfr/ibr/003/astm.f963.2011.html): 4.13.2 hinge gap admitting 3/16 in (5 mm) must admit 1/2 in (13 mm), moveable parts over 1/2 lb; 4.18.1 accessible clearances 5 mm then 13 mm; 4.18.2 holes in thin rigid material, 6 mm rod to 10 mm depth then 13 mm rod, children under 60 months; 4.18.4 accessible parts of power-driven mechanisms in toys for children under 60 months **shall not present a pinch or laceration hazard**; 4.6.1 small-parts cylinder 31.7 mm (1.25 in) diameter, depth 25.4-57.1 mm (16 CFR 1501); 4.14.1 cords under 18 months no longer than 300 mm under 2.25 kg load, any loop must not pass the head probe; breakaway release below 22.2 N (5 lbf) 4.14.1.1.
- **ISO 8124-1** (secondary, search snippet only): hinge 5 to 12 mm for toys over 0.25 kg; resilient projectile above 0.08 J limited to 2500 J/m2 (which is 0.25 J/cm2, not equal to the 0.16 J/cm2 EN 71 arrow figure; treat as different clauses). Full text not read.
- **IEC 61032 probes** (secondary, vendor pages): probe 19 = 5.6 mm finger for children under 36 months; probe 18 = 8.6 mm for older children.
- **EN 1176-1:2017** (https://nobelcert.com/DataFiles/FreeUpload/EN%201176-1%20(2017).pdf, read from text): completely bound openings, probes C or E must not pass unless probe D also passes (4.2.7.2 a); force applied to probe 222 +/- 5 N (D.2.1.2); finger rods 8 mm and 25 mm, if the 8 mm passes then 25 mm must pass, and 8 mm must not lock in (D.10); 8.6 mm rod then 12 mm rod rule; 230 mm minimum for openings at suspended bridges. Probe dimensions are in Figure D.1 (drawing, not extracted). Vendor pages quote 89, 130, 230 mm for C, E, D (unverified). The standard itself notes probe D is based on an older child.
- **Cots:** ASTM F1169 / 16 CFR 1219 slat spacing at most 2 3/8 in (about 60 mm), from search snippet (https://law.resource.org/pub/us/cfr/ibr/003/astm.f1169.2013.html not read in full). EN 716-1 (secondary blog only, unverified): bar spacing 45-65 mm, 7 mm cone must not pass mesh holes, 60 mm cone must not pass slats, mounting holes 7-12 mm prohibited.
- **ISO 13854:2017 Table 1** (sample, https://cdn.standards.iteh.ai/samples/66459/46ce3b47790e4526802912dbbd921354/ISO-13854-2017.pdf): body 500 mm, head 300 mm, leg 180 mm; the rest of the table was cut off in the sample. Text requires "the unpredictable behaviour of children and their body dimensions" to be assessed separately when children may be present, so this is **adult-based**.
- **ISO 13482:2014** (https://cdn.standards.iteh.ai/samples/53820/5ddca453a6d141e5a558f4f791ea3229/ISO-13482-2014.pdf; quoted in search snippets): states that no exhaustive, internationally recognised pain/injury data exist for impact and that relevance for children and elderly must be considered. **No numeric limits for children.**
- **IEC 62368-1 / IEC 60335-1, -2-2:** only qualitative class definitions (MS1 no pain, MS2 painful but no injury, MS3 injury) found; **no numeric limits obtained**.

### B3. Crush and strangulation numbers
- Static crush threshold 150 N for fingers/hand, from door standards; Mewes D, Mauser F (2003), *Int J Occup Saf Ergon* 9(2):177-191, https://www.tandfonline.com/doi/abs/10.1080/10803548.2003.11076562 ; IFA report 0031 (2015): "150 N for static force", no dynamic threshold exists, https://www.dguv.de/medien/ifa/en/pub/ada/pdf_en/aifa0031e.pdf **[S/D, adult]**.
- Neck compression, adult: jugular about 2 kg (about 20 N), carotid 2.5-10 kg, trachea 8-12 kg (Puschel 2004, via https://www.forensicmed.co.uk/pathology/pressure-to-the-neck/); Iserson 1984 *Ann Emerg Med* quoted at 3.5 kg carotid. **Adult, secondary. No child-specific values verified.** A Wisconsin slide deck lists "child" values (11 lb, 4.4 lb, 33 lb) that I could not verify and appear to be adult figures; not used.
- **Child airway (direct measurement, anesthetised):** Walker RWM et al. 2010, *Br J Anaesth* 104:71-74, https://pubmed.ncbi.nlm.nih.gov/19942611/ (n=30, 3 months-15 y, bronchoscopy): mean external cricoid force compressing the subglottic airway by 50% or more was **10.5 N, as low as 5 N under 1 y, 15-25 N in teenagers**; adult cricoid pressure is 30 N. **[M]** This is the only child-specific neck-compression number I could verify, and it points to about 5 N being enough to distort an infant's airway.
- Crash-derived neck tension limits (EEVC WG12 Doc. 514, 2008, Table 3, ECE R94 scaled, https://soobiesurgeons.com/wp-content/uploads/2016/08/EEVC_WG1218_DOC514_Q-dummies__Criteria-April_2008.pdf) **[D, scaled from adult]**: upper neck tension Fz Q0 (newborn) 433 N, Q1 (1 y) 951 N, Q1.5 1080 N. Q3/Q6 columns were garbled in extraction. These are serious-injury criteria for crash loading, not comfort or strangulation limits.

### B4. Pain pressure in children vs adults
- Nikolajsen L et al. 2011, *J Child Orthop* 5:173-, 50 children aged 4-12 y (orthopaedic patients), hand-held algometer, 1 cm2 probe (search snippets): pressure pain threshold **183.1 kPa (SD 90.7) leg, 179.1 kPa (SD 97.4) thenar**; not age-dependent. https://pubmed.ncbi.nlm.nih.gov/22654978/ **[M]**
- Pedersen LK et al. 2020, *Scand J Pain* 20(2):339-344, n=30 aged 6-16 y: baseline crus 248 kPa, thenar 195 kPa, falling to 171 / 179 kPa before surgery and 146 / 161 kPa the day after. https://pubmed.ncbi.nlm.nih.gov/32007949/ **[M]** (values from abstract-level text).
- Conversion **[D]**: 179 kPa = 17.9 N/cm2. Roughly mean minus 1 SD gives about 82 kPa = 8 N/cm2 (assumes normality). Methods differ from ISO/TS 15066 (round rubber 1 cm2 slow ramp vs flat 1.96 cm2 metal, 75th percentile), so this is a magnitude comparison only; but it shows the ISO hand values are not safe to reuse for a child.
- I did **not** find a pediatric scaling factor for ISO/TS 15066 body-region limits or a finger crush injury threshold for children.

## Candidate conservative values (for review)

| Quantity | Candidate | Source / label | Adult-derived? |
|---|---|---|---|
| Smallest neck circumference (2-6 y) | 200 mm observed min (P5 216 mm) => neck diameter about 64 mm (P5 69 mm) | Snyder 1977 raw file [D from M] | No (child), but 2-6 y only |
| Neck <=2 y | about 208-213 mm (approx P5 from abstract mean/SD); infant <12 mo unknown | Santos 2015 abstract [D, low conf.] | No; infants unresolved |
| Wrist circumference | 92 mm min (P5 100 mm), diameter about 29-32 mm | Snyder 1977 raw [D from M] | No |
| Wrist breadth | 21 mm min (P5 24 mm) | Snyder 1977 raw | No |
| Upper/forearm circumference | 126 / 131 mm min (P5 136-140 mm at 2-<3 y); WHO 12 mo P5 124 mm | Snyder 1977 raw; WHO 2007 | No |
| Chest circumference | 434 mm min at 3-<4 y; P5 461 mm at 2-<3 y | Snyder 1977 raw | No |
| Head breadth | 119 mm min; P5 125 mm (2-<3 y) | Snyder 1977 raw | No |
| Finger first-joint size | 8.3-8.7 mm (hole test) | Snyder 1977 raw; EN 71 5 mm probe [S] | No |
| Hand clearance | hand passes a 38 mm hole | Snyder 1977 raw | No |
| Gap rule (fingers) | reject 5-12 mm gaps (5 mm admitted, then 12 mm must be admitted) | EN 71-1 4.10.3; ASTM F963 4.13.2 (13 mm) [S] | Child-oriented |
| Finger probe (under 36 mo) | 5.6 mm | IEC 61032 probe 19 (secondary) | Child-oriented |
| Small parts | 31.7 mm cylinder | ASTM F963 4.6.1 / 16 CFR 1501 [S] | Child |
| Head entrapment | 60 mm slat gap upper limit (cots) | ASTM F1169 (snippet) | Child |
| Neck compression | 5 N distorts infant airway | Walker 2010 [M] | Child (only verified one) |
| Child pain threshold, hand | about 8 N/cm2 (mean minus 1 SD) to 18 N/cm2 (mean) | Nikolajsen 2011 [D] | Child |
| Child pull force (design tug load) | 1-hand 48 N (2-<2.5 y) to 112 N (3-4 y); 2-hand up to 165 N max (2.5-3 y); infants 9-11 mo about 29 N P95 | CPSC child strength reports, App. D [M] | Child |
| Grip force P95 (3-5 y) | 66-87 N (Swedish); CPSC dynamometer 66-91 N | Ramirez-Osuna 2026; CPSC [M] | Child |
| Quasi-static contact force, hand | 140 N (adult); neck 150 N; face 65 N | ISO/TS 15066 A.2 [S] | **ADULT, must not be transferred** |
| Quasi-static pressure | 110-300 N/cm2 | ISO/TS 15066 A.2 [S] | **ADULT, must not be transferred** |
| Static finger crush | 150 N | Mewes & Mauser 2003 / IFA 0031 | **ADULT** |
| Neck tension IARV | Q0 433 N, Q1 951 N | EEVC Doc. 514 [D] | Scaled from adult, crash context |
| ISO 13854 gaps | body 500, head 300, leg 180 mm | ISO 13854 Table 1 | **ADULT** |

Unresolved: infant neck/wrist/finger dimensions (<2 y); pediatric strangulation/airway occlusion by force; child-specific crush thresholds; full CPSC pull tables for the remaining tasks and 12-23 months (only the rows above were read); numeric limits in ISO 13482 and IEC 62368/60335.
