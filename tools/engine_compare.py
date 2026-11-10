"""Slice .3mf projects through two CuraEngine builds with identical input and compare per-layer geometry.

Written for the Cura 5.13 -> 5.14 port: run the old and new engine on the same models and report,
per print feature (;TYPE:), total extrusion length, plus travels, retracts and per-layer differences.

What it does, per .3mf (the repo root's *.3mf and ../A5*/*.3mf):
  1. Bakes the build item's transform into a binary STL centred on 0,0 (the engine adds the bed-centre
     offset itself when machine_center_is_zero is false).
  2. Resolves a full settings set from packaging/fdmprinter.def.json, the .3mf's embedded machine
     definition and its profile stack, evaluating formulas in Python.
  3. Runs `CuraEngine slice -r resolved.json -o X.gcode` for both engines, from the model's folder
     (a 5.13 engine resolves the model name against the working directory), 120 s timeout each.

It compares two engines against each other; it does NOT reproduce Cura's own resolved settings exactly.
Cura's stack evaluation differs in places (the A55 Right Wing hung in support towers here but slices in
~14 s in real Cura). Confirm anything surprising in Cura itself.

Traps found while building it: ;TIME:6666 and dummy MINX values in a 5.13 command-line header are
placeholders; the embedded .def.json is flattened (formulas start with "=", defaults are strings).

Usage: engine_compare.py <repo> <workdir> <engineA> <engineB> [--only substr] [--skip-slice]
  The old-def baseline for detecting machine overrides is read from git (featherprint-5.13 branch).
"""
import argparse, collections, configparser, glob, json, math, os, re, struct, subprocess, sys, time, zipfile
import xml.etree.ElementTree as ET

CORE = "{http://schemas.microsoft.com/3dmanufacturing/core/2015/02}"


def flatten(tree):
    out = {}

    def walk(n):
        for k, v in n.items():
            out[k] = v
            if isinstance(v, dict) and "children" in v:
                walk(v["children"])

    walk(tree)
    return out


def cfg_values(text):
    c = configparser.ConfigParser(interpolation=None, strict=False)
    c.optionxform = str
    c.read_string(text)
    return dict(c["values"]) if "values" in c else {}


def stack(z, cfg_name):
    c = configparser.ConfigParser(interpolation=None, strict=False)
    c.read_string(z.read(cfg_name).decode())
    return [c["containers"][str(i)] for i in range(len(c["containers"]))]


class Resolver:
    def __init__(self, defs, layers):
        self.defs, self.layers, self.memo, self.active = defs, layers, {}, set()
        self.env = _Env(self)

    def cast(self, key, v):
        t = self.defs.get(key, {}).get("type", "str")
        if isinstance(v, str):
            s = v.strip()
            if t == "bool":
                return s.lower() in ("true", "on", "yes", "1")
            if t in ("int", "extruder", "optional_extruder"):
                try:
                    return int(float(s))
                except ValueError:
                    return v
            if t == "float":
                try:
                    return float(s)
                except ValueError:
                    return v
            return v
        if t == "int" and isinstance(v, (int, float)) and not isinstance(v, bool):
            return int(round(v))
        if t == "float" and isinstance(v, (int, float)) and not isinstance(v, bool):
            return float(v)
        return v

    def eval(self, key, expr):
        return self.cast(key, eval(expr, self.env))

    def get(self, key):
        if key in self.memo:
            return self.memo[key]
        d = self.defs.get(key)
        if key in self.active:  # formula cycle: break it on the default
            return self.cast(key, d.get("default_value")) if d else 0
        self.active.add(key)
        try:
            r = self._compute(key, d)
        finally:
            self.active.discard(key)
        self.memo[key] = r
        return r

    def _compute(self, key, d):
        for layer in self.layers:
            if key in layer:
                v = layer[key]
                if isinstance(v, str) and v.startswith("="):
                    return self.eval(key, v[1:])
                return self.cast(key, v)
        if d is None:
            raise KeyError(key)
        if "value" in d:
            try:
                return self.eval(key, str(d["value"]))
            except Exception as e:
                FORMULA_FAILS[key] = repr(e)
        return self.cast(key, d.get("default_value"))


FORMULA_FAILS = {}


class _Env(dict):
    def __init__(self, r):
        super().__init__()
        self.r = r
        self.update(math=math, max=max, min=min, sum=sum, any=any, all=all, abs=abs, int=int, float=float, str=str,
                    bool=bool, len=len, round=round, map=map, sorted=sorted, range=range, list=list,
                    resolveOrValue=lambda k: r.get(k), extruderValue=lambda e, k: r.get(k),
                    extruderValues=lambda k: [r.get(k)], defaultExtruderPosition=lambda: "0",
                    anyExtruderWithMaterial=lambda k: 0, anyExtruderNrWithOrDefault=lambda k: 0,
                    valueFromContainer=lambda *a: 0, valueFromExtruderContainer=lambda *a: 0)

    def __missing__(self, k):
        return self.r.get(k)


def prepare(repo, path, out_dir, fork_old):
    z = zipfile.ZipFile(path)
    names = z.namelist()
    g_cfg = next(n for n in names if n.endswith(".global.cfg"))
    e_cfg = next(n for n in names if n.endswith(".extruder.cfg"))

    def inst(nm):
        p = f"Cura/{nm}.inst.cfg"
        return cfg_values(z.read(p).decode()) if p in names else {}

    def layered(cfg):
        # Skip the last entry (definition) and material/variant placeholders.
        return [inst(c) for c in stack(z, cfg)[:-1]]

    gl, el = layered(g_cfg), layered(e_cfg)
    layers = []
    for i in range(max(len(gl), len(el))):
        if i < len(el):
            layers.append(el[i])
        if i < len(gl):
            layers.append(gl[i])

    mat = z.read(next(n for n in names if n.endswith(".fdm_material"))).decode()
    density = float(re.search(r"<density>(.*?)</density>", mat).group(1))
    diameter = float(re.search(r"<diameter>(.*?)</diameter>", mat).group(1))
    guid = re.search(r"<GUID>(.*?)</GUID>", mat).group(1)
    layers.append({"material_diameter": diameter, "material_bed_temperature": 60, "material_print_temperature": 200,
                   "material_standby_temperature": 175, "cool_fan_speed": 100})

    defs = flatten(json.load(open(os.path.join(repo, "packaging/fdmprinter.def.json"), encoding="utf-8"))["settings"])
    machine = {}
    for nm in names:
        if nm.endswith(".def.json"):
            # The embedded definition is flattened: every formula has a leading "=" and every default is
            # a string. Treat an entry as a machine override only if it differs from the fork's old def.
            for k, v in flatten(json.loads(z.read(nm))["settings"]).items():
                if v.get("type") == "category":
                    continue
                o = fork_old.get(k)
                ev = v.get("value")
                ev = ev[1:] if isinstance(ev, str) and ev.startswith("=") else ev
                if o is None:
                    machine[k] = {**v, "value": ev} if ev is not None else {kk: vv for kk, vv in v.items() if kk != "value"}
                elif ev is not None and str(ev) != str(o.get("value")):
                    machine[k] = {"value": ev}
                elif ev is None and str(v.get("default_value")) != str(o.get("default_value")):
                    machine[k] = {"default_value": v.get("default_value"), "_plain": True}
    for k, m in machine.items():
        plain = m.pop("_plain", False)
        defs[k] = {**defs.get(k, {}), **m}
        if plain:
            defs[k].pop("value", None)

    r = Resolver(defs, layers)
    vals = {}
    for k, d in defs.items():
        if d.get("type") == "category":
            continue
        v = r.get(k)
        vals[k] = v
    # Tokens Cura's StartSliceJob supplies, and blank start/end G-code to keep comparison on geometry.
    vals.update(material_density=density, material_spool_weight=750, material_spool_cost=0, material_name="PLA",
                material_guid=guid, material_id="generic_pla", material_type="PLA", material_brand="Generic",
                print_temperature=vals.get("material_print_temperature", 200),
                print_bed_temperature=vals.get("material_bed_temperature", 60),
                machine_start_gcode=";start", machine_end_gcode=";end", extruder_nr=0)

    def ser(v):
        if isinstance(v, bool):
            return "true" if v else "false"
        return v if isinstance(v, str) else json.dumps(v)

    sv = {k: ser(v) for k, v in vals.items() if v is not None}

    x = ET.fromstring(z.read("3D/3dmodel.model"))
    objs = {o.get("id"): o for o in x.iter(CORE + "object")}
    item = next(x.iter(CORE + "item"))
    t = [float(a) for a in item.get("transform").split()]
    mesh = objs[item.get("objectid")].find(CORE + "mesh")
    vs = [(float(v.get("x")), float(v.get("y")), float(v.get("z"))) for v in mesh.iter(CORE + "vertex")]
    w = [(a * t[0] + b * t[3] + c * t[6] + t[9], a * t[1] + b * t[4] + c * t[7] + t[10], a * t[2] + b * t[5] + c * t[8] + t[11]) for a, b, c in vs]
    tris = [(int(f.get("v1")), int(f.get("v2")), int(f.get("v3"))) for f in mesh.iter(CORE + "triangle")]
    minz = min(p[2] for p in w)
    # Keep the rotation from the 3mf but centre the model on 0,0: the engine itself adds the bed-centre
    # offset (machine_width/2, machine_depth/2) when machine_center_is_zero is false.
    cx = (min(p[0] for p in w) + max(p[0] for p in w)) / 2
    cy = (min(p[1] for p in w) + max(p[1] for p in w)) / 2
    w = [(p[0] - cx, p[1] - cy, p[2]) for p in w]
    stl = os.path.join(out_dir, "model.stl")
    with open(stl, "wb") as f:
        f.write(b"\0" * 80 + struct.pack("<I", len(tris)))
        for a, b, c in tris:
            f.write(struct.pack("<12fH", 0, 0, 0, *w[a][:2], w[a][2] - minz, *w[b][:2], w[b][2] - minz, *w[c][:2], w[c][2] - minz, 0))
    resolved = {"global": sv, "extruder.0": sv, "model.stl": {"extruder_nr": "0", "mesh_rotation_matrix": "[[1,0,0],[0,1,0],[0,0,1]]"}}
    rj = os.path.join(out_dir, "resolved.json")
    json.dump(resolved, open(rj, "w", encoding="utf-8"))
    summary = {k: vals.get(k) for k in ("infill_pattern", "layer_height", "layer_height_0", "wall_line_count", "support_enable", "top_layers", "bottom_layers", "initial_bottom_layers", "infill_sparse_density", "adhesion_type", "retraction_hop_enabled", "machine_width")}
    return rj, summary, len(tris)


def slice_one(exe, rj, gcode, log):
    t = time.time()
    with open(log, "w", encoding="utf-8", errors="replace") as lf:
        try:
            p = subprocess.run([exe, "slice", "-r", rj, "-o", gcode], cwd=os.path.dirname(rj), stdout=lf, stderr=subprocess.STDOUT, timeout=120)
        except subprocess.TimeoutExpired:
            return "TIMEOUT", time.time() - t
    return p.returncode, time.time() - t


MOVE = re.compile(r"[GXYZEF]-?[\d.]+")


def analyse(gcode):
    layers = collections.defaultdict(lambda: collections.Counter())
    bbox = [1e9, 1e9, -1e9, -1e9]
    x = y = z = e = 0.0
    abs_e = True
    layer, typ, travel, extruded, retracts, hops, prev_e_retract = None, "NONE", 0.0, 0.0, 0, 0, False
    meta = {}
    for line in open(gcode, encoding="utf-8", errors="replace"):
        if line[0] == ";":
            if line.startswith(";LAYER:"):
                layer = int(line[7:])
            elif line.startswith(";TYPE:"):
                typ = line[6:].strip()
            elif line.startswith(";TIME:"):
                meta["time"] = float(line[6:])
            continue
        if line.startswith("M82"):
            abs_e = True
        elif line.startswith("M83"):
            abs_e = False
        elif line.startswith("G92"):
            m = re.search(r"E(-?[\d.]+)", line)
            if m:
                e = float(m.group(1))
        elif line[:3] in ("G0 ", "G1 ", "G2 ", "G3 "):
            nx, ny, nz, ne = x, y, z, None
            for tok in line.split(";")[0].split()[1:]:
                c = tok[0]
                if c == "X":
                    nx = float(tok[1:])
                elif c == "Y":
                    ny = float(tok[1:])
                elif c == "Z":
                    nz = float(tok[1:])
                elif c == "E":
                    ne = float(tok[1:])
            d = math.hypot(nx - x, ny - y)
            de = 0.0
            if ne is not None:
                de = (ne - e) if abs_e else ne
                e = ne if abs_e else e
            if layer is not None:
                c = layers[layer]
                if de > 1e-5 and d > 0:
                    c[typ] += d
                    c["_filament"] += de
                    bbox = [min(bbox[0], nx), min(bbox[1], ny), max(bbox[2], nx), max(bbox[3], ny)]
                elif d > 0:
                    c["_travel"] += d
                    c["_travel_moves"] += 1
                if de < -1e-5:
                    c["_retracts"] += 1
                if nz > z + 1e-4 and d == 0 and ne is None:
                    c["_hops"] += 1
            x, y, z = nx, ny, nz
    return dict(layers=dict(layers), bbox=bbox, meta=meta)


def totals(a):
    t = collections.Counter()
    for c in a["layers"].values():
        t.update(c)
    return t


def compare(a, b, name, report):
    ta, tb = totals(a), totals(b)
    report.append(f"\n=== {name}: layers {len(a['layers'])} -> {len(b['layers'])}; bbox {[round(v,2) for v in a['bbox']]} -> {[round(v,2) for v in b['bbox']]}; est.time {a['meta'].get('time')} -> {b['meta'].get('time')}")
    keys = sorted(set(ta) | set(tb))
    for k in keys:
        va, vb = ta.get(k, 0), tb.get(k, 0)
        pct = (vb - va) / va * 100 if va else float("inf") if vb else 0
        report.append(f"  {k:22s} {va:14.1f} {vb:14.1f}  {pct:+8.2f}%")
    per = []
    for n in sorted(set(a["layers"]) | set(b["layers"])):
        la, lb = a["layers"].get(n, {}), b["layers"].get(n, {})
        va = sum(v for k, v in la.items() if not k.startswith("_") and k != "SUPPORT" and not k.startswith("SUPPORT"))
        vb = sum(v for k, v in lb.items() if not k.startswith("_") and k != "SUPPORT" and not k.startswith("SUPPORT"))
        if va or vb:
            per.append((abs(vb - va) / max(va, vb, 1e-6), n, va, vb))
    per.sort(reverse=True)
    n_any = sum(1 for p in per if p[0] > 1e-3)
    n_big = sum(1 for p in per if p[0] > 0.02)
    report.append(f"  model-extrusion per layer: {n_any}/{len(per)} differ >0.1%, {n_big} differ >2%. Worst: " + "; ".join(f"L{n} {va:.0f}->{vb:.0f}" for _, n, va, vb in per[:6]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("repo"); ap.add_argument("work"); ap.add_argument("a"); ap.add_argument("b")
    ap.add_argument("--only"); ap.add_argument("--skip-slice", action="store_true")
    o = ap.parse_args()
    repo = os.path.abspath(o.repo)
    fork_old = flatten(json.loads(subprocess.check_output(["git", "-C", repo, "show", "featherprint-5.13:packaging/fdmprinter.def.json"]))["settings"])
    models = glob.glob(os.path.join(repo, "*.3mf")) + glob.glob(os.path.join(repo, "..", "A5*", "*.3mf"))
    report = []
    for m in models:
        tag = os.path.basename(m)[:-4].replace(" ", "_")
        if o.only and o.only not in tag:
            continue
        d = os.path.join(o.work, tag)
        os.makedirs(d, exist_ok=True)
        rj, summary, ntris = prepare(repo, m, d, fork_old)
        print(tag, ntris, "tris", summary, flush=True)
        res = {}
        for lab, exe in (("A", o.a), ("B", o.b)):
            g = os.path.join(d, lab + ".gcode")
            if not o.skip_slice:
                rc, dt = slice_one(os.path.abspath(exe), os.path.abspath(rj), os.path.abspath(g), os.path.join(d, lab + ".log"))
                print(f"  {lab}: exit {rc} in {dt:.1f}s", flush=True)
                report.append(f"{tag} {lab}: exit {rc} in {dt:.1f}s")
            if os.path.exists(g) and os.path.getsize(g):
                res[lab] = analyse(g)
        if len(res) == 2:
            compare(res["A"], res["B"], tag, report)
    print("formula eval fails:", json.dumps(FORMULA_FAILS, indent=0)[:1500])
    print("\n".join(report))


main()
