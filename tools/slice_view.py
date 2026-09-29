"""Slice view: renders the Corrugated decomposition debug dump.

Produce a dump by slicing with the environment variables
    FP_SLICE_DUMP=<directory>           where to write layerNNNN_partK.json
    FP_SLICE_DUMP_LAYERS=<a-b | n>      which layers (gcode numbering, i.e. Cura's layer number - 1); default all
then render it:
    python tools/slice_view.py <directory> [layers, e.g. 140-150 or 352,565] [-o out.png] [--zoom x0,x1,y0,y1]

Each layer/part gets two panels:
  left  - Triangulation (VBCT Stages 1-9) domains
  right - Medial Axis domains over the Voronoi graph: kept axis (dark green), corner-pruned (light grey),
          spur-pruned (orange)
Domains: left wall red, right wall blue, Ring walls purple/teal, caps as black x (start) / black o (end).
"""
import argparse, glob, json, os, re

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_layers(spec):
    if not spec:
        return None
    out = set()
    for part in spec.split(","):
        if "-" in part:
            a, b = part.split("-")
            out.update(range(int(a), int(b) + 1))
        else:
            out.add(int(part))
    return out


def draw_domains(ax, domains):
    for d in domains or []:
        ring = d["kind"] == "ring"
        for key, col in (("left", "#7b3294" if ring else "tab:red"), ("right", "#008080" if ring else "tab:blue")):
            pts = d[key]
            if pts:
                ax.plot([p[0] for p in pts], [p[1] for p in pts], color=col, lw=1.4, zorder=3)
        if d.get("cap_start"):
            ax.plot(*d["cap_start"], "kx", ms=7, mew=1.5, zorder=4)
        if d.get("cap_end"):
            ax.plot(*d["cap_end"], "ko", ms=5, mfc="none", mew=1.2, zorder=4)


def draw_outline(ax, outline):
    for c in outline:
        xs = [p[0] for p in c] + [c[0][0]]
        ys = [p[1] for p in c] + [c[0][1]]
        ax.plot(xs, ys, color="black", lw=0.5, zorder=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump_dir")
    ap.add_argument("layers", nargs="?")
    ap.add_argument("-o", "--out", default="slice_view.png")
    ap.add_argument("--zoom", help="x0,x1,y0,y1 in mm")
    ap.add_argument("--no-pruned", action="store_true", help="hide corner-pruned Voronoi edges")
    args = ap.parse_args()
    want = parse_layers(args.layers)
    files = []
    for f in sorted(glob.glob(os.path.join(args.dump_dir, "layer*_part*.json"))):
        m = re.search(r"layer(\d+)_part(\d+)", f)
        if want is None or int(m.group(1)) in want:
            files.append(f)
    if not files:
        raise SystemExit("no dump files matched")
    if len(files) > 24:
        print(f"{len(files)} files matched; rendering the first 24")
        files = files[:24]
    fig, axs = plt.subplots(len(files), 2, figsize=(12, 5.5 * len(files)), squeeze=False)
    for row, f in zip(axs, files):
        d = json.load(open(f))
        title = f"layer {d['layer']} (Cura {d['layer'] + 1}) part {d['part']}"
        for ax, name in zip(row, ("cdt", "voronoi")):
            draw_outline(ax, d["outline"])
            if name == "voronoi":
                for e in d["graph"]:
                    if e["s"] == 1 and args.no_pruned:
                        continue
                    col, lw = {0: ("darkgreen", 1.0), 1: ("#cccccc", 0.5), 2: ("orange", 1.0)}[e["s"]]
                    ax.plot([p[0] for p in e["p"]], [p[1] for p in e["p"]], color=col, lw=lw, zorder=2)
            doms = d[name]
            if doms is None:
                ax.text(0.5, 0.5, "rejected", transform=ax.transAxes, ha="center", color="red")
            else:
                draw_domains(ax, doms)
            kinds = ",".join(x["kind"][0].upper() for x in doms) if doms else "-"
            ax.set_title(f"{title} - {'Triangulation' if name == 'cdt' else 'Medial Axis'} [{kinds}]", fontsize=9)
            ax.set_aspect("equal")
            if args.zoom:
                x0, x1, y0, y1 = (float(v) for v in args.zoom.split(","))
                ax.set_xlim(x0, x1)
                ax.set_ylim(y0, y1)
            ax.tick_params(labelsize=7)
    plt.tight_layout()
    plt.savefig(args.out, dpi=90)
    print("wrote", args.out)


if __name__ == "__main__":
    main()
