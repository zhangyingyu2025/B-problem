"""JSON CLI and deterministic SVG inspection plot for B1."""
from __future__ import annotations

import argparse
import hashlib
import html
import json
import math
from pathlib import Path
import sys

from b1_geometry import solve_b1


def render_svg(result, title="B1 synthetic geometry"):
    """Render geometry directly as vector SVG. Not a spatial clipping solver."""
    pts = result["vertices"]
    circle = result["diameter_circle"]
    minimum = result["minimum_enclosing_circle"]
    if not pts:
        msg = html.escape(result["status"])
        return (f'<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="260">'
                f'<rect width="100%" height="100%" fill="#f8fafc"/>'
                f'<text x="40" y="65" font-family="sans-serif" font-size="25">{html.escape(title)}</text>'
                f'<text x="40" y="120" font-family="sans-serif" font-size="20">Status: {msg}; no finite polygon plotted.</text></svg>')
    bounds = list(pts)
    for c in (circle, minimum):
        if c:
            x, y = c["center"]
            r = c["radius_m"]
            bounds.extend([(x-r, y-r), (x+r, y+r)])
    xmin, xmax = min(p[0] for p in bounds), max(p[0] for p in bounds)
    ymin, ymax = min(p[1] for p in bounds), max(p[1] for p in bounds)
    span = max(xmax-xmin, ymax-ymin, 1.0)
    cx, cy = (xmin+xmax)/2, (ymin+ymax)/2
    scale = 460/(span*1.2)
    def xy(p):
        return 345+(p[0]-cx)*scale, 350-(p[1]-cy)*scale
    def coord(p):
        x, y = xy(p)
        return f"{x:.6f},{y:.6f}"
    provenance_label = "Synthetic validation case" if result.get("data_kind") == "synthetic" else "Input observations"
    elements = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="680" viewBox="0 0 1000 680">',
                '<rect width="1000" height="680" fill="#f8fafc"/>',
                '<g font-family="Segoe UI,Arial,sans-serif" fill="#152333">',
                f'<text x="40" y="45" font-size="25" font-weight="600">{html.escape(title)}</text>',
                f'<text x="40" y="75" font-size="15">{provenance_label} | Coordinates in metres | Equal x/y scale</text>',
                '<rect x="45" y="105" width="600" height="505" rx="8" fill="white" stroke="#d7dfe7"/>']
    elements.append(f'<polygon points="{" ".join(coord(p) for p in pts)}" fill="#dbeafe" fill-opacity="0.75" stroke="#1d4ed8" stroke-width="2"/>')
    for c, color, dash in ((minimum, "#047857", ""), (circle, "#be123c", 'stroke-dasharray="7 5"')):
        if c:
            x, y = xy(c["center"])
            elements.append(f'<circle cx="{x:.6f}" cy="{y:.6f}" r="{c["radius_m"]*scale:.6f}" fill="none" stroke="{color}" stroke-width="2" {dash}/>')
            elements.append(f'<path d="M{x-4},{y}h8 M{x},{y-4}v8" stroke="{color}"/>')
    if result["farthest_pair"]:
        a, b = result["farthest_pair"]
        elements.append(f'<polyline points="{coord(a)} {coord(b)}" fill="none" stroke="#be123c" stroke-width="2"/>')
    for i, p in enumerate(pts):
        x, y = xy(p)
        elements.append(f'<circle cx="{x:.6f}" cy="{y:.6f}" r="4" fill="#1d4ed8"/><text x="{x+8:.6f}" y="{y-8:.6f}" font-size="13">V{i}</text>')
    lines = [("#1d4ed8", "Feasible polygon"), ("#be123c", "Farthest-pair midpoint circle"), ("#047857", "Minimum enclosing circle")]
    for i, (color, label) in enumerate(lines):
        yy = 140+40*i
        elements.append(f'<line x1="675" x2="700" y1="{yy}" y2="{yy}" stroke="{color}" stroke-width="3"/><text x="710" y="{yy+5}" font-size="14">{label}</text>')
    rows = [f'Status: {result["status"]}', f'Vertices: {len(pts)}']
    if circle:
        rows += [f'Diameter: {result["diameter_m"]:.8g} m', f'Radius D/2: {circle["radius_m"]:.8g} m',
                 f'Same-diameter cover: {str(circle["covers"]).upper()}',
                 f'Min. cover diameter: {minimum["diameter_m"]:.8g} m',
                 f'Independent check: {result["circle_crosscheck_passed"]}']
    for i, line in enumerate(rows):
        elements.append(f'<text x="675" y="{290+31*i}" font-size="14">{html.escape(line)}</text>')
    elements.append('<text x="45" y="640" font-size="14">Blue vertices satisfy all bearing half-planes. The drawing window does not constrain the solver.</text>')
    elements.append('</g></svg>')
    return '\n'.join(elements)+'\n'


def load_payload(path):
    def reject_constant(value):
        raise ValueError(f"non-finite JSON constant: {value}")
    def unique_keys(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON key: {key}")
            result[key] = value
        return result
    data = json.loads(path.read_text(encoding="utf-8-sig"), parse_constant=reject_constant,
                      object_pairs_hook=unique_keys)
    if not isinstance(data, dict) or "observations" not in data:
        raise ValueError("input must be an object containing observations")
    allowed = {"observations", "error_deg", "case_id", "data_kind", "description", "true_target"}
    if set(data)-allowed:
        raise ValueError(f"unknown top-level fields: {sorted(set(data)-allowed)}")
    return data


def main(argv=None):
    parser = argparse.ArgumentParser(description="B1 bearing-wedge diameter and circle coverage")
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--svg", type=Path)
    args = parser.parse_args(argv)
    try:
        # Resolve paths before writing: an output must never replace an input.
        targets = [args.output] + ([args.svg] if args.svg else [])
        resolved = [p.resolve() for p in targets]
        if args.input.resolve() in resolved or len(set(resolved)) != len(resolved):
            raise ValueError("input, JSON output and SVG output must use distinct paths")
        payload = load_payload(args.input)
        result = solve_b1(payload["observations"], payload.get("error_deg", 1.0))
        result["case_id"] = payload.get("case_id", args.input.stem)
        result["data_kind"] = payload.get("data_kind", "unspecified")
        result["input_sha256"] = hashlib.sha256(args.input.read_bytes()).hexdigest()
        encoded = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
        if args.svg:
            args.svg.parent.mkdir(parents=True, exist_ok=True)
            title = f'B1 {result["case_id"]} ({result["data_kind"]})'
            args.svg.write_text(render_svg(result, title), encoding="utf-8")
        print(f'{result["case_id"]}: {result["status"]}; D={result["diameter_m"]}')
        return 3 if result["status"] == "numerically_uncertain" else 0
    except (ValueError, OSError) as exc:
        print(f"Input/output error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
