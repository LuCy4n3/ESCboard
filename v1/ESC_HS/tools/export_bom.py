#!/usr/bin/env python3
"""Extract a hierarchical KiCad XML netlist and reconcile it with the PCB.

Generate the XML with kicad-cli sch export netlist --format kicadxml first.
Values, symbols and footprints are preserved independently when they disagree.
"""
import argparse
import collections
import csv
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET


def natural(value):
    return [int(x) if x.isdigit() else x.lower() for x in re.split(r"(\d+)", value)]


def sexpr(text):
    """Read KiCad's nested text format without evaluating any file content."""
    stack = []
    result = None
    for match in re.finditer(r'\(|\)|"(?:\\.|[^"\\])*"|[^\s()]+', text):
        token = match.group()
        if token == "(":
            node = []
            if stack:
                stack[-1].append(node)
            stack.append(node)
        elif token == ")":
            if not stack:
                raise ValueError("Unbalanced KiCad parentheses")
            result = stack.pop()
        else:
            if token.startswith('"'):
                token = re.sub(r'\\([\\"nrt])', lambda m: {
                    "n": "\n", "r": "\r", "t": "\t"
                }.get(m[1], m[1]), token[1:-1])
            stack[-1].append(token)
    if stack or result is None:
        raise ValueError("Incomplete KiCad file")
    return result


def children(node, key):
    return [x for x in node[1:] if isinstance(x, list) and x and x[0] == key]


def first(node, key, default=""):
    found = children(node, key)
    return found[0][1] if found and len(found[0]) > 1 else default


def pcb_parts(path):
    root = sexpr(path.read_text(encoding="utf-8"))
    result = {}
    for fp in children(root, "footprint"):
        props = {x[1]: x[2] for x in children(fp, "property")}
        ref = props.get("Reference", "")
        if not ref:
            raise ValueError("Missing PCB reference")
        attr = children(fp, "attr")
        result.setdefault(ref, []).append({
            "value": props.get("Value", ""), "footprint": fp[1],
            "side": first(fp, "layer"),
            "attrs": attr[0][1:] if attr else [],
            "position_mm": children(fp, "at")[0][1:3],
        })
    return result


def extract(netlist, board):
    root = ET.parse(netlist).getroot()
    pcb = pcb_parts(board)
    details = []
    for comp in root.findall("components/comp"):
        ref = comp.attrib["ref"]
        fields = {x.attrib["name"]: (x.text or "").strip()
                  for x in comp.findall("fields/field")}
        properties = {x.attrib["name"]: x.get("value", "") for x in comp.findall("property")}
        lib = comp.find("libsource")
        value = comp.findtext("value", "")
        footprint = comp.findtext("footprint", "")
        parts = pcb.get(ref, [])
        part = parts[0] if parts else None
        excluded = "exclude_from_bom" in properties
        dnp = "dnp" in properties
        notes = []
        if not part:
            notes.append("Not on PCB")
        else:
            if len(parts) > 1:
                notes.append(f"Duplicate PCB reference ({len(parts)} footprints); schematic quantity retained")
            if any(footprint != p["footprint"] for p in parts):
                notes.append("Schematic/PCB footprint mismatch")
            if any(value != p["value"] for p in parts):
                notes.append("Schematic/PCB value mismatch")
            if any(excluded != ("exclude_from_bom" in p["attrs"]) for p in parts):
                notes.append("Schematic/PCB BOM exclusion mismatch")
            if any(dnp != ("dnp" in p["attrs"]) for p in parts):
                notes.append("Schematic/PCB DNP mismatch")
        if footprint == "Connector_PCBEdge:Conn_Power":
            notes.append("PCB copper connection pad; no separate component to purchase")
        elif value in ("", "~"):
            notes.append("Missing component value")
        elif value in ("R", "D", "93LCxxBxxOT", "WE-PD2_3521"):
            notes.append("Exact value/part requires selection")
        if not footprint:
            notes.append("Missing footprint")
        if lib.get("part") == "IRFB7530PbF" and "IRFS7530" in footprint:
            notes.append("Symbol IRFB7530PbF vs footprint IRFS7530PBF: confirm package/part")
        if value.startswith("SQ3427CEV") and lib.get("part") == "Si2319CDS":
            notes.append("Value SQ3427CEV vs symbol/datasheet Si2319CDS: confirm selected device")
        details.append({
            "Reference": ref, "Value": value, "Footprint": footprint,
            "Symbol": f"{lib.get('lib', '')}:{lib.get('part', '')}",
            "Manufacturer": fields.get("MF") or fields.get("MANUFACTURER", ""),
            "MPN": fields.get("MP") or fields.get("MPN", ""),
            "Datasheet": comp.findtext("datasheet", ""),
            "Sheet": comp.find("sheetpath").get("names"),
            "Source file": properties.get("Sheetfile", ""),
            "PCB value": "; ".join(sorted({p["value"] for p in parts})),
            "PCB footprint": "; ".join(sorted({p["footprint"] for p in parts})),
            "Side": "; ".join(sorted({p["side"] for p in parts})),
            "PCB instances": len(parts),
            "DNP": "Yes" if dnp else "No",
            "Excluded from BOM": "Yes" if excluded else "No",
            "Assembly quantity": 0 if excluded or dnp else 1,
            "Notes": "; ".join(notes),
        })
    details.sort(key=lambda x: natural(x["Reference"]))
    refs = [x["Reference"] for x in details]
    if len(refs) != len(set(refs)):
        raise ValueError("Duplicate schematic references")
    grouped = collections.defaultdict(list)
    key_fields = ["Value", "Footprint", "Symbol", "Manufacturer", "MPN", "Datasheet", "Notes"]
    for row in details:
        if row["Assembly quantity"]:
            grouped[tuple(row[k] for k in key_fields)].append(row)
    groups = []
    for key, rows in grouped.items():
        entry = {k: v for k, v in zip(key_fields, key)}
        entry.update({"Quantity": len(rows), "References": ", ".join(x["Reference"] for x in rows),
                      "Sheets": ", ".join(sorted({x["Sheet"] for x in rows}))})
        groups.append(entry)
    groups.sort(key=lambda x: natural(x["References"].split(", ")[0]))
    missing = sorted(set(refs) - set(pcb), key=natural)
    extra = sorted(set(pcb) - set(refs), key=natural)
    issues = [{"Reference": x["Reference"], "Notes": x["Notes"]} for x in details if x["Notes"]]
    return {
        "summary": {"schematic_components": len(details),
                    "pcb_footprints": sum(len(x) for x in pcb.values()),
                    "pcb_duplicates": {k: v for k, v in pcb.items() if len(v) > 1},
                    "assembly_components": sum(x["Assembly quantity"] for x in details),
                    "assembly_groups": len(groups),
                    "excluded_from_bom": sum(x["Excluded from BOM"] == "Yes" for x in details),
                    "dnp": sum(x["DNP"] == "Yes" for x in details),
                    "schematic_only": missing, "pcb_only": extra,
                    "issues": issues,
                    "source_netlist_tool": root.findtext("design/tool"),
                    "source_export_date": root.findtext("design/date"),
                    "schematic_sha256": {s.findtext("title_block/source"):
                        hashlib.sha256((board.parent / s.findtext("title_block/source")).read_bytes()).hexdigest()
                        for s in root.findall("design/sheet")},
                    "pcb_sha256": hashlib.sha256(board.read_bytes()).hexdigest()},
        "groups": groups, "details": details,
    }


def write_csv(path, fields, rows):
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def write_report(path, data):
    s = data["summary"]
    issues = collections.defaultdict(list)
    for row in data["details"]:
        if row["Notes"]:
            issues[row["Notes"]].append(row["Reference"])
    lines = ["# ESC_HS bill of materials", "",
             "Generated from the supplied `esc_design/esc_design.kicad_sch` hierarchy",
             "and reconciled against `esc_design/esc_design.kicad_pcb`. Values and",
             "footprint identifiers are preserved as authored. The earlier provisional",
             "hardware checklist is replaced by this source-derived BOM.", "",
             "## Files", "",
             "- [Excel BOM](outputs/esc_hs_bom/BOM.xlsx): grouped parts, component inventory and review findings.",
             "- [Grouped CSV](BOM.csv): quantities, references, values, footprints and confirmation notes.",
             "- [Component CSV](BOM_components.csv): each schematic reference, including exclusions and PCB counts.",
             "- [Source data and reconciliation](bom_data.json): extracted fields, findings and source hashes.", "",
             "## Counts and population rules", "",
             f"- {s['schematic_components']} schematic component references across {len(s['schematic_sha256'])} sheets.",
             f"- {s['pcb_footprints']} PCB footprints. Duplicate references: {', '.join(s['pcb_duplicates']) or 'none'}.",
             f"- {s['assembly_components']} schematic BOM entries in {s['assembly_groups']} groups.",
             f"- {s['excluded_from_bom']} references excluded by the schematic BOM flags. DNP references: {s['dnp']}.",
             f"- Schematic-only references: {', '.join(s['schematic_only']) or 'none'}. PCB-only references: {', '.join(s['pcb_only']) or 'none'}.", "",
             "Quantities follow the schematic population flags. The duplicate PCB `R34`",
             "is reported separately and does not silently add another schematic part.",
             "`P1`–`P5` are PCB copper connection pads included by the source BOM flags,",
             "not five separately purchased components. Four mounting holes (`H1`–`H4`)",
             "and four solder jumpers (`JP1`–`JP3`, `JP5`) are excluded. `JP4` remains included.", "",
             "## Items requiring confirmation", "",
             "| References | Finding |", "| --- | --- |"]
    for note, refs in issues.items():
        lines.append(f"| {', '.join(refs)} | {note} |")
    lines.append("")
    for row in data["details"]:
        if "Schematic/PCB footprint mismatch" in row["Notes"]:
            lines += [f"`{row['Reference']}` schematic footprint: `{row['Footprint']}`. PCB footprint:",
                      f"`{row['PCB footprint']}`. This identifier comparison does not establish a geometry difference.", ""]
    for ref, parts in s["pcb_duplicates"].items():
        locations = "; ".join(f"{p['side']} at ({', '.join(p['position_mm'])}) mm" for p in parts)
        lines += [f"Duplicated `{ref}` locations in the supplied PCB coordinates: {locations}.", ""]
    lines += [
              "The BOM preserves specific part-like values such as `STSPIN32G4`,",
              "`WSLP27265L000FEA`, `MAX74810ARMZ-RL` and `TMUX6136PWR`. An explicit MPN",
              "field is left blank where the design has not populated it. No supplier",
              "stock, pricing, approved replacements or missing component ratings are invented.",
              "Passive voltage/power ratings and tolerances, and the unresolved generic",
              "parts above, must be selected before this becomes a purchasing BOM.", "",
              "## Design portability and review scope", "",
              "`U1` is explicitly STSPIN32G4, matching the firmware family. This is the",
              "ESC_HS design, separate from the neighboring STM32H750/DRV8323 ESC_HF board.",
              "The included `sym-lib-table` refers to an absolute external path. Only the",
              "IRFS7530PbF custom footprint library is bundled; the other custom footprint",
              "definitions are embedded in the PCB. Bundle external libraries and make",
              "their paths project-relative for portable editing.", "",
              "The review covers component fields, hierarchy, population flags and",
              "schematic/PCB reconciliation. It does not include full electrical ERC,",
              "layout DRC, device-rating approval or hardware validation. Firmware and",
              "schematic/PCB files have not been altered.", "",
              "## Regenerate CSV and report", "",
              "From the `v1/ESC_HS/` project directory in PowerShell:", "", "```powershell",
              "New-Item -ItemType Directory -Force captures",
              "& 'C:\\Program Files\\KiCad\\8.0\\bin\\kicad-cli.exe' sch export netlist --format kicadxml --output captures/bom_netlist.xml esc_design/esc_design.kicad_sch",
              "python tools/export_bom.py --netlist captures/bom_netlist.xml --pcb esc_design/esc_design.kicad_pcb --out hardware/bom_data.json --csv",
              "```", "", "Adjust the KiCad executable path for your installation. These commands refresh",
              "the CSV files, JSON and this report. The Excel file is a snapshot of this",
              "export and must be refreshed separately after design changes.", "",
              "## Grouped source BOM", "",
              "| Qty | Value | References | Footprint |", "| ---: | --- | --- | --- |"]
    for row in data["groups"]:
        lines.append(f"| {row['Quantity']} | {row['Value']} | {row['References']} | {row['Footprint']} |")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--netlist", required=True, type=Path)
    parser.add_argument("--pcb", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--csv", action="store_true", help="Also export grouped and component CSV files")
    args = parser.parse_args()
    data = extract(args.netlist, args.pcb)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if args.csv:
        write_csv(args.out.parent / "BOM.csv", ["Quantity", "References", "Value", "Footprint",
                  "Symbol", "Manufacturer", "MPN", "Datasheet", "Sheets", "Notes"], data["groups"])
        write_csv(args.out.parent / "BOM_components.csv", list(data["details"][0]), data["details"])
        write_report(args.out.parent / "BOM.md", data)
    print(json.dumps(data["summary"], ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
