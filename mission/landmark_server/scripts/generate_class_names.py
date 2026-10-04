#!/usr/bin/env python3
"""Generate the landmark class name tables from vortex_msgs.

usage: generate_class_names.py LandmarkType.msg LandmarkSubtype.msg OUT.hpp

Every `uint16 NAME = value` constant of LandmarkType is a type. Every
constant of LandmarkSubtype is named <TYPE>_<SHORT> after the type it belongs
to (the longest type name that prefixes it). A new class is then one line in
vortex_msgs: the map reads its name in the config without a change here.
"""

import re
import sys

CONST = re.compile(r"^\s*uint16\s+([A-Z][A-Z0-9_]*)\s*=\s*(\d+)")


def constants(path):
    out = []
    for line in open(path):
        m = CONST.match(line.split("#", 1)[0])
        if m:
            out.append((m.group(1), int(m.group(2))))
    return out


def main():
    type_msg, subtype_msg, out_path = sys.argv[1:4]
    types = constants(type_msg)
    subtypes = []
    for name, value in constants(subtype_msg):
        owners = [t for t in types if name.startswith(t[0] + "_")]
        if owners:
            tname, tvalue = max(owners, key=lambda t: len(t[0]))
            short = name[len(tname) + 1:]
        else:
            # TORPEDO_ICON_FIRE belongs to TORPEDO_BOARD: the one type that
            # shares its first word.
            first = name.split("_")[0]
            owners = [t for t in types if t[0].split("_")[0] == first]
            if len(owners) != 1:
                sys.exit(f"{subtype_msg}: {name}: no type in {type_msg} it "
                         f"belongs to (name subtypes <TYPE>_<NAME>)")
            tname, tvalue = owners[0]
            short = name[len(first) + 1:]
        subtypes.append((tvalue, name, short, value))
    lines = [
        "// Generated from vortex_msgs LandmarkType.msg and LandmarkSubtype.msg",
        "// by scripts/generate_class_names.py. Do not edit.",
        "#pragma once",
        "#include <cstdint>",
        "",
        "namespace vortex::mission::generated {",
        "",
        "struct TypeName {",
        "    const char* name;",
        "    uint16_t value;",
        "};",
        "struct SubtypeName {",
        "    uint16_t type;",
        "    const char* full;",
        "    const char* short_name;",
        "    uint16_t value;",
        "};",
        "",
        "inline constexpr TypeName kTypes[] = {",
    ]
    lines += [f'    {{"{n}", {v}}},' for n, v in types]
    lines += ["};", "", "inline constexpr SubtypeName kSubtypes[] = {"]
    lines += [f'    {{{t}, "{full}", "{short}", {v}}},' for t, full, short, v in subtypes]
    lines += ["};", "", "}  // namespace vortex::mission::generated", ""]
    text = "\n".join(lines)
    try:
        if open(out_path).read() == text:
            return  # unchanged: no rebuild of what includes it
    except FileNotFoundError:
        pass
    with open(out_path, "w") as f:
        f.write(text)


if __name__ == "__main__":
    main()
