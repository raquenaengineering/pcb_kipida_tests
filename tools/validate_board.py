"""Run Ki-PIDA on the open kipida_test_4l board and read the Kelvin sense pads.

Needs KiCad running with kipida_test_4l.kicad_pcb open in the PCB editor (one
KiCad instance only), and a Python with Ki-PIDA's requirements installed (for
example the Ki-PIDA development venv):

    <KiPIDA>/.venv/Scripts/python.exe tools/validate_board.py [KiPIDA folder] [grid_mm ...]

As a submodule of Ki-PIDA (examples/pcb_kipida_tests) the Ki-PIDA folder is found
by itself, so from the Ki-PIDA root:

    .venv/Scripts/python.exe examples/pcb_kipida_tests/tools/validate_board.py [grid_mm ...]

In a standalone checkout of this repo, give the Ki-PIDA folder first.

Default grids: 0.1 and 0.025 mm. For each grid it prints one JSON line with the
drop between the sense pads (mV) per structure, then a KIPIDA_RESULTS block for
tools/generate_kipida_test_4l.py (paste it there and run the generator with
--spec-only to refresh VALIDATION.md).
"""
import contextlib
import io
import json
import os
import sys
import time

def _is_kipida(folder):
    return os.path.isdir(os.path.join(folder, "kipida"))


def _parse_args(args):
    """(Ki-PIDA folder, grids). The folder is optional when this repo sits inside Ki-PIDA."""
    if args and not _is_number(args[0]):
        folder, args = os.path.abspath(args[0]), args[1:]
    else:
        # tools/ -> pcb_kipida_tests/ -> examples/ -> Ki-PIDA root
        folder = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    if not _is_kipida(folder):
        sys.exit(f"Ki-PIDA not found in {folder}\n{__doc__}")
    return folder, [float(g) for g in args] or [0.1, 0.025]


def _is_number(text):
    try:
        float(text)
        return True
    except ValueError:
        return False


KIPIDA_DIR, GRIDS = _parse_args(sys.argv[1:])
sys.path.insert(0, KIPIDA_DIR)

import kipy  # noqa: E402
import numpy as np  # noqa: E402
from kipida.core.config_manager import load_config  # noqa: E402
from kipida.simulation.system import SystemSimulator  # noqa: E402

# Structure -> (net, reference sense terminal, other sense terminal); drop = V(first) - V(second).
# Sense pad = pad 2 of each terminal.
PAIRS = {
    "A1": ("VAL_A1_W050", "J1", "J2"), "A2": ("VAL_A2_W100", "J3", "J4"), "A3": ("VAL_A3_W200", "J5", "J6"),
    "B1": ("VAL_B1_NECK", "J7", "J8"), "C1": ("VAL_C1_IN1", "J9", "J10"), "F1": ("VAL_F1_BOTTOM", "J11", "J12"),
    "F2": ("VAL_F2_PLANE_UNDER", "J13", "J14"), "D1": ("VAL_D1_PLANE", "J19", "J20"),
    "E1": ("VAL_E1_VIA01", "J21", "J22"), "E2": ("VAL_E2_VIA04", "J23", "J24"), "E3": ("VAL_E3_VIA10", "J25", "J26"),
    "H1": ("VAL_H1_HEATER", "J27", "J28"),
    "G1 L1": ("VAL_G1_BUS", "J15", "J16"), "G1 L2": ("VAL_G1_BUS", "J15", "J17"), "G1": ("VAL_G1_BUS", "J15", "J18"),
    "I1 12V": ("VAL_I1_12V", "J29", "J31"), "I1": ("VAL_I1_5V", "J32", "J30"),
    "K1": ("VAL_K1_PTH", "J34", "J35"),
}
EXPECTED_FOOTPRINTS = 37


def run(board, rails, grid):
    sim = SystemSimulator(board, log_callback=lambda m: None)
    with contextlib.redirect_stdout(io.StringIO()):
        t = time.perf_counter()
        res = sim.run(rails, grid_size_mm=grid)
        elapsed = time.perf_counter() - t

    def sense(net, ref):
        r = res.rails.get(net)
        if r is None:
            return None
        nodes = sim.get_mesh_nodes(r.mesh, ref, ["2"])
        return float(np.mean([r.results[n] for n in nodes])) if nodes else None

    drops = {}
    for sid, (net, a, b) in PAIRS.items():
        va, vb = sense(net, a), sense(net, b)
        drops[sid] = None if va is None or vb is None else va - vb
    return drops, elapsed


def main():
    board = kipy.KiCad(timeout_ms=120000).get_board()
    n_fp = len(list(board.get_footprints()))
    if n_fp != EXPECTED_FOOTPRINTS:
        print(f"WARNING: {n_fp} footprints on the open board, expected {EXPECTED_FOOTPRINTS}: "
              "is kipida_test_4l (current version) the board open in KiCad?", file=sys.stderr)
    proj = board.document.project
    rails = load_config(os.path.join(proj.path, f"{proj.name}.kipida.json"))

    results = {}
    for grid in GRIDS:
        drops, elapsed = run(board, rails, grid)
        results[grid] = drops
        print(json.dumps({"grid": grid, "elapsed_s": round(elapsed, 2),
                          "mV": {k: None if v is None else round(v * 1e3, 4) for k, v in drops.items()}}))

    if len(GRIDS) == 2:
        coarse, fine = (results[g] for g in GRIDS)
        print(f"\n# Paste into generate_kipida_test_4l.py (mesh {GRIDS[0]:g} / {GRIDS[1]:g} mm):")
        print("KIPIDA_RESULTS = {")
        for sid in PAIRS:
            c, f = coarse[sid], fine[sid]
            if c is not None and f is not None:
                print(f'    "{sid}": ({c:.6g}, {f:.6g}),')
        print("}")


if __name__ == "__main__":
    main()
