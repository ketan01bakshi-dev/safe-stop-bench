"""python -m plant.ci [l0 | l1 | mutants | l2 | all | release [--approve --by NAME] | version]   (P6 plant-model pipeline; exit 0 green, 1 red, 5 BLOCKED by the tool)"""
from __future__ import annotations

import sys

from plant.ci import omc, pipeline


def cli(argv: list[str]) -> int:
    stage = argv[0] if argv else "all"
    if stage == "release":
        by = argv[argv.index("--by") + 1] if "--by" in argv else ""
        return pipeline.release(approve="--approve" in argv, by=by)
    if stage == "version":
        print(omc.version())
        return 0
    if stage not in ("l0", "l1", "mutants", "l2", "all"):
        print(__doc__)
        return 2
    return pipeline.main(stage)


if __name__ == "__main__":
    sys.exit(cli(sys.argv[1:]))
