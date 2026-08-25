#!/usr/bin/env python3
"""Regenerate world_swarm.yaml's auto-generated survivors block from survivors.yaml.

Survivor placement is edited in one place (config/survivors.yaml) — this script
folds it into the world config Gazebo actually reads. To add/remove/move a
survivor: edit survivors.yaml, then re-run this script (or just run
scripts/run_simulation.sh, which runs it automatically before every launch).

Everything between the BEGIN/END markers in the target world file is replaced
wholesale on every run; nothing outside those markers is touched.
"""

__author__ = 'NIDAR RescueSwarm'
__license__ = 'BSD-3-Clause'

import argparse
from pathlib import Path

import yaml

BEGIN_MARKER = '# --- BEGIN AUTO-GENERATED SURVIVORS (do not edit by hand) ---'
END_MARKER = '# --- END AUTO-GENERATED SURVIVORS ---'

# Half-width of the square test arena survivors must stay inside (Phase 0.4:
# a 30m x 30m area centered on the world origin, i.e. x and y both in [-15, 15]).
ARENA_HALF_WIDTH_M = 15.0


def load_survivors(survivors_file: Path) -> list[dict]:
    """Load and validate survivor entries from survivors.yaml."""
    with open(survivors_file, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f) or {}

    survivors = config.get('survivors', [])
    if not survivors:
        raise ValueError(f'No survivors found in {survivors_file}')

    seen_names = set()
    for s in survivors:
        for field in ('name', 'model_type', 'x', 'y', 'z', 'yaw'):
            if field not in s:
                raise ValueError(f'Survivor entry {s} is missing required field "{field}"')
        if s['name'] in seen_names:
            raise ValueError(f'Duplicate survivor name "{s["name"]}" in {survivors_file}')
        seen_names.add(s['name'])
        if abs(s['x']) > ARENA_HALF_WIDTH_M or abs(s['y']) > ARENA_HALF_WIDTH_M:
            raise ValueError(
                f'Survivor "{s["name"]}" at ({s["x"]}, {s["y"]}) falls outside the '
                f'{ARENA_HALF_WIDTH_M * 2:.0f}m x {ARENA_HALF_WIDTH_M * 2:.0f}m arena '
                f'(x and y must both be within +/-{ARENA_HALF_WIDTH_M}m of the origin).'
            )

    return survivors


def render_objects_block(survivors: list[dict]) -> str:
    """Render the survivors list as the `objects:` YAML block."""
    lines = ['objects:']
    for s in survivors:
        lines.append(f'  - model_type: "{s["model_type"]}"')
        lines.append(f'    model_name: "{s["name"]}"')
        lines.append(f'    xyz: [{s["x"]}, {s["y"]}, {s["z"]}]')
        lines.append(f'    rpy: [0.0, 0.0, {s["yaw"]}]')
    return '\n'.join(lines)


def sync(survivors_file: Path, world_file: Path) -> int:
    """Replace the marked block in world_file with survivors_file's contents.

    :return: number of survivors written
    """
    survivors = load_survivors(survivors_file)
    objects_block = render_objects_block(survivors)

    world_text = world_file.read_text(encoding='utf-8')
    if BEGIN_MARKER not in world_text or END_MARKER not in world_text:
        raise ValueError(
            f'{world_file} is missing the BEGIN/END AUTO-GENERATED SURVIVORS markers. '
            'Add them once by hand (see project_gazebo/config/world_swarm.yaml), '
            'then this script will manage everything between them.'
        )

    before = world_text.split(BEGIN_MARKER)[0]
    after = world_text.split(END_MARKER)[1]
    new_text = f'{before}{BEGIN_MARKER}\n{objects_block}\n{END_MARKER}{after}'

    world_file.write_text(new_text, encoding='utf-8')
    return len(survivors)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '-s', '--survivors_file',
        type=str,
        default=str(Path(__file__).resolve().parent.parent / 'config' / 'survivors.yaml'),
        help='Path to survivors.yaml')
    parser.add_argument(
        '-w', '--world_file',
        type=str,
        default=str(Path(__file__).resolve().parent.parent / 'config' / 'world_swarm.yaml'),
        help='Path to the world config file to update')
    args = parser.parse_args()

    count = sync(Path(args.survivors_file), Path(args.world_file))
    print(f'Synced {count} survivor(s) from {args.survivors_file} into {args.world_file}')
