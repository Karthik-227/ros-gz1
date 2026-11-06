"""CSV logging shared by the offline simulator and the live SITL runner.

Both produce the *same* schema, which is the point: a run flown in Gazebo and
the equivalent run from the point-mass model can be dropped into the same
plotting script and overlaid. Run metadata (law, gains, initial conditions)
goes in as ``#`` comment lines ahead of the header, so the file is still a
valid CSV for anything that skips comments -- :mod:`guidance_sim.plot_run`
does, and so does ``numpy.genfromtxt(..., comments='#')``.
"""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

#: Column order for the quantities every run logs. Anything a law adds via
#: ``extra_log()`` is appended after these, sorted, so the schema stays stable.
BASE_COLUMNS = [
    't',
    'R', 'Rdot', 'Vc', 'lam', 'lamdot', 'gam', 'gam_t', 'eps',
    'Vp', 'Vt', 'nu', 't_go', 'zem_perp',
    'latax', 'a_n', 'a_e', 'a_d', 'a_mag',
    'p_p_n', 'p_p_e', 'p_p_d', 'v_p_n', 'v_p_e', 'v_p_d',
    'p_t_n', 'p_t_e', 'p_t_d', 'v_t_n', 'v_t_e', 'v_t_d',
]


def sample(t: float, eng, a_cmd, extra: dict | None = None) -> dict:
    """Build one log row from an engagement and the command applied to it."""
    a_cmd = np.asarray(a_cmd, dtype=float)
    row = {'t': float(t)}
    row.update(eng.as_dict())
    row['latax'] = eng.latax_of(a_cmd)
    row['a_n'], row['a_e'], row['a_d'] = (float(x) for x in a_cmd)
    row['a_mag'] = float(np.linalg.norm(a_cmd))
    if extra:
        row.update({k: float(v) for k, v in extra.items()})
    return row


class RunLogger:
    """Collect rows in memory and write them out as one CSV."""

    def __init__(self, meta: dict | None = None):
        self.meta = dict(meta or {})
        self.rows: list[dict] = []

    def add(self, row: dict) -> None:
        self.rows.append(row)

    def log(self, t: float, eng, a_cmd, extra: dict | None = None) -> None:
        """Convenience wrapper around :func:`sample`."""
        self.rows.append(sample(t, eng, a_cmd, extra))

    def columns(self) -> list[str]:
        """Base columns in canonical order, then any extras, sorted."""
        seen: set[str] = set()
        for row in self.rows:
            seen.update(row)
        extras = sorted(seen - set(BASE_COLUMNS))
        return [c for c in BASE_COLUMNS if c in seen] + extras

    def write(self, path: str | Path) -> Path:
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        cols = self.columns()
        with path.open('w', newline='') as fh:
            for key, val in self.meta.items():
                fh.write(f'# {key}: {val}\n')
            writer = csv.DictWriter(fh, fieldnames=cols, restval='')
            writer.writeheader()
            for row in self.rows:
                writer.writerow({c: row.get(c, '') for c in cols})
        return path


def read_run(path: str | Path) -> tuple[dict[str, np.ndarray], dict[str, str]]:
    """Read a run CSV back as ``(columns, metadata)``.

    ``columns`` maps each header name to a float array. Empty cells come
    back as NaN so plots leave a gap instead of failing. Note that ``inf``,
    which ``t_go`` writes whenever the engagement is not closing, round-trips
    as a genuine infinity rather than NaN -- clip or mask it before plotting
    that column.
    """
    path = Path(path).expanduser()
    meta: dict[str, str] = {}
    data_lines: list[str] = []
    with path.open() as fh:
        for line in fh:
            if line.startswith('#'):
                key, _, val = line[1:].partition(':')
                meta[key.strip()] = val.strip()
            else:
                data_lines.append(line)

    reader = csv.DictReader(data_lines)
    cols: dict[str, list[float]] = {name: [] for name in (reader.fieldnames or [])}
    for row in reader:
        for name in cols:
            try:
                cols[name].append(float(row[name]))
            except (TypeError, ValueError):
                cols[name].append(float('nan'))
    return {k: np.asarray(v, dtype=float) for k, v in cols.items()}, meta
