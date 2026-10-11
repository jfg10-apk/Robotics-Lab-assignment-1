"""Compare all perturbation runs with the reference, one image per parameter."""

from pathlib import Path
import sys

# Keep tools/csv.py from shadowing Python's standard csv module.
if __package__ in (None, ""):
    sys.path[0] = str(Path(__file__).resolve().parents[1])

import csv
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

TOOLS_DIR = Path(__file__).resolve().parent
RESULTS_DIR = TOOLS_DIR / "results"
TIME_COLUMN = "Time (s)"
MAX_EFFORT_COLUMN = "Current actuator max effort (N*m)"
CASES = {
    "ombro": "Shoulder perturbation",
    "cotovelo": "Elbow perturbation",
    "pulso": "Wrist perturbation",
    "referencia": "Reference swing (no external torque)",
}


def load_results():
    """Group all numeric CSV recordings by experiment name."""
    experiments = {}
    for path in sorted(RESULTS_DIR.rglob("*.csv")):
        try:
            with path.open(newline="", encoding="utf-8-sig") as file:
                reader = csv.reader(file)
                columns = next(reader)
                values = np.array(list(reader), dtype=float)
            if values.size == 0:
                print(f"Sem amostras: {path}")
                continue
            if (TIME_COLUMN not in columns or values.ndim != 2
                    or values.shape[1] != len(columns)
                    or not np.isfinite(values).all()):
                raise ValueError("Colunas ou valores invalidos.")
        except (OSError, ValueError, StopIteration, csv.Error) as error:
            print(f"CSV ignorado: {path}; {error}")
            continue
        run_name = str(path.parent.relative_to(RESULTS_DIR))
        experiments.setdefault(path.stem, []).append((run_name, columns, values))
        print(f"Lido: {path.relative_to(RESULTS_DIR)}; {len(values)} amostras.")
    return experiments


def plot_parameter(column, experiments, references):
    """Plot one parameter, with torque curves and a dashed reference per case."""
    cases = [case for case in CASES if case in experiments and case != "referencia"]
    cases += [case for case in experiments if case not in CASES]
    if not cases:
        cases = ["referencia"]
    fig, axes = plt.subplots(len(cases), 1, figsize=(12, 3 * len(cases)), squeeze=False)
    legend = {}
    for axis, case in zip(axes.flat, cases):
        recordings = ([] if case == "referencia" else experiments[case]) + references
        for index, (run_name, headers, values) in enumerate(recordings):
            if column not in headers:
                continue
            is_reference = run_name.startswith("Reference")
            line, = axis.plot(
                values[:, headers.index(TIME_COLUMN)],
                values[:, headers.index(column)],
                label=run_name,
                color="black" if is_reference else f"C{index % 10}",
                linestyle="--" if is_reference else "-",
                linewidth=1.8 if is_reference else 1.2,
            )
            legend[run_name] = line
        axis.set_title(CASES.get(case, case))
        axis.set_xlabel(TIME_COLUMN)
        unit = column.split("(", 1)[1].rstrip(")") if "(" in column else "N m"
        axis.set_ylabel(unit)
        axis.grid(True, alpha=0.3)
    is_ball = column.startswith("Ball ") or column == MAX_EFFORT_COLUMN
    title = column if is_ball else f"Actuator effort: {column} (N m)"
    fig.suptitle(title, fontsize=14)
    fig.legend(list(legend.values()), list(legend), loc="lower center",
               ncol=min(5, len(legend)), title="Torque runs and normal swing", fontsize=9)
    legend_height = 0.3 * (math.ceil(len(legend) / 5) + 1) + 0.2
    height = 2.8 * len(cases) + legend_height + 0.6
    fig.set_size_inches(12, height)
    fig.tight_layout(rect=(0, legend_height / height, 1, 1 - 0.6 / height))
    if column == MAX_EFFORT_COLUMN:
        filename = "max_actuator_effort"
    elif is_ball:
        filename = column.split(" (")[0].lower().replace(" ", "_")
    else:
        filename = f"actuator_{column}"
    output = TOOLS_DIR / f"{filename}.png"
    fig.savefig(output, dpi=180)
    plt.close(fig)
    print(f"Grafico guardado: {output}")


def main():
    experiments = load_results()
    if not experiments:
        print(f"Nao existem CSV com amostras validas em {RESULTS_DIR}.")
        return
    references = []
    for run_name, headers, values in experiments.get("referencia", []):
        # Identical reference recordings represent the same normal swing.
        if not any(headers == previous_headers and np.array_equal(values, previous_values)
                   for _, previous_headers, previous_values in references):
            references.append((f"Reference ({run_name})", headers, values))
    if not references:
        print("Falta referencia.csv: executar os ensaios para guardar o swing normal.")
        return
    if len(references) == 1:
        _, headers, values = references[0]
        references = [("Reference (0 N m)", headers, values)]
    # Include every recorded parameter; simulation time is the horizontal axis.
    columns = list(dict.fromkeys(
        column for recordings in experiments.values()
        for _, headers, _ in recordings for column in headers
        if column != TIME_COLUMN
    ))
    for column in columns:
        plot_parameter(column, experiments, references)


if __name__ == "__main__":
    main()
