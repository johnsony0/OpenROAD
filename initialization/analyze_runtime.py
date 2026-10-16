#!/usr/bin/env python3

import argparse
import csv
from statistics import fmean
from pathlib import Path

import matplotlib.pyplot as plt


STEP_GROUPS = {
    "Floorplanning": (
        "Design initialization",
        "Floorplanning",
        "Macro placement",
        "Tapcell insertion",
        "Power distribution network",
        "Filler placement",
    ),
    "Global placement": ("Global placement",),
    "Detailed placement": (
        "Pre-CTS detailed placement",
        "Detailed placement",
    ),
    "Timing optimization": (
        "Pre-CTS timing optimization",
        "Pre-CTS timing reports",
        "Setup and hold timing repair",
    ),
    "Clock tree synthesis": ("Clock tree synthesis",),
    "Global routing": ("Global routing",),
    "Detailed routing": ("Detailed routing",),
    "Finishing": ("Extraction", "Final reporting"),
}

COLORS = (
    "#4e79a7",
    "#f28e2b",
    "#76b7b2",
    "#e15759",
    "#59a14f",
    "#edc948",
    "#b07aa1",
    "#9c755f",
)


def read_runtime_csv(path):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or reader.fieldnames[0] != "section":
            raise ValueError("CSV must start with a 'section' column")

        pairs = reader.fieldnames[1:]
        if not pairs:
            raise ValueError("CSV contains no PDK/design columns")

        raw_timings = {pair: {} for pair in pairs}
        for row in reader:
            section = row["section"]
            if section in raw_timings[pairs[0]]:
                raise ValueError(f"Duplicate section in CSV: {section}")
            for pair in pairs:
                value = row.get(pair, "")
                try:
                    seconds = float(value) if value else 0.0
                except ValueError as error:
                    raise ValueError(
                        f"Invalid runtime for {pair}, {section}: {value!r}"
                    ) from error
                if seconds < 0:
                    raise ValueError(
                        f"Negative runtime for {pair}, {section}: {seconds}"
                    )
                raw_timings[pair][section] = seconds

    known_sections = {
        section for sections in STEP_GROUPS.values() for section in sections
    }
    ignored_sections = {
        "Post-GRT antenna repair",
        "Post-DRT antenna repair",
        "DRT input export",
    }
    unexpected_sections = set().union(*(set(rows) for rows in raw_timings.values()))
    unexpected_sections -= known_sections | ignored_sections
    if unexpected_sections:
        raise ValueError(
            "Uncategorized CSV sections: " + ", ".join(sorted(unexpected_sections))
        )

    timings = {
        pair: {
            group: sum(raw_timings[pair].get(section, 0.0) for section in sections)
            for group, sections in STEP_GROUPS.items()
        }
        for pair in pairs
    }
    return timings


def write_summary_csv(timings, output_path):
    pairs = list(timings)
    groups = list(STEP_GROUPS)
    totals = {pair: sum(timings[pair].values()) for pair in pairs}
    with output_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            ["portion", *pairs, "average_seconds", "average_percent"]
        )
        for group in groups:
            values = [timings[pair][group] for pair in pairs]
            percentages = [
                timings[pair][group] / totals[pair] * 100
                if totals[pair]
                else 0.0
                for pair in pairs
            ]
            writer.writerow(
                [
                    group,
                    *(f"{value:.3f}" for value in values),
                    f"{fmean(values):.3f}",
                    f"{fmean(percentages):.2f}",
                ]
            )
    print(f"Saved portion averages to {output_path}")


def plot_timings(timings, output_path, summary_path, show):
    pairs = list(timings)
    groups = list(STEP_GROUPS)
    totals = {pair: sum(timings[pair].values()) for pair in pairs}
    if not any(totals.values()):
        raise ValueError("CSV has no included runtime data to plot")
    write_summary_csv(timings, summary_path)

    figure_height = max(6.5, 0.55 * len(pairs) + 2.5)
    figure, (percent_axis, seconds_axis) = plt.subplots(
        ncols=2, figsize=(17, figure_height), sharey=True
    )

    for axis, percentage in ((percent_axis, True), (seconds_axis, False)):
        left = [0.0] * len(pairs)
        for group, color in zip(groups, COLORS):
            values = [
                (timings[pair][group] / totals[pair] * 100)
                if percentage and totals[pair]
                else timings[pair][group]
                for pair in pairs
            ]
            axis.barh(
                pairs,
                values,
                left=left,
                color=color,
                edgecolor="white",
                linewidth=0.4,
                label=group,
            )
            for index, (value, start) in enumerate(zip(values, left)):
                min_label_width = (
                    2.5
                    if percentage
                    else max(1.0, max(totals.values()) * 0.045)
                )
                if value > min_label_width:
                    label = f"{value:.0f}%" if percentage else f"{value:.1f}s"
                    axis.text(
                        start + value / 2,
                        index,
                        label,
                        ha="center",
                        va="center",
                        fontsize=8,
                    )
            left = [start + value for start, value in zip(left, values)]

        axis.set_xlabel("Share of total runtime (%)" if percentage else "Runtime distribution (Seconds)")
        axis.grid(axis="x", alpha=0.25)
        axis.set_axisbelow(True)
        axis.invert_yaxis()

    # percent_axis.set_title("Runtime distribution")
    percent_axis.set_xlim(0, 100)
    # seconds_axis.set_title("Runtime by step (total at bar end)")
    seconds_axis.set_xlim(0, max(totals.values()) * 1.15)
    for index, pair in enumerate(pairs):
        seconds_axis.annotate(
            f"{totals[pair]:.1f}s",
            xy=(totals[pair], index),
            xytext=(4, 0),
            textcoords="offset points",
            va="center",
            fontsize=8,
        )

    handles, labels = seconds_axis.get_legend_handles_labels()
    legend_columns = (len(groups) + 1) // 2
    legend_order = [
        index
        for column in range(legend_columns)
        for index in (column, column + legend_columns)
        if index < len(groups)
    ]
    figure.legend(
        [handles[index] for index in legend_order],
        [labels[index] for index in legend_order],
        loc="lower center",
        ncol=legend_columns,
        bbox_to_anchor=(0.5, 0.015),
    )
    '''figure.text(
        0.5,
        0.92,
        "Antenna repair and DRT input export are excluded from totals and charts.",
        ha="center",
        va="top",
        fontsize=9,
    )'''
    figure.tight_layout(rect=(0, 0.09, 1, 0.9))
    figure.savefig(output_path, dpi=180, bbox_inches="tight")
    print(f"Saved runtime charts to {output_path}")
    print("Runtime breakdown (seconds, included steps):")
    for pair in pairs:
        breakdown = ", ".join(
            f"{group}={timings[pair][group]:.3f}" for group in groups
        )
        print(f"{pair}: total={totals[pair]:.3f}; {breakdown}")

    if show:
        plt.show()
    plt.close(figure)


def main():
    default_input = Path(__file__).with_name("runtime_data.csv")
    default_output = Path(__file__).with_name("runtime_analysis.png")
    default_summary = Path(__file__).with_name("runtime_summary.csv")
    parser = argparse.ArgumentParser(
        description="Plot merged OpenROAD flow runtimes from the timing CSV."
    )
    parser.add_argument(
        "csv",
        nargs="?",
        type=Path,
        default=default_input,
        help=f"runtime CSV (default: {default_input})",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=default_output,
        help=f"output image path (default: {default_output})",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="save the chart without opening an interactive window",
    )
    parser.add_argument(
        "--summary-csv",
        type=Path,
        default=default_summary,
        help=f"output grouped timing CSV (default: {default_summary})",
    )
    args = parser.parse_args()

    if not args.csv.is_file():
        parser.error(f"runtime CSV not found: {args.csv}")
    plot_timings(
        read_runtime_csv(args.csv),
        args.output,
        args.summary_csv,
        not args.no_show,
    )


if __name__ == "__main__":
    main()
