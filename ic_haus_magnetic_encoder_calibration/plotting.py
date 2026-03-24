"""Per-iteration diagnostic plots for encoder calibration.

Generates three types of figures:

1. **Raw waveforms** - master and nonius 14-bit ADC signals vs sample index.
2. **Residuals bar chart** - the 8 analog residual values for the current
   iteration with the convergence threshold line.
3. **Residuals trend** - line chart showing how each residual evolves
   across iterations (updated cumulatively).

Each figure is saved as a PNG and, optionally, shown interactively.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import matplotlib
import matplotlib.pyplot as plt

if TYPE_CHECKING:
    from pathlib import Path

    import mu_3sl_interface as mu_3sl
    from matplotlib.figure import Figure

logger = logging.getLogger(__name__)

# Convergence threshold (mirrors calibrator._RESIDUAL_THRESHOLD)
_THRESHOLD = 1.0

# Labels for the 8 residual values
_RESIDUAL_LABELS = [
    "M gx",
    "M voss",
    "M vosc",
    "M ph",
    "N gx",
    "N voss",
    "N vosc",
    "N ph",
]


def _extract_residuals(
    analyze_result: mu_3sl.AnalyzeResult,
) -> list[float]:
    """Return the 8 residual values as a flat list.

    Returns:
        List of 8 floats: [master gx, voss, vosc, ph, nonius gx, voss, vosc, ph].
    """
    m = analyze_result.relative_master_track_adjustments()
    n = analyze_result.relative_nonius_track_adjustments()
    return [
        m.cosine_gain_lsb,
        m.sine_offset_lsb,
        m.cosine_offset_lsb,
        m.phase_lsb,
        n.cosine_gain_lsb,
        n.sine_offset_lsb,
        n.cosine_offset_lsb,
        n.phase_lsb,
    ]


def _save_and_show(fig: Figure, path: Path, *, interactive: bool) -> None:
    fig.savefig(path, dpi=150, bbox_inches="tight")
    logger.info("Saved plot: %s", path)
    if interactive:
        fig.show()
        plt.pause(0.1)
    else:
        plt.close(fig)


def _ensure_backend(*, interactive: bool) -> None:
    """Set the matplotlib backend once, before creating any figures."""
    current = plt.get_backend().lower()
    if interactive and current == "agg":
        matplotlib.use("TkAgg", force=True)
    elif not interactive and current != "agg":
        matplotlib.use("Agg", force=True)


def plot_raw_waveforms(
    master_raw: list[int],
    nonius_raw: list[int],
    *,
    encoder: int,
    iteration: int,
    output_dir: Path,
    interactive: bool = False,
) -> Path:
    """Plot raw master and nonius ADC waveforms.

    Args:
        master_raw: 14-bit master samples.
        nonius_raw: 14-bit nonius samples.
        encoder: Encoder channel number.
        iteration: Current calibration iteration.
        output_dir: Directory for PNG output.
        interactive: Whether to also display the plot window.

    Returns:
        Path to the saved PNG file.
    """
    _ensure_backend(interactive=interactive)
    fig, (ax_m, ax_n) = plt.subplots(2, 1, sharex=True, figsize=(12, 6))
    fig.suptitle(f"Encoder {encoder} - Iteration {iteration}: Raw Waveforms")

    ax_m.plot(master_raw, linewidth=0.4)
    ax_m.set_ylabel("Master (14-bit)")
    ax_m.set_title(f"Master track ({len(master_raw)} samples)")

    ax_n.plot(nonius_raw, linewidth=0.4, color="tab:orange")
    ax_n.set_ylabel("Nonius (14-bit)")
    ax_n.set_xlabel("Sample index")
    ax_n.set_title(f"Nonius track ({len(nonius_raw)} samples)")

    path = output_dir / f"enc{encoder}_iter{iteration}_raw.png"
    _save_and_show(fig, path, interactive=interactive)
    return path


def plot_residuals_bar(
    analyze_result: mu_3sl.AnalyzeResult,
    *,
    encoder: int,
    iteration: int,
    output_dir: Path,
    interactive: bool = False,
) -> Path:
    """Plot a bar chart of the 8 analog residuals vs the convergence threshold.

    Args:
        analyze_result: Result from ``Calibration.analyze_raw_data()``.
        encoder: Encoder channel number.
        iteration: Current calibration iteration.
        output_dir: Directory for PNG output.
        interactive: Whether to also display the plot window.

    Returns:
        Path to the saved PNG file.
    """
    _ensure_backend(interactive=interactive)
    residuals = _extract_residuals(analyze_result)
    abs_residuals = [abs(r) for r in residuals]

    fig, ax = plt.subplots(figsize=(10, 5))
    colors = ["tab:green" if v <= _THRESHOLD else "tab:red" for v in abs_residuals]
    ax.bar(_RESIDUAL_LABELS, abs_residuals, color=colors)
    ax.axhline(_THRESHOLD, color="black", linestyle="--", linewidth=1, label="Threshold (1.0 LSB)")
    ax.set_ylabel("|Residual| (LSB)")
    ax.set_title(f"Encoder {encoder} - Iteration {iteration}: Residuals")
    ax.legend()

    # Annotate values
    for i, v in enumerate(abs_residuals):
        ax.text(i, v + 0.05, f"{v:.2f}", ha="center", va="bottom", fontsize=8)

    path = output_dir / f"enc{encoder}_iter{iteration}_residuals.png"
    _save_and_show(fig, path, interactive=interactive)
    return path


def plot_residuals_trend(
    history: dict[int, list[list[float]]],
    *,
    encoder: int,
    output_dir: Path,
    interactive: bool = False,
) -> Path:
    """Plot the evolution of each residual across iterations.

    Args:
        history: ``{encoder_number: [[8 residuals iter1], [8 residuals iter2], ...]}``.
        encoder: Encoder channel number.
        output_dir: Directory for PNG output.
        interactive: Whether to also display the plot window.

    Returns:
        Path to the saved PNG file.
    """
    _ensure_backend(interactive=interactive)
    data = history[encoder]
    iterations = list(range(1, len(data) + 1))

    fig, ax = plt.subplots(figsize=(10, 6))
    for idx, label in enumerate(_RESIDUAL_LABELS):
        values = [abs(row[idx]) for row in data]
        ax.plot(iterations, values, marker="o", label=label)

    ax.axhline(_THRESHOLD, color="black", linestyle="--", linewidth=1, label="Threshold")
    ax.set_xlabel("Iteration")
    ax.set_ylabel("|Residual| (LSB)")
    ax.set_title(f"Encoder {encoder} - Residuals Trend")
    ax.set_xticks(iterations)
    ax.legend(fontsize=8, ncol=2)

    path = output_dir / f"enc{encoder}_residuals_trend.png"
    _save_and_show(fig, path, interactive=interactive)
    return path
