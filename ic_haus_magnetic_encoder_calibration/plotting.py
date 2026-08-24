"""Per-iteration diagnostic plots for encoder calibration.

Generates three types of figures:

1. **Raw waveforms** - master and nonius 14-bit ADC signals vs sample index.
2. **Residuals bar chart** - the 8 analog residual values for the current
   iteration with the convergence threshold line.
3. **Residuals trend** - line chart showing how each residual evolves
   across iterations (updated cumulatively).

Each figure is saved as a PNG.
"""

import logging
import math
import tempfile
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
from matplotlib.figure import Figure

logger = logging.getLogger(__name__)

# Convergence threshold
RESIDUAL_THRESHOLD = 1.0

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


def _save_and_show(fig: Figure, path: Path) -> None:
    fig.savefig(path, dpi=150, bbox_inches="tight")
    logger.info(f"Saved plot: {path}")
    plt.close(fig)


def _ensure_backend() -> None:
    """Set the matplotlib backend once, before creating any figures."""
    current = plt.get_backend().lower()
    if current != "agg":
        matplotlib.use("Agg", force=True)


def warm_matplotlib_cache() -> None:
    """Force matplotlib to build its font cache before time-critical work.

    The first call to ``savefig()`` triggers a full font enumeration that
    can block the Python GIL for several seconds.  When PDO exchange is
    active, this stall is long enough to trip the EtherCAT watchdog.
    Calling this function **before** PDO activation eliminates that risk.
    """
    _ensure_backend()
    fig, ax = plt.subplots()
    ax.set_title("warm-up")
    warmup_path = Path(tempfile.gettempdir()) / ".mpl_warmup.png"
    fig.savefig(warmup_path, dpi=50)
    plt.close(fig)
    warmup_path.unlink(missing_ok=True)
    logger.debug("Matplotlib font cache warmed up.")


def _plot_raw_waveforms(
    master_raw: list[int],
    nonius_raw: list[int],
    *,
    encoder: int,
    iteration: int,
    output_dir: Path,
) -> Path:
    """Plot raw master and nonius ADC waveforms.

    Args:
        master_raw: 14-bit master samples.
        nonius_raw: 14-bit nonius samples.
        encoder: Encoder channel number.
        iteration: Current calibration iteration.
        output_dir: Directory for PNG output.

    Returns:
        Path to the saved PNG file.
    """
    _ensure_backend()
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
    _save_and_show(fig, path)
    return path


def _plot_residuals_bar(
    residuals: list[float],
    *,
    encoder: int,
    iteration: int,
    output_dir: Path,
) -> Path:
    """Plot a bar chart of the 8 analog residuals vs the convergence threshold.

    Args:
        residuals: List of 8 residual values [M gx, voss, vosc, ph, N gx, voss, vosc, ph].
        encoder: Encoder channel number.
        iteration: Current calibration iteration.
        output_dir: Directory for PNG output.

    Returns:
        Path to the saved PNG file.
    """
    _ensure_backend()
    abs_residuals = [abs(r) for r in residuals]

    fig, ax = plt.subplots(figsize=(10, 5))
    colors = ["tab:green" if v <= RESIDUAL_THRESHOLD else "tab:red" for v in abs_residuals]
    ax.bar(_RESIDUAL_LABELS, abs_residuals, color=colors)
    ax.axhline(
        RESIDUAL_THRESHOLD, color="black", linestyle="--", linewidth=1, label="Threshold (1.0 LSB)"
    )
    ax.set_ylabel("|Residual| (LSB)")
    ax.set_title(f"Encoder {encoder} - Iteration {iteration}: Residuals")
    ax.legend()

    # Annotate values
    for i, v in enumerate(abs_residuals):
        ax.text(i, v + 0.05, f"{v:.2f}", ha="center", va="bottom", fontsize=8)

    path = output_dir / f"enc{encoder}_iter{iteration}_residuals.png"
    _save_and_show(fig, path)
    return path


def _plot_residuals_trend(
    history: dict[int, list[list[float]]],
    *,
    encoder: int,
    output_dir: Path,
) -> Path:
    """Plot the evolution of each residual across iterations.

    Args:
        history: ``{encoder_number: [[8 residuals iter1], [8 residuals iter2], ...]}``.
        encoder: Encoder channel number.
        output_dir: Directory for PNG output.

    Returns:
        Path to the saved PNG file.
    """
    _ensure_backend()
    data = history[encoder]
    iterations = list(range(1, len(data) + 1))

    fig, ax = plt.subplots(figsize=(10, 6))
    for idx, label in enumerate(_RESIDUAL_LABELS):
        values = [abs(row[idx]) for row in data]
        ax.plot(iterations, values, marker="o", label=label)

    ax.axhline(RESIDUAL_THRESHOLD, color="black", linestyle="--", linewidth=1, label="Threshold")
    ax.set_xlabel("Iteration")
    ax.set_ylabel("|Residual| (LSB)")
    ax.set_title(f"Encoder {encoder} - Residuals Trend")
    ax.set_xticks(iterations)
    ax.legend(fontsize=8, ncol=2)

    path = output_dir / f"enc{encoder}_residuals_trend.png"
    _save_and_show(fig, path)
    return path


def _plot_nonius_track_offset_table(
    encoder: int,
    phase_error: list[int],
    track_offset_curve: list[int],
    phase_margin: list[int],
    single_turn_position: list[float],
    continuous_single_turn_position: list[float],
    nonius_phase_range_limit: int,
    output_dir: Path,
) -> None:

    if (
        not phase_error
        or not track_offset_curve
        or not phase_margin
        or not single_turn_position
        or not continuous_single_turn_position
        or not nonius_phase_range_limit
    ):
        logger.warning(
            f"Encoder {encoder}: nonius curve data is empty; skipping plot. "
            "Did you call optimized_nonius_track_offset_table() first?"
        )
        return

    _ensure_backend()

    single_turns_start_index = []
    single_turns_start_index.append(0)
    single_turns_start_index.extend(
        i
        for i in range(1, int(len(single_turn_position)))
        if abs(single_turn_position[i] - single_turn_position[i - 1]) > (360 / 2)
    )
    single_turns_start_index.append(len(single_turn_position) - 1)

    nonius_curve_fig, (nonius_curve_plot, nonius_curve_continuous_plot) = plt.subplots(
        2, 1, figsize=(16, 9), layout="constrained"
    )
    if nonius_curve_fig.canvas.manager is not None:
        nonius_curve_fig.canvas.manager.set_window_title("Nonius Curves")

    for i in range(1, int(len(single_turns_start_index))):
        x = single_turn_position[
            single_turns_start_index[i - 1] : (single_turns_start_index[i] - 1)
        ]
        label_hidden_prefix = ""
        if i > 1:
            label_hidden_prefix = "_"
        nonius_curve_plot.plot(
            x,
            phase_error[single_turns_start_index[i - 1] : (single_turns_start_index[i] - 1)],
            label=label_hidden_prefix + "Error",
            color="red",
            alpha=0.9,
        )

    for i in range(1, int(len(single_turns_start_index))):
        x = single_turn_position[
            single_turns_start_index[i - 1] : (single_turns_start_index[i] - 1)
        ]
        label_hidden_prefix = ""
        if i > 1:
            label_hidden_prefix = "_"
        nonius_curve_plot.plot(
            x,
            phase_margin[single_turns_start_index[i - 1] : (single_turns_start_index[i] - 1)],
            label=label_hidden_prefix + "Result",
            color="lime",
            alpha=0.9,
        )

    for i in range(1, int(len(single_turns_start_index))):
        x = single_turn_position[
            single_turns_start_index[i - 1] : (single_turns_start_index[i] - 1)
        ]
        label_hidden_prefix = ""
        if i > 1:
            label_hidden_prefix = "_"
        nonius_curve_plot.plot(
            x,
            track_offset_curve[single_turns_start_index[i - 1] : (single_turns_start_index[i] - 1)],
            label=label_hidden_prefix + "SPO",
            color="midnightblue",
            alpha=0.9,
        )

    optimize_number_of_ticks = len(nonius_curve_plot.get_xticks())
    plot_range = max(single_turn_position) - min(single_turn_position)
    optimal_ticker_base = (
        math.pow(2, round(math.log2(plot_range / optimize_number_of_ticks / 90))) * 90
    )
    degree_loc = plticker.MultipleLocator(base=optimal_ticker_base)
    nonius_curve_plot.xaxis.set_major_locator(degree_loc)

    nonius_curve_plot.set_title("Nonius Curve")
    nonius_curve_plot.legend(loc="upper right")
    nonius_curve_plot.set_xlabel("Reference Angle (degrees)")
    nonius_curve_plot.set_ylabel("Track Error (LSB)")
    nonius_curve_plot.axhline(nonius_phase_range_limit, linewidth=1.0, ls="-", color="cyan")
    nonius_curve_plot.axhline(-nonius_phase_range_limit, linewidth=1.0, ls="-", color="cyan")

    nonius_curve_plot.axhline(max(phase_error), linewidth=0.5, ls="-", color="red")
    nonius_curve_plot.axhline(min(phase_error), linewidth=0.5, ls="-", color="red")

    nonius_curve_plot.axhline(max(phase_margin), linewidth=0.5, ls="-", color="lime")
    nonius_curve_plot.axhline(min(phase_margin), linewidth=0.5, ls="-", color="lime")

    nonius_curve_continuous_plot.plot(
        continuous_single_turn_position, phase_error, label="Error", color="red", alpha=0.9
    )
    nonius_curve_continuous_plot.plot(
        continuous_single_turn_position, phase_margin, label="Result", color="lime", alpha=0.9
    )
    nonius_curve_continuous_plot.plot(
        continuous_single_turn_position,
        track_offset_curve,
        label="SPO",
        color="midnightblue",
        alpha=0.9,
    )
    nonius_curve_continuous_plot.set_title("Continuous Nonius Curve")
    nonius_curve_continuous_plot.legend(loc="upper right")
    nonius_curve_continuous_plot.set_xlabel("Reference Angle (degrees)")
    nonius_curve_continuous_plot.set_ylabel("Track Error (LSB)")
    nonius_curve_continuous_plot.axhline(
        nonius_phase_range_limit, linewidth=1.0, ls="-", color="cyan"
    )
    nonius_curve_continuous_plot.axhline(
        -nonius_phase_range_limit, linewidth=1.0, ls="-", color="cyan"
    )

    path = output_dir / f"enc{encoder}_nonius_curve.png"
    _save_and_show(nonius_curve_fig, path)
