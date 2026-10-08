import pytest

from ic_haus_magnetic_encoder_calibration.plotting import (
    PREVIOUS_RUN_DIR_NAME,
    RESIDUAL_THRESHOLD,
    _plot_nonius_track_offset_table,
    _plot_raw_waveforms,
    _plot_residuals_bar,
    _plot_residuals_trend,
    ensure_output_dir_writable,
    prepare_output_dir,
    warm_matplotlib_cache,
)


class TestOutputDir:
    def test_raises_clear_error_when_os_reports_directory_not_writable(
        self, tmp_path, monkeypatch
    ) -> None:
        """Test that a clear error is raised when the output directory is not writable."""
        output_dir = tmp_path / "calibration_output"
        output_dir.mkdir()
        monkeypatch.setattr(
            "ic_haus_magnetic_encoder_calibration.plotting.os.access",
            lambda *_: False,
        )

        with pytest.raises(SystemExit, match="is not writable."):
            ensure_output_dir_writable(output_dir)

    def test_creates_missing_output_directory(self, tmp_path) -> None:
        """Test that the output directory is created if it does not exist."""
        output_dir = tmp_path / "calibration_output"
        prepare_output_dir(output_dir)

        # Check that the output directory was created
        assert output_dir.is_dir()
        # Check that no archived-data-folder was created
        assert not (output_dir / PREVIOUS_RUN_DIR_NAME).exists()

    def test_archives_existing_contents(self, tmp_path) -> None:
        """Test that existing contents are archived.

        Files from a previous run exist on the folder, but no archived files folder exists.

        """
        # Create initial output directory with contents but no archived data nor folder
        output_dir = tmp_path / "calibration_output"
        output_dir.mkdir()
        (output_dir / "old_result.json").write_text("old", encoding="utf-8")

        # Run the function to prepare the output directory
        prepare_output_dir(output_dir)

        # Check that the previous run contents have been moved correctly
        previous_run_dir = output_dir / PREVIOUS_RUN_DIR_NAME
        assert not (output_dir / "old_result.json").exists()
        assert (previous_run_dir / "old_result.json").read_text(encoding="utf-8") == "old"

    def test_prior_archived_files_get_overwritten(self, tmp_path) -> None:
        """Test that existing contents are archived.

        Files from a previous run exist on the folder AND the archived files folder exists.

        """
        # Create initial output directory with contents
        output_dir = tmp_path / "calibration_output"
        output_dir.mkdir()
        (output_dir / "old_result.json").write_text("old", encoding="utf-8")
        # Create archived data folder with files
        previous_run_dir = output_dir / PREVIOUS_RUN_DIR_NAME
        previous_run_dir.mkdir()
        (previous_run_dir / "stale_result.json").write_text("stale", encoding="utf-8")

        # Run the function to prepare the output directory
        prepare_output_dir(output_dir)

        # Check that the previous run contents have been moved correctly and no old files remain
        assert not (output_dir / "old_result.json").exists()
        assert (previous_run_dir / "old_result.json").read_text(encoding="utf-8") == "old"
        assert not (previous_run_dir / "stale_result.json").exists()

    def test_raises_clear_error_when_prepare_output_dir_fails(self, tmp_path, monkeypatch) -> None:
        """Test that a clear error is raised when the output directory preparation fails."""
        output_dir = tmp_path / "calibration_output"
        output_dir.mkdir()
        monkeypatch.setattr(
            "ic_haus_magnetic_encoder_calibration.plotting.os.access",
            lambda *_: False,
        )

        with pytest.raises(SystemExit, match="is not writable."):
            prepare_output_dir(output_dir)


class TestPlotRawWaveforms:
    def test_saves_png(self, tmp_path) -> None:
        master_raw = [100, 200, 300, 400]
        nonius_raw = [50, 60, 70, 80]

        path = _plot_raw_waveforms(
            master_raw, nonius_raw, encoder=1, iteration=2, output_dir=tmp_path
        )

        assert path == tmp_path / "enc1_iter2_raw.png"
        assert path.exists()

    def test_filename_uses_encoder_and_iteration(self, tmp_path) -> None:
        path = _plot_raw_waveforms([1, 2], [3, 4], encoder=5, iteration=9, output_dir=tmp_path)

        assert path.name == "enc5_iter9_raw.png"


class TestPlotResidualsBar:
    def test_saves_png(self, tmp_path) -> None:
        residuals = [0.1, -0.2, 0.3, -1.5, 0.05, 0.6, -0.7, 2.0]

        path = _plot_residuals_bar(residuals, encoder=1, iteration=1, output_dir=tmp_path)

        assert path == tmp_path / "enc1_iter1_residuals.png"
        assert path.exists()

    def test_handles_all_zero_residuals(self, tmp_path) -> None:
        residuals = [0.0] * 8

        path = _plot_residuals_bar(residuals, encoder=2, iteration=3, output_dir=tmp_path)

        assert path.exists()

    def test_handles_values_above_threshold(self, tmp_path) -> None:
        residuals = [RESIDUAL_THRESHOLD + 1] * 8

        path = _plot_residuals_bar(residuals, encoder=3, iteration=1, output_dir=tmp_path)

        assert path.exists()


class TestPlotResidualsTrend:
    def test_saves_png(self, tmp_path) -> None:
        history = {
            1: [
                [0.1, -0.2, 0.3, -1.5, 0.05, 0.6, -0.7, 2.0],
                [0.05, -0.1, 0.2, -0.5, 0.02, 0.3, -0.4, 1.0],
            ]
        }

        path = _plot_residuals_trend(history, encoder=1, output_dir=tmp_path)

        assert path == tmp_path / "enc1_residuals_trend.png"
        assert path.exists()

    def test_single_iteration_history(self, tmp_path) -> None:
        history = {4: [[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]]}

        path = _plot_residuals_trend(history, encoder=4, output_dir=tmp_path)

        assert path.exists()


class TestPlotNoniusTrackOffsetTable:
    def test_saves_png(self, tmp_path) -> None:
        phase_error = [0.5, -0.3, 0.2, -0.1, 0.4, -0.2]
        track_offset_curve = [1.0, 1.1, 0.9, 1.0, 1.05, 0.95]
        phase_margin = [0.1, 0.2, 0.1, 0.15, 0.12, 0.18]
        single_turn_position = [0.0, 60.0, 120.0, 180.0, 240.0, 300.0]
        continuous_single_turn_position = [0.0, 60.0, 120.0, 180.0, 240.0, 300.0]

        _plot_nonius_track_offset_table(
            encoder=1,
            phase_error=phase_error,
            track_offset_curve=track_offset_curve,
            phase_margin=phase_margin,
            single_turn_position=single_turn_position,
            continuous_single_turn_position=continuous_single_turn_position,
            nonius_phase_range_limit=1,
            output_dir=tmp_path,
        )

        assert (tmp_path / "enc1_nonius_curve.png").exists()

    @pytest.mark.parametrize(
        "missing_arg",
        [
            "phase_error",
            "track_offset_curve",
            "phase_margin",
            "single_turn_position",
            "continuous_single_turn_position",
        ],
    )
    def test_skips_plot_when_data_is_empty(self, tmp_path, missing_arg) -> None:
        kwargs: dict[str, object] = {
            "phase_error": [0.5, -0.3],
            "track_offset_curve": [1.0, 1.1],
            "phase_margin": [0.1, 0.2],
            "single_turn_position": [0.0, 60.0],
            "continuous_single_turn_position": [0.0, 60.0],
            "nonius_phase_range_limit": 1,
        }
        kwargs[missing_arg] = []

        _plot_nonius_track_offset_table(encoder=1, output_dir=tmp_path, **kwargs)

        assert not any(tmp_path.iterdir())

    def test_skips_plot_when_phase_range_limit_is_zero(self, tmp_path) -> None:
        _plot_nonius_track_offset_table(
            encoder=1,
            phase_error=[0.5, -0.3],
            track_offset_curve=[1.0, 1.1],
            phase_margin=[0.1, 0.2],
            single_turn_position=[0.0, 60.0],
            continuous_single_turn_position=[0.0, 60.0],
            nonius_phase_range_limit=0,
            output_dir=tmp_path,
        )

        assert not any(tmp_path.iterdir())


class TestWarmMatplotlibCache:
    def test_does_not_raise(self) -> None:
        warm_matplotlib_cache()
