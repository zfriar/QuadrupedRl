import json
import pathlib
import shutil

import pytest


def _ensure_removed(path: pathlib.Path):
    if path.exists():
        if path.is_file():
            path.unlink()
        else:
            shutil.rmtree(path)


@pytest.mark.parametrize(
    "xml_content, expected_color, expected_pct_range",
    [
        (
            "<?xml version='1.0'?><coverage line-rate='0.75'></coverage>",
            "red",
            (74, 76),
        ),
        (
            "<?xml version='1.0'?><coverage line-rate='0.85'></coverage>",
            "yellow",
            (84, 86),
        ),
        (
            "<?xml version='1.0'?><coverage line-rate='0.92'></coverage>",
            "yellowgreen",
            (91, 93),
        ),
        (
            "<?xml version='1.0'?><coverage line-rate='0.97'></coverage>",
            "brightgreen",
            (96, 98),
        ),
    ],
)
def test_badge_color_branches(
    tmp_path, xml_content, expected_color, expected_pct_range
):
    """Ensure the badge script maps coverage percentages to the correct colors."""
    repo_root = pathlib.Path.cwd()
    target = repo_root / "coverage.xml"
    badges_dir = repo_root / "badges"

    _ensure_removed(target)
    _ensure_removed(badges_dir)

    target.write_text(xml_content)

    # import the module and run main
    import importlib

    mod = importlib.import_module("scripts.make_coverage_badge")
    mod.main()

    payload = json.loads((badges_dir / "coverage.json").read_text())
    assert payload["color"] == expected_color
    pct = int(payload["message"].rstrip("%"))
    assert expected_pct_range[0] <= pct <= expected_pct_range[1]

    _ensure_removed(badges_dir)
    _ensure_removed(target)


def test_fallback_line_rate_on_nested_element(tmp_path):
    """If root has no line-rate but a nested element does, use it."""
    repo_root = pathlib.Path.cwd()
    target = repo_root / "coverage.xml"
    badges_dir = repo_root / "badges"

    _ensure_removed(target)
    _ensure_removed(badges_dir)

    # place the line-rate on a nested package element
    xml = "<?xml version='1.0'?><coverage><packages><package line-rate='0.80' /></packages></coverage>"
    target.write_text(xml)

    import importlib

    mod = importlib.import_module("scripts.make_coverage_badge")
    mod.main()
    payload = json.loads((badges_dir / "coverage.json").read_text())
    # 80% => falls into the yellow branch (>=80 and <90)
    assert payload["color"] == "yellow"

    _ensure_removed(badges_dir)
    _ensure_removed(target)


def test_malformed_line_rate_falls_back_to_zero(tmp_path):
    """Non-numeric line-rate should result in 0% and red color."""
    repo_root = pathlib.Path.cwd()
    target = repo_root / "coverage.xml"
    badges_dir = repo_root / "badges"

    _ensure_removed(target)
    _ensure_removed(badges_dir)

    target.write_text(
        "<?xml version='1.0'?><coverage line-rate='not-a-number'></coverage>"
    )

    import importlib

    mod = importlib.import_module("scripts.make_coverage_badge")
    mod.main()
    payload = json.loads((badges_dir / "coverage.json").read_text())
    assert payload["color"] == "red"
    assert payload["message"] == "0%"

    _ensure_removed(badges_dir)
    _ensure_removed(target)


def test_script_prints_written_message(tmp_path, capsys):
    """Ensure the script prints a confirmation message when writing the badge."""
    repo_root = pathlib.Path.cwd()
    target = repo_root / "coverage.xml"
    badges_dir = repo_root / "badges"

    _ensure_removed(target)
    _ensure_removed(badges_dir)

    target.write_text("<?xml version='1.0'?><coverage line-rate='0.99'></coverage>")

    import importlib

    mod = importlib.import_module("scripts.make_coverage_badge")
    mod.main()

    captured = capsys.readouterr()
    assert "Wrote" in captured.out
    assert (badges_dir / "coverage.json").exists()

    _ensure_removed(badges_dir)
    _ensure_removed(target)


def test_make_coverage_badge_creates_json(tmp_path):
    # create a small coverage.xml with line-rate attribute
    cov_xml = tmp_path / "coverage.xml"
    cov_xml.write_text(
        """<?xml version='1.0'?>
<coverage line-rate="0.875"></coverage>
"""
    )

    # copy it to repo root (script reads coverage.xml in cwd)
    repo_root = pathlib.Path.cwd()
    target = repo_root / "coverage.xml"
    _ensure_removed(target)
    cov_xml.replace(target)

    # ensure badges dir removed
    badges_dir = repo_root / "badges"
    _ensure_removed(badges_dir)

    # import and call the script's main function so coverage is measured
    import scripts
    import scripts.make_coverage_badge

    scripts.make_coverage_badge.main()

    out = badges_dir / "coverage.json"
    assert out.exists()
    payload = json.loads(out.read_text())
    assert payload["schemaVersion"] == 1
    assert payload["label"] == "coverage"
    # 0.875 -> 88%
    assert payload["message"].endswith("%")
    pct = int(payload["message"].rstrip("%"))
    assert 87 <= pct <= 89

    # cleanup generated coverage.xml
    _ensure_removed(target)
    _ensure_removed(badges_dir)


def test_make_coverage_badge_color_branches(tmp_path):
    """Cover additional color/branch logic in the badge script."""
    repo_root = pathlib.Path.cwd()
    target = repo_root / "coverage.xml"
    badges_dir = repo_root / "badges"

    import scripts
    import scripts.make_coverage_badge

    # case: very high coverage -> brightgreen
    _ensure_removed(target)
    _ensure_removed(badges_dir)
    target.write_text("<?xml version='1.0'?><coverage line-rate='0.97'></coverage>")
    scripts.make_coverage_badge.main()
    payload = json.loads((badges_dir / "coverage.json").read_text())
    assert payload["color"] == "brightgreen"

    # cleanup
    _ensure_removed(badges_dir)

    # case: malformed/missing line-rate -> fallback/exc -> pct == 0 -> red
    target.write_text("<?xml version='1.0'?><coverage></coverage>")
    scripts.make_coverage_badge.main()
    payload = json.loads((badges_dir / "coverage.json").read_text())
    assert payload["color"] == "red"

    # cleanup
    _ensure_removed(target)
    _ensure_removed(badges_dir)


def test_make_coverage_badge_missing_file(tmp_path):
    # Ensure SystemExit occurs when coverage.xml missing
    repo_root = pathlib.Path.cwd()
    target = repo_root / "coverage.xml"
    _ensure_removed(target)

    import scripts
    import scripts.make_coverage_badge

    with pytest.raises(SystemExit) as exc:
        scripts.make_coverage_badge.main()
    assert exc.value.code == 2
