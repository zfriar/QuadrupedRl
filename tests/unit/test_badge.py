import json
import pathlib

import pytest


def _ensure_removed(path: pathlib.Path):
    if path.exists():
        if path.is_file():
            path.unlink()
        else:
            for f in path.iterdir():
                f.unlink()
            path.rmdir()


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
