"""The nose-lift module flies PX4's Takeoff and Land as modes registered with the commander (NLF_MODES). In the
heartbeat such a mode is only "Auto, sub-mode 10 + n"; the module names it when it registers, and the app has to show
that name rather than "Auto12" on the mode pill."""
from pathlib import Path

import yaml

from airframe_designer.px4.link import EXTERNAL_MODE_LINE
from airframe_designer.server.app import mode_name

MODULE = Path(__file__).parents[1] / "firmware" / "atlas" / "src" / "atlas_nose_lift"


def custom(main, sub=0):
    return (main << 16) | (sub << 24)


def test_an_external_mode_is_named_by_what_registered_it():
    named = {}
    # at registration, and again on entering the mode for a link that was not there at boot
    for line in ("ATLAS Land (external mode 1) replaces Land", "ATLAS Takeoff (external mode 2)"):
        assert len(line) <= 50, "one STATUSTEXT"
        m = EXTERNAL_MODE_LINE.search(line)
        named[int(m.group(2))] = m.group(1)
    assert mode_name(custom(4, 11), named) == "ATLAS Land"
    assert mode_name(custom(4, 12), named) == "ATLAS Takeoff"
    assert mode_name(custom(4, 13), named) == "External 3", "an external mode nobody named"
    assert mode_name(custom(4, 12)) == "External 2", "a link that missed the registration line"
    assert mode_name(custom(4, 6), named) == "Land" and mode_name(custom(4, 2)) == "Takeoff"
    assert mode_name(custom(3)) == "Position"


def test_the_module_says_the_line_the_app_reads():
    src = (MODULE / "AtlasNoseLift.cpp").read_text()
    assert '"%s (external mode %d) replaces %s"' in src and '"%s (external mode %d)"' in src
    assert "enable_replace_internal_mode=true" in src, "the modes replace PX4's, they are not extra ones"


def test_the_modes_are_on_by_default_and_can_be_switched_off():
    p = yaml.safe_load((MODULE / "params.yaml").read_text())["parameters"][0]["definitions"]["NLF_MODES"]
    assert p["default"] == 1 and p["min"] == 0 and p["max"] == 1
