"""Guards the packaging trap that stopped this package from ever launching.

This is not a style check. `nidar_detection/package.xml` once contained a
doubled ASCII hyphen inside an XML comment, which is illegal XML. Nothing
reported an error anywhere:

  * colcon built the package successfully,
  * `ros2 pkg list` showed it,
  * the install tree was complete and correct,

but colcon could not parse the manifest, so it identified the package as
plain `python` instead of `ros.ament_python` (`colcon list` prints the build
type it chose). The `ament_python` build task is the ONLY thing that adds the
`ament_prefix_path` environment hook, so without it the package never reached
AMENT_PREFIX_PATH, and every `ros2 launch nidar_detection ...` failed with
"package 'nidar_detection' not found" -- listing a search path that simply
did not include it.

The visible symptom was three sessions of "the detection nodes produce no
topics", investigated as a DDS problem, a camera problem, and a model
problem. It was a punctuation problem.

So: assert the manifests parse, and assert the export block that decides the
build type is intact. Both are cheap; the failure they prevent is not.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

#: The workspace's src/ directory: .../src/nidar_detection/tests/this_file.py
SRC_DIR = Path(__file__).resolve().parents[2]


def _nidar_manifests():
    if not SRC_DIR.is_dir():
        return []
    return sorted(SRC_DIR.glob('nidar_*/package.xml'))


def _manifest_ids(paths):
    return [p.parent.name for p in paths]


MANIFESTS = _nidar_manifests()


@pytest.mark.skipif(not MANIFESTS, reason='not running from a source checkout')
@pytest.mark.parametrize('manifest', MANIFESTS, ids=_manifest_ids(MANIFESTS))
def test_package_manifest_is_well_formed_xml(manifest):
    """An unparseable manifest downgrades the package's build type silently."""
    try:
        ET.parse(manifest)
    except ET.ParseError as e:
        pytest.fail(
            f'{manifest.parent.name}/package.xml is not well-formed XML: {e}. '
            f'The usual cause is a doubled ASCII hyphen inside an XML comment, '
            f'which is illegal there -- reword the comment. This does not fail '
            f'the build; it makes ros2 launch/run unable to find the package.')


@pytest.mark.skipif(not MANIFESTS, reason='not running from a source checkout')
@pytest.mark.parametrize('manifest', MANIFESTS, ids=_manifest_ids(MANIFESTS))
def test_package_manifest_declares_a_build_type(manifest):
    """The <export><build_type> is what selects colcon's ament build task,
    and that task is what installs the ament_prefix_path hook."""
    root = ET.parse(manifest).getroot()
    build_types = [b.text.strip() for b in root.findall('./export/build_type')
                   if b.text]
    assert build_types, (
        f'{manifest.parent.name}/package.xml has no <export><build_type>; '
        f'colcon would fall back to a non-ament build type and skip the '
        f'ament_prefix_path hook.')
    assert build_types[0] in ('ament_python', 'ament_cmake'), \
        f'{manifest.parent.name}: unexpected build_type {build_types[0]!r}'


@pytest.mark.skipif(not MANIFESTS, reason='not running from a source checkout')
@pytest.mark.parametrize('manifest', MANIFESTS, ids=_manifest_ids(MANIFESTS))
def test_package_manifest_name_matches_its_directory(manifest):
    """A name/directory mismatch breaks the ament index marker path, which is
    the other half of what makes a package findable at runtime."""
    root = ET.parse(manifest).getroot()
    declared = root.findtext('name', '').strip()
    assert declared == manifest.parent.name, \
        f'{manifest.parent.name}/package.xml declares name {declared!r}'
