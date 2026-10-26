"""Catch broken installed resources and include arguments after package relocation."""
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


class PackageResourcesTest(unittest.TestCase):
    def test_legacy_publisher_has_no_second_optical_transform(self):
        path = ROOT.parents[1] / 'easy_handeye/easy_handeye/launch/aubo_realsense_publish_only.launch'
        tree = ET.parse(path)
        args = {arg.get('name'): arg.get('default') for arg in tree.findall('arg')}
        self.assertEqual(args['tracking_base_frame'], 'camera_color_optical_frame')
        self.assertEqual(tree.findall('node'), [])
        self.assertEqual(len(tree.findall('include')), 1)

    def test_local_scripts_configs_and_document_links_resolve(self):
        cmake = (ROOT/'CMakeLists.txt').read_text()
        for script in (ROOT/'scripts').glob('*.py'):
            self.assertIn('scripts/'+script.name, cmake)
        for document in [ROOT/'README.md'] + list((ROOT/'docs').glob('*.md')):
            for link in re.findall(r'\[[^\]]+\]\(([^)]+)\)', document.read_text(encoding='utf-8')):
                if '://' not in link and not link.startswith('#'):
                    self.assertTrue((document.parent/link.split('#')[0]).is_file(),
                                    '{}: {}'.format(document.name, link))
        for launch in (ROOT/'launch').glob('*.launch'):
            tree = ET.parse(launch)
            for node in tree.findall('.//node'):
                if node.get('pkg') == 'aubo_handeye_validation':
                    self.assertTrue((ROOT/'scripts'/node.get('type')).is_file())
            for item in tree.iter():
                for value in item.attrib.values():
                    match = re.fullmatch(r'\$\(find aubo_handeye_validation\)/([^$]+)', value)
                    if match:
                        self.assertTrue((ROOT/match.group(1)).is_file(), value)

    def test_dependencies_and_direct_include_arguments_match(self):
        manifest = ET.parse(ROOT/'package.xml')
        dependencies = {item.text for item in manifest.findall('exec_depend')}
        self.assertFalse(dependencies & {'aubo_sorting', 'aubo_sorting_core'})
        for launch in (ROOT/'launch').glob('*.launch'):
            tree = ET.parse(launch)
            for node in tree.findall('.//node'):
                if node.get('pkg') != 'aubo_handeye_validation':
                    self.assertIn(node.get('pkg'), dependencies)
            for include in tree.findall('.//include'):
                match = re.fullmatch(r'\$\(find ([^)]+)\)/(.*)', include.get('file'))
                self.assertIsNotNone(match)
                package, path = match.groups()
                self.assertIn(package, dependencies)
                source = ROOT.parent/package/path
                if package.startswith('aubo_'):
                    self.assertTrue(source.is_file(), source)
                    declared = {arg.get('name') for arg in ET.parse(source).findall('arg')}
                    for passed in include.findall('arg'):
                        self.assertIn(passed.get('name'), declared, str(source))


if __name__ == '__main__':
    unittest.main()
