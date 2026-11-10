"""The remote workstation exposes telemetry and projection, never camera frames."""
import os
import re
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from app import App


class ProjectionOnlyTests(unittest.TestCase):
    def test_projection_page_retains_the_elements_used_by_its_controller(self):
        root = Path(__file__).resolve().parents[1] / 'dist'
        html = (root / 'index.html').read_text()
        controller = (root / 'app.js').read_text()
        identifiers = set(re.findall(r'id="([^"]+)"', html))
        referenced = set(re.findall(r"(?:text\(|\$\()'([^']+)'", controller))
        self.assertEqual(referenced - identifiers, set())

    def test_camera_is_not_exposed_even_if_a_provider_offers_it(self):
        invoked = []
        provider = SimpleNamespace(camera_enabled=True, camera=lambda: invoked.append(True))
        with patch.dict(os.environ, {'AUBO_DASHBOARD_MODE': 'ros',
            'DASHBOARD_PASSWORD': 'private-test-password-long', 'DASHBOARD_SESSION_KEY': 's' * 40}):
            application = App(provider)
        token = application.sign('login', {'exp': application.clock() + 60})
        headers = {'Authorization': 'Bearer ' + token}
        self.assertEqual(application.route('GET', '/api/camera', headers)[0], 404)
        self.assertEqual(invoked, [])
        _, _, html = application.route('GET', '/')
        self.assertNotIn(b'camera-image', html)
        self.assertIn(b'id="map"', html)


if __name__ == '__main__':
    unittest.main()
