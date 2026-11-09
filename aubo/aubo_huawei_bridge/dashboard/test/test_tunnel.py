from contextlib import redirect_stdout
import importlib.machinery
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock, patch

ROOT=Path(__file__).resolve().parents[2]
loader=importlib.machinery.SourceFileLoader('aubo_tunnel_test',str(ROOT/'scripts/huawei_tunnel'))
module=loader.load_module()


class TunnelTests(unittest.TestCase):
    def test_native_config_reuse_ros_remapping_and_private_child_proxy_environment(self):
        with tempfile.TemporaryDirectory() as tmp:
            home=Path(tmp)
            native=home/'.cpolar/cpolar.yml'
            native.parent.mkdir()
            token='TEST_ONLY_PRIVATE_ACCOUNT_TOKEN'
            native.write_text('authtoken: '+token+'\n')
            native.chmod(0o600)
            binary=home/'.local/bin/cpolar'
            binary.parent.mkdir(parents=True)
            binary.touch()
            response=MagicMock()
            response.headers={'X-Aubo-Workstation':'ros'}
            opener=MagicMock()
            opener.open.return_value=response
            output=io.StringIO()
            with patch.object(module.Path,'home',return_value=home), patch.object(module.shutil,'which',return_value=None), \
                 patch.object(module.sys,'argv',['huawei_tunnel','__name:=node','__log:=log']), \
                 patch.object(module,'build_opener',return_value=opener), patch.object(module.os,'execve') as execute, \
                 patch.dict(module.os.environ,{'HTTPS_PROXY':'http://test-proxy.invalid:1'}), redirect_stdout(output):
                module.main()
                self.assertIn('HTTPS_PROXY',module.os.environ)
            args=execute.call_args[0]
            self.assertEqual(args[0],str(binary))
            self.assertIn(str(native),args[1])
            self.assertIn('127.0.0.1:8099',args[1])
            self.assertIn('127.0.0.1:4040',args[1])
            self.assertNotIn('HTTPS_PROXY',args[2])
            self.assertNotIn(token,output.getvalue())
            self.assertNotIn(token,' '.join(args[1]))

    def test_configuration_paste_is_private_and_never_overwrites_an_account(self):
        with tempfile.TemporaryDirectory() as tmp:
            home=Path(tmp)
            output=io.StringIO()
            with patch.object(module.Path,'home',return_value=home), \
                 patch.object(module.sys,'argv',['huawei_tunnel','--configure']), \
                 patch.object(module.sys.stdin,'isatty',return_value=True), \
                 patch.object(module.getpass,'getpass',return_value='TEST_ONLY_SECRET_TOKEN'),redirect_stdout(output):
                module.main()
                with self.assertRaises(FileExistsError):
                    module.main()
            target=home/'.config/aubo/huawei/cpolar.yml'
            self.assertEqual(target.stat().st_mode & 0o777,0o600)
            self.assertNotIn('TEST_ONLY_SECRET_TOKEN',output.getvalue())


if __name__=='__main__':
    unittest.main()
