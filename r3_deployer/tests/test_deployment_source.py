"""Offline coverage for HTTPS package validation and local deployment selection."""

import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from urllib.error import URLError
from urllib.parse import unquote, urlsplit
from urllib.request import Request

from r3_deployer import deploy, remote, settings


def _bundle_manifest(files):
    return {'schema':'r3.pi-deploy.v1','owner_directory':remote.OWNER_DIR.as_posix(),
            'files':{name:hashlib.sha256(data).hexdigest() for name,data in files.items()}}


class _HttpsResponse(io.BytesIO):
    def __init__(self, content, length=None):
        super().__init__(content)
        self.headers = {'Content-Length':str(length if length is not None else len(content))}


class _HttpsSource:
    def __init__(self, files):
        self.files = files
        self.requests = []

    def open(self, request, timeout):
        path = unquote(urlsplit(request.full_url).path).removeprefix('/bundle/')
        self.requests.append((path,request.get_header('Authorization')))
        return _HttpsResponse(self.files[path])


class DeploymentSourceTests(unittest.TestCase):
    def setUp(self):
        self.files = {'app/run_auto.py':b'print("main")\n','app/task_runner.py':b'pass\n',
                      'owner/'+remote.OWNER_NAME:b'# owner payload\n'}
        self.manifest = _bundle_manifest(self.files)
        self.values = dict(settings.DEFAULTS,remote_url='https://packages.example/bundle/manifest.json')

    def _opener(self, *, files=None):
        return _HttpsSource({'manifest.json':json.dumps(self.manifest).encode(),**(files or self.files)})

    def test_saved_old_settings_default_to_remote_and_keep_local_fallback(self):
        with patch.object(settings,'_read',return_value={'local_path':'D:/old_robot_bundle'}):
            values = settings.load()
        self.assertEqual(values['deployment_source'],'remote')
        self.assertEqual(values['remote_url'],settings.DEFAULT_REMOTE_URL)
        self.assertEqual(values['local_path'],'D:/old_robot_bundle')
        self.assertEqual(settings.validate(dict(values,deployment_source='local',remote_url=''))['local_path'],
                         'D:/old_robot_bundle')

    def test_remote_url_formats_normalize_and_require_https(self):
        hosted = 'https://raw.githubusercontent.com/owner/project'
        for original, expected in (
            ('https://packages.example/bundle/', 'https://packages.example/bundle'),
            ('https://packages.example/bundle/manifest.json', 'https://packages.example/bundle'),
            ('https://github.com/owner/project.git', hosted+'/main/pi_deploy_bundle'),
            ('https://github.com/owner/project/tree/dev/releases/pi', hosted+'/dev/releases/pi'),
            ('https://github.com/owner/project/blob/dev/releases/pi/manifest.json', hosted+'/dev/releases/pi'),
        ):
            with self.subTest(original=original):
                self.assertEqual(settings.RemoteBundle_NormalizeUrl(original),expected)
        for original in ('http://packages.example/bundle','https://user@packages.example/bundle',
                         'https://packages.example/../bundle','https://packages.example/%2e%2e/bundle'):
            with self.subTest(original=original), self.assertRaises(ValueError):
                settings.RemoteBundle_NormalizeUrl(original)
        redirect = deploy._HttpsRedirect()
        with self.assertRaises(URLError):
            redirect.redirect_request(Request('https://packages.example/bundle'),None,302,'Found',{},
                                      'http://packages.example/bundle')

    def test_remote_bundle_fetches_only_manifest_members_and_ignores_local_path(self):
        opener = self._opener()
        logs = []
        with patch.object(deploy,'build_opener',return_value=opener), patch.object(
                deploy,'_local_package',side_effect=AssertionError('remote should not open or build local files')):
            manifest, files = deploy._package(dict(self.values,local_path=''),logs.append)
        self.assertEqual(json.loads(manifest),self.manifest)
        self.assertEqual(files,self.files)
        self.assertCountEqual(opener.requests,[(name,None) for name in ('manifest.json',*self.files)])
        self.assertIn('远程包下载并校验完成，共 3 个文件。',logs)

    def test_invalid_remote_manifest_is_rejected_before_downloading_payload(self):
        for key, digest in (('app/../unrelated.txt','0'*64), ('app/file',0), ('app','0'*64)):
            with self.subTest(key=key):
                self.manifest['files'][key] = digest
                opener = self._opener()
                with patch.object(deploy,'build_opener',return_value=opener), self.assertRaises(ValueError):
                    deploy._package(self.values,lambda _:None)
                self.assertEqual(opener.requests,[('manifest.json',None)])
                del self.manifest['files'][key]

    def test_wrong_remote_hash_blocks_deployment_before_ssh_and_does_not_fall_back(self):
        opener = self._opener(files={**self.files,'app/task_runner.py':b'outdated content'})
        with patch.object(deploy,'build_opener',return_value=opener), patch.object(
                deploy,'_connect',side_effect=AssertionError('SSH must wait for a verified payload')), patch.object(
                deploy,'_local_package',side_effect=AssertionError('explicit selection must be respected')):
            with self.assertRaisesRegex(ValueError,'哈希不匹配'):
                deploy.operate('deploy',self.values,'test-password',lambda _:None)

    def test_local_source_reads_the_configured_directory_without_network(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            (folder/'manifest.json').write_text(json.dumps(self.manifest),encoding='utf-8')
            for name,content in self.files.items():
                path = folder/name
                path.parent.mkdir(parents=True,exist_ok=True)
                path.write_bytes(content)
            values = dict(self.values,deployment_source='local',local_path=temporary,remote_url='')
            with patch.object(deploy,'build_opener',side_effect=AssertionError('local deployment must stay offline')):
                _, files = deploy._package(settings.validate(values),lambda _:None)
            self.assertEqual(files,self.files)

    def test_download_limit_does_not_trust_content_length(self):
        for length in (0, 10):
            with self.subTest(declared_length=length), patch.object(
                    deploy,'monotonic',return_value=0), self.assertRaisesRegex(ValueError,'大小限制'):
                opener = type('Opener',(),{'open':lambda *_args,**_kwargs:_HttpsResponse(b'0123456789',length)})()
                deploy._remote_bundle_read(opener,'https://packages.example/bundle','manifest.json',5,1,'')
