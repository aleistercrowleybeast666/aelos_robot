"""Offline coverage for HTTPS package validation and local deployment selection."""

import hashlib
import io
import json
from pathlib import Path
import stat
import tempfile
import unittest
from unittest.mock import Mock, patch
from urllib.error import HTTPError, URLError
from urllib.parse import unquote, urlsplit
from urllib.request import Request
from zipfile import ZipFile,ZipInfo

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
                      'app/warm_client.py':b'# control entry\n',
                      'owner/'+remote.OWNER_NAME:b'# owner payload\n'}
        self.manifest = _bundle_manifest(self.files)
        self.values = dict(settings.DEFAULTS,remote_url='https://packages.example/bundle/manifest.json')

    def _opener(self, *, files=None):
        return _HttpsSource({'manifest.json':json.dumps(self.manifest).encode(),**(files or self.files)})

    def _archive(self, *, overrides=None, extra=None, symlink=None):
        artifact = io.BytesIO()
        payload = {'manifest.json':json.dumps(self.manifest).encode(),**self.files,**(overrides or {}),
                   **(extra or {})}
        with ZipFile(artifact,'w') as archive:
            archive.writestr('aelos_robot-main/README.md',b'Unrelated to the deployment bundle.')
            for name,data in payload.items():
                entry = ZipInfo(deploy._GITHUB_ARCHIVE_BUNDLE+name)
                if name == symlink:
                    entry.create_system = 3
                    entry.external_attr = (stat.S_IFLNK|0o777)<<16
                archive.writestr(entry,data)
        return artifact.getvalue()

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
        self.assertIn('远程包下载并校验完成，共 4 个文件。',logs)

    def test_status_entry_is_required_even_with_a_consistent_manifest(self):
        missing = dict(self.files)
        del missing['app/warm_client.py']
        with self.assertRaisesRegex(ValueError, 'warm_client.py'):
            deploy._manifest_files(json.dumps(_bundle_manifest(missing)))
        with self.assertRaisesRegex(ValueError, '为空.*warm_client.py'):
            deploy._verify_bundle_file(b'', 'app/warm_client.py', hashlib.sha256(b'').hexdigest())

    def test_default_github_reads_one_archive_without_fetching_unrelated_files(self):
        opener = Mock()
        opener.open.return_value = _HttpsResponse(self._archive())
        with patch.object(deploy,'build_opener',return_value=opener):
            manifest,files = deploy._package(settings.DEFAULTS,lambda _:None)
        self.assertEqual(json.loads(manifest),self.manifest)
        self.assertEqual(files,self.files)
        opener.open.assert_called_once()
        url = urlsplit(opener.open.call_args.args[0].full_url)
        self.assertEqual(url.geturl().split('?')[0],deploy._GITHUB_ARCHIVE_URL)

    def test_default_github_archive_transport_failure_uses_verified_raw_files(self):
        source_files = {'manifest.json':json.dumps(self.manifest).encode(),**self.files}
        logs = []

        def respond(request, timeout):
            url = urlsplit(request.full_url)
            if url.hostname == 'codeload.github.com':
                raise HTTPError(url.geturl(),404,'Not found',None,None)
            path = url.path.split('/pi_deploy_bundle/',1)[1]
            return _HttpsResponse(source_files[path])

        opener = Mock()
        opener.open.side_effect = respond
        with patch.object(deploy,'build_opener',return_value=opener):
            _,files = deploy._package(settings.DEFAULTS,logs.append)
        self.assertEqual(files,self.files)
        self.assertEqual(opener.open.call_count,len(self.files)+2)
        self.assertIn('GitHub 压缩包暂不可用，改用文件下载。',logs)

    def test_default_github_archive_integrity_failures_never_use_raw_or_ssh(self):
        for options, expected_error in (
                ({'overrides':{'app/task_runner.py':b'altered'}},'哈希不匹配'),
                ({'symlink':'app/task_runner.py'},'链接、特殊文件'),
                ({'extra':{'app/extra.py':b'unlisted'}},'与清单不符')):
            with self.subTest(options=options):
                opener = Mock()
                opener.open.return_value = _HttpsResponse(self._archive(**options))
                with patch.object(deploy,'build_opener',return_value=opener), patch.object(
                        deploy,'_connect',side_effect=AssertionError('invalid archive must not reach SSH')):
                    with self.assertRaisesRegex(ValueError,expected_error):
                        deploy.operate('deploy',settings.DEFAULTS,'test-password',lambda _:None)
                opener.open.assert_called_once()

    def test_remote_transport_retries_connection_reset_and_truncated_body(self):
        opener = Mock()
        opener.open.side_effect = (
            URLError(OSError(10054,'connection reset')),
            _HttpsResponse(b'cut',10),
            _HttpsResponse(b'complete'),
        )
        with patch.object(deploy,'monotonic',return_value=0), patch.object(deploy,'sleep') as pause:
            result = deploy._remote_bundle_read(opener,'https://packages.example/bundle',
                                                'manifest.json',20,1,'')
        self.assertEqual(result,b'complete')
        self.assertEqual(opener.open.call_count,3)
        self.assertEqual(pause.call_count,2)

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
