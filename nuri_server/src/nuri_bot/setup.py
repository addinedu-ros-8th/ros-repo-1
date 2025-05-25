import os
import stat
import sys
from setuptools import find_packages, setup
from setuptools.command.install import install
from setuptools.command.develop import develop

class CustomInstallCommand(install):
    """Custom install command to fix shebang after installation"""
    
    def run(self):
        # 기본 설치 실행
        install.run(self)
        # shebang 수정
        self.fix_shebang()
    
    def fix_shebang(self):
        """설치된 console_scripts의 shebang을 수정"""
        venv_path = "/home/pinky/venv/pinky/bin/python"
        
        print(f"[INFO] Fixing shebang to use: {venv_path}")
        
        # console_scripts로 설치된 실행 파일들 찾기
        if hasattr(self, 'install_scripts') and self.install_scripts:
            script_dir = self.install_scripts
        else:
            # 기본 스크립트 설치 경로 추정
            script_dir = os.path.join(os.path.dirname(sys.executable))
        
        # console_scripts에 정의된 스크립트 이름들
        target_scripts = ['main_node', 'cam_node']
        
        print(f"[INFO] Looking for scripts in: {script_dir}")
        
        for script_name in target_scripts:
            target_file = os.path.join(script_dir, script_name)
            
            if os.path.isfile(target_file):
                try:
                    # 파일 읽기
                    with open(target_file, 'r') as f:
                        content = f.read()
                    
                    # shebang 라인 수정
                    lines = content.split('\n')
                    if lines and lines[0].startswith('#!'):
                        old_shebang = lines[0]
                        lines[0] = f'#!{venv_path}'
                        
                        # 파일 쓰기
                        with open(target_file, 'w') as f:
                            f.write('\n'.join(lines))
                        
                        # 실행 권한 확인/설정
                        current_permissions = os.stat(target_file).st_mode
                        if not (current_permissions & stat.S_IEXEC):
                            os.chmod(target_file, current_permissions | stat.S_IEXEC)
                        
                        print(f"[Success] Fixed shebang for {target_file}")
                        print(f"    Old: {old_shebang}")
                        print(f"    New: #!{venv_path}")
                    else:
                        print(f"[Warning] No shebang found in {target_file}")
                    
                except Exception as e:
                    print(f"[Error] Failed to fix shebang for {target_file}: {e}")
            else:
                print(f"[Warning] Script not found: {target_file}")

class CustomDevelopCommand(develop):
    """Custom develop command to fix shebang after development installation"""
    
    def run(self):
        # 기본 develop 설치 실행
        develop.run(self)
        # shebang 수정 (install과 동일한 로직 사용)
        installer = CustomInstallCommand(self.distribution)
        installer.install_scripts = self.install_scripts
        installer.fix_shebang()

package_name = 'nuri_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='hhm9124@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_node = nuri_bot.main_node:main',
            # 'stt_node = nuri_bot.stt_node:main',
            # 'tts_node = nuri_bot.tts_node:main',
            'cam_node = nuri_bot.cam_node:main',
        ],
    },
    cmdclass={
        'install': CustomInstallCommand,
        'develop': CustomDevelopCommand,
    },
)