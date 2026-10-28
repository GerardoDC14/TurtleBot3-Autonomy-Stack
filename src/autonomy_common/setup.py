from setuptools import setup

setup(
    name='autonomy_common',
    version='0.1.0',
    packages=['autonomy_common'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/autonomy_common']),
        ('share/autonomy_common', ['package.xml']),
    ],
    install_requires=['setuptools'],
    maintainer='Gerardo Escobar',
    maintainer_email='123440177+GerardoDC14@users.noreply.github.com',
    description='Shared asynchronous Nav2 mission-task handling.',
    license='Apache-2.0',
)
