from setuptools import setup

package_name = 'rqt_cfw11_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/plugin.xml', 'resource/MyPlugin.ui']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='CFW11 plugin for RQT',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'rqt_gui.plugins': [
            'my_plugin = rqt_cfw11_gui.my_plugin:MyPlugin'
        ],
    },
)
