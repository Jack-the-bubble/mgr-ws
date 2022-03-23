from setuptools import setup

setup(
    name='PatrolMapDivider',
    version='0.1.0',
    author='Marcin Skrzypkowski',
    author_email='mj.skrzypkowski@gmail.com',
    packages=['patrol_map_divider_ros'], # no idea why, if the folder name is the same as package, the conf file is invisible to rospy
    description='An awesome package that does something',
    install_requires=["pytest"],
)
