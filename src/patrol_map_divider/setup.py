from setuptools import setup

setup(
    name='PatrolMapDivider',
    version='0.1.0',
    author='Marcin Skrzypkowski',
    author_email='mj.skrzypkowski@gmail.com',
    # no idea why, if the folder name is the same as package,
    # the conf file is invisible to rospy
    packages=['patrol_map_divider_ros'],
    description='An awesome package that does something',
    install_requires=["pytest"],
)
