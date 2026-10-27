from glob import glob
from setuptools import setup

setup(name='rail_stage1', version='0.1.0', packages=['rail_stage1'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/rail_stage1']),
                  ('share/rail_stage1', ['package.xml']),
                  ('share/rail_stage1/config', glob('config/*')),
                  ('share/rail_stage1/launch', glob('launch/*'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Vlad', maintainer_email='vlad@example.com', license='MIT',
      description='Modular map-based rail discovery and independent Nav2 approach',
      entry_points={'console_scripts': ['rail_finder = rail_stage1.finder:main',
                                        'rail_approach = rail_stage1.approach:main',
                                        'rail_visual_entry = rail_stage1.entry:main']})
