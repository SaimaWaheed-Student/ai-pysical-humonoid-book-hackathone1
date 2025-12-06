from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vla_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), # Add launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='ROS 2 nodes for Vision-Language-Action integration',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'whisper_transcriber = vla_nodes.whisper_transcriber:main',
            'gpt4_task_decomposer = vla_nodes.gpt4_task_decomposer:main',
            'task_to_action_translator = vla_nodes.task_to_action_translator:main',
            'clip_object_recognizer = vla_nodes.clip_object_recognizer:main',
            'grounding_dino_detector = vla_nodes.grounding_dino_detector:main',
            'sam_segmenter = vla_nodes.sam_segmenter:main',
        ],
    },
)
