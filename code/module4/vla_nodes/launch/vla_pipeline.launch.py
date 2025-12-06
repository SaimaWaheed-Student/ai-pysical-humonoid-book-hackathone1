from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Whisper Transcriber Node
        Node(
            package='vla_nodes',
            executable='whisper_transcriber',
            name='whisper_transcriber_node',
            output='screen',
            parameters=[
                # Add any specific parameters for whisper_transcriber here
            ]
        ),

        # GPT-4 Task Decomposer Node
        Node(
            package='vla_nodes',
            executable='gpt4_task_decomposer',
            name='gpt4_task_decomposer_node',
            output='screen',
            parameters=[
                # Add any specific parameters for gpt4_task_decomposer here
            ],
            remappings=[
                ('transcribed_text', '/transcribed_text') # Connects to Whisper output
            ]
        ),

        # Task to Action Translator Node
        Node(
            package='vla_nodes',
            executable='task_to_action_translator',
            name='task_to_action_translator_node',
            output='screen',
            parameters=[
                # Add any specific parameters for task_to_action_translator here
            ],
            remappings=[
                ('decomposed_tasks', '/decomposed_tasks') # Connects to GPT-4 decomposer output
            ]
        ),

        # CLIP Object Recognizer Node
        Node(
            package='vla_nodes',
            executable='clip_object_recognizer',
            name='clip_object_recognizer_node',
            output='screen',
            parameters=[
                # Add any specific parameters for clip_object_recognizer here
            ],
            remappings=[
                ('camera/image_raw', '/realsense_d435/image_raw'), # Assuming a camera source
                ('object_recognition_prompt', '/object_recognition_prompt') # From language model or direct
            ]
        ),

        # Grounding DINO Detector Node
        Node(
            package='vla_nodes',
            executable='grounding_dino_detector',
            name='grounding_dino_detector_node',
            output='screen',
            parameters=[
                # Add any specific parameters for grounding_dino_detector here
            ],
            remappings=[
                ('camera/image_raw', '/realsense_d435/image_raw'), # Assuming a camera source
                ('detection_prompt', '/detection_prompt') # From language model or direct
            ]
        ),

        # SAM Segmenter Node
        Node(
            package='vla_nodes',
            executable='sam_segmenter',
            name='sam_segmenter_node',
            output='screen',
            parameters=[
                # Add any specific parameters for sam_segmenter here
            ],
            remappings=[
                ('camera/image_raw', '/realsense_d435/image_raw'), # Assuming a camera source
                ('segmentation_prompt', '/segmentation_prompt') # From language model or direct/other node
            ]
        ),
    ])
