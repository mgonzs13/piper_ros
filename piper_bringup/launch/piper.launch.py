# MIT License

# Copyright (c) 2024  Miguel Ángel González Santamarta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import os
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchContext
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from huggingface_hub import hf_hub_download


def generate_launch_description():

    def run_piper(context: LaunchContext, repo, model, config, model_path, config_path):
        repo = str(context.perform_substitution(repo))
        model = str(context.perform_substitution(model))
        config = str(context.perform_substitution(config))
        model_path = str(context.perform_substitution(model_path))
        config_path = str(context.perform_substitution(config_path))

        if not model_path:
            model_path = hf_hub_download(
                repo_id=repo, filename=model, force_download=False
            )

        if not config_path:
            config_path = hf_hub_download(
                repo_id=repo, filename=config, force_download=False
            )

        params = {
            "chunk": LaunchConfiguration("chunk", default=512),
            "frame_id": LaunchConfiguration("frame_id", default=""),
            "model_path": LaunchConfiguration("model", default=model_path),
            "model_config_path": LaunchConfiguration(
                "model_config_path", default=config_path
            ),
            "speaker_id": LaunchConfiguration("speaker_id", default="0"),
            "noise_scale": LaunchConfiguration("noise_scale", default=0.667),
            "length_scale": LaunchConfiguration("length_scale", default=1.0),
            "noise_w": LaunchConfiguration("noise_w", default=0.8),
            "sentence_silence_seconds": LaunchConfiguration(
                "sentence_silence_seconds", default=0.2
            ),
            "silence_phonemes": LaunchConfiguration("silence_phonemes", default="[0]"),
            "silence_seconds": LaunchConfiguration("silence_seconds", default="[0.0]"),
        }

        return (
            Node(
                package="piper_ros",
                executable="piper_node",
                name="piper_node",
                parameters=[params],
                remappings=[("audio", "audio/out")],
            ),
        )

    model_repo = LaunchConfiguration("model_repo")
    model_repo_cmd = DeclareLaunchArgument(
        "model_repo",
        default_value="rhasspy/piper-voices",
        description="Hugging Face model repo for piper",
    )

    model_filename = LaunchConfiguration("model_filename")
    model_filename_cmd = DeclareLaunchArgument(
        "model_filename",
        default_value="en/en_US/lessac/low/en_US-lessac-low.onnx",
        description="Hugging Face model filename for piper",
    )

    config_filename = LaunchConfiguration("config_filename")
    config_filename_cmd = DeclareLaunchArgument(
        "config_filename",
        default_value="en/en_US/lessac/low/en_US-lessac-low.onnx.json",
        description="Hugging Face config filename for piper",
    )

    model_path = LaunchConfiguration("model_path")
    model_path_cmd = DeclareLaunchArgument(
        "model_path",
        default_value="",
        description="Local path to the model file for piper",
    )

    config_path = LaunchConfiguration("config_path")
    config_path_cmd = DeclareLaunchArgument(
        "config_path",
        default_value="",
        description="Local path to the config file for piper",
    )

    return LaunchDescription(
        [
            model_repo_cmd,
            model_filename_cmd,
            config_filename_cmd,
            model_path_cmd,
            config_path_cmd,
            OpaqueFunction(
                function=run_piper,
                args=[
                    model_repo,
                    model_filename,
                    config_filename,
                    model_path,
                    config_path,
                ],
            ),
            Node(
                package="audio_common",
                executable="audio_player_node",
                name="player_node",
                namespace="audio",
                parameters=[
                    {
                        "format": LaunchConfiguration("format", default=8),
                        "channels": LaunchConfiguration("channels", default=2),
                    }
                ],
                remappings=[("audio", "out")],
                condition=IfCondition(
                    PythonExpression(
                        [LaunchConfiguration("launch_audio_player", default=True)]
                    )
                ),
            ),
        ]
    )
