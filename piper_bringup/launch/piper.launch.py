# MIT License

# Copyright (c) 2024 Miguel Ángel González Santamarta

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


from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    params = {
        "chunk": LaunchConfiguration("chunk", default=512),
        "frame_id": LaunchConfiguration("frame_id", default=""),
        "model_repo": LaunchConfiguration("model_repo", default="rhasspy/piper-voices"),
        "model_filename": LaunchConfiguration(
            "model_filename", default="en/en_US/lessac/low/en_US-lessac-low.onnx"
        ),
        "model_path": LaunchConfiguration("model", default=""),
        "model_config_repo": LaunchConfiguration(
            "model_config_repo", default="rhasspy/piper-voices"
        ),
        "model_config_filename": LaunchConfiguration(
            "model_config_filename", default="rhasspy/piper-voices"
        ),
        "model_config_path": LaunchConfiguration("model_config_path", default=""),
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

    return LaunchDescription(
        [
            Node(
                package="piper_ros",
                executable="piper_node",
                name="piper_node",
                parameters=[params],
                remappings=[("audio", "audio/out")],
            ),
            Node(
                package="audio_common",
                executable="audio_player_node",
                name="player_node",
                namespace="audio",
                parameters=[
                    {
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
