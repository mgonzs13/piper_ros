// MIT License
//
// Copyright (c) 2024 Miguel Ángel González Santamarta
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef PIPER_ROS__PIPER_NODE_HPP
#define PIPER_ROS__PIPER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <map>
#include <memory>
#include <queue>
#include <string>

#include "audio_common_msgs/action/tts.hpp"
#include "audio_common_msgs/msg/audio_stamped.hpp"

#include "json.hpp"
#include "piper.hpp"

namespace piper_ros {

using TTS = audio_common_msgs::action::TTS;
using GoalHandleTTS = rclcpp_action::ServerGoalHandle<TTS>;

struct RunConfig {
  // Path to espeak-ng data directory (default is next to piper executable)
  std::string e_speak_data_path;

  // Path to .onnx voice file
  std::string model_path;

  // Path to JSON voice config file
  std::string model_config_path;

  // Path to libtashkeel ort model
  // https://github.com/mush42/libtashkeel/
  std::string tashkeel_model_path;

  // Numerical id of the default speaker (multi-speaker voices)
  piper::SpeakerId speaker_id;

  // Amount of noise to add during audio generation
  float noise_scale;

  // Speed of speaking (1 = normal, < 1 is faster, > 1 is slower)
  float length_scale;

  // Variation in phoneme lengths
  float noise_w;

  // Seconds of silence to add after each sentence
  float sentence_silence_seconds;

  // Seconds of extra silence to insert after a single phoneme
  std::map<piper::Phoneme, float> phoneme_silence_seconds;
};

class PiperNode : public rclcpp_lifecycle::LifecycleNode {

public:
  PiperNode();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &);

protected:
  struct RunConfig run_config;
  piper::PiperConfig piper_config;
  piper::Voice voice;

private:
  int chunk_;
  std::string frame_id_;

  // queue
  std::queue<std::shared_ptr<GoalHandleTTS>> goal_queue_;
  std::recursive_mutex goal_queue_lock_;
  std::shared_ptr<GoalHandleTTS> current_goal_handle_;

  // audio pub
  std::mutex pub_lock_;
  std::unique_ptr<rclcpp::Rate> pub_rate;
  rclcpp::Publisher<audio_common_msgs::msg::AudioStamped>::SharedPtr
      player_pub_;

  // action server
  rclcpp_action::Server<TTS>::SharedPtr action_server_;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const TTS::Goal> goal);
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleTTS> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleTTS> goal_handle);
  void execute_callback(const std::shared_ptr<GoalHandleTTS> goal_handle);
  void run_next_goal();
};
} // namespace piper_ros

#endif