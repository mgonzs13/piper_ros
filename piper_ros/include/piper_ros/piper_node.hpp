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

/**
 * @brief Alias for the TTS action type.
 */
using TTS = audio_common_msgs::action::TTS;

/**
 * @brief Alias for the TTS action goal handle.
 */
using GoalHandleTTS = rclcpp_action::ServerGoalHandle<TTS>;

/**
 * @brief Configuration structure for running the Piper TTS system.
 */
struct RunConfig {
  /// Path to espeak-ng data directory (default is next to piper executable)
  std::string e_speak_data_path;

  /// Path to .onnx voice file
  std::string model_path;

  /// Path to JSON voice config file
  std::string model_config_path;

  /// Path to libtashkeel ort model https://github.com/mush42/libtashkeel/
  std::string tashkeel_model_path;

  /// Numerical id of the default speaker (multi-speaker voices)
  piper::SpeakerId speaker_id;

  /// Amount of noise to add during audio generation
  float noise_scale;

  /// Speed of speaking (1 = normal, < 1 is faster, > 1 is slower)
  float length_scale;

  /// Variation in phoneme lengths
  float noise_w;

  /// Seconds of silence to add after each sentence
  float sentence_silence_seconds;

  /// Seconds of extra silence to insert after a single phoneme
  std::map<piper::Phoneme, float> phoneme_silence_seconds;
};

/**
 * @brief ROS 2 Lifecycle Node for managing the Piper TTS system.
 */
class PiperNode : public rclcpp_lifecycle::LifecycleNode {

public:
  /**
   * @brief Constructor for the PiperNode class.
   */
  PiperNode();

  /**
   * @brief Callback for configuring the node.
   * @param state The current lifecycle state.
   * @return The result of the configuration process.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &state);

  /**
   * @brief Callback for activating the node.
   * @param state The current lifecycle state.
   * @return The result of the activation process.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &state);

  /**
   * @brief Callback for deactivating the node.
   * @param state The current lifecycle state.
   * @return The result of the deactivation process.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &state);

  /**
   * @brief Callback for cleaning up the node.
   * @param state The current lifecycle state.
   * @return The result of the cleanup process.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &state);

  /**
   * @brief Callback for shutting down the node.
   * @param state The current lifecycle state.
   * @return The result of the shutdown process.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &state);

protected:
  /// Configuration for running the Piper TTS system.
  struct RunConfig run_config;
  /// Piper configuration object.
  piper::PiperConfig piper_config;
  /// Voice object for TTS generation.
  piper::Voice voice;

private:
  /// Chunk size for audio processing.
  int chunk_;
  /// Frame ID for audio messages.
  std::string frame_id_;

  /// Queue for managing TTS goals.
  std::queue<std::shared_ptr<GoalHandleTTS>> goal_queue_;
  /// Mutex for synchronizing access to the goal queue.
  std::recursive_mutex goal_queue_lock_;
  /// Handle for the current TTS goal being processed.
  std::shared_ptr<GoalHandleTTS> current_goal_handle_;

  /// Mutex for synchronizing access to the audio publisher.
  std::mutex pub_lock_;
  /// Rate for publishing audio messages.
  std::unique_ptr<rclcpp::Rate> pub_rate;
  /// Publisher for audio messages.
  rclcpp::Publisher<audio_common_msgs::msg::AudioStamped>::SharedPtr
      player_pub_;

  /// Action server for handling TTS goals.
  rclcpp_action::Server<TTS>::SharedPtr action_server_;

  /**
   * @brief Handle a new TTS goal.
   * @param uuid The unique identifier for the goal.
   * @param goal The goal message.
   * @return The response to the goal request.
   */
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const TTS::Goal> goal);

  /**
   * @brief Handle a request to cancel a TTS goal.
   * @param goal_handle The handle for the goal to be canceled.
   * @return The response to the cancel request.
   */
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleTTS> goal_handle);

  /**
   * @brief Handle an accepted TTS goal.
   * @param goal_handle The handle for the accepted goal.
   */
  void handle_accepted(const std::shared_ptr<GoalHandleTTS> goal_handle);

  /**
   * @brief Execute the callback for a TTS goal.
   * @param goal_handle The handle for the goal being executed.
   */
  void execute_callback(const std::shared_ptr<GoalHandleTTS> goal_handle);

  /**
   * @brief Process the next goal in the queue.
   */
  void run_next_goal();
};
} // namespace piper_ros

#endif