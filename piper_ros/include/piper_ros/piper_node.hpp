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

#include <memory>
#include <mutex>
#include <queue>
#include <string>

#include "audio_common_msgs/action/tts.hpp"
#include "audio_common_msgs/msg/audio_stamped.hpp"

#include <piper.h>

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
 * @brief ROS 2 Lifecycle Node for managing the Piper TTS system.
 *
 * This node wraps the libpiper C API to provide text-to-speech
 * functionality as a ROS 2 action server. It downloads voice models
 * from HuggingFace Hub and streams synthesized audio as
 * AudioStamped messages.
 *
 * @sa https://github.com/OHF-Voice/piper1-gpl
 */
class PiperNode : public rclcpp_lifecycle::LifecycleNode {

public:
  /**
   * @brief Constructor for the PiperNode class.
   *
   * Declares all ROS 2 parameters and resolves the espeak-ng data path.
   */
  PiperNode();

  /**
   * @brief Destructor. Frees piper synthesizer resources if still active.
   */
  ~PiperNode();

  /**
   * @brief Callback for configuring the node.
   *
   * Reads parameters, downloads the voice model and config from
   * HuggingFace Hub if paths are not provided directly.
   *
   * @param state The current lifecycle state.
   * @return SUCCESS on successful configuration.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &state);

  /**
   * @brief Callback for activating the node.
   *
   * Creates the piper synthesizer, sets up the audio publisher
   * and TTS action server.
   *
   * @param state The current lifecycle state.
   * @return SUCCESS on successful activation, FAILURE if synthesizer
   *         creation fails.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &state);

  /**
   * @brief Callback for deactivating the node.
   *
   * Resets the publisher, action server, and frees the piper
   * synthesizer.
   *
   * @param state The current lifecycle state.
   * @return SUCCESS.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &state);

  /**
   * @brief Callback for cleaning up the node.
   * @param state The current lifecycle state.
   * @return SUCCESS.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &state);

  /**
   * @brief Callback for shutting down the node.
   * @param state The current lifecycle state.
   * @return SUCCESS.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &state);

private:
  /** @brief Piper text-to-speech synthesizer (opaque C handle). */
  piper_synthesizer *synth_;

  /** @brief Chunk size in samples for audio publication. */
  int chunk_;
  /** @brief Frame ID attached to published AudioStamped headers. */
  std::string frame_id_;

  /** @brief Path to the .onnx voice model file. */
  std::string model_path_;
  /** @brief Path to the JSON voice config file. */
  std::string model_config_path_;
  /** @brief Path to the espeak-ng data directory. */
  std::string espeak_data_path_;

  /** @brief Numerical speaker id for multi-speaker voices. */
  int speaker_id_;
  /** @brief Amount of noise added during audio generation. */
  float noise_scale_;
  /** @brief Speed of speaking (1 = normal, < 1 faster, > 1 slower). */
  float length_scale_;
  /** @brief Variation in phoneme lengths during synthesis. */
  float noise_w_scale_;
  /** @brief Seconds of silence inserted between sentences. */
  float sentence_silence_seconds_;

  /** @brief FIFO queue of pending TTS goal handles. */
  std::queue<std::shared_ptr<GoalHandleTTS>> goal_queue_;
  /** @brief Mutex protecting @ref goal_queue_ and @ref current_goal_handle_. */
  std::recursive_mutex goal_queue_lock_;
  /** @brief Handle for the TTS goal currently being executed. */
  std::shared_ptr<GoalHandleTTS> current_goal_handle_;

  /** @brief Mutex serializing audio chunk publication. */
  std::mutex pub_lock_;
  /** @brief Rate limiter matching audio chunk / sample-rate cadence. */
  std::unique_ptr<rclcpp::Rate> pub_rate;
  /** @brief Publisher for AudioStamped messages. */
  rclcpp::Publisher<audio_common_msgs::msg::AudioStamped>::SharedPtr
      player_pub_;

  /** @brief Action server exposing the ~/say TTS action. */
  rclcpp_action::Server<TTS>::SharedPtr action_server_;

  /**
   * @brief Handle a new TTS goal request.
   * @param uuid The unique identifier for the goal.
   * @param goal The goal message containing the text to synthesize.
   * @return ACCEPT_AND_EXECUTE.
   */
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const TTS::Goal> goal);

  /**
   * @brief Handle a request to cancel a TTS goal.
   * @param goal_handle The handle for the goal to be canceled.
   * @return ACCEPT.
   */
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleTTS> goal_handle);

  /**
   * @brief Handle an accepted TTS goal by queuing it for execution.
   * @param goal_handle The handle for the accepted goal.
   */
  void handle_accepted(const std::shared_ptr<GoalHandleTTS> goal_handle);

  /**
   * @brief Execute the TTS synthesis and publish audio for a goal.
   *
   * Runs piper synthesis, converts float output to int16, and
   * publishes audio chunks at the appropriate rate.
   *
   * @param goal_handle The handle for the goal being executed.
   */
  void execute_callback(const std::shared_ptr<GoalHandleTTS> goal_handle);

  /**
   * @brief Dequeue and start the next pending TTS goal, if any.
   */
  void run_next_goal();
};
} // namespace piper_ros

#endif
