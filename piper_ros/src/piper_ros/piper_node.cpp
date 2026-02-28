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

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "audio_common_msgs/action/tts.hpp"
#include "audio_common_msgs/msg/audio_stamped.hpp"
#include "huggingface_hub.h"

#include "piper_ros/piper_node.hpp"

using namespace piper_ros;
using std::placeholders::_1;
using std::placeholders::_2;

PiperNode::PiperNode()
    : rclcpp_lifecycle::LifecycleNode("piper_node"), synth_(nullptr),
      pub_rate(nullptr) {

  this->declare_parameter<int>("chunk", 512);
  this->declare_parameter<std::string>("frame_id", "");

  this->declare_parameter<std::string>("model_repo", "");
  this->declare_parameter<std::string>("model_filename", "");
  this->declare_parameter<std::string>("model_path", "");
  this->declare_parameter<std::string>("model_config_repo", "");
  this->declare_parameter<std::string>("model_config_filename", "");
  this->declare_parameter<std::string>("model_config_path", "");

  this->declare_parameter<int>("speaker_id", 0);
  this->declare_parameter<float>("noise_scale", 0.667f);
  this->declare_parameter<float>("length_scale", 1.0f);
  this->declare_parameter<float>("noise_w_scale", 0.8f);
  this->declare_parameter<float>("sentence_silence_seconds", 0.2f);

  this->espeak_data_path_ =
      ament_index_cpp::get_package_share_directory("piper_vendor") +
      "/espeak-ng-data";
}

std::string download_model(const std::string &repo_id,
                           const std::string &filename) {

  if (repo_id.empty() || filename.empty()) {
    return "";
  }

  auto result = huggingface_hub::hf_hub_download(repo_id, filename);

  if (result.success) {
    return result.path;
  } else {
    return "";
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PiperNode::on_configure(const rclcpp_lifecycle::State &) {

  std::string model_repo;
  std::string model_filename;
  std::string model_config_repo;
  std::string model_config_filename;

  RCLCPP_INFO(get_logger(), "[%s] Configuring...", this->get_name());

  this->get_parameter("chunk", this->chunk_);
  this->get_parameter("frame_id", this->frame_id_);

  this->get_parameter("model_repo", model_repo);
  this->get_parameter("model_filename", model_filename);
  this->get_parameter("model_path", this->model_path_);
  this->get_parameter("model_config_repo", model_config_repo);
  this->get_parameter("model_config_filename", model_config_filename);
  this->get_parameter("model_config_path", this->model_config_path_);

  this->get_parameter("speaker_id", this->speaker_id_);
  this->get_parameter("noise_scale", this->noise_scale_);
  this->get_parameter("length_scale", this->length_scale_);
  this->get_parameter("noise_w_scale", this->noise_w_scale_);
  this->get_parameter("sentence_silence_seconds",
                      this->sentence_silence_seconds_);

  // Download model
  if (this->model_path_.empty()) {
    this->model_path_ = download_model(model_repo, model_filename);
  }

  if (this->model_config_path_.empty()) {

    if (model_config_repo.empty()) {
      model_config_repo = model_repo;
    }

    if (model_config_filename.empty()) {
      model_config_filename = model_filename + ".json";
    }

    this->model_config_path_ =
        download_model(model_config_repo, model_config_filename);
  }

  RCLCPP_INFO(get_logger(), "[%s] Configured", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PiperNode::on_activate(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(get_logger(), "[%s] Activating...", this->get_name());
  RCLCPP_INFO(get_logger(), "Loading voice from %s (config=%s)",
              this->model_path_.c_str(), this->model_config_path_.c_str());

  // Create piper synthesizer
  const char *config_path = this->model_config_path_.empty()
                                ? nullptr
                                : this->model_config_path_.c_str();

  this->synth_ = piper_create(this->model_path_.c_str(), config_path,
                              this->espeak_data_path_.c_str());

  if (!this->synth_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create piper synthesizer");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::FAILURE;
  }

  // Audio pub
  this->player_pub_ =
      this->create_publisher<audio_common_msgs::msg::AudioStamped>(
          "audio", rclcpp::SensorDataQoS());

  // Action server
  this->action_server_ = rclcpp_action::create_server<TTS>(
      this, "say", std::bind(&PiperNode::handle_goal, this, _1, _2),
      std::bind(&PiperNode::handle_cancel, this, _1),
      std::bind(&PiperNode::handle_accepted, this, _1));

  RCLCPP_INFO(get_logger(), "[%s] Activated", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PiperNode::on_deactivate(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(get_logger(), "[%s] Deactivating...", this->get_name());

  this->player_pub_.reset();
  this->player_pub_ = nullptr;

  this->action_server_.reset();
  this->action_server_ = nullptr;

  if (this->synth_) {
    piper_free(this->synth_);
    this->synth_ = nullptr;
  }

  RCLCPP_INFO(get_logger(), "[%s] Deactivated", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PiperNode::on_cleanup(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(get_logger(), "[%s] Cleaning up...", this->get_name());
  RCLCPP_INFO(get_logger(), "[%s] Cleaned up", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PiperNode::on_shutdown(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(get_logger(), "[%s] Shutting down...", this->get_name());
  RCLCPP_INFO(get_logger(), "[%s] Shutted down", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse
PiperNode::handle_goal(const rclcpp_action::GoalUUID &uuid,
                       std::shared_ptr<const TTS::Goal> goal) {
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
PiperNode::handle_cancel(const std::shared_ptr<GoalHandleTTS> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Canceling TTS...");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PiperNode::handle_accepted(
    const std::shared_ptr<GoalHandleTTS> goal_handle) {

  std::lock_guard<std::recursive_mutex> lock(this->goal_queue_lock_);
  this->goal_queue_.push(goal_handle);

  if (this->current_goal_handle_ == nullptr ||
      !this->current_goal_handle_->is_active()) {
    this->run_next_goal();
  }
}

void PiperNode::run_next_goal() {
  std::lock_guard<std::recursive_mutex> lock(this->goal_queue_lock_);

  if (!this->goal_queue_.empty()) {
    this->current_goal_handle_ = this->goal_queue_.front();
    this->goal_queue_.pop();
    std::thread{std::bind(&PiperNode::execute_callback, this, _1),
                this->current_goal_handle_}
        .detach();

  } else {
    this->current_goal_handle_ = nullptr;
  }
}

void PiperNode::execute_callback(
    const std::shared_ptr<GoalHandleTTS> goal_handle) {

  const auto goal = goal_handle->get_goal();
  std::string text = goal->text;

  auto result = std::make_shared<TTS::Result>();
  result->text = text;

  // Set up synthesis options
  piper_synthesize_options options =
      piper_default_synthesize_options(this->synth_);
  options.speaker_id = this->speaker_id_;
  options.noise_scale = this->noise_scale_;
  options.length_scale = this->length_scale_;
  options.noise_w_scale = this->noise_w_scale_;

  // Generate audio using streaming API
  std::vector<int16_t> audio_buffer;
  int sample_rate = 22050; // default

  try {
    int ret = piper_synthesize_start(this->synth_, text.c_str(), &options);
    if (ret != PIPER_OK) {
      throw std::runtime_error("Failed to start synthesis");
    }

    piper_audio_chunk chunk;
    while (true) {
      ret = piper_synthesize_next(this->synth_, &chunk);
      if (ret == PIPER_DONE) {
        break;
      }
      if (ret != PIPER_OK) {
        throw std::runtime_error("Error during synthesis");
      }

      sample_rate = chunk.sample_rate;

      // Convert float samples to int16_t
      for (size_t i = 0; i < chunk.num_samples; i++) {
        float sample = std::clamp(chunk.samples[i], -1.0f, 1.0f);
        audio_buffer.push_back(static_cast<int16_t>(sample * 32767.0f));
      }

      // Add sentence silence
      if (this->sentence_silence_seconds_ > 0.0f) {
        int silence_samples = static_cast<int>(this->sentence_silence_seconds_ *
                                               chunk.sample_rate);
        audio_buffer.insert(audio_buffer.end(), silence_samples, 0);
      }
    }

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Error while generating audio: %s",
                 e.what());
    this->run_next_goal();
    goal_handle->abort(result);
    return;
  }

  // Create rate
  std::unique_lock<std::mutex> lock(this->pub_lock_);
  this->run_next_goal();

  if (this->pub_rate == nullptr) {
    std::chrono::nanoseconds period((int)(1e9 * this->chunk_ / sample_rate));
    this->pub_rate = std::make_unique<rclcpp::Rate>(period);
  }

  // Publish the audio data in chunks
  for (size_t i = 0; i < audio_buffer.size(); i += this->chunk_) {

    int min_size = std::min(this->chunk_, (int)(audio_buffer.size() - i));
    std::vector<int16_t> data(&audio_buffer[i], &audio_buffer[i + min_size]);

    int pad_size = this->chunk_ - data.size();
    if (pad_size > 0) {
      data.insert(data.end(), pad_size, 0);
    }

    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    }

    auto msg = audio_common_msgs::msg::AudioStamped();
    msg.header.stamp = this->get_clock()->now();
    msg.audio.audio_data.int16_data = data;
    msg.audio.info.channels = 1;
    msg.audio.info.chunk = this->chunk_;
    msg.audio.info.format = 8;
    msg.audio.info.rate = sample_rate;

    auto feedback = std::make_shared<TTS::Feedback>();
    feedback->audio = msg;

    this->player_pub_->publish(msg);
    goal_handle->publish_feedback(feedback);
    this->pub_rate->sleep();
  }

  goal_handle->succeed(result);
}
