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
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#if __has_include("rclcpp/version.h")
#include "rclcpp/version.h"
#if RCLCPP_VERSION_GTE(32, 0, 0)
#include <ament_index_cpp/get_package_share_path.hpp>
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

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
    : rclcpp_lifecycle::LifecycleNode("piper_node"), synth_(nullptr) {

  this->declare_parameter<int>("chunk", 512);
  this->declare_parameter<std::string>("frame_id", "");

  // Model parameters
  this->declare_parameter<std::string>("model.repo", "");
  this->declare_parameter<std::string>("model.filename", "");
  this->declare_parameter<std::string>("model.path", "");
  this->declare_parameter<std::string>("model.config_repo", "");
  this->declare_parameter<std::string>("model.config_filename", "");
  this->declare_parameter<std::string>("model.config_path", "");

  // Synthesis parameters
  this->declare_parameter<int>("synthesis.speaker_id", 0);
  this->declare_parameter<float>("synthesis.noise_scale", 0.667f);
  this->declare_parameter<float>("synthesis.length_scale", 1.0f);
  this->declare_parameter<float>("synthesis.noise_w_scale", 0.8f);
  this->declare_parameter<float>("synthesis.sentence_silence_seconds", 0.2f);

  std::string package_path;

#if __has_include("rclcpp/version.h")
#if RCLCPP_VERSION_GTE(32, 0, 0)
  package_path =
      ament_index_cpp::get_package_share_path("piper_vendor").string();
#else
  package_path = ament_index_cpp::get_package_share_directory("piper_vendor");
#endif
#else
  package_path = ament_index_cpp::get_package_share_directory("piper_vendor");
#endif

  this->espeak_data_path_ = package_path + "/espeak-ng-data";
}

PiperNode::~PiperNode() {
  this->stop_worker();

  if (this->synth_) {
    piper_free(this->synth_);
    this->synth_ = nullptr;
  }
}

namespace {

std::string download_model(const std::string &repo_id,
                           const std::string &filename) {

  if (repo_id.empty() || filename.empty()) {
    return "";
  }

  try {
    auto result = huggingface_hub::hf_hub_download(repo_id, filename);

    if (result.success && !result.path.empty()) {
      return result.path;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("piper_node"),
                 "Error downloading %s from %s: %s", filename.c_str(),
                 repo_id.c_str(), e.what());
  }

  return "";
}

} // namespace

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PiperNode::on_configure(const rclcpp_lifecycle::State &) {

  std::string model_repo;
  std::string model_filename;
  std::string model_config_repo;
  std::string model_config_filename;

  RCLCPP_INFO(get_logger(), "[%s] Configuring...", this->get_name());

  this->get_parameter("chunk", this->chunk_);
  this->get_parameter("frame_id", this->frame_id_);

  if (this->chunk_ <= 0) {
    RCLCPP_ERROR(get_logger(), "Invalid chunk size: %d (must be > 0)",
                 this->chunk_);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::FAILURE;
  }

  this->get_parameter("model.repo", model_repo);
  this->get_parameter("model.filename", model_filename);
  this->get_parameter("model.path", this->model_path_);
  this->get_parameter("model.config_repo", model_config_repo);
  this->get_parameter("model.config_filename", model_config_filename);
  this->get_parameter("model.config_path", this->model_config_path_);

  this->get_parameter("synthesis.speaker_id", this->speaker_id_);
  this->get_parameter("synthesis.noise_scale", this->noise_scale_);
  this->get_parameter("synthesis.length_scale", this->length_scale_);
  this->get_parameter("synthesis.noise_w_scale", this->noise_w_scale_);
  this->get_parameter("synthesis.sentence_silence_seconds",
                      this->sentence_silence_seconds_);

  // Download model
  if (this->model_path_.empty()) {
    this->model_path_ = download_model(model_repo, model_filename);
  }

  if (this->model_path_.empty() ||
      !std::filesystem::exists(this->model_path_)) {
    RCLCPP_ERROR(get_logger(),
                 "Voice model not found (model.path='%s', repo='%s', "
                 "filename='%s')",
                 this->model_path_.c_str(), model_repo.c_str(),
                 model_filename.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::FAILURE;
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

  if (this->model_config_path_.empty()) {
    // piper_create() falls back to "<model_path>.json" when no config path
    // is given, so accept that file if it exists.
    const std::string fallback_config_path = this->model_path_ + ".json";

    if (std::filesystem::exists(fallback_config_path)) {
      this->model_config_path_ = fallback_config_path;
    }
  }

  if (this->model_config_path_.empty() ||
      !std::filesystem::exists(this->model_config_path_)) {
    RCLCPP_ERROR(get_logger(),
                 "Voice config not found (model.config_path='%s', repo='%s', "
                 "filename='%s')",
                 this->model_config_path_.c_str(), model_config_repo.c_str(),
                 model_config_filename.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::FAILURE;
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

  try {
    this->synth_ = piper_create(this->model_path_.c_str(), config_path,
                                this->espeak_data_path_.c_str());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create piper synthesizer: %s",
                 e.what());
    this->synth_ = nullptr;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::FAILURE;
  }

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

  // Worker thread that serializes goal execution
  {
    std::lock_guard<std::mutex> lock(this->goal_queue_lock_);
    this->stop_worker_ = false;
  }

  this->worker_ = std::thread(&PiperNode::worker_loop, this);

  RCLCPP_INFO(get_logger(), "[%s] Activated", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PiperNode::on_deactivate(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(get_logger(), "[%s] Deactivating...", this->get_name());

  // Stop the worker before releasing any resource it may be using
  this->stop_worker();

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

  this->stop_worker();

  RCLCPP_INFO(get_logger(), "[%s] Cleaned up", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PiperNode::on_shutdown(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(get_logger(), "[%s] Shutting down...", this->get_name());

  // Stop the worker before releasing any resource it may be using
  this->stop_worker();

  this->player_pub_.reset();
  this->action_server_.reset();

  if (this->synth_) {
    piper_free(this->synth_);
    this->synth_ = nullptr;
  }

  RCLCPP_INFO(get_logger(), "[%s] Shut down", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse
PiperNode::handle_goal(const rclcpp_action::GoalUUID &uuid,
                       std::shared_ptr<const TTS::Goal> goal) {
  (void)uuid;

  if (goal->text.empty()) {
    RCLCPP_WARN(this->get_logger(), "Rejected TTS goal with empty text");
    return rclcpp_action::GoalResponse::REJECT;
  }

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

  std::unique_lock<std::mutex> lock(this->goal_queue_lock_);

  if (this->stop_worker_) {
    // The node is deactivating/shutting down, so no new goal can be queued.
    lock.unlock();

    auto result = std::make_shared<TTS::Result>();
    result->text = goal_handle->get_goal()->text;

    try {
      goal_handle->abort(result);
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "Failed to abort goal: %s", e.what());
    }
    return;
  }

  this->goal_queue_.push(goal_handle);
  this->goal_queue_cv_.notify_one();
}

void PiperNode::worker_loop() {
  while (true) {
    std::shared_ptr<GoalHandleTTS> goal_handle;

    {
      std::unique_lock<std::mutex> lock(this->goal_queue_lock_);
      this->goal_queue_cv_.wait(lock, [this] {
        return this->stop_worker_.load() || !this->goal_queue_.empty();
      });

      if (this->stop_worker_) {
        // Pending goals are aborted by stop_worker().
        return;
      }

      goal_handle = this->goal_queue_.front();
      this->goal_queue_.pop();
    }

    // A goal canceled while waiting in the queue must not be synthesized.
    if (goal_handle->is_canceling()) {
      auto result = std::make_shared<TTS::Result>();
      result->text = goal_handle->get_goal()->text;

      try {
        goal_handle->canceled(result);
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Failed to cancel goal: %s", e.what());
      }
      continue;
    }

    this->execute_callback(goal_handle);
  }
}

void PiperNode::stop_worker() {
  {
    std::lock_guard<std::mutex> lock(this->goal_queue_lock_);
    this->stop_worker_ = true;
  }
  this->goal_queue_cv_.notify_all();

  if (this->worker_.joinable()) {
    this->worker_.join();
  }

  // Abort goals that were queued but never executed, otherwise their clients
  // would wait forever.
  std::queue<std::shared_ptr<GoalHandleTTS>> pending_goals;
  {
    std::lock_guard<std::mutex> lock(this->goal_queue_lock_);
    std::swap(pending_goals, this->goal_queue_);
  }

  while (!pending_goals.empty()) {
    auto goal_handle = pending_goals.front();
    pending_goals.pop();

    if (goal_handle == nullptr || !goal_handle->is_active()) {
      continue;
    }

    auto result = std::make_shared<TTS::Result>();
    result->text = goal_handle->get_goal()->text;

    try {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else {
        goal_handle->abort(result);
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "Failed to abort pending goal: %s",
                  e.what());
    }
  }
}

void PiperNode::execute_callback(
    const std::shared_ptr<GoalHandleTTS> goal_handle) {

  const auto goal = goal_handle->get_goal();
  const std::string text = goal->text;

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
  std::vector<float> audio_buffer;
  int sample_rate = 0;
  bool canceled = false;
  bool stopped = false;

  try {
    int ret = piper_synthesize_start(this->synth_, text.c_str(), &options);
    if (ret != PIPER_OK) {
      throw std::runtime_error("Failed to start synthesis");
    }

    while (true) {
      if (this->stop_worker_) {
        stopped = true;
        break;
      }

      if (goal_handle->is_canceling()) {
        canceled = true;
        break;
      }

      piper_audio_chunk chunk{};
      ret = piper_synthesize_next(this->synth_, &chunk);

      // Any return value other than OK/DONE is an actual error.
      if (ret != PIPER_OK && ret != PIPER_DONE) {
        throw std::runtime_error("Error during synthesis");
      }

      // Do not discard the chunk if piper_synthesize_next() returned
      // PIPER_DONE. Some piper1-gpl versions still send data in the final
      // chunk.
      if (chunk.samples != nullptr && chunk.num_samples > 0) {
        sample_rate = chunk.sample_rate;

        audio_buffer.insert(audio_buffer.end(), chunk.samples,
                            chunk.samples + chunk.num_samples);

        // Add silence between sentences, but never after the final chunk.
        if (this->sentence_silence_seconds_ > 0.0f && !chunk.is_last &&
            ret != PIPER_DONE) {
          const size_t silence_samples =
              static_cast<size_t>(this->sentence_silence_seconds_ *
                                  static_cast<float>(chunk.sample_rate));
          audio_buffer.insert(audio_buffer.end(), silence_samples, 0.0f);
        }
      }

      // PIPER_DONE means that there will be no more chunks.
      if (ret == PIPER_DONE) {
        break;
      }
    }

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Error while generating audio: %s",
                 e.what());
    goal_handle->abort(result);
    return;
  }

  if (canceled) {
    goal_handle->canceled(result);
    return;
  }

  if (stopped) {
    goal_handle->abort(result);
    return;
  }

  if (sample_rate <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid sample rate: %d", sample_rate);
    goal_handle->abort(result);
    return;
  }

  // Publish the synthesized audio only after synthesis has completed.
  const std::chrono::nanoseconds period(
      static_cast<int64_t>(1e9 * static_cast<double>(this->chunk_) /
                           static_cast<double>(sample_rate)));

  rclcpp::Rate pub_rate(period);

  // Publish the audio data in chunks
  for (size_t i = 0; i < audio_buffer.size(); i += this->chunk_) {

    const size_t remaining = audio_buffer.size() - i;
    const size_t data_size =
        std::min(static_cast<size_t>(this->chunk_), remaining);
    std::vector<float> data(audio_buffer.begin() + i,
                            audio_buffer.begin() + i + data_size);

    // AudioStamped chunks are required to have exactly chunk_ samples. Zero-pad
    // only the final message.
    if (data.size() < static_cast<size_t>(this->chunk_)) {
      data.resize(this->chunk_, 0.0f);
    }

    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    }

    if (this->stop_worker_) {
      goal_handle->abort(result);
      return;
    }

    auto msg = audio_common_msgs::msg::AudioStamped();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = this->frame_id_;
    msg.audio.audio_data.float32_data = data;
    msg.audio.info.channels = 1;
    msg.audio.info.chunk = this->chunk_;
    msg.audio.info.format = 1;
    msg.audio.info.rate = sample_rate;

    auto feedback = std::make_shared<TTS::Feedback>();
    feedback->audio = msg;

    this->player_pub_->publish(msg);
    goal_handle->publish_feedback(feedback);

    // Do not sleep after the final chunk.
    if (i + data_size < audio_buffer.size()) {
      pub_rate.sleep();
    }
  }

  // A cancel request may race with the final chunk, in which case succeed()
  // fails and the goal must be canceled instead.
  try {
    goal_handle->succeed(result);
  } catch (const std::exception &e) {
    RCLCPP_WARN(this->get_logger(), "Failed to succeed goal: %s", e.what());

    if (goal_handle->is_canceling()) {
      try {
        goal_handle->canceled(result);
      } catch (const std::exception &cancel_error) {
        RCLCPP_WARN(this->get_logger(), "Failed to cancel goal: %s",
                    cancel_error.what());
      }
    }
  }
}
