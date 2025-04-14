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
    : rclcpp_lifecycle::LifecycleNode("piper_node"), pub_rate(nullptr) {

  this->declare_parameter<int>("chunk", 512);
  this->declare_parameter<std::string>("frame_id", "");

  this->declare_parameter<std::string>("model_repo", "");
  this->declare_parameter<std::string>("model_filename", "");
  this->declare_parameter<std::string>("model_path", "");
  this->declare_parameter<std::string>("model_config_repo", "");
  this->declare_parameter<std::string>("model_config_filename", "");
  this->declare_parameter<std::string>("model_config_path", "");

  this->declare_parameter<long int>("speaker_id", 0);
  this->declare_parameter<float>("noise_scale", 0.667f);
  this->declare_parameter<float>("length_scale", 1.0f);
  this->declare_parameter<float>("noise_w", 0.8f);
  this->declare_parameter<float>("sentence_silence_seconds", 0.2f);
  this->declare_parameter<std::vector<long int>>("silence_phonemes",
                                                 std::vector<long int>({}));
  this->declare_parameter<std::vector<double>>("silence_seconds",
                                               std::vector<double>({}));

  this->run_config.e_speak_data_path =
      ament_index_cpp::get_package_share_directory("piper_vendor") +
      "/espeak-ng-data";

  this->run_config.tashkeel_model_path =
      ament_index_cpp::get_package_share_directory("piper_vendor") +
      "/libtashkeel_model.ort";
}

std::string download_model(const std::string &repo_id,
                           const std::string &filename) {

  if (repo_id.empty() || filename.empty()) {
    return "";
  }

  auto result = huggingface_hub::hf_hub_download_with_shards(repo_id, filename);

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

  std::vector<long int> silence_phonemes;
  std::vector<double> silence_seconds;

  RCLCPP_INFO(get_logger(), "[%s] Configuring...", this->get_name());

  this->get_parameter("chunk", this->chunk_);
  this->get_parameter("frame_id", this->frame_id_);

  this->get_parameter("model_repo", model_repo);
  this->get_parameter("model_filename", model_filename);
  this->get_parameter("model_path", this->run_config.model_path);
  this->get_parameter("model_config_repo", model_config_repo);
  this->get_parameter("model_config_filename", model_config_filename);
  this->get_parameter("model_config_path", this->run_config.model_config_path);

  this->get_parameter("speaker_id", this->run_config.speaker_id);
  this->get_parameter("noise_scale", this->run_config.noise_scale);
  this->get_parameter("length_scale", this->run_config.length_scale);
  this->get_parameter("noise_w", this->run_config.noise_w);
  this->get_parameter("sentence_silence_seconds",
                      this->run_config.sentence_silence_seconds);

  this->get_parameter("silence_phonemes", silence_phonemes);
  this->get_parameter("silence_seconds", silence_seconds);

  // download model
  if (this->run_config.model_path.empty()) {
    this->run_config.model_path = download_model(model_repo, model_filename);
  }

  if (this->run_config.model_config_path.empty()) {

    if (model_config_repo.empty()) {
      model_config_repo = model_repo;
    }

    if (model_config_filename.empty()) {
      model_config_filename = model_filename + ".json";
    }

    this->run_config.model_config_path =
        download_model(model_config_repo, model_config_filename);
  }

  if (silence_phonemes.size() != silence_seconds.size()) {
    RCLCPP_ERROR(this->get_logger(),
                 "silence_phonemes (%ld) and silence_seconds (%ld) must have "
                 "the same size",
                 silence_phonemes.size(), silence_seconds.size());

  } else {
    for (size_t i = 0; i < silence_phonemes.size(); i++) {
      this->run_config.phoneme_silence_seconds.insert(
          {silence_phonemes.at(i), silence_phonemes.at(i)});
    }
  }

  RCLCPP_INFO(get_logger(), "[%s] Configured", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PiperNode::on_activate(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(get_logger(), "[%s] Activating...", this->get_name());

  RCLCPP_INFO(get_logger(), "Loading voice from %s (config=%s)",
              this->run_config.model_path.c_str(),
              this->run_config.model_config_path.c_str());
  std::optional<long int> temp_speaker_id = this->run_config.speaker_id;
  loadVoice(this->piper_config, this->run_config.model_path,
            this->run_config.model_config_path, this->voice, temp_speaker_id);

  if (this->voice.phonemizeConfig.phonemeType == piper::eSpeakPhonemes) {
    RCLCPP_INFO(this->get_logger(), "Voice uses eSpeak phonemes (%s)",
                this->voice.phonemizeConfig.eSpeak.voice.c_str());
    this->piper_config.eSpeakDataPath = this->run_config.e_speak_data_path;

  } else {
    // Not using eSpeak
    this->piper_config.useESpeak = false;
  }

  // Enable libtashkeel for Arabic
  if (this->voice.phonemizeConfig.eSpeak.voice == "ar") {
    this->piper_config.useTashkeel = true;
    this->piper_config.tashkeelModelPath = this->run_config.tashkeel_model_path;
  }

  // Init piper
  piper::initialize(this->piper_config);

  // Scales
  if (this->run_config.noise_scale) {
    this->voice.synthesisConfig.noiseScale = this->run_config.noise_scale;
  }

  if (this->run_config.length_scale) {
    this->voice.synthesisConfig.lengthScale = this->run_config.length_scale;
  }

  if (this->run_config.noise_w) {
    voice.synthesisConfig.noiseW = this->run_config.noise_w;
  }

  if (this->run_config.sentence_silence_seconds) {
    voice.synthesisConfig.sentenceSilenceSeconds =
        this->run_config.sentence_silence_seconds;
  }

  // if phonemeSilenceSeconds
  if (this->run_config.phoneme_silence_seconds.size()) {
    if (!this->voice.synthesisConfig.phonemeSilenceSeconds) {
      // Overwrite
      this->voice.synthesisConfig.phonemeSilenceSeconds =
          this->run_config.phoneme_silence_seconds;

    } else {
      // Merge
      for (const auto &[phoneme, silence_seconds] :
           this->run_config.phoneme_silence_seconds) {
        this->voice.synthesisConfig.phonemeSilenceSeconds->try_emplace(
            phoneme, silence_seconds);
      }
    }
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

  // Create audio
  piper::SynthesisResult piper_result;
  std::vector<int16_t> audio_buffer;

  try {
    textToAudio(this->piper_config, this->voice, text, audio_buffer,
                piper_result, NULL);

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
    std::chrono::nanoseconds period(
        (int)(1e9 * this->chunk_ / this->voice.synthesisConfig.sampleRate));
    this->pub_rate = std::make_unique<rclcpp::Rate>(period);
  }

  // Initialize the audio message
  audio_common_msgs::msg::AudioStamped msg;
  msg.header.frame_id = this->frame_id_;

  // Publish the audio data in chunks
  std::vector<float> data(this->chunk_);

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
    msg.audio.info.channels = this->voice.synthesisConfig.channels;
    msg.audio.info.chunk = this->chunk_;
    msg.audio.info.format = 8;
    msg.audio.info.rate = this->voice.synthesisConfig.sampleRate;

    auto feedback = std::make_shared<TTS::Feedback>();
    feedback->audio = msg;

    this->player_pub_->publish(msg);
    goal_handle->publish_feedback(feedback);
    this->pub_rate->sleep();
  }

  goal_handle->succeed(result);
}