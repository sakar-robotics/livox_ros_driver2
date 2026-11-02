//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
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
//

#include "lds_lidar.h"

#include <stdio.h>
#include <string.h>
#include <future>
#include <memory>
#include <mutex>
#include <thread>

#ifdef WIN32
#include <winsock2.h>
#include <ws2def.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif // WIN32
#include "livox_lidar_api.h"
#include "comm/comm.h"
#include "comm/pub_handler.h"

#include "parse_cfg_file/parse_cfg_file.h"
#include "parse_cfg_file/parse_livox_lidar_cfg.h"

#include "call_back/lidar_common_callback.h"
#include "call_back/livox_lidar_callback.h"

using namespace std;

namespace livox_ros {

/** Const varible ------------------------------------------------------------*/
/** For callback use only */
LdsLidar *g_lds_ldiar = nullptr;

/** Global function for common use -------------------------------------------*/

/** Lds lidar function -------------------------------------------------------*/
LdsLidar::LdsLidar(double publish_freq)
    : Lds(publish_freq, kSourceRawLidar), 
      auto_connect_mode_(true),
      whitelist_count_(0),
      is_initialized_(false) {
  memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));
  ResetLdsLidar();
}

LdsLidar::~LdsLidar() {}

void LdsLidar::ResetLdsLidar(void) { ResetLds(kSourceRawLidar); }



bool LdsLidar::InitLdsLidar(const std::string& path_name) {
  if (is_initialized_) {
    printf("Lds is already inited!\n");
    return false;
  }

  if (g_lds_ldiar == nullptr) {
    g_lds_ldiar = this;
  }

  path_ = path_name;
  if (!InitLidars()) {
    return false;
  }
  SetLidarPubHandle();
  if (!Start()) {
    return false;
  }
  is_initialized_ = true;
  return true;
}

bool LdsLidar::InitLidars() {
  if (!ParseSummaryConfig()) {
    return false;
  }
  std::cout << "config lidar type: " << static_cast<int>(lidar_summary_info_.lidar_type) << std::endl;

  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    if (!InitLivoxLidar()) {
      return false;
    }
  }
  return true;
}


bool LdsLidar::Start() {
  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    if (!LivoxLidarStart()) {
      return false;
    }
  }
  return true;
}

bool LdsLidar::ParseSummaryConfig() {
  return ParseCfgFile(path_).ParseSummaryInfo(lidar_summary_info_);
}

bool LdsLidar::InitLivoxLidar() {
#ifdef BUILDING_ROS2
  DisableLivoxSdkConsoleLogger();
#endif

  // parse user config
  LivoxLidarConfigParser parser(path_);
  std::vector<UserLivoxLidarConfig> user_configs;
  if (!parser.Parse(user_configs)) {
    std::cout << "failed to parse user-defined config" << std::endl;
  }

  // SDK initialization
  if (!LivoxLidarSdkInit(path_.c_str())) {
    std::cout << "Failed to init livox lidar sdk." << std::endl;
    return false;
  }

  // fill in lidar devices
  for (auto& config : user_configs) {
    uint8_t index = 0;
    int8_t ret = g_lds_ldiar->cache_index_.GetFreeIndex(kLivoxLidarType, config.handle, index);
    if (ret != 0) {
      std::cout << "failed to get free index, lidar ip: " << IpNumToString(config.handle) << std::endl;
      continue;
    }
    LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[index]);
    p_lidar->lidar_type = kLivoxLidarType;
    p_lidar->livox_config = config;
    p_lidar->handle = config.handle;

    LidarExtParameter lidar_param;
    lidar_param.handle = config.handle;
    lidar_param.lidar_type = kLivoxLidarType;
    if (config.pcl_data_type == kLivoxLidarCartesianCoordinateLowData) {
      // temporary resolution
      lidar_param.param.roll  = config.extrinsic_param.roll;
      lidar_param.param.pitch = config.extrinsic_param.pitch;
      lidar_param.param.yaw   = config.extrinsic_param.yaw;
      lidar_param.param.x     = config.extrinsic_param.x / 10;
      lidar_param.param.y     = config.extrinsic_param.y / 10;
      lidar_param.param.z     = config.extrinsic_param.z / 10;
    } else {
      lidar_param.param.roll  = config.extrinsic_param.roll;
      lidar_param.param.pitch = config.extrinsic_param.pitch;
      lidar_param.param.yaw   = config.extrinsic_param.yaw;
      lidar_param.param.x     = config.extrinsic_param.x;
      lidar_param.param.y     = config.extrinsic_param.y;
      lidar_param.param.z     = config.extrinsic_param.z;
    }
    pub_handler().AddLidarsExtParam(lidar_param);
  }

  SetLivoxLidarInfoChangeCallback(LivoxLidarCallback::LidarInfoChangeCallback, g_lds_ldiar);
  return true;
}

void LdsLidar::SetLidarPubHandle() {
  pub_handler().SetPointCloudsCallback(LidarCommonCallback::OnLidarPointClounCb, g_lds_ldiar);
  pub_handler().SetImuDataCallback(LidarCommonCallback::LidarImuDataCallback, g_lds_ldiar);

  double publish_freq = Lds::GetLdsFrequency();
  pub_handler().SetPointCloudConfig(publish_freq);
}

bool LdsLidar::LivoxLidarStart() {
  return true;
}

namespace {

// The Mid-360's IDLE: motor and laser off, network up. Livox-SDK2 names 0x02 kLivoxLidarWakeUp
// after the HAP, whose states differ, so only a Mid-360 is sent it.
constexpr uint8_t kMid360Idle = 0x02;

const char* WorkStateName(int state) {
  switch (state) {
    case 0x01: return "SAMPLING";
    case 0x02: return "IDLE";
    case 0x04: return "ERROR";
    case 0x05: return "SELFCHECK";
    case 0x06: return "MOTOR_STARTUP";
    case 0x08: return "UPGRADE";
    case 0x09: return "READY";
    default: return state < 0 ? "NOT_ANSWERING" : "UNKNOWN";
  }
}

// Shared with the SDK callback, so an answer that comes after the wait gave up is harmless.
using Answer = std::shared_ptr<std::promise<int>>;

void Deliver(void* client_data, int value) {
  std::unique_ptr<Answer> answer(static_cast<Answer*>(client_data));
  (*answer)->set_value(value);
}

void OnWorkModeSet(livox_status status, uint32_t, LivoxLidarAsyncControlResponse* response,
                   void* client_data) {
  int ret = status;
  if (status == kLivoxLidarStatusSuccess) {
    ret = response == nullptr ? static_cast<int>(kLivoxLidarStatusFailure) : response->ret_code;
  }
  Deliver(client_data, ret);
}

void OnStateQueried(livox_status status, uint32_t, LivoxLidarDiagInternalInfoResponse* response,
                    void* client_data) {
  int state = status != kLivoxLidarStatusSuccess ? status : kLivoxLidarStatusFailure;
  if (status == kLivoxLidarStatusSuccess && response != nullptr && response->ret_code == 0) {
    uint16_t off = 0;
    for (uint16_t i = 0; i < response->param_num; ++i) {
      auto* kv = reinterpret_cast<LivoxLidarKeyValueParam*>(&response->data[off]);
      if (kv->key == kKeyCurWorkState) {
        state = kv->value[0];
      }
      off += sizeof(uint16_t) * 2 + kv->length;
    }
  }
  Deliver(client_data, state);
}

// Sends one request and waits for its callback until `deadline`: the lidar's answer (>= 0) or an
// SDK status (< 0).
template <typename Send>
int Ask(Send send, std::chrono::steady_clock::time_point deadline) {
  auto answer = std::make_shared<std::promise<int>>();
  std::future<int> future = answer->get_future();
  auto* held = new Answer(answer);
  const livox_status sent = send(held);
  if (sent != kLivoxLidarStatusSuccess) {
    delete held;  // the SDK calls back only for a request it sent
    return sent;
  }
  if (future.wait_until(deadline) != std::future_status::ready) {
    return kLivoxLidarStatusTimeout;
  }
  return future.get();
}

// The lidar's current work state (0x01 SAMPLING, ...), or an SDK status (< 0).
int QueryWorkState(uint32_t handle, std::chrono::steady_clock::time_point deadline) {
  return Ask([&](void* held) {
    return QueryLivoxLidarInternalInfo(handle, OnStateQueried, held);
  }, deadline);
}

}  // namespace

std::vector<uint32_t> LdsLidar::Mid360Handles() {
  std::lock_guard<std::mutex> lock(dev_types_mutex_);
  std::vector<uint32_t> handles;
  for (const auto& lidar : dev_types_) {
    if (lidar.second == kLivoxLidarTypeMid360) {
      handles.push_back(lidar.first);
    }
  }
  return handles;
}

std::string LdsLidar::WorkStateSummary(std::chrono::milliseconds timeout) {
  const auto handles = Mid360Handles();
  if (handles.empty()) {
    return "NO_LIDAR";
  }
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  if (handles.size() == 1) {
    return WorkStateName(QueryWorkState(handles.front(), deadline));
  }
  std::string summary;
  for (const uint32_t handle : handles) {
    summary += (summary.empty() ? "" : ", ") + IpNumToString(handle) + ": " +
               WorkStateName(QueryWorkState(handle, deadline));
  }
  return summary;
}

void LdsLidar::NoteLidarType(uint32_t handle, uint8_t dev_type) {
  std::lock_guard<std::mutex> lock(dev_types_mutex_);
  dev_types_[handle] = dev_type;
}

LivoxLidarWorkMode LdsLidar::TargetWorkMode(uint32_t handle) {
  std::lock_guard<std::mutex> lock(dev_types_mutex_);
  const auto it = dev_types_.find(handle);
  if (!sampling_ && it != dev_types_.end() && it->second == kLivoxLidarTypeMid360) {
    return static_cast<LivoxLidarWorkMode>(kMid360Idle);
  }
  return kLivoxLidarNormal;
}

bool LdsLidar::SetSampling(bool sampling, std::chrono::milliseconds timeout, std::string& message) {
  using Clock = std::chrono::steady_clock;
  const auto handles = Mid360Handles();
  if (handles.empty()) {
    message = "no Mid-360 connected";
    return false;
  }

  const uint8_t mode = sampling ? static_cast<uint8_t>(kLivoxLidarNormal) : kMid360Idle;
  const auto start = Clock::now();
  const auto deadline = start + timeout;
  for (const uint32_t handle : handles) {
    const int ret = Ask([&](void* held) {
      return SetLivoxLidarWorkMode(handle, static_cast<LivoxLidarWorkMode>(mode), OnWorkModeSet,
                                   held);
    }, deadline);
    if (ret != 0) {
      message = IpNumToString(handle) + (ret < 0 ? " did not answer " : " refused ") +
                WorkStateName(mode) + " (" + std::to_string(ret) + ")";
      return false;
    }
  }
  // Only now: the lidars have taken the mode, and keep it even across a power cut. A request that
  // failed before this changes nothing, also for a lidar that reconnects later.
  sampling_ = sampling;

  // From IDLE the motor has to start first: ~6.5 s to SAMPLING.
  for (const uint32_t handle : handles) {
    int state = QueryWorkState(handle, deadline);
    while (state != mode) {
      if (Clock::now() + std::chrono::milliseconds(200) >= deadline) {
        message = IpNumToString(handle) + " took " + WorkStateName(mode) + " but is " +
                  WorkStateName(state) + " after " + std::to_string(timeout.count()) + " ms";
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      state = QueryWorkState(handle, deadline);
    }
  }
  const auto elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count();
  message = std::string(WorkStateName(mode)) + " after " + std::to_string(elapsed) + " ms";
  return true;
}

int LdsLidar::DeInitLdsLidar(void) {
  if (!is_initialized_) {
    printf("LiDAR data source is not exit");
    return -1;
  }

  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    if (idle_on_exit_) {
      std::string message;
      const bool idle = SetSampling(false, std::chrono::seconds(2), message);
      std::cout << (idle ? "lidar idle on exit: " : "failed to idle the lidar on exit: ")
                << message << std::endl;
    }
    LivoxLidarSdkUninit();
    printf("Livox Lidar SDK Deinit completely!\n");
  }

  return 0;
}

void LdsLidar::PrepareExit(void) { DeInitLdsLidar(); }

}  // namespace livox_ros
