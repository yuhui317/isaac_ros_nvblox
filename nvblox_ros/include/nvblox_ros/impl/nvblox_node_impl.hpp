// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef NVBLOX_ROS__IMPL__NVBLOX_NODE_IMPL_HPP_
#define NVBLOX_ROS__IMPL__NVBLOX_NODE_IMPL_HPP_

#include <nvblox/utils/delays.h>
#include <nvblox/utils/rates.h>
#include <nvblox/utils/timing.h>

#include <list>
#include <memory>
#include <string>
#include <vector>

// 学习入口：ROS 消息/服务的有界队列模板；位姿就绪筛选与具体积分解耦，返回失败没有重入队语义。
namespace nvblox
{

template<typename QueuedType>
// 复制并移除所有就绪项后解锁处理；process_function 的 bool 返回值被忽略，所以失败不会自动重试。
void NvbloxNode::processQueue(
  std::unique_ptr<std::list<QueuedType>> & queue_ptr, std::mutex * queue_mutex_ptr,
  ReadyCheckFunctionType<QueuedType> ready_check_function,
  ProcessFunctionType<QueuedType> process_function)
{
  timing::Timer ros_total_timer("ros/process_queue");
  if (queue_ptr->empty()) {
    return;
  }

  // 只在遍历/移除阶段持锁，GPU 积分在解锁后发生；这不等于就绪项的处理结果会重新检查。
  // To avoid holding onto the queue lock longer than necessary, we first create copies of the
  // elements in the queue before processing them.
  std::unique_lock<std::mutex> lock(*queue_mutex_ptr);
  std::vector<QueuedType> items_to_process;
  items_to_process.reserve(queue_ptr->size());

  typename std::list<QueuedType>::iterator itr = queue_ptr->begin();
  while (itr != queue_ptr->end()) {
    if (ready_check_function(*itr)) {
      items_to_process.push_back(*itr);
      itr = queue_ptr->erase(itr);
    } else {
      ++itr;
    }
  }
  lock.unlock();

  // 处理返回值未被使用；定位刚可用但后续转换失败的项也已从输入队列移走。
  // Process all items
  for (auto item : items_to_process) {
    process_function(item);
  }
}


template<typename QueuedType>
// 互斥追加输入；达到最大长度后丢弃最旧项，并按配置记录累计丢队列项。
void NvbloxNode::pushOntoQueue(
  const std::string & queue_name, QueuedType item,
  std::unique_ptr<std::list<QueuedType>> & queue_ptr,
  std::mutex * queue_mutex_ptr)
{
  timing::Timer lock_timer("ros/push_onto_queue");
  std::unique_lock<std::mutex> lock(*queue_mutex_ptr);
  // Size should never grow larger than allowed since this is the only place where we push to the
  // queue.
  CHECK(queue_ptr->size() <= static_cast<size_t>(params_.maximum_input_queue_length));

  queue_ptr->emplace_back(item);
  const int queue_ptr_size = queue_ptr->size();

  // maximum_input_queue_length 是条数限制；丢最旧消息可以限制延迟但会丢失观测。
  // Remove oldest item if queue is full
  if (queue_ptr_size > params_.maximum_input_queue_length) {
    queue_ptr->pop_front();
    lock.unlock();

    // Print some debug info
    if (params_.print_queue_drops_to_console) {
      number_of_dropped_queued_items_[queue_name]++;
      auto & clk = *get_clock();
      RCLCPP_INFO_STREAM_THROTTLE(
        get_logger(), clk, params_.print_statistics_on_console_period_ms,
        "Dropped an item from: " <<
          queue_name << ". Size of queue: " << queue_ptr_size <<
          ". Total number of dropped items is: " <<
          number_of_dropped_queued_items_[queue_name]);
    }
  } else {
    lock.unlock();
  }
}

}  // namespace nvblox

#endif  // NVBLOX_ROS__IMPL__NVBLOX_NODE_IMPL_HPP_
