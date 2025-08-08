enum class ExecType { Timer, Subscription };

struct AvailableExecutable {
  // common
  rclcpp::AnyExecutable any;
  std::string node_name;
  ExecType type;

  // for topics
  std::string topic_name;
};

bool set_any_executable(std::vector<AvailableExecutable>& all_callbacks, int idx, rclcpp::AnyExecutable& any_executable) {
  if(all_callbacks[idx].type == ExecType::Timer) {
        // Check that the timer should be called still, i.e. it wasn't canceled.
        any_executable.data = all_callbacks[idx].any.timer->call();
        if (!any_executable.data) return false;
        any_executable.timer = all_callbacks[idx].any.timer;
        any_executable.callback_group = all_callbacks[idx].any.callback_group;
  }
  else {
      any_executable.subscription = all_callbacks[idx].any.subscription;
      any_executable.callback_group = all_callbacks[idx].any.callback_group;
  }
  return true;
}

bool Executor::get_next_ready_executable_custom(AnyExecutable & any_executable)
{
  TRACETOOLS_TRACEPOINT(rclcpp_executor_get_next_ready);
  
  std::vector<AvailableExecutable> available_next_callbacks;

  if (!wait_result_.has_value() || wait_result_->kind() != rclcpp::WaitResultKind::Ready) {
    return false;
  }

  { // timers
    size_t current_timer_index = 0;
    while (true) {
      auto [timer, timer_index] = wait_result_->peek_next_ready_timer(current_timer_index);
      if (!timer) break;
      current_timer_index = timer_index;
      
      auto entity_iter = current_collection_.timers.find(timer->get_timer_handle().get());
      if (entity_iter != current_collection_.timers.end()) {
        auto callback_group = entity_iter->second.callback_group.lock();
        if (!callback_group || !callback_group->can_be_taken_from()) {
          current_timer_index++;
          continue;
        }
        // At this point the timer is either ready for execution or was perhaps
        // it was canceled, based on the result of call(), but either way it
        // should not be checked again from peek_next_ready_timer(), so clear
        // it from the wait result.
        wait_result_->clear_timer_with_index(current_timer_index);

        rclcpp::AnyExecutable timer_executable;
        timer_executable.timer = timer;
        timer_executable.callback_group = callback_group;

        // auto nbi = timer->get_node_base_interface().lock();
        std::string node_name = "UNKNOWN";
        std::string topic_name = "";
        
        available_next_callbacks.push_back(AvailableExecutable{std::move(timer_executable), std::move(node_name), ExecType::Timer, topic_name});
      }
      current_timer_index++;
    }
  }

  { // topic subscriptions
    while (auto subscription = wait_result_->next_ready_subscription()) {
      auto entity_iter = current_collection_.subscriptions.find(
        subscription->get_subscription_handle().get());
      if (entity_iter != current_collection_.subscriptions.end()) {
        auto callback_group = entity_iter->second.callback_group.lock();
        if (!callback_group || !callback_group->can_be_taken_from()) continue;
        
        rclcpp::AnyExecutable topic_callback;
        topic_callback.subscription = subscription;
        topic_callback.callback_group = callback_group;
        std::string node_name = subscription->get_node_base()->get_fully_qualified_name();
        std::string topic_name = subscription->get_topic_name(); 


        available_next_callbacks.push_back(AvailableExecutable{std::move(topic_callback), std::move(node_name), ExecType::Timer, topic_name});
      }
    }
  }

  // if (!valid_executable) {
  //   while (auto service = wait_result_->next_ready_service()) {
  //     auto entity_iter = current_collection_.services.find(service->get_service_handle().get());
  //     if (entity_iter != current_collection_.services.end()) {
  //       auto callback_group = entity_iter->second.callback_group.lock();
  //       if (!callback_group || !callback_group->can_be_taken_from()) {
  //         continue;
  //       }
  //       any_executable.service = service;
  //       any_executable.callback_group = callback_group;
  //       valid_executable = true;
  //       break;
  //     }
  //   }
  // }

  // if (!valid_executable) {
  //   while (auto client = wait_result_->next_ready_client()) {
  //     auto entity_iter = current_collection_.clients.find(client->get_client_handle().get());
  //     if (entity_iter != current_collection_.clients.end()) {
  //       auto callback_group = entity_iter->second.callback_group.lock();
  //       if (!callback_group || !callback_group->can_be_taken_from()) {
  //         continue;
  //       }
  //       any_executable.client = client;
  //       any_executable.callback_group = callback_group;
  //       valid_executable = true;
  //       break;
  //     }
  //   }
  // }

  // if (!valid_executable) {
  //   while (auto waitable = wait_result_->next_ready_waitable()) {
  //     auto entity_iter = current_collection_.waitables.find(waitable.get());
  //     if (entity_iter != current_collection_.waitables.end()) {
  //       auto callback_group = entity_iter->second.callback_group.lock();
  //       if (!callback_group || !callback_group->can_be_taken_from()) {
  //         continue;
  //       }
  //       any_executable.waitable = waitable;
  //       any_executable.callback_group = callback_group;
  //       any_executable.data = waitable->take_data();
  //       valid_executable = true;
  //       break;
  //     }
  //   }
  // } 

  if(available_next_callbacks.size() > 0) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Number of callbacks available: %d", (int) available_next_callbacks.size());
    for(auto it=available_next_callbacks.begin(); it != available_next_callbacks.end(); ++it){
      if(it->type == ExecType::Subscription) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "TOPIC: Node %s, topic: %s", it->node_name.c_str(), it->topic_name.c_str());
      } 
      else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "TIMER: Node %s", it->node_name.c_str());
      }
    } 
  }

  if (any_executable.callback_group) {
    if (any_executable.callback_group->type() == CallbackGroupType::MutuallyExclusive) {
      assert(any_executable.callback_group->can_be_taken_from().load());
      any_executable.callback_group->can_be_taken_from().store(false);
    }
  }

  for(int i=0; i<(int)available_next_callbacks.size(); ++i){
    if(set_any_executable(available_next_callbacks, i, any_executable)) {
      return true;
    }
    else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Was unable to set idx: %d", i);
    }
  }
  return false;
}

bool Executor::get_next_executable_custom(AnyExecutable & any_executable, std::chrono::nanoseconds timeout)
{
  bool success = false;
  // Check to see if there are any subscriptions or timers needing service
  // TODO(wjwwood): improve run to run efficiency of this function
  success = get_next_ready_executable_custom(any_executable);
  // If there are none
  if (!success) {
    // Wait for subscriptions or timers to work on
    wait_for_work(timeout);
    if (!spinning.load()) {
      return false;
    }
    // Try again
    success = get_next_ready_executable_custom(any_executable);
  }
  return success;
}