#pragma once

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ros {

class Duration;

class Time {
public:
  Time() : time_(0, 0, RCL_ROS_TIME) {}
  explicit Time(double seconds)
      : time_(static_cast<int64_t>(std::max(0.0, seconds) * 1000000000.0), RCL_ROS_TIME) {}
  Time(int32_t sec, uint32_t nsec)
      : time_(sec < 0 ? 0 : sec, sec < 0 ? 0U : nsec, RCL_ROS_TIME) {}
  Time(const builtin_interfaces::msg::Time& msg) : time_(clamp_message_time(msg), RCL_ROS_TIME) {}
  Time(const rclcpp::Time& time) : time_(time) {}

  static Time now();

  double toSec() const { return time_.seconds(); }
  int64_t nanoseconds() const { return time_.nanoseconds(); }
  bool isZero() const { return time_.nanoseconds() == 0; }
  builtin_interfaces::msg::Time to_msg() const {
    builtin_interfaces::msg::Time msg;
    const int64_t ns = time_.nanoseconds();
    msg.sec = static_cast<int32_t>(ns / 1000000000LL);
    msg.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    return msg;
  }

  operator builtin_interfaces::msg::Time() const { return to_msg(); }
  operator rclcpp::Time() const { return time_; }

  Time& operator+=(const Duration& duration);

private:
  static builtin_interfaces::msg::Time clamp_message_time(const builtin_interfaces::msg::Time& msg) {
    if (msg.sec >= 0) {
      return msg;
    }
    builtin_interfaces::msg::Time zero;
    zero.sec = 0;
    zero.nanosec = 0;
    return zero;
  }

  rclcpp::Time time_;
};

class Duration {
public:
  Duration() : duration_(0, 0) {}
  explicit Duration(double seconds)
      : duration_(std::chrono::nanoseconds(static_cast<int64_t>(seconds * 1000000000.0))) {}
  explicit Duration(std::chrono::nanoseconds duration) : duration_(duration) {}
  Duration(int32_t sec, uint32_t nsec) : duration_(sec, nsec) {}
  explicit Duration(const rclcpp::Duration& duration) : duration_(duration) {}

  double toSec() const { return duration_.seconds(); }
  int64_t nanoseconds() const { return duration_.nanoseconds(); }
  std::chrono::nanoseconds to_chrono() const {
    return std::chrono::nanoseconds(duration_.nanoseconds());
  }
  void sleep() const { rclcpp::sleep_for(to_chrono()); }

  operator rclcpp::Duration() const { return duration_; }

private:
  rclcpp::Duration duration_;
};

inline Time Time::now() { return Time(rclcpp::Clock(RCL_ROS_TIME).now()); }

static const Time TIME_MIN(std::numeric_limits<int32_t>::min(), 0U);
static const Time TIME_MAX(std::numeric_limits<int32_t>::max(), 999999999U);

inline Time& Time::operator+=(const Duration& duration) {
  time_ = time_ + static_cast<rclcpp::Duration>(duration);
  return *this;
}

inline Time operator+(const Time& lhs, const Duration& rhs) {
  Time result(lhs);
  result += rhs;
  return result;
}

inline Duration operator-(const Time& lhs, const Time& rhs) {
  return Duration(std::chrono::nanoseconds(lhs.nanoseconds() - rhs.nanoseconds()));
}

inline Duration operator-(const builtin_interfaces::msg::Time& lhs, const Time& rhs) {
  return Time(lhs) - rhs;
}

inline Duration operator-(const Time& lhs, const builtin_interfaces::msg::Time& rhs) {
  return lhs - Time(rhs);
}

inline bool operator<(const Time& lhs, const Time& rhs) {
  return lhs.nanoseconds() < rhs.nanoseconds();
}
inline bool operator>(const Time& lhs, const Time& rhs) { return rhs < lhs; }
inline bool operator<=(const Time& lhs, const Time& rhs) { return !(rhs < lhs); }
inline bool operator>=(const Time& lhs, const Time& rhs) { return !(lhs < rhs); }
inline bool operator==(const Time& lhs, const Time& rhs) {
  return lhs.nanoseconds() == rhs.nanoseconds();
}
inline bool operator!=(const Time& lhs, const Time& rhs) { return !(lhs == rhs); }

class Rate {
public:
  explicit Rate(double hz) : rate_(hz) {}
  void sleep() { rate_.sleep(); }

private:
  rclcpp::Rate rate_;
};

struct TimerEvent {
  Time last_real;
  Time current_real;
};

class Timer {
public:
  Timer() = default;
  explicit Timer(rclcpp::TimerBase::SharedPtr timer) : timer_(std::move(timer)) {}

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

class Publisher {
public:
  Publisher() = default;

  template <typename MessageT>
  explicit Publisher(std::shared_ptr<rclcpp::Publisher<MessageT>> publisher)
      : publisher_(std::move(publisher)) {}

  template <typename MessageT>
  void publish(const MessageT& msg) const {
    using CleanT = typename std::remove_cv<typename std::remove_reference<MessageT>::type>::type;
    std::static_pointer_cast<rclcpp::Publisher<CleanT>>(publisher_)->publish(msg);
  }

  template <typename MessageT>
  void publish(const std::shared_ptr<MessageT>& msg) const {
    if (msg) {
      publish(*msg);
    }
  }

  size_t getNumSubscribers() const {
    return publisher_ ? publisher_->get_subscription_count() : 0;
  }

  explicit operator bool() const { return static_cast<bool>(publisher_); }

private:
  std::shared_ptr<rclcpp::PublisherBase> publisher_;
};

class Subscriber {
public:
  Subscriber() = default;
  explicit Subscriber(rclcpp::SubscriptionBase::SharedPtr subscription)
      : subscription_(std::move(subscription)) {}
  void shutdown() { subscription_.reset(); }

private:
  rclcpp::SubscriptionBase::SharedPtr subscription_;
};

class TransportHints {
public:
  TransportHints& tcpNoDelay() { return *this; }
};

namespace detail {

inline std::string normalize_parameter_name(std::string name) {
  std::replace(name.begin(), name.end(), '/', '.');
  while (!name.empty() && name.front() == '.') {
    name.erase(name.begin());
  }
  return name;
}

template <typename T>
struct function_traits : function_traits<decltype(&T::operator())> {};

template <typename R, typename A>
struct function_traits<R (*)(A)> {
  using arg_type = A;
};

template <typename C, typename R, typename A>
struct function_traits<R (C::*)(A)> {
  using arg_type = A;
};

template <typename C, typename R, typename A>
struct function_traits<R (C::*)(A) const> {
  using arg_type = A;
};

template <typename T>
struct message_from_arg {
  using clean_type = typename std::remove_cv<typename std::remove_reference<T>::type>::type;
  using type = clean_type;
};

template <typename M>
struct message_from_arg<std::shared_ptr<M>> {
  using type = typename std::remove_cv<M>::type;
};

template <typename M>
struct message_from_arg<const std::shared_ptr<M>&> {
  using type = typename std::remove_cv<M>::type;
};

template <typename M>
struct message_from_arg<std::shared_ptr<const M>> {
  using type = M;
};

template <typename M>
struct message_from_arg<const std::shared_ptr<const M>&> {
  using type = M;
};

template <typename CallbackT, typename MessageT>
void invoke_free_callback(CallbackT&& callback, typename MessageT::ConstSharedPtr msg) {
  if constexpr (std::is_invocable_v<CallbackT, typename MessageT::ConstSharedPtr>) {
    callback(msg);
  } else if constexpr (std::is_invocable_v<CallbackT, const typename MessageT::ConstSharedPtr&>) {
    callback(msg);
  } else if constexpr (std::is_invocable_v<CallbackT, const MessageT&>) {
    callback(*msg);
  } else {
    callback(*msg);
  }
}

template <typename MethodT, typename ObjT, typename MessageT>
void invoke_member_callback(MethodT method, ObjT* object, typename MessageT::ConstSharedPtr msg) {
  if constexpr (std::is_invocable_v<MethodT, ObjT*, typename MessageT::ConstSharedPtr>) {
    (object->*method)(msg);
  } else if constexpr (std::is_invocable_v<MethodT, ObjT*, const typename MessageT::ConstSharedPtr&>) {
    (object->*method)(msg);
  } else if constexpr (std::is_invocable_v<MethodT, ObjT*, const MessageT&>) {
    (object->*method)(*msg);
  } else {
    (object->*method)(*msg);
  }
}

}  // namespace detail

class NodeHandle {
public:
  NodeHandle();
  explicit NodeHandle(const std::string& ns);
  explicit NodeHandle(const NodeHandle&) = default;

  rclcpp::Node::SharedPtr node() const { return node_; }
  rclcpp::Node* raw() const { return node_.get(); }

  bool ok() const { return rclcpp::ok(); }

  template <typename T>
  void param(const std::string& name, T& value, const T& default_value) const {
    const auto parameter_name = detail::normalize_parameter_name(name);
    if (!node_->has_parameter(parameter_name)) {
      node_->declare_parameter<T>(parameter_name, default_value);
    }
    value = node_->get_parameter(parameter_name).get_value<T>();
  }

  template <typename T>
  bool getParam(const std::string& name, T& value) const {
    const auto parameter_name = detail::normalize_parameter_name(name);
    if (!node_->has_parameter(parameter_name)) {
      node_->declare_parameter<T>(parameter_name, T{});
    }
    return node_->get_parameter(parameter_name, value);
  }

  template <typename T>
  void setParam(const std::string& name, const T& value) const {
    const auto parameter_name = detail::normalize_parameter_name(name);
    if (!node_->has_parameter(parameter_name)) {
      node_->declare_parameter<T>(parameter_name, value);
    }
    node_->set_parameter(rclcpp::Parameter(parameter_name, value));
  }

  template <typename MessageT>
  Publisher advertise(const std::string& topic, size_t queue_size, bool latch = false) const {
    rclcpp::QoS qos(queue_size);
    if (latch) {
      qos.transient_local().reliable();
    }
    return Publisher(node_->create_publisher<MessageT>(resolve_topic(topic), qos));
  }

  template <typename CallbackT>
  Subscriber subscribe(const std::string& topic, size_t queue_size, CallbackT&& callback,
                       const TransportHints& = TransportHints()) const {
    using ArgT = typename detail::function_traits<typename std::decay<CallbackT>::type>::arg_type;
    using MessageT = typename detail::message_from_arg<ArgT>::type;
    return subscribe<MessageT>(topic, queue_size, std::forward<CallbackT>(callback));
  }

  template <typename MessageT, typename CallbackT>
  Subscriber subscribe(const std::string& topic, size_t queue_size, CallbackT&& callback,
                       const TransportHints& = TransportHints()) const {
    using StoredCallbackT = typename std::decay<CallbackT>::type;
    auto stored_callback =
        std::make_shared<StoredCallbackT>(std::forward<CallbackT>(callback));
    auto subscription = node_->create_subscription<MessageT>(
        resolve_topic(topic), rclcpp::QoS(queue_size),
        [stored_callback](typename MessageT::ConstSharedPtr msg) {
          detail::invoke_free_callback<StoredCallbackT&, MessageT>(*stored_callback, msg);
        });
    return Subscriber(subscription);
  }

  template <typename MethodT, typename ObjT>
  Subscriber subscribe(const std::string& topic, size_t queue_size, MethodT method, ObjT* object,
                       const TransportHints& = TransportHints()) const {
    using ArgT = typename detail::function_traits<MethodT>::arg_type;
    using MessageT = typename detail::message_from_arg<ArgT>::type;
    return subscribe<MessageT>(topic, queue_size, method, object);
  }

  template <typename MessageT, typename MethodT, typename ObjT>
  Subscriber subscribe(const std::string& topic, size_t queue_size, MethodT method, ObjT* object,
                       const TransportHints& = TransportHints()) const {
    auto subscription = node_->create_subscription<MessageT>(
        resolve_topic(topic), rclcpp::QoS(queue_size),
        [method, object](typename MessageT::ConstSharedPtr msg) {
          detail::invoke_member_callback<MethodT, ObjT, MessageT>(method, object, msg);
        });
    return Subscriber(subscription);
  }

  template <typename CallbackT>
  Timer createTimer(const Duration& period, CallbackT&& callback) const {
    using StoredCallbackT = typename std::decay<CallbackT>::type;
    auto stored_callback =
        std::make_shared<StoredCallbackT>(std::forward<CallbackT>(callback));
    auto last_real = std::make_shared<Time>(Time::now());
    auto timer = node_->create_wall_timer(period.to_chrono(), [stored_callback, last_real]() {
      TimerEvent event;
      event.last_real = *last_real;
      event.current_real = Time::now();
      *last_real = event.current_real;
      (*stored_callback)(event);
    });
    return Timer(timer);
  }

  template <typename MethodT, typename ObjT>
  Timer createTimer(const Duration& period, MethodT method, ObjT* object) const {
    auto last_real = std::make_shared<Time>(Time::now());
    auto timer = node_->create_wall_timer(period.to_chrono(), [method, object, last_real]() {
      TimerEvent event;
      event.last_real = *last_real;
      event.current_real = Time::now();
      *last_real = event.current_real;
      (object->*method)(event);
    });
    return Timer(timer);
  }

private:
  std::string resolve_topic(const std::string& topic) const {
    if (!private_ns_ || topic.empty() || topic.front() == '/' || topic.front() == '~') {
      return topic;
    }
    return "~/" + topic;
  }

  rclcpp::Node::SharedPtr node_;
  bool private_ns_{false};
};

inline rclcpp::Node::SharedPtr& global_node() {
  static rclcpp::Node::SharedPtr node;
  return node;
}

inline rclcpp::Logger get_logger() {
  return global_node() ? global_node()->get_logger() : rclcpp::get_logger("fuel_ros2_compat");
}

inline rclcpp::Clock::SharedPtr get_clock() {
  return global_node() ? global_node()->get_clock() : std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
}

inline void init(int argc, char** argv, const std::string& name) {
  if (!rclcpp::ok()) {
    rclcpp::init(argc, argv);
  }
  global_node() = std::make_shared<rclcpp::Node>(name);
}

inline bool ok() { return rclcpp::ok(); }
inline void spin() { rclcpp::spin(global_node()); }
inline void spinOnce() { rclcpp::spin_some(global_node()); }
inline void shutdown() { rclcpp::shutdown(); }

inline NodeHandle::NodeHandle() : node_(global_node()) {
  if (!node_) {
    node_ = std::make_shared<rclcpp::Node>("fuel_ros2_compat_node");
    global_node() = node_;
  }
}

inline NodeHandle::NodeHandle(const std::string& ns) : NodeHandle() {
  private_ns_ = (ns == "~");
}

namespace this_node {
inline std::string getName() { return global_node() ? global_node()->get_name() : std::string{}; }
}  // namespace this_node

class Exception : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

}  // namespace ros

namespace builtin_interfaces {
namespace msg {
inline bool operator<(const Time& lhs, const Time& rhs) {
  return std::tie(lhs.sec, lhs.nanosec) < std::tie(rhs.sec, rhs.nanosec);
}
inline bool operator>(const Time& lhs, const Time& rhs) { return rhs < lhs; }
}  // namespace msg
}  // namespace builtin_interfaces

#define ROS_INFO(...) RCLCPP_INFO(::ros::get_logger(), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(::ros::get_logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(::ros::get_logger(), __VA_ARGS__)
#define ROS_DEBUG(...) RCLCPP_DEBUG(::ros::get_logger(), __VA_ARGS__)
#define ROS_INFO_STREAM(args) RCLCPP_INFO_STREAM(::ros::get_logger(), args)
#define ROS_WARN_STREAM(args) RCLCPP_WARN_STREAM(::ros::get_logger(), args)
#define ROS_ERROR_STREAM(args) RCLCPP_ERROR_STREAM(::ros::get_logger(), args)
#define ROS_DEBUG_STREAM(args) RCLCPP_DEBUG_STREAM(::ros::get_logger(), args)
#define ROS_INFO_STREAM_THROTTLE(period, args) RCLCPP_INFO_STREAM(::ros::get_logger(), args)
#define ROS_WARN_STREAM_THROTTLE(period, args) RCLCPP_WARN_STREAM(::ros::get_logger(), args)
#define ROS_ERROR_STREAM_THROTTLE(period, args) RCLCPP_ERROR_STREAM(::ros::get_logger(), args)
#define ROS_INFO_THROTTLE(period, ...) ROS_INFO(__VA_ARGS__)
#define ROS_WARN_THROTTLE(period, ...) ROS_WARN(__VA_ARGS__)
#define ROS_ERROR_THROTTLE(period, ...) ROS_ERROR(__VA_ARGS__)
#define ROS_ERROR_COND(cond, ...) \
  do {                            \
    if (cond) {                   \
      ROS_ERROR(__VA_ARGS__);     \
    }                             \
  } while (0)
#define ROS_ASSERT(cond) assert(cond)
#define ROS_ASSERT_MSG(cond, ...) \
  do {                            \
    if (!(cond)) {                \
      ROS_ERROR(__VA_ARGS__);     \
      assert(cond);               \
    }                             \
  } while (0)
