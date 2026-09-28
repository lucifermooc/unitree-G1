#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <unitree_hg/msg/bms_state.hpp>

namespace g1_nav_bridge {

class BatteryStateBridge final : public rclcpp::Node {
 public:
  BatteryStateBridge() : Node("g1_battery_state_bridge") {
    const auto input_topic =
        declare_parameter<std::string>("input_topic", "/lf/bmsstate");
    const auto output_topic =
        declare_parameter<std::string>("output_topic", "/battery_state");
    location_ = declare_parameter<std::string>("location", "G1 battery");

    publisher_ = create_publisher<sensor_msgs::msg::BatteryState>(
        output_topic, rclcpp::QoS(10).reliable());
    subscription_ = create_subscription<unitree_hg::msg::BmsState>(
        input_topic, rclcpp::SensorDataQoS(),
        std::bind(&BatteryStateBridge::OnBmsState, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "G1 battery bridge: %s -> %s",
                input_topic.c_str(), output_topic.c_str());
  }

 private:
  void OnBmsState(const unitree_hg::msg::BmsState::SharedPtr message) {
    sensor_msgs::msg::BatteryState battery;
    battery.header.stamp = now();

    const auto nan = std::numeric_limits<float>::quiet_NaN();
    battery.voltage = nan;
    for (const auto voltage_mv : message->bmsvoltage) {
      if (voltage_mv > 0) {
        battery.voltage = static_cast<float>(voltage_mv) / 1000.0F;
        break;
      }
    }

    battery.current = static_cast<float>(message->current) / 1000.0F;
    battery.percentage = std::clamp(
        static_cast<float>(message->soc) / 100.0F, 0.0F, 1.0F);

    battery.temperature = nan;
    for (const auto temperature : message->temperature) {
      if (temperature != 0) {
        battery.temperature = static_cast<float>(temperature);
        break;
      }
    }

    battery.charge = nan;
    battery.capacity = nan;
    battery.design_capacity = nan;

    constexpr float kCurrentDeadband = 0.05F;
    if (battery.current < -kCurrentDeadband) {
      battery.power_supply_status =
          sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    } else if (battery.current > kCurrentDeadband) {
      battery.power_supply_status =
          sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
    } else if (battery.percentage >= 0.99F) {
      battery.power_supply_status =
          sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
    } else {
      battery.power_supply_status =
          sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
    }

    // SOH 是百分比，而 BatteryState health 是故障枚举，不能直接转换。
    battery.power_supply_health =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    battery.power_supply_technology =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    battery.present = true;

    battery.cell_voltage.reserve(message->cell_vol.size());
    for (const auto cell_mv : message->cell_vol) {
      if (cell_mv > 0) {
        battery.cell_voltage.push_back(
            static_cast<float>(cell_mv) / 1000.0F);
      }
    }
    // BMS 温度探头并不与单体电芯一一对应。
    battery.cell_temperature.clear();
    battery.location = location_;

    publisher_->publish(battery);
    // 每 5 s 一条的电量播报会淹没导航日志；降为 DEBUG。
    // 需要时用 --log-level g1_battery_state_bridge:=debug 打开，
    // 或直接订阅 /battery_state 看数据。
    RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Battery: %.1f%%, %.3f V, %.3f A, temp=%.1f C, cells=%zu",
        battery.percentage * 100.0F, battery.voltage, battery.current,
        battery.temperature, battery.cell_voltage.size());
  }

  std::string location_;
  rclcpp::Subscription<unitree_hg::msg::BmsState>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_;
};

}  // namespace g1_nav_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<g1_nav_bridge::BatteryStateBridge>());
  rclcpp::shutdown();
  return 0;
}
