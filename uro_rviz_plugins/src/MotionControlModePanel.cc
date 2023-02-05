#include "MotionControlModePanel.hh"

#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QCheckBox>

#include <chrono>
#include <thread>
#include <mutex>

using namespace std::placeholders;
using namespace std::chrono_literals;
using namespace uro_rviz_plugins;

MotionControlModePanel::MotionControlModePanel(QWidget *parent)
    : rviz_common::Panel(parent)
{
  QVBoxLayout *vbox_layout = new QVBoxLayout();
  QHBoxLayout *hbox_layout = new QHBoxLayout();
  QLabel *label = new QLabel("Main mode");
  checkbox_off = new QCheckBox("OFF");
  checkbox_wbc = new QCheckBox("WBC");
  checkbox_rov = new QCheckBox("ROV");
  QPushButton *push_button = new QPushButton("Apply");
  label_status = new QLabel("");
  hbox_layout->addWidget(label);
  hbox_layout->addWidget(checkbox_off);
  hbox_layout->addWidget(checkbox_wbc);
  hbox_layout->addWidget(checkbox_rov);
  vbox_layout->addLayout(hbox_layout);
  vbox_layout->addWidget(push_button);
  vbox_layout->addWidget(label_status);
  vbox_layout->addStretch();
  this->setLayout(vbox_layout);
  connect(push_button, &QPushButton::clicked, this, &MotionControlModePanel::Apply);
  connect(this, &MotionControlModePanel::completed, this, &MotionControlModePanel::OnCompleted);
}

void MotionControlModePanel::onInitialize()
{
  node = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  client = node->create_client<uro_control_msgs::srv::ControllerManager>("/motion_control_node/controller_manager");
}

void MotionControlModePanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void MotionControlModePanel::load(const rviz_common::Config &config)
{
  rviz_common::Panel::load(config);
}

void MotionControlModePanel::Apply()
{
  auto request = std::make_shared<uro_control_msgs::srv::ControllerManager::Request>();

  request->main_mode = 0; // uro_control_msgs::srv::ControllerManager::OFF;
  if (checkbox_wbc->isChecked())
  {
    request->main_mode = 1; // uro_control_msgs::srv::ControllerManager::WBC;
  }
  else if (checkbox_rov->isChecked())
  {
    request->main_mode = 2; // uro_control_msgs::srv::ControllerManager::ROV;
  }

  this->setEnabled(false);
  std::thread([&]
              {
                auto result = client->async_send_request(request);

                std::chrono::seconds s(1);
                while (!client->wait_for_service(s))
                {
                  if (!rclcpp::ok())
                  {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return;
                  }
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
                }

                // if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
                // {
                //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "success: %s message: %s", result.get()->success ? "true" : "false", result.get()->message);
                // }
                // else
                // {
                //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
                // }
                while (rclcpp::ok())
                {
                  if (result.wait_for(3s) == std::future_status::ready)
                  {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "success: %s message: %s", result.get()->success ? "true" : "false", result.get()->message.c_str());
                    QString status;
                    status.sprintf("success: %s message: %s", result.get()->success ? "true" : "false", result.get()->message.c_str());
                    emit completed(status);
                    break;
                  }
                } //
              })
      .detach();
}

void MotionControlModePanel::OnCompleted(const QString& status)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), status.toStdString());
  label_status->setText(status);
  this->setEnabled(true);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(uro_rviz_plugins::MotionControlModePanel, rviz_common::Panel)