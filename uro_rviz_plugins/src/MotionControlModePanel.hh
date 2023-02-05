#ifndef URO_RVIZ_PLUGINS_MOTION_CONTROL_PANEL_HPP
#define URO_RVIZ_PLUGINS_MOTION_CONTROL_PANEL_HPP

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <QtWidgets>
#endif
#include <uro_control_msgs/msg/motion_control_mode.hpp>
#include <uro_control_msgs/srv/controller_manager.hpp>

namespace uro_rviz_plugins
{

    class MotionControlModePanel : public rviz_common::Panel
    {
        Q_OBJECT
    public:
        MotionControlModePanel(QWidget *parent = nullptr);

        virtual void onInitialize();
        virtual void load(const rviz_common::Config &config);
        virtual void save(rviz_common::Config config) const;

    signals:
        void completed(const QString& status);

    private slots:
        void Apply();
        void OnCompleted(const QString& status);

    protected:
        rclcpp::Node::SharedPtr node;
        rclcpp::Client<uro_control_msgs::srv::ControllerManager>::SharedPtr client;

        QCheckBox *checkbox_off;
        QCheckBox *checkbox_wbc;
        QCheckBox *checkbox_rov;
        QLabel *label_status;

    private:
        rclcpp::Client<uro_control_msgs::srv::ControllerManager>::SharedFuture future;
    };
}

#endif // URO_RVIZ_PLUGINS_MOTION_CONTROL_PANEL_HPP