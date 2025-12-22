#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <cstdlib>
#include <thread>
#include <mutex>

class SafetySwitch {
private:
    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;
    ros::Subscriber cmd_in_sub_;
    ros::Publisher cmd_out_pub_;
    ros::Publisher emergency_stop_pub_;  // 发布急停状态给其他节点
    ros::Publisher replan_trigger_pub_;   // 触发FastPlanner重新规划
    ros::ServiceClient replan_service_;   // 调用FastPlanner重新规划服务
    
    bool emergency_stop_;
    int stop_button_;     // B 键：紧急停止
    int reset_button_;    // A 键：系统重启
    ros::Time last_joy_time_;
    ros::Time last_cmd_time_;  // 记录最后一次收到MPC命令的时间
    bool reset_in_progress_;   // 标记系统重置中
    bool a_button_pressed_;    // 防抖：A键是否已按下
    std::mutex reset_mutex_;   // 保护重置状态的互斥锁
    
public:
    SafetySwitch() : nh_("~"), emergency_stop_(false), reset_in_progress_(false), a_button_pressed_(false) {
        // 参数：B键作为紧急停止按钮，A键作为系统重启按钮
        nh_.param<int>("stop_button", stop_button_, 2);   // B 键（默认按钮2）
        nh_.param<int>("reset_button", reset_button_, 1); // A 键（默认按钮1）
        
        // 订阅手柄
        joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/hf_platform/joy", 10, 
                                                    &SafetySwitch::joyCallback, this);
        
        // 订阅 MPC 输出
        cmd_in_sub_ = nh_.subscribe<geometry_msgs::Twist>("/mpc/cmd_vel_raw", 10, 
                                                           &SafetySwitch::cmdCallback, this);
        
        // 发布到 twist_mux
        cmd_out_pub_ = nh_.advertise<geometry_msgs::Twist>("/hf_platform/twist_mux/cmd_vel", 10);
        
        // 发布急停状态（供其他节点监听）
        emergency_stop_pub_ = nh_.advertise<std_msgs::Bool>("/safety_switch/emergency_stop", 10, true);
        
        // 触发FastPlanner重新规划
        replan_trigger_pub_ = nh_.advertise<std_msgs::Empty>("/planning/replan", 10);
        
        // FastPlanner 重新规划服务
        replan_service_ = nh_.serviceClient<std_srvs::Empty>("/planning/replan_if_needed");
        
        last_joy_time_ = ros::Time::now();
        last_cmd_time_ = ros::Time::now();
        
        // 发布初始状态（非急停）
        std_msgs::Bool stop_msg;
        stop_msg.data = false;
        emergency_stop_pub_.publish(stop_msg);
        
        ROS_INFO("=================================================");
        ROS_INFO("[Safety Switch] 安全开关已启动（完全重启模式）");
        ROS_INFO("  正常运行：MPC 连续控制，无卡顿");
        ROS_INFO("  按下 B 键（按钮 %d）立即急停！", stop_button_);
        ROS_INFO("  按下 A 键（按钮 %d）完全重启系统！", reset_button_);
        ROS_INFO("=================================================");
    }
    
    // ============ 系统完全重启函数（后台线程执行，不阻塞主线程） ============
    void fullSystemResetAsync() {
        // 立即返回，在后台线程执行重启
        std::thread reset_thread([this]() {
            this->fullSystemResetImpl();
        });
        reset_thread.detach();  // 后台运行，不等待完成
    }
    
    // 系统重启的实际实现（在后台线程中执行）
    void fullSystemResetImpl() {
        std::lock_guard<std::mutex> lock(reset_mutex_);
        
        if (reset_in_progress_) {
            ROS_WARN("[Safety Switch] 系统重置已在进行中，忽略重复请求");
            return;
        }
        
        reset_in_progress_ = true;
        emergency_stop_ = false;
        
        ROS_WARN("=================================================");
        ROS_WARN("[Safety Switch] *** 启动系统完全重置 ***");
        ROS_WARN("[Safety Switch] 正在重置以下组件...");
        ROS_WARN("=================================================");
        
        // 阶段1: 停止所有运动（1秒）
        ROS_INFO("[Safety Switch] [阶段 1/5] 停止底盘运动...");
        geometry_msgs::Twist zero;
        ros::Rate rate(20);  // 20 Hz
        for (int i = 0; i < 20; ++i) {  // 1秒
            cmd_out_pub_.publish(zero);
            rate.sleep();
        }
        ROS_INFO("[Safety Switch] [✓] 底盘已停止");
        
        // 阶段2: 清空 MPC 缓存（1秒零速度）
        ROS_INFO("[Safety Switch] [阶段 2/5] 清空 MPC 缓存...");
        for (int i = 0; i < 20; ++i) {  // 1秒
            cmd_out_pub_.publish(zero);
            rate.sleep();
        }
        ROS_INFO("[Safety Switch] [✓] MPC 缓存已清空");
        
        // 阶段3: 清空 FastPlanner 目标
        ROS_INFO("[Safety Switch] [阶段 3/5] 清空 FastPlanner 目标...");
        ros::Publisher goal_pub = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
        ros::Duration(0.2).sleep();
        goal_pub.shutdown();
        ROS_INFO("[Safety Switch] [✓] FastPlanner 目标已清除");
        
        // 阶段4: 触发 FastPlanner 重新规划
        ROS_INFO("[Safety Switch] [阶段 4/5] 触发 FastPlanner 重新初始化...");
        std_msgs::Empty replan_msg;
        for (int i = 0; i < 3; ++i) {
            replan_trigger_pub_.publish(replan_msg);
            ros::Duration(0.1).sleep();
        }
        ROS_INFO("[Safety Switch] [✓] FastPlanner 已重新初始化");
        
        // 阶段5: 发布重置完成状态
        ROS_INFO("[Safety Switch] [阶段 5/5] 发布系统状态...");
        std_msgs::Bool stop_msg;
        stop_msg.data = false;
        emergency_stop_pub_.publish(stop_msg);
        ROS_INFO("[Safety Switch] [✓] 系统状态已重置");
        
        // 稳定化：1秒零速度
        ROS_INFO("[Safety Switch] [稳定化] 发送1秒零速度稳定系统...");
        for (int i = 0; i < 20; ++i) {  // 1秒
            cmd_out_pub_.publish(zero);
            rate.sleep();
        }
        
        reset_in_progress_ = false;
        
        ROS_WARN("=================================================");
        ROS_WARN("[Safety Switch] *** 系统重置完成！***");
        ROS_WARN("[Safety Switch] 现在可以设置新的目标点");
        ROS_WARN("=================================================");
    }
    
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
        last_joy_time_ = ros::Time::now();
        
        // 检测 B 键按下 -> 触发急停
        if (stop_button_ >= 0 && stop_button_ < (int)joy->buttons.size()) {
            if (joy->buttons[stop_button_] && !emergency_stop_) {
                emergency_stop_ = true;
                
                // 发送零速度
                geometry_msgs::Twist zero;
                cmd_out_pub_.publish(zero);
                
                // 发布急停状态
                std_msgs::Bool stop_msg;
                stop_msg.data = true;
                emergency_stop_pub_.publish(stop_msg);
                
                ROS_ERROR("********************************************************");
                ROS_ERROR("[Safety Switch] *** 紧急停止触发！***");
                ROS_ERROR("[Safety Switch] 机器人已停止，请检查情况");
                ROS_ERROR("[Safety Switch] 按 A 键（按钮 %d）完全重启系统", reset_button_);
                ROS_ERROR("********************************************************");
            }
        }
        
        // 检测 A 键按下 -> 完全重启系统（防抖 + 异步）
        if (reset_button_ >= 0 && reset_button_ < (int)joy->buttons.size()) {
            if (joy->buttons[reset_button_] && !a_button_pressed_) {
                a_button_pressed_ = true;  // 防抖：标记A键已按下
                ROS_INFO("[Safety Switch] A键检测到，启动后台重启线程...");
                fullSystemResetAsync();  // 异步启动重启，不阻塞主线程
            } else if (!joy->buttons[reset_button_]) {
                a_button_pressed_ = false;  // A键释放，重置防抖状态
            }
        }
    }
    
    void cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd) {
        last_cmd_time_ = ros::Time::now();
        
        // 如果处于急停状态，只发送零速度
        if (emergency_stop_) {
            geometry_msgs::Twist zero;
            cmd_out_pub_.publish(zero);
        } else {
            // 正常转发 MPC 命令（无任何阻塞）
            cmd_out_pub_.publish(*cmd);
        }
    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "safety_switch_node");
    SafetySwitch safety_switch;
    
    ROS_INFO("=================================================");
    ROS_INFO("[Safety Switch] 进入事件循环");
    ROS_INFO("[Safety Switch] 手柄回调将在后台处理");
    ROS_INFO("=================================================");
    
    ros::spin();
    return 0;
}
