// 包含必要的ROS2头文件
#include "rclcpp/rclcpp.hpp"              // ROS2客户端库的核心功能
#include "std_msgs/msg/string.hpp"        // 标准字符串消息类型

// 定义自定义的发布者节点类，继承自rclcpp::Node
class MyPublisher: public rclcpp::Node
{
public:
    // 构造函数：初始化节点和成员变量
    MyPublisher(): Node("my_publisher"),  // 调用基类构造函数，设置节点名为"my_publisher"
                   count_(0)              // 初始化计数器为0
    {
        // 创建发布者（Publisher）
        // 参数说明：
        // - "my_message": 话题名称
        // - 10: 队列大小（缓存的消息数量）
        publisher_ = this->create_publisher<std_msgs::msg::String>("my_message", 10);
        
        // 创建定时器（Timer），每秒触发一次
        // 参数说明：
        // - std::chrono::seconds(1): 定时器周期为1秒
        // - std::bind(...): 绑定回调函数，当定时器触发时调用timer_callback方法
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MyPublisher::timer_callback, this)
        );
    }

private:
    // 定时器回调函数 - 当定时器触发时自动调用
    void timer_callback()
    {
        // 创建一个字符串消息对象
        auto message = std_msgs::msg::String();
        
        // 设置消息内容：包含问候语和计数器值
        message.data = "Hello, ROS2! " + std::to_string(count_++);
        
        // 在终端输出日志信息（INFO级别）
        // RCLCPP_INFO是ROS2的日志宏，类似于printf但功能更强大
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        
        // 发布消息到话题
        publisher_->publish(message);
    }

    // 成员变量声明
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  // 发布者指针
    rclcpp::TimerBase::SharedPtr timer_;                             // 定时器指针
    size_t count_;                                                   // 消息计数器
};

// 主函数 - 程序的入口点
int main(int argc, char * argv[])
{
    // 初始化ROS2客户端库
    // 必须在使用任何ROS2功能之前调用
    rclcpp::init(argc, argv);
    
    // 创建节点对象
    // std::make_shared是智能指针，用于自动管理内存
    auto node = std::make_shared<MyPublisher>();
    
    // 进入事件循环，保持节点运行
    // spin()会阻塞当前线程，等待和处理事件（如定时器、订阅消息等）
    rclcpp::spin(node);
    
    // 关闭ROS2客户端库，清理资源
    rclcpp::shutdown();
    
    return 0;
}