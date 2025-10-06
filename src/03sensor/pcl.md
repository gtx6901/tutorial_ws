# PCL 简要科普
## 1. 安装 PCL 库（Ubuntu 22.04 + ROS 2 Humble/Foxy 示例）

在 Ubuntu 22.04 上，PCL 可以通过系统包安装，也可以通过源码安装，但为了简单，推荐 **apt 安装**：

```bash
# 更新软件源
sudo apt update
sudo apt upgrade -y

# 安装 PCL 基础库
sudo apt install -y libpcl-dev

# 安装 ROS2 与 PCL 交互所需依赖（假设已安装 ROS 2）
sudo apt install -y ros-humble-pcl-ros
```

安装完成后，可以通过以下命令检查：

```bash
pkg-config --modversion pcl_common
```

应该输出 PCL 版本号，比如 `1.12.1`。

---

## 2. PCL 基本概念

### 2.1 什么是 PCL

PCL（Point Cloud Library）是一个开源的 C++ 点云处理库。  
在 ROS 里，点云数据一般以 `sensor_msgs/msg/PointCloud2` 的形式传输，但直接操作这种消息比较麻烦，PCL 提供了 **友好的点云数据结构和处理方法**。

### 2.2 点云基本类型

* `pcl::PointXYZ`：每个点包含 `x, y, z` 坐标  
* `pcl::PointXYZI`：每个点包含 `x, y, z` + 强度 `intensity`  
* `pcl::PointXYZRGB`：每个点包含 `x, y, z` + 颜色信息  

点云通常这样存储：

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
```

---

## 3. 读取 PCD 文件

PCL 提供了读取 `.pcd` 文件的接口：

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

if (pcl::io::loadPCDFile<pcl::PointXYZ>("example.pcd", *cloud) == -1) {
    PCL_ERROR("Couldn't read file example.pcd\n");
    return -1;
}

std::cout << "Loaded " << cloud->width * cloud->height 
          << " points from example.pcd" << std::endl;
```

## 4. 与 ROS2 PointCloud2 的互转

ROS2 中点云消息类型是 `sensor_msgs/msg/PointCloud2`，PCL 提供了转换方法：

```cpp
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

// PCL -> ROS2
sensor_msgs::msg::PointCloud2 ros_cloud;
pcl::toROSMsg(*cloud, ros_cloud);
ros_cloud.header.frame_id = "lidar_link";

// ROS2 -> PCL
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::fromROSMsg(ros_cloud, *pcl_cloud);
```


## 5. 坐标系变换

如果希望对点云进行坐标变换，可以用`Eigen::Affine3f`配合`pcl::transformPointCloud`函数：

```cpp
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>

// 创建齐次变换矩阵
Eigen::Affine3f transform = Eigen::Affine3f::Identity();

// 沿x轴平移1米
transform.translation() << 1.0, 0.0, 0.0;

Eigen::Quaternionf quat;
quat = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())      // 绕X轴旋转角度
     * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())      // 绕Y轴旋转角度  
     * Eigen::AngleAxisf(M_PI/6, Eigen::Vector3f::UnitZ()); // 绕Z轴旋转30度
transform.rotate(quat);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_odom(new pcl::PointCloud<pcl::PointXYZ>);
pcl::transformPointCloud(*cloud_, *cloud_odom, transform);
```

## 6. TIPS

* RViz 里要打开 `PointCloud2` 显示，设置 `Fixed Frame` 对应 frame_id。
* 可以用 LLM 查函数用法，但**禁止直接生成整段作业代码**。