# 前言

## 问题

* 在 open space 空间下规划出⼀条⾃⻋到停⻋位的⽆碰撞轨迹 满⾜平滑约束 可跟踪
* 考虑动态障碍物约束
* 在路径不可⽤的情况下 具备重规划能⼒
* 重规划时能够做到⽆缝切换 即从原路径⽆缝切换到重规划路径 ⽆明显体感
* 规划频率 10HZ

## 预实现目标

* 在 100ms 内规划出一条轨迹(包括速度)
* 路径满足平滑约束 无 kappa 过大的问题
* 规划成功率 > 80%
* 可在 ROS 中可视化看到仿真结果 包括控制 

## 预计时间

* 2 个月

---

# 前期准备

## 环境配置和前期知识储备

* 预计 4 月 15 号完成

### 2023-04-08

* 配置 windows 下的 ros  
	[参考文章](https://blog.csdn.net/qq_40344790/article/details/129115083?csdn_share_tail=%7B%22type%22%3A%22blog%22%2C%22rType%22%3A%22article%22%2C%22rId%22%3A%22129115083%22%2C%22source%22%3A%22qq_40344790%22%7D)  
	[参考视频](https://www.bilibili.com/video/BV1y54y1w7Ka/?spm_id_from=333.337.search-card.all.click&vd_source=c68460d92fb2b166884357ad8c98de03)
* 安装过程并不顺利
* [安装 ros2](https://ms-iot.github.io/ROSOnWindows/GettingStarted/SetupRos2.html)
* 准备了解一下 ros 的相关概念和语法？
* ros 进行自动驾驶开发过程？

### 2023-04-09

* 网站老是崩 进行离线安装
* 下载又慢又容易崩
* [基于ros的路径规划](https://www.zhihu.com/search?type=content&q=%E5%9F%BA%E4%BA%8Eros%E7%9A%84%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92)
* [ros入门](https://www.bilibili.com/video/BV1Ci4y1L7ZZ/?spm_id_from=333.1007.top_right_bar_window_custom_collection.content.click&vd_source=c68460d92fb2b166884357ad8c98de03)
* [ros入门文档](http://www.autolabor.com.cn/book/ROSTutorials/chapter1.html)
* 为了以后方便还是准备装 linux
* 之前 virtualbox 在 win11 出来的时候有 bug 就删掉了 现在应该已经修复了

### 2023-04-10

* 完成 linux 下的 ros 安装
* ros 集成开发环境搭建

### 2023-04-11 -> 04-16	

* 了解 ros 的开发过程
* 了解 ros 的通信机制
* ros 的话题通信、服务通信和参数服务器
* ros 自定义头文件和源文件调用
* ros 的常用组件 --- TF 坐标变换 --- rosbag --- rqt 工具箱

## 机器人仿真

* 预计 4 月 20 号完成

### 2023-04-16 -> 04-18

* URDF 语法
* xacro
* gazebo

## 机器人导航

### 2023-04-18 -> 2023-04-20 

* 这个应该是最重要的知识部分
* 导航模块
* 导航实现
* 接下来准备开始 coding

---

# 日志

## 日期

### 2023-04-20

* 创建项目仓库
* 完成项目环境配置

### 2023-04-22

* 创建车辆模型
* 完成初始车辆姿态订阅
* 完成目标车辆姿态订阅

### 2023-04-23

* 实现对传入的车辆姿态信息解析
```word
初始姿态使用PoseWithCovarianceStamped 而目标姿态使用PoseStamped 二者都是带有参考系和时间戳的姿态 
主要的区别在于PoseWithCovarianceStamped消息类型包含一个协方差矩阵 用于表示姿态的不确定性
```
* 安装了 Eigen 但还是报错 No such file or directory -> 因为 eigen 库默认安装在 /usr/include/eigen3/Eigen 需映射到 /usr/include 路径下  
```word
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen 
```
* 对传入的车辆信息进行初始化 并解析位姿信息
* 测试话题正常能否正常发布

### 2023-04-24

* 了解 `A*` 算法和 `混合A*` 算法
* 重温 `Dijkstra`算法
* 百度的 apollo 项目中的`混合A*` 算法的代码中 直接将拓展的距离设置成了栅格的对角线 也就是说子节点和父节点不会占据一个栅格 -> 是不是因为这样才不适合狭窄的空间下情况 而不是`混合A*` 不适合？

### 2023-04-26

* 实现地图的订阅
* 继续了解 `混合A*` 算法

### 2023-05-07

* 创建规划策略(planning_method)相关文件(.h .cpp)
* 设置参数 -> 包括转向角、转向惩罚、倒车惩罚等
* 修改地图的消息类型

### 2023-05-08

* 设置车辆形状
* 抽象化状态节点
* 创建时间类 方便计算时间差

### 2023-05-09

* 开始编写`混合A*算法部分
* `混合A*` 初始化

### 2023-05-11

* 继续`混合A*`算法部分
* debug
* RS(Reed_Shepp)?

### 2023-05-12

* 测试程序
* debug
* 生成的路径发布失败(显示失败)

### 2023-05-13

* 设置地图参数
* 实现地图在 rviz 中显示
* 节点搜索部分
* 启发函数

### 2023-05-15

* 继续 search() 部分
* 包括相邻节点搜索 路径障碍物检测 边界检测 Bresenham算法

### 2023-05-16

* 修改路径发布 topic
* 搜索树

---