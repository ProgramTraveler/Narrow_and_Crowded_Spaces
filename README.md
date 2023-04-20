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

# 开发日志

### 2023-04-20

* 创建项目仓库
* 完成项目环境配置

---