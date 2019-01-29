# ShopRobot_ROS

HDU ShopRobot ROS coding

2019.1.29 版本0.0.1
@done 完成行为树的移植,以及黑板数据结构的基类
@done 与mcu的串口通信,协议可以看infantry_info,@TODO 整理为文档到docs下
@done web服务器的架构,以及进程之间的队列通信

##文件架构

-common
    此目录为整个项目公用代码仓库
-docs
    此目录为文档仓库
-example
    此目录为实例,意在能理解统一接口
-modules
    此目录为具体开发
    -decision
        任务决策的内容
    -driver
        与外界的交流,具体为各种通信
    -planning
        为具体的规划,eg:路径规划,识别等
    -stsyem
        为整个系统的配置与通信协议
-tools
    此目录为dubug工具

注释模板
```C++
//
// @brief <简述>
// -<具体实现，可不加>
// @param <参数说明>
// - <参数的特定要求，可不加>
// - 
// -
// -
// @author
// - <作者名字><联系方式>
```