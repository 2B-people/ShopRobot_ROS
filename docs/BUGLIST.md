# BUG_LIST

`bug0001`
2018.11.21:  main_interface中的MAIN需要在namespace外使用，不然编译时会出现
```shell
/usr/lib/gcc/x86_64-linux-gnu/5/../../../x86_64-linux-gnu/crt1.o：在函数‘_start’中：
(.text+0x20)：对‘main’未定义的引用

```
`bug0002`
2018.11.21: cmakelist中的library的名字需要注意，编译时会出现找不到对应library

`bug0003`
2018.11.25: main函数不能放在namespace中，会出现main函数找不到的情况，同*bug0001*
```shell
/usr/lib/gcc/x86_64-linux-gnu/5/../../../x86_64-linux-gnu/crt1.o：在函数‘_start’中：
(.text+0x20)：对‘main’未定义的引用

```

`bug0004`
2018.11.29: 对于通信协议，要先确定协议在coding，不然会发生，不得不重新写！！

`bug0005`
2018.11.29：在函数需要改变多于一个值时，要传指针进去

`bug0006`
2019.11.29：使用uint32_t出现的问题
```c++
uint32_t bug = -3;
cout<<bug<<endl;
```
会打印出 -3补码的十进制？？
<!-- TODO(nqq) 解决这个bug，现在只能避免这样赋值-->
```shell
～$ ./a.out
4294967293
```

`bug007`
“was not declared in this scope”是一个错误信息，在编译的时候会遇到。其含义为标识符在其出现的地方是未被定义的。
出现该错误的时候，会同时把未定义的变量名显示出来。比如如下程序：
```cpp
int main()
{
printf("%d",i);//这个i是没定义的。
}
```
就会显示一个'i' was not declared in this scope或者类似的错误信息出来。
对于变量和函数，如果未定义都会出现这个错误。
该错误出现时，需要根据出现该错误的行号及名称，查找对应名称变量或函数的作用，一般有如下几种可能：
1 忘记定义。写代码的时候疏忽了，导致一些变量直接使用但没有定义。只要对应定义相应的函数或变量即可。
2 拼写错误。写代码的时候，敲错了字符。比如sum敲成了Sum, average敲成averge等。对应修改即可。
3 作用域不正确。在超出变量和函数的作用域部分使用了该变量或函数。需要通过定义位置，或增加声明的手段，加大变量的作用域使其包含引用位置

`bug008`
```shell
error: ISO C++ forbids declaration of ‘init
Serial’ with no type [-fpermissive] McuSerial::initSerial()
```
`bug009`
```shell
 error: ‘i’ was not declared in this scope
   data |= buff[i];
```
ctrl-v时注意改值，不然gg！


`bug010`
```shell
CMakeFiles/decision_test.dir/src/decision_test.cpp.o：在函数‘BlackTest::BlackTest()’中：
decision_test.cpp:(.text._ZN9BlackTestC2Ev[_ZN9BlackTestC5Ev]+0x19)：对‘vtable for BlackTest’未定义的引用
collect2: error: ld returned 1 exit status
ShopRobot_ROS/modules/decision/decision/CMakeFiles/decision_test.dir/build.make:114: recipe for target '/home/nqq09/Shop_robot_WS/devel/lib/decision/decision_test' failed
make[2]: *** [/home/nqq09/Shop_robot_WS/devel/lib/decision/decision_test] Error 1
CMakeFiles/Makefile2:2544: recipe for target 'ShopRobot_ROS/modules/decision/decision/CMakeFiles/decision_test.dir/all' failed
make[1]: *** [ShopRobot_ROS/modules/decision/decision/CMakeFiles/decision_test.dir/all] Error 2
Makefile:138: recipe for target 'all' failed
make: *** [all] Error 2
Invoking "make -j4 -l4" failed
```
```c++
    //错误
    ~CoordinateDir();
    //解决
    virtual ~CoordinateDir() =default;

```
猜测是上面的调用问题
`解决`
不是上述问题,是虚基类的继承问题,是编译器报虚函数没有被实现的
是因为基类中如果虚函数没有缺省的实现，最好就声明成纯虚函数。
另外在对各种成员函数包括构造函数之类的最好提供其实现，即使为空也可以。

`bug011`
ros::time 没有重载条件运算符,使用时注意得转化为double来进入判断
`重要` 重载了(是我傻逼),应该用以下方法来判断定时
```c++
        if(at_now - begin_ >= ros::Duration(3));
        //ros::time-ros::time 两个Time相减返回Duration类型
```

`bug012`
```c++
    DirBase::Ptr GetDirPtr(std::string key);
```
此方法只能return基类的指针,所以在使用的时候注意使用std::dynamic_pointer_cast<Temp>来转化类指针类型

`bug013`
```shell
make[2]: 警告：检测到时钟错误。您的创建可能是不完整的。
Scanning dependencies of target decision_test
make[2]: Warning: File '/home/nqq09/Shop_robot_WS/devel/lib/decision/decision_test' has modification time 27184 s in the future
```
???莫名解决,再运行就没有了,神奇,网上说的时cmake时间的问题,猜测是在win10上修改时,影响了cmakelist时间线

`bug014`
```shell
/home/nqq09/Shop_robot_WS/src/ShopRobot_ROS/modules/decision/decision/include/decision/behavior_node.hpp:11:38: fatal error: blackboard/black_board.hpp: 没有那个文件或目录
```
原因:
cmake里没有加以下语句
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp 
)
这是给外部包提供依赖所使用的代码