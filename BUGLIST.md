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