# BUG_LIST

2018.11.21:  main_interface中的MAIN需要在namespace外使用，不然编译时会出现
```shell
/usr/lib/gcc/x86_64-linux-gnu/5/../../../x86_64-linux-gnu/crt1.o：在函数‘_start’中：
(.text+0x20)：对‘main’未定义的引用

```

2018.11.21: cmakelist中的library的名字需要注意，编译时会出现找不到对应library