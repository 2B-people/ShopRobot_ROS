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
2018.11.25: main函数不能放在namespace中，会出现main函数找不到的情况，同