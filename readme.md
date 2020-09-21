# 底盘can驱动

*这份代码专门针对的是小车的底盘can驱动的代码*

第一步是安装can驱动，安装方法之前写过。这份代码是直接下载下来，catkin_make安装即可。

提供了两种使用方式，一种是rosrun can_bus start_can   一种是rosrun can_bus teleop

teleop是方便使用键盘进行控制的操作方式，而start_can则就是正常得启动。

如果没有底盘can信息的话，会导致程序一直在readcan信息中一直循环，ctrl+c都不能终止。但只要有一条信息发送出来，就可以终止了。

使用之前一定要修改标定表的位置

