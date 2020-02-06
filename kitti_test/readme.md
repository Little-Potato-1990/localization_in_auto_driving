
#### 文件说明
本文件夹内的文件是为了测试利用KITTI数据集生成的bag文件，bag文件在我的百度网盘里可以下载。

网盘地址：https://pan.baidu.com/s/1TyXbifoTHubu3zt4jZ90Wg 提取码: n9ys

bag文件可以在文件夹“KITTI数据集/转换后的bag文件/2011_10_03”中找到，由于百度网盘单个文件大小有限制，所以我做了分卷压缩，下载完成之后需要在当前目录下输入如下指令，把他们再合成一个文件才能解压
```
cat bag_file*>bag.tar.gz
```

如果想自己用原始数据转换生成bag文件，可以参考我对应的[博客](https://zhuanlan.zhihu.com/p/104875159)里的方法。原始数据在下载的文件夹“KITTI数据集/RawData原始数据”里可以找到。

#### 测试方法
执行步骤如下：
- 启动ros
```
roscore
```
- 启动rviz
打开终端，cd到“kitti_test”文件夹下，输入指令
```
rviz display_bag.rviz
```
- 播放bag
打开终端，cd到bag所在目录，输入指令
```
rosbag play kitti_2011_10_03_drive_0027_synced.bag
```