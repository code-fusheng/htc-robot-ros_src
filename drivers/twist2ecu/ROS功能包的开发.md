# 1 ROS功能包的开发

### 1.1 创建ROS工作空间

- 第一步：创建文件夹

  ```
  - -  twist2ecu
  ```

  在这里我们创建的文件夹叫twist2ecu

- 第二步：在该文件夹目录下创建src子文件夹

  ```
  - - twist2ecu
  		- - src
  ```

- 第三步：在twist2ecu文件夹下打开终端, 输入`catkin_make`

  ![image-20231031125014121](/home/mhy/.config/Typora/typora-user-images/image-20231031125014121.png)

  这样一个名为twist2ecu的ROS空间就创建好了

  ```
  - - twist2ecu
  		- - build(将程序中调用到的类库和你的程序链接起来, 后续不会展开这个文件夹)
  		- - devel(没太懂干嘛的, 感觉和build很像, 一堆头文件和链接库, 后续不会展开这个文件夹)
  		- - src(源码, 后续主要展开描述这个文件)
  		- - .catkin_workspace(这是个txt文件, 我不知道用来干嘛的)
  ```

### 1.2 构建好src文件夹

- 第一步：创建好所需要的ros包并添加依赖

  ```
  cd src
  catkin_create_pkg trans roscpp rospy std_msgs
  ```

  其中trans是我们自定义创建的ros功能包, 下面会详细说

- 第二步：创建好第三方数据格式文件, 方便后续自动化生成头文件

  ```
  cd src
  mkdir msg
  ```

  再将你对应的msg数据格式文件放入其中就好了, 在这里我们使用的是ecu.msg

### 1.3 源码及其依赖构建

到这一步我们就要详细的去写代码了, 我们在下一个文档去说明

