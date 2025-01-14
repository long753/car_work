#### 文档说明

### 项目名称：智能小车

### 主要文件说明

## 1. 主程序文件：icar.cpp

# 1.1初始化：

1.初始化模型  smartcar.model_init();
2.初始化通信  smartcar.uart_connect();
3.初始化相机  smartcar.imagecapture.camera_init();
4.启动相机    smartcar.imagecapture.run();
5.启动AI推理  smartcar.ai_process_mul();

# 1.2主要工作情况：

这是原工程的主要的工作模式，在主可执行文件中，我们通过视觉的检测来判定当前场景，然后根据场景的不同，调用不同的场景类来进行相应的工作。具体的工作模式如下：

1.环岛--RingScene
2.救援--RescueScene
3.追车--RacingScene
4.危险--DangerScene
5.桥接--BridgeScene
6.停车--ParkingScene

# 1.3本届竞赛的主要工程：

在学长的原代码的基础上，本届竞赛的主要工程主要有以下几个方面，可以发现有3种情况需要我们自行完成，同时，学长要求我们自己写出环岛的代码。不过幸好学长把检测部分的代码给我们了，我们只需要完成环岛的运动控制部分即可。

1.环岛--RingScene       （相同or类似）
2.餐饮--RestaurantScene
3.充电--ChargingScene
4.躲避--AvoidScene
5.桥接--BridgeScene     （相同or类似）
6.停车--ParkingScene    （相同or类似）

## 2. 核心类：SmartCar.h、SmartCar.cpp

## 3. 控制文件夹：control

## 4. 通信文件夹：communication

## 5. 驱动文件夹：driver

## 6. 显示文件夹：display

## 7. 其他文件夹：utils、test、examples、docs

## 类关系图

### 基本规范

1. 类用驼峰命名法
2. 函数使用下划线和全小写字母组合
3. 变量名也使用全小写加上下划线组合