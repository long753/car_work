# 文档说明

## 项目名称：SmartCar-Competition

## 主要文件说明

### 1. 主程序文件：icar.cpp

#### 1.1初始化：

1.初始化模型  smartcar.model_init();  
2.初始化通信  smartcar.uart_connect();  
3.初始化相机  smartcar.imagecapture.camera_init();  
4.启动相机    smartcar.imagecapture.run();  
5.启动AI推理  smartcar.ai_process_mul();  

#### 1.2主要工作情况：

这是原工程的主要的工作模式，在主可执行文件中，我们通过视觉的检测来判定当前场景，然后根据场景的不同，调用不同的场景类来进行相应的工作。具体的工作模式如下：  

1.环岛--RingScene  
2.救援--RescueScene  
3.追车--RacingScene  
4.危险--DangerScene  
5.桥--BridgeScene  
6.停车--ParkingScene  

#### 1.3本届竞赛的主要工作：

在学长的原代码的基础上，本届竞赛的主要工程主要有以下几个方面，可以发现有3种情况需要我们自行完成。同时，学长还要求我们自己写出环岛的代码，目前的主要工作是完成环岛部分的代码，包括环岛检测:detection/RingDetection.cpp   detection/RingDetection.h 环岛控制(我还没创)  

1.环岛--RingScene       （相同or类似）  
2.餐饮--RestaurantScene  
3.充电--ChargingScene  
4.躲避--AvoidScene  
5.桥接--BridgeScene     （相同or类似）  
6.停车--ParkingScene    （相同or类似）  

### 2. 核心类：SmartCar.h、SmartCar.cpp

这是本工程的核心类，包含了智能小车的基本功能，如：初始化、通信、相机、AI推理、场景切换等。

#### 2.1 成员变量：


#### 2.2 成员函数：


### 3. 控制文件夹：control

### 4. 检测文件夹：detection

该文件夹中包含了桥场景、危险场景、停车场景、救援场景、追车场景、环岛场景的检测代码。  

#### 4.1桥检测

#### 4.2危险检测

#### 4.3停车检测

#### 4.4救援检测

#### 4.5追车检测    

#### 4.6环岛检测

### 5. 驱动文件夹：driver

### 6. 元素文件夹：element

### 7. 通用头文件夹：include

### 8. 外工具文件夹：tool

### 9. 内工具文件夹：utils

### 10. 视觉文件夹：vision

### 11. 其他文件夹or文件：

均为配置文件或者说明文件等辅助性文件，无需过多关注，只需在必要时间修改即可。

## 类关系图

### 主要类及其内容：

#### 1. Uart 类
用途：负责与小车硬件的串口通信，发送控制命令和接收传感器数据。  
关键函数：  
open()：打开串口连接。  
startReceive()：启动接收线程，处理接收到的数据。  
carControl(motorSpeed, servoPwm)：通过串口发送控制命令，调整小车的速度和方向。  

#### 2. ImageCapture 类
用途：负责图像捕捉和预处理。  
关键成员：  
rgb_image：存储捕捉到的彩色图像。  
image_process：图像处理模块，执行图像二值化等操作。  
image_update_flag：标志是否有新图像更新。  

#### 3. Detection 类
用途：负责 AI 物体检测，进行环境感知。  
关键函数：  
inference(cv::Mat m)：执行 AI 推理，处理图像并生成检测结果。  
成员变量：  
results：存储 AI 推理的结果。  

#### 4. Findline 类
用途：负责赛道线的识别和处理。　　
关键函数：　　
search_line(cv::Mat imageBinary)：在二值化图像中搜索赛道线。　　
you_dan_diao_width() 和 zuo_dan_diao_width()：计算左右边界宽度，用于判断小车位置。　　
midline_calculate()：计算中线位置，用于导航。　　
成员变量：　　
line_type：当前线条类型，如直线、曲线等。　　
loseflag：标志是否失去赛道线。　　

#### 5. 场景处理模块
Ring、Cross、Bridge、Danger、Parking、Racing、Rescue 类：　　
用途：分别处理不同的驾驶场景，执行特定的控制逻辑。　　
关键函数：　　
process(...)：处理特定场景下的逻辑，调整小车行为。　　
成员变量：　　
各自场景特定的参数和状态。　　

####　6. Control 类
用途：管理小车的运动控制参数，如速度和方向。　　
成员变量：　　
motorSpeed：电机速度。　　
servoPwm：舵机 PWM 信号。　　
turnP 和 turnD：PID 控制参数。　　
qianzhan：前瞻控制参数。　　
功能：　　
error_cal(findline)：计算控制误差。　　
servo_control(error, findline, scene)：调整舵机以校正方向。　　

####　7. IcarShow 类
用途：用于调试模式下的显示和可视化，实时展示小车状态和图像处理结果。　　
关键函数：　　
show_init()：初始化显示窗口。　　
run()：运行显示逻辑。　　
ai_draw_result(img, results)：绘制 AI 检测结果。　　
get_Scene_string(scene)：获取当前场景的字符串表示。　　
成员变量：　　
debug_image1 和 debug_image2：用于显示的图像。　　
debug_image1_mutex 和 debug_image2_mutex：保护图像数据的互斥锁。　　

## 基本规范

1. 类用驼峰命名法
2. 函数使用下划线和全小写字母组合
3. 变量名也使用全小写加上下划线组合