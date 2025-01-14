#include "common.hpp"
#include "stop_watch.hpp"
#include <atomic>
#include <iostream>
#include <libserial/SerialPort.h>
#include <string>

using namespace LibSerial;

// USB通信帧
#define USB_FRAME_HEAD 0x42 // USB通信帧头
#define USB_FRAME_LENMIN 4  // USB通信帧最短字节长度
#define USB_FRAME_LENMAX 12 // USB通信帧最长字节长度

// USB通信地址
#define USB_ADDR_CARCTRL 1 // 智能车速度+方向控制
#define USB_ADDR_Angle 2 // 陀螺仪
#define USB_ADDR_BUZZER 4  // 蜂鸣器音效控制
#define USB_ADDR_LED 5     // LED灯效控制
#define USB_ADDR_KEYINPUT 6     // 按键信息

class Uart {

private:
  /**

  **/

  /**
   * @brief 串口通信结构体
   *
   */
  typedef struct {
    bool start;                           // 开始接收标志
    uint8_t index;                        // 接收序列
    uint8_t buffRead[USB_FRAME_LENMAX];   // 临时缓冲数据
    uint8_t buffFinish[USB_FRAME_LENMAX]; // 校验成功数据
  } SerialStruct;

  std::jthread threadRec; // 串口接收子线程
  std::shared_ptr<SerialPort> serialPort = nullptr;
  std::string portName; // 端口名字
  bool isOpen = false;
  SerialStruct serialStr; // 串口通信数据结构体

  /**
   * @brief 32位数据内存对齐/联合体
   *
   */
  typedef union {
    uint8_t buff[4];
    float float32;
    int int32;
  } Bit32Union;

  /**
   * @brief 16位数据内存对齐/联合体
   *
   */
  typedef union {
    uint8_t buff[2];
    int int16;
    uint16_t uint16;
  } Bit16Union;
  std::atomic<bool> serialState;
  int receiveBytes(unsigned char &charBuffer, size_t msTimeout = 0) {

    try {

      serialPort->ReadByte(charBuffer, msTimeout);
    } catch (const ReadTimeout &) {
      return -2;
    } catch (const NotOpen &) {
      return -1;
    }
    return 0;
  };
enum KEYTYPE{   
      NOKEY = 0,
      GO ,
      STOP,
};

  /**
   * @brief
   *
   * @param data
   * @return int
   */
  int transmitByte(unsigned char data) {
    // try检测语句块有没有异常
    try {
      serialPort->WriteByte(data); // 写数据到串口
    } catch (const std::runtime_error &) // catch捕获并处理 try 检测到的异常。
    {
      std::cerr << "The Write() runtime_error." << std::endl;
      return -2;
    } catch (const NotOpen &) // catch捕获并处理 try 检测到的异常。
    {
      std::cerr << "Port Not Open ..." << std::endl;
      return -1;
    }
    serialPort->DrainWriteBuffer(); // 等待，直到写缓冲区耗尽，然后返回。
    return 0;
  }

public:
  uint16_t lastReceivedKey = 0;
  std::atomic<float> angle_yaw = 0;
  Uart(const std::string &port) : portName(port), serialState(true){};

  ~Uart() noexcept
  { 
    printf(" uart thread exit!\n");
    carControl(0, PWMSERVOMID);
    serialState.store(false);
    if (serialPort != nullptr) {
      serialPort->Close();
      serialPort = nullptr;
    }
    isOpen = false;
  };
  bool keypress = false;

  enum Buzzer {
    BUZZER_OK = 0,   // 确认
    BUZZER_WARNNING, // 报警
    BUZZER_FINISH,   // 完成
    BUZZER_DING,     // 提示
    BUZZER_START,    // 开机
  };

  /**
   * @brief 启动串口通信
   *
   * @param port 串口号
   * @return int
   */
  int open(void) {
    serialPort = std::make_shared<SerialPort>();
    if (serialPort == nullptr) {
      std::cerr << "Serial Create Failed ." << std::endl;
      return -1;
    }
    // try检测语句块有没有异常
    try {
      serialPort->Open(portName);                     // 打开串口
      serialPort->SetBaudRate(BaudRate::BAUD_115200); // 设置波特率
      serialPort->SetCharacterSize(CharacterSize::CHAR_SIZE_8); // 8位数据位
      serialPort->SetFlowControl(FlowControl::FLOW_CONTROL_NONE); // 设置流控
      serialPort->SetParity(Parity::PARITY_NONE);                 // 无校验
      serialPort->SetStopBits(StopBits::STOP_BITS_1); // 1个停止位
    } catch (const OpenFailed &) // catch捕获并处理 try 检测到的异常。
    {
      std::cerr << "Serial port: " << portName << "open failed ..."
                << std::endl;
      isOpen = false;
      return -2;
    } catch (const AlreadyOpen &) // catch捕获并处理 try 检测到的异常。
    {
      std::cerr << "Serial port: " << portName << "open failed ..."
                << std::endl;
      isOpen = false;
      return -3;
    } catch (...) // catch捕获并处理 try 检测到的异常。
    {
      std::cerr << "Serial port: " << portName << " recv exception ..."
                << std::endl;
      isOpen = false;
      return -4;
    }

    serialStr.start = false;
    serialStr.index = 0;
    isOpen = true;

    return 0;
  }

  /**
   * @brief 启动接收子线程
   *
   */
  void startReceive(void) {
    if (!isOpen){
      return;
    } // 串口是否正常打开
      

    // 启动串口接收子线程
    threadRec = std::jthread([this]() {
      while (serialState.load()) {
        receiveCheck(); // 串口接收校验
      }
    });
  }



  /**
   * @brief 串口接收校验
   *
   */
  void receiveCheck(void) {
    if (!isOpen) // 串口是否正常打开
      return;

    uint8_t resByte = 0;
    int ret = receiveBytes(resByte, 1000);
    if (ret == 0) {
      if (resByte == USB_FRAME_HEAD && !serialStr.start) // 监听帧头
      {
        serialStr.start = true;                   // 开始接收数据
        serialStr.buffRead[0] = resByte;          // 获取帧头
        serialStr.buffRead[2] = USB_FRAME_LENMIN; // 初始化帧长
        serialStr.index = 1;
      } else if (serialStr.index == 2) // 接收帧的长度
      {
        serialStr.buffRead[serialStr.index] = resByte;
        serialStr.index++;
        if (resByte > USB_FRAME_LENMAX ||
            resByte < USB_FRAME_LENMIN) // 帧长错误
        {
          serialStr.buffRead[2] = USB_FRAME_LENMIN; // 重置帧长
          serialStr.index = 0;
          serialStr.start = false; // 重新监听帧长
        }
      } else if (serialStr.start &&
                 serialStr.index < USB_FRAME_LENMAX) // 开始接收数据
      {
        serialStr.buffRead[serialStr.index] = resByte; // 读取数据
        serialStr.index++;                             // 索引下移
      }

      // 帧长接收完毕
      if ((serialStr.index >= USB_FRAME_LENMAX ||
           serialStr.index >= serialStr.buffRead[2]) &&
          serialStr.index > USB_FRAME_LENMIN) // 检测是否接收完数据
      {
        uint8_t check = 0; // 初始化校验和
        uint8_t length = USB_FRAME_LENMIN;
        length = serialStr.buffRead[2]; // 读取本次数据的长度
        for (int i = 0; i < length - 1; i++){
          check += serialStr.buffRead[i]; // 累加校验和
        }
          

        if (check == serialStr.buffRead[length - 1]) // 校验和相等
        {
          memcpy(serialStr.buffFinish, serialStr.buffRead,
                 USB_FRAME_LENMAX); // 储存接收的数据
          dataTransform();
        }

        serialStr.index = 0;     // 重新开始下一轮数据接收
        serialStr.start = false; // 重新监听帧头
      }
    }
  }


  /**
   * @brief 串口通信协议数据转换
   */  
  
  void processKeyPress(uint8_t byte1, uint8_t byte2) 
  {
    Bit16Union keyUnion;
    keyUnion.buff[0] = byte1;
    keyUnion.buff[1] = byte2;
    uint16_t rawKey = keyUnion.uint16;  // 从字节中得到原始的 uint16_t 值
    if (rawKey <= STOP) {
        KEYTYPE receivedKey = static_cast<KEYTYPE>(rawKey);  
        lastReceivedKey = receivedKey; 
        std::cout << receivedKey << std::endl;

    } else {
        std::cerr << "无效的按键值：" << rawKey << std::endl;
    }
  }  
  bool validateChecksum(const std::vector<uint8_t>& data, size_t expectedLength) 
  {
      uint8_t sum = 0;
      for (size_t i = 0; i < expectedLength - 1; ++i) {
          sum += data[i];
      }
      return sum == data[expectedLength - 1];
  }
  void dataTransform(void) {
  // if(serialStr.buffFinish[0]==0x42){
  //     std::cout << "ok" << std::endl;
  // }
  // if(serialStr.buffFinish[2]==0x06){
  //     std::cout << "anjian" << std::endl;
  // }
  // if(serialStr.buffFinish[2]==0x08){
  //     std::cout << "tuoluoyi" << std::endl;
  // }
    switch (serialStr.buffFinish[1]) {
      case USB_ADDR_KEYINPUT: // 处理按键输入
        // 创建一个 vector 并初始化，包含前6个元素，用于校验
        if (validateChecksum(std::vector<uint8_t>(serialStr.buffFinish, serialStr.buffFinish + 6), 6)) {
            
           processKeyPress(serialStr.buffFinish[3], serialStr.buffFinish[4]);
        } else {
            std::cerr << "校验和错误，按键输入数据有误。" << std::endl;
        }
        break;
      case USB_ADDR_Angle:
        Bit32Union bit32_union;
        memcpy(bit32_union.buff, &(serialStr.buffFinish[3]), 4);
        angle_yaw = bit32_union.float32;
        // std::cout<<"receive imu"<<std::endl;
        break;
      default:
        std::cerr << "收到未知的地址类型：" << static_cast<int>(serialStr.buffFinish[1]) << std::endl;
        break;
    }
}



  /**
   * @brief 车辆速度+方向控制
   *
   * @param speed 速度：m/s
   * @param servo 方向：PWM（500~2500）
   */
  void carControl(float speed, uint16_t servo) {
    if (!isOpen)
      return;

    uint8_t buff[11];  // 多发送一个字节
    uint8_t check = 0; // 校验位
    Bit32Union bit32U;
    Bit16Union bit16U;

    buff[0] = USB_FRAME_HEAD;   // 通信帧头
    buff[1] = USB_ADDR_CARCTRL; // 地址
    buff[2] = 10;               // 帧长

    bit32U.float32 = speed; // X轴线速度
    for (int i = 0; i < 4; i++)
      buff[i + 3] = bit32U.buff[i];

    bit16U.uint16 = servo; // Y轴线速度
    buff[7] = bit16U.buff[0];
    buff[8] = bit16U.buff[1];

    for (int i = 0; i < 9; i++)
      check += buff[i];
    buff[9] = check; // 校验位

    // 循环发送数据
    for (size_t i = 0; i < 11; i++){
      transmitByte(buff[i]);
    }
      
  }

  /**
   * @brief 蜂鸣器音效控制
   *
   * @param sound
   */
  void buzzerSound(Buzzer sound) {
    if (!isOpen)
      return;
    uint8_t buff[6];   // 多发送一个字节
    uint8_t check = 0; // 校验位

    buff[0] = USB_FRAME_HEAD;  // 帧头
    buff[1] = USB_ADDR_BUZZER; // 地址
    buff[2] = 5;               // 帧长
    switch (sound) {
    case Buzzer::BUZZER_OK: // 确认
      buff[3] = 0;
      break;
    case Buzzer::BUZZER_WARNNING: // 报警
      buff[3] = 1;
      break;
    case Buzzer::BUZZER_FINISH: // 完成
      buff[3] = 2;
      break;
    case Buzzer::BUZZER_DING: // 提示
      buff[3] = 3;
      break;
    case Buzzer::BUZZER_START: // 开机
      buff[3] = 4;
      break;
    }

    for (size_t i = 0; i < 4; i++){
      check += buff[i];
    }
      
    buff[4] = check;

    // 循环发送数据
    for (size_t i = 0; i < 6; i++)
      transmitByte(buff[i]);
  }
};

