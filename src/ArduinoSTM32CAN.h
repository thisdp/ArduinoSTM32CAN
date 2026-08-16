/**
 * @file    ArduinoSTM32CAN.h
 * @brief   Arduino-style CAN/CANFD library for STM32 (HAL-based)
 *
 * 封装 STM32 HAL FDCAN，提供 Arduino 风格 API：
 *   setRx(), setTx(), begin(), write(), available(), read(), onReceive()
 *
 * 支持的 STM32 系列：
 *   - STM32G4  (G431/G441/G471/G473/G474/G483/G484/G491)
 *   - STM32H7  (H723/H725/H730/H733/H735/H742/H743/H745/H747/H750/H753/H755/H757/H7A3/H7B0/H7B3)
 *   - STM32G0  (G071/G081/G0B1/G0C1)
 *   - STM32L5  (L552/L562)
 *   - STM32U5  (U575/U585/U595/U599/U5A5/U5A9)
 *   - STM32H5  (H503/H563/H573)
 *   - STM32WB  (WB55 等)
 *
 * 用法示例：
 * @code
 *   #include "ArduinoSTM32CAN.h"
 *
 *   STM32CAN can(FDCAN1);
 *
 *   void setup() {
 *       can.setRx(PA11);  // PA11 = FDCAN1_RX, 自动检测 AF9
 *       can.setTx(PA12);  // PA12 = FDCAN1_TX, 自动检测 AF9
 *       can.begin(500000, 5000000);      // 仲裁段 500k, 数据段 5M
 *   }
 *
 *   void loop() {
 *       STM32CAN::Message msg;
 *       if (can.read(msg)) {
 *           // 处理 msg
 *       }
 *   }
 * @endcode
 */

#ifndef _ARDUINO_STM32_CAN_H_
#define _ARDUINO_STM32_CAN_H_

#include <stdint.h>

#ifdef __cplusplus

// =========================================================================
// 自动检测 STM32 系列，包含对应的 HAL 头文件
// =========================================================================
#ifndef __MAIN_H
  #if defined(__has_include)
    #if __has_include("stm32g4xx_hal.h")
      #include "stm32g4xx_hal.h"
    #elif __has_include("stm32h7xx_hal.h")
      #include "stm32h7xx_hal.h"
    #elif __has_include("stm32g0xx_hal.h")
      #include "stm32g0xx_hal.h"
    #elif __has_include("stm32l5xx_hal.h")
      #include "stm32l5xx_hal.h"
    #elif __has_include("stm32u5xx_hal.h")
      #include "stm32u5xx_hal.h"
    #elif __has_include("stm32h5xx_hal.h")
      #include "stm32h5xx_hal.h"
    #elif __has_include("stm32mp1xx_hal.h")
      #include "stm32mp1xx_hal.h"
    #elif __has_include("stm32wbxx_hal.h")
      #include "stm32wbxx_hal.h"
    #else
      #error "ArduinoSTM32CAN: 未检测到 STM32 HAL 头文件，请先 #include 你的 stm32xx_hal.h"
    #endif
  #elif defined(STM32G431xx) || defined(STM32G441xx) || defined(STM32G471xx) || \
        defined(STM32G473xx) || defined(STM32G474xx) || defined(STM32G483xx) || \
        defined(STM32G484xx) || defined(STM32G491xx) || defined(STM32G4A1xx) || \
        defined(STM32G4)
    #include "stm32g4xx_hal.h"
  #elif defined(STM32H723xx) || defined(STM32H725xx) || defined(STM32H730xx) || \
        defined(STM32H733xx) || defined(STM32H735xx) || defined(STM32H742xx) || \
        defined(STM32H743xx) || defined(STM32H745xx) || defined(STM32H747xx) || \
        defined(STM32H750xx) || defined(STM32H753xx) || defined(STM32H755xx) || \
        defined(STM32H757xx) || defined(STM32H7A3xx) || defined(STM32H7B0xx) || \
        defined(STM32H7B3xx) || defined(STM32H7)
    #include "stm32h7xx_hal.h"
  #elif defined(STM32G071xx) || defined(STM32G081xx) || defined(STM32G0B1xx) || \
        defined(STM32G0C1xx) || defined(STM32G0)
    #include "stm32g0xx_hal.h"
  #elif defined(STM32L552xx) || defined(STM32L562xx) || defined(STM32L5)
    #include "stm32l5xx_hal.h"
  #elif defined(STM32U575xx) || defined(STM32U585xx) || defined(STM32U595xx) || \
        defined(STM32U599xx) || defined(STM32U5A5xx) || defined(STM32U5A9xx) || \
        defined(STM32U5)
    #include "stm32u5xx_hal.h"
  #elif defined(STM32H503xx) || defined(STM32H563xx) || defined(STM32H573xx) || \
        defined(STM32H5)
    #include "stm32h5xx_hal.h"
  #elif defined(STM32WB)
    #include "stm32wbxx_hal.h"
  #else
    #error "ArduinoSTM32CAN: 不支持的 STM32 系列，请先 #include 你的 stm32xx_hal.h"
  #endif
#endif  // __MAIN_H

#if !defined(FDCAN1) && !defined(FDCAN)
  #error "ArduinoSTM32CAN: 当前 STM32 系列不支持 FDCAN (需要 G4/H7/G0/L5/U5/H5/WB)"
#endif

// HAL 回调前向声明 (C 链接，与 STM32 HAL 一致)
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef*, uint32_t);

/**
 * @brief STM32CAN — Arduino 风格 CAN/CANFD 库
 *
 * 参考 HardwareSerial 设计：
 *   - 构造函数传入外设实例 (FDCAN1, FDCAN2 等)
 *   - setRx() / setTx() 指定引脚
 *   - begin() 完成初始化
 *
 * 不依赖 CubeMX 的 CAN 配置 — 库内部处理 GPIO、时钟、时钟源。
 */
class STM32CAN {
public:
    /* ================================================================ *
     * 公开类型
     * ================================================================ */

    /** @brief 帧格式模式 */
    enum Mode : uint8_t {
        CLASSIC   = 0,  // 经典 CAN (最大 8 字节)
        FD_NO_BRS = 1,  // CANFD 无比特率切换
        FD_BRS    = 2,  // CANFD 带比特率切换 (推荐)
    };

    /** @brief 接收到的 CAN/CANFD 消息 */
    struct Message {
        uint32_t id;           // 消息 ID
        bool     extended;     // 是否为扩展帧
        bool     fd;           // 是否为 CANFD 帧
        bool     brs;          // 是否启用了比特率切换
        bool     esi;          // 错误状态指示
        uint8_t  len;          // 实际数据长度 (0..64)
        uint8_t  data[64];     // 数据缓冲区
        uint16_t timestamp;    // 硬件时间戳
        uint8_t  filterIndex;  // 匹配的滤波器索引
    };

    /** @brief 详细配置结构体（高级用户使用） */
    struct Config {
        Mode mode = FD_BRS;

        // 仲裁段 (Nominal) 比特率参数
        uint32_t nominalPrescaler = 10;
        uint32_t nominalTimeSeg1  = 29;
        uint32_t nominalTimeSeg2  = 2;
        uint32_t nominalSJW       = 1;

        // 数据段 (Data) 比特率参数
        uint32_t dataPrescaler = 4;
        uint32_t dataTimeSeg1  = 5;
        uint32_t dataTimeSeg2  = 2;
        uint32_t dataSJW       = 1;

        // FDCAN 内核时钟频率 (Hz)，0 = 自动检测
        uint32_t fdcanClockHz = 0;

        // 其他
        bool autoRetransmission = false;
        bool transmitPause      = false;
        bool protocolException  = true;
    };

    /* ================================================================ *
     * 构造 / 析构
     * ================================================================ */

    /**
     * @brief 构造函数 — 参考 HardwareSerial(USART1) 模式
     * @param instance  FDCAN 外设实例: FDCAN1, FDCAN2 等
     */
    STM32CAN(FDCAN_GlobalTypeDef* instance = FDCAN1);

    ~STM32CAN();

    /* ================================================================ *
     * 引脚配置 (必须在 begin() 之前调用)
     * ================================================================ */

    /**
     * @brief 设置 RX/TX 引脚 (简化版 — 自动检测端口和 AF)
     * @param pin  STM32 Arduino 引脚编号 (如 PA11, PB7)
     *
     * 例: can.setRx(PA11);   // 自动检测 GPIOA, GPIO_PIN_11, AF9
     *     can.setTx(PB6);    // 自动检测 GPIOB, GPIO_PIN_6, AF9
     */
    void setRx(uint32_t pin);
    void setTx(uint32_t pin);

    /**
     * @brief 设置 RX/TX 引脚 (高级版 — 手动指定端口、引脚和 AF)
     * @param port  GPIO 端口 (GPIOA, GPIOB, ...)
     * @param pin   GPIO 引脚 (GPIO_PIN_0 .. GPIO_PIN_15)
     * @param af    Alternate Function 编号，默认 AF9 (FDCAN 最常用)
     *
     * 例: can.setRx(GPIOA, GPIO_PIN_11);       // PA11, AF9
     *     can.setRx(GPIOH, GPIO_PIN_14, 11);   // PH14, AF11 (部分 H7)
     */
    void setRx(GPIO_TypeDef* port, uint16_t pin, uint8_t af = 9);
    void setTx(GPIO_TypeDef* port, uint16_t pin, uint8_t af = 9);

    /* ================================================================ *
     * 初始化
     * ================================================================ */

    /**
     * @brief 使用详细配置初始化
     */
    bool begin(const Config& config);

    /**
     * @brief 便捷初始化 — 只需指定波特率
     * @param nominalBaud   仲裁段波特率 (如 500000)
     * @param dataBaud      数据段波特率 (如 5000000)
     * @param mode          帧格式 (默认 FD_BRS)
     * @param fdcanClockHz  FDCAN 内核时钟 (0 = 自动检测)
     */
    bool begin(uint32_t nominalBaud, uint32_t dataBaud = 0,
               Mode mode = FD_BRS, uint32_t fdcanClockHz = 0);

    void end();

    /* ================================================================ *
     * 发送
     * ================================================================ */

    bool write(uint32_t id, const uint8_t* data, uint8_t len,
               bool isFD = true, bool brs = true, bool extended = false);
    bool write(const Message& msg);

    /* ================================================================ *
     * 接收 (轮询)
     * ================================================================ */

    int available();
    bool read(Message& msg);

    /* ================================================================ *
     * 接收 (中断回调)
     * ================================================================ */

    void onReceive(void (*callback)(const Message&));

    /* ================================================================ *
     * 滤波器
     * ================================================================ */

    bool setFilter(uint32_t id, uint32_t mask,
                   bool extended = false, uint8_t fifo = 0);
    bool setFilterRange(uint32_t id1, uint32_t id2,
                        bool extended = false, uint8_t fifo = 0);
    void clearFilters();

    /* ================================================================ *
     * 状态
     * ================================================================ */

    bool isStarted() const { return _started; }
    uint8_t getErrorCounter();
    uint8_t getTxErrorCounter();
    uint8_t getRxErrorCounter();
    uint8_t getBusState();

    /* ================================================================ *
     * 周期任务
     * ================================================================ */

    void loop();

private:
    /* ================================================================ *
     * 内部方法
     * ================================================================ */

    bool _initPeripheral();
    bool _configFdcan(const Config& cfg);
    bool _configTdc();
    bool _configGlobalFilter();
    bool _calcBitTiming(uint32_t baud, uint32_t clock,
                        uint32_t& prescaler, uint32_t& tseg1, uint32_t& tseg2,
                        bool isDataPhase = false);
    uint32_t _encodeDlc(uint8_t len);
    uint8_t _decodeDlc(uint32_t dlc);
    uint32_t _getFdcanClock();

    /* ================================================================ *
     * 成员变量
     * ================================================================ */

    FDCAN_HandleTypeDef _handle;          // 内部 HAL 句柄 (值类型)
    bool                _started;
    bool                _interruptMode;
    Mode                _mode;
    uint8_t             _filterCount;
    uint8_t             _extFilterCount;

    // 引脚配置
    GPIO_TypeDef* _rxPort;   // RX GPIO 端口
    uint16_t      _rxPin;    // RX GPIO 引脚号
    uint8_t       _rxAF;     // RX Alternate Function
    GPIO_TypeDef* _txPort;   // TX GPIO 端口
    uint16_t      _txPin;    // TX GPIO 引脚号
    uint8_t       _txAF;     // TX Alternate Function
    bool          _pinsSet;  // 用户是否设置了引脚

    // 回调
    void (*_rxCallback)(const Message&);

    // 静态实例指针 (用于 HAL 中断回调)
    static STM32CAN* _instance;

    friend void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef*, uint32_t);
};

#endif /* __cplusplus */
#endif /* _ARDUINO_STM32_CAN_H_ */