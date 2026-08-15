/**
 * @file    ArduinoSTM32CAN.cpp
 * @brief   STM32CAN 库实现
 */

#include "ArduinoSTM32CAN.h"

// =========================================================================
// 静态成员
// =========================================================================
STM32CAN* STM32CAN::_instance = nullptr;

// =========================================================================
// 构造 / 析构
// =========================================================================

STM32CAN::STM32CAN(FDCAN_GlobalTypeDef* instance)
    : _started(false)
    , _interruptMode(false)
    , _mode(FD_BRS)
    , _filterCount(0)
    , _rxPort(nullptr)
    , _rxPin(0)
    , _rxAF(9)
    , _txPort(nullptr)
    , _txPin(0)
    , _txAF(9)
    , _pinsSet(false)
    , _rxCallback(nullptr)
{
    // 初始化 HAL 句柄 (内部管理，不再依赖 CubeMX)
    _handle.Instance = instance;
    _handle.State    = HAL_FDCAN_STATE_RESET;
}

STM32CAN::~STM32CAN()
{
    end();
}

// =========================================================================
// 引脚配置
// =========================================================================

// 将 STM32 Arduino 引脚编号转换为 GPIO 端口
static GPIO_TypeDef* _pinToPort(uint32_t pin)
{
    uint8_t idx = (pin >> 4) & 0x0F;
    switch (idx) {
        case 0:  return GPIOA;
        case 1:  return GPIOB;
        case 2:  return GPIOC;
        case 3:  return GPIOD;
        case 4:  return GPIOE;
        case 5:  return GPIOF;
        case 6:  return GPIOG;
        case 7:
#ifdef GPIOH
            return GPIOH;
#else
            return nullptr;
#endif
#ifdef GPIOI
        case 8:  return GPIOI;
#endif
#ifdef GPIOJ
        case 9:  return GPIOJ;
#endif
#ifdef GPIOK
        case 10: return GPIOK;
#endif
        default: return nullptr;
    }
}

void STM32CAN::setRx(uint32_t pin)
{
    GPIO_TypeDef* port = _pinToPort(pin);
    uint16_t pinMask = (uint16_t)(1 << (pin & 0x0F));
    setRx(port, pinMask, 9);
}

void STM32CAN::setTx(uint32_t pin)
{
    GPIO_TypeDef* port = _pinToPort(pin);
    uint16_t pinMask = (uint16_t)(1 << (pin & 0x0F));
    setTx(port, pinMask, 9);
}

void STM32CAN::setRx(GPIO_TypeDef* port, uint16_t pin, uint8_t af)
{
    _rxPort  = port;
    _rxPin   = pin;
    _rxAF    = af;
    _pinsSet = true;
}

void STM32CAN::setTx(GPIO_TypeDef* port, uint16_t pin, uint8_t af)
{
    _txPort  = port;
    _txPin   = pin;
    _txAF    = af;
    _pinsSet = true;
}

// =========================================================================
// 初始化
// =========================================================================

bool STM32CAN::begin(const Config& config)
{
    if (_started) {
        end();
    }

    // 1. 初始化外设 (时钟 + GPIO，不依赖 CubeMX)
    if (!_initPeripheral()) {
        return false;
    }

    // 2. 配置 FDCAN 内核
    if (!_configFdcan(config)) {
        return false;
    }

    // 3. Tx Delay Compensation (CANFD 必须)
    if (config.mode != CLASSIC) {
        if (!_configTdc()) {
            return false;
        }
    }

    // 4. 全局滤波器
    if (!_configGlobalFilter()) {
        return false;
    }

    // 5. 启动 FDCAN
    if (HAL_FDCAN_Start(&_handle) != HAL_OK) {
        return false;
    }

    _mode        = config.mode;
    _started     = true;
    _filterCount = 0;

    // 注册为静态实例 (用于 HAL 中断回调)
    _instance = this;

    // 6. 如果之前已通过 onReceive() 注册了回调，则激活中断
    if (_interruptMode && _rxCallback != nullptr) {
        HAL_FDCAN_ActivateNotification(
            &_handle,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
            0);
    }

    return true;
}

bool STM32CAN::begin(uint32_t nominalBaud, uint32_t dataBaud, Mode mode, uint32_t fdcanClockHz)
{
    Config cfg;
    cfg.mode = mode;
    cfg.fdcanClockHz = fdcanClockHz;

    uint32_t clock = fdcanClockHz;
    if (clock == 0) {
        clock = _getFdcanClock();
    }

    uint32_t prescaler, tseg1, tseg2;
    if (!_calcBitTiming(nominalBaud, clock, prescaler, tseg1, tseg2, false)) {
        return false;
    }
    cfg.nominalPrescaler = prescaler;
    cfg.nominalTimeSeg1  = tseg1;
    cfg.nominalTimeSeg2  = tseg2;
    cfg.nominalSJW       = 1;

    if (mode != CLASSIC) {
        if (dataBaud == 0) {
            dataBaud = nominalBaud;
        }
        if (!_calcBitTiming(dataBaud, clock, prescaler, tseg1, tseg2, true)) {
            return false;
        }
        cfg.dataPrescaler = prescaler;
        cfg.dataTimeSeg1  = tseg1;
        cfg.dataTimeSeg2  = tseg2;
        cfg.dataSJW       = 1;
    }

    return begin(cfg);
}

void STM32CAN::end()
{
    if (_started) {
        HAL_FDCAN_Stop(&_handle);
        HAL_FDCAN_DeInit(&_handle);
        _started = false;
    }

    if (_instance == this) {
        _instance = nullptr;
    }

    _filterCount   = 0;
    _interruptMode = false;
}

// =========================================================================
// 发送
// =========================================================================

bool STM32CAN::write(uint32_t id, const uint8_t* data, uint8_t len,
                     bool isFD, bool brs, bool extended)
{
    if (!_started || data == nullptr) {
        return false;
    }

    if (len > 64) {
        len = 64;
    }
    if (!isFD && len > 8) {
        len = 8;
    }

    FDCAN_TxHeaderTypeDef txHeader;
    txHeader.Identifier          = id;
    txHeader.IdType              = extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    txHeader.TxFrameType         = FDCAN_DATA_FRAME;
    txHeader.DataLength          = _encodeDlc(len);
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch       = (isFD && brs) ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    txHeader.FDFormat            = isFD ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker       = 0;

    return (HAL_FDCAN_AddMessageToTxFifoQ(&_handle, &txHeader, (uint8_t*)data) == HAL_OK);
}

bool STM32CAN::write(const Message& msg)
{
    return write(msg.id, msg.data, msg.len, msg.fd, msg.brs, msg.extended);
}

// =========================================================================
// 接收 (轮询)
// =========================================================================

int STM32CAN::available()
{
    if (!_started) return 0;
    return (int)HAL_FDCAN_GetRxFifoFillLevel(&_handle, FDCAN_RX_FIFO0);
}

bool STM32CAN::read(Message& msg)
{
    if (!_started) return false;
    if (available() == 0) return false;

    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[64] = {0};

    if (HAL_FDCAN_GetRxMessage(&_handle, FDCAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
        return false;
    }

    msg.id          = rxHeader.Identifier;
    msg.extended    = (rxHeader.IdType == FDCAN_EXTENDED_ID);
    msg.fd          = (rxHeader.FDFormat == FDCAN_FD_CAN);
    msg.brs         = (rxHeader.BitRateSwitch == FDCAN_BRS_ON);
    msg.esi         = (rxHeader.ErrorStateIndicator == FDCAN_ESI_PASSIVE);
    msg.len         = _decodeDlc(rxHeader.DataLength);
    msg.timestamp   = (uint16_t)rxHeader.RxTimestamp;
    msg.filterIndex = (uint8_t)rxHeader.FilterIndex;

    for (uint8_t i = 0; i < msg.len; i++) {
        msg.data[i] = rxData[i];
    }

    return true;
}

// =========================================================================
// 接收 (中断回调)
// =========================================================================

void STM32CAN::onReceive(void (*callback)(const Message&))
{
    _rxCallback    = callback;
    _interruptMode = (callback != nullptr);

    if (_interruptMode) {
        if (_started) {
            HAL_FDCAN_ActivateNotification(
                &_handle,
                FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                0);
        }
    } else {
        if (_started) {
            HAL_FDCAN_DeactivateNotification(
                &_handle,
                FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
        }
    }
}

// =========================================================================
// 滤波器
// =========================================================================

bool STM32CAN::setFilter(uint32_t id, uint32_t mask,
                          bool extended, uint8_t fifo)
{
    if (!_started) return false;

    FDCAN_FilterTypeDef filter;
    filter.IdType       = extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    filter.FilterIndex  = _filterCount;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = (fifo == 0) ? FDCAN_FILTER_TO_RXFIFO0
                                      : FDCAN_FILTER_TO_RXFIFO1;
    filter.FilterID1    = id;
    filter.FilterID2    = mask;

    if (HAL_FDCAN_ConfigFilter(&_handle, &filter) != HAL_OK) {
        return false;
    }

    _filterCount++;
    return true;
}

bool STM32CAN::setFilterRange(uint32_t id1, uint32_t id2,
                               bool extended, uint8_t fifo)
{
    if (!_started) return false;

    FDCAN_FilterTypeDef filter;
    filter.IdType       = extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    filter.FilterIndex  = _filterCount;
    filter.FilterType   = FDCAN_FILTER_RANGE;
    filter.FilterConfig = (fifo == 0) ? FDCAN_FILTER_TO_RXFIFO0
                                      : FDCAN_FILTER_TO_RXFIFO1;
    filter.FilterID1    = id1;
    filter.FilterID2    = id2;

    if (HAL_FDCAN_ConfigFilter(&_handle, &filter) != HAL_OK) {
        return false;
    }

    _filterCount++;
    return true;
}

void STM32CAN::clearFilters()
{
    for (uint8_t i = 0; i < _filterCount; i++) {
        FDCAN_FilterTypeDef filter;
        filter.IdType       = FDCAN_STANDARD_ID;
        filter.FilterIndex  = i;
        filter.FilterType   = FDCAN_FILTER_MASK;
        filter.FilterConfig = FDCAN_FILTER_DISABLE;
        filter.FilterID1    = 0;
        filter.FilterID2    = 0;
        HAL_FDCAN_ConfigFilter(&_handle, &filter);
    }
    _filterCount = 0;
}

// =========================================================================
// 状态
// =========================================================================

uint8_t STM32CAN::getErrorCounter()
{
    if (!_started) return 0;
    uint32_t psr = _handle.Instance->PSR;
    uint8_t tec = (uint8_t)((psr >> 16) & 0x7F);
    uint8_t rec = (uint8_t)((psr >> 8) & 0x7F);
    return (tec > rec) ? tec : rec;
}

uint8_t STM32CAN::getTxErrorCounter()
{
    if (!_started) return 0;
    return (uint8_t)((_handle.Instance->PSR >> 16) & 0x7F);
}

uint8_t STM32CAN::getRxErrorCounter()
{
    if (!_started) return 0;
    return (uint8_t)((_handle.Instance->PSR >> 8) & 0x7F);
}

uint8_t STM32CAN::getBusState()
{
    if (!_started) return 0xFF;
    uint32_t psr = _handle.Instance->PSR;
    if (psr & FDCAN_PSR_BO) return 3;
    if (psr & FDCAN_PSR_EP) return 2;
    if (psr & FDCAN_PSR_EW) return 1;
    return 0;
}

// =========================================================================
// 周期任务
// =========================================================================

void STM32CAN::loop()
{
    if (!_started) return;

    // 仅在轮询模式下触发回调 (中断模式下由 ISR 直接调用)
    if (!_interruptMode && _rxCallback != nullptr) {
        int count = available();
        while (count > 0) {
            Message msg;
            if (read(msg)) {
                _rxCallback(msg);
            }
            count--;
        }
    }
}

// =========================================================================
// 内部方法
// =========================================================================

bool STM32CAN::_initPeripheral()
{
    // 1. 使能 FDCAN 时钟
    __HAL_RCC_FDCAN_CLK_ENABLE();

    // 2. 配置 FDCAN 时钟源 = PCLK1 (APB1)
    RCC_PeriphCLKInitTypeDef periphClk;
    periphClk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    periphClk.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&periphClk) != HAL_OK) {
        return false;
    }

    // 3. 如果用户设置了引脚，直接配置 GPIO
    if (_pinsSet) {
        // RX 引脚
        if (_rxPort != nullptr) {
            GPIO_InitTypeDef gpio = {0};
            gpio.Pin       = _rxPin;
            gpio.Mode      = GPIO_MODE_AF_PP;
            gpio.Pull      = GPIO_NOPULL;
            gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
            gpio.Alternate = _rxAF;
            HAL_GPIO_Init(_rxPort, &gpio);
        }

        // TX 引脚
        if (_txPort != nullptr) {
            GPIO_InitTypeDef gpio = {0};
            gpio.Pin       = _txPin;
            gpio.Mode      = GPIO_MODE_AF_PP;
            gpio.Pull      = GPIO_NOPULL;
            gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
            gpio.Alternate = _txAF;
            HAL_GPIO_Init(_txPort, &gpio);
        }

        // 使能 GPIO 时钟
        if (_rxPort == GPIOA || _txPort == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
        if (_rxPort == GPIOB || _txPort == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
        if (_rxPort == GPIOC || _txPort == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
#ifdef GPIOD
        if (_rxPort == GPIOD || _txPort == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
#endif
#ifdef GPIOE
        if (_rxPort == GPIOE || _txPort == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
#endif
#ifdef GPIOF
        if (_rxPort == GPIOF || _txPort == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
#endif
#ifdef GPIOG
        if (_rxPort == GPIOG || _txPort == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
#endif
#ifdef GPIOH
        if (_rxPort == GPIOH || _txPort == GPIOH) __HAL_RCC_GPIOH_CLK_ENABLE();
#endif
    }
    // 否则依赖 HAL_FDCAN_MspInit (CubeMX 生成的回调，在 HAL_FDCAN_Init 内调用)

    return true;
}

bool STM32CAN::_configFdcan(const Config& cfg)
{
    _handle.Init.ClockDivider       = FDCAN_CLOCK_DIV1;
    _handle.Init.AutoRetransmission = cfg.autoRetransmission ? ENABLE : DISABLE;
    _handle.Init.TransmitPause      = cfg.transmitPause ? ENABLE : DISABLE;
    _handle.Init.ProtocolException  = cfg.protocolException ? ENABLE : DISABLE;
    _handle.Init.TxFifoQueueMode    = FDCAN_TX_FIFO_OPERATION;

    switch (cfg.mode) {
        case CLASSIC:
            _handle.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
            break;
        case FD_NO_BRS:
            _handle.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
            break;
        case FD_BRS:
        default:
            _handle.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
            break;
    }

    _handle.Init.Mode = FDCAN_MODE_NORMAL;

    _handle.Init.NominalPrescaler     = cfg.nominalPrescaler;
    _handle.Init.NominalSyncJumpWidth = cfg.nominalSJW;
    _handle.Init.NominalTimeSeg1      = cfg.nominalTimeSeg1;
    _handle.Init.NominalTimeSeg2      = cfg.nominalTimeSeg2;

    _handle.Init.DataPrescaler      = cfg.dataPrescaler;
    _handle.Init.DataSyncJumpWidth  = cfg.dataSJW;
    _handle.Init.DataTimeSeg1       = cfg.dataTimeSeg1;
    _handle.Init.DataTimeSeg2       = cfg.dataTimeSeg2;

    _handle.Init.StdFiltersNbr = 14;  // 分配标准帧滤波器 RAM (G4 最大 28)
    _handle.Init.ExtFiltersNbr = 0;

    return (HAL_FDCAN_Init(&_handle) == HAL_OK);
}

bool STM32CAN::_configTdc()
{
    if (HAL_FDCAN_ConfigTxDelayCompensation(&_handle, 0, 0) != HAL_OK) {
        return false;
    }
    return (HAL_FDCAN_EnableTxDelayCompensation(&_handle) == HAL_OK);
}

bool STM32CAN::_configGlobalFilter()
{
    return (HAL_FDCAN_ConfigGlobalFilter(
        &_handle,
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE
    ) == HAL_OK);
}

// =========================================================================
// 比特率计算
// =========================================================================

bool STM32CAN::_calcBitTiming(uint32_t baud, uint32_t clock,
                               uint32_t& prescaler, uint32_t& tseg1, uint32_t& tseg2,
                               bool isDataPhase)
{
    if (baud == 0 || clock == 0) return false;

    const uint32_t prescalerMin = 1;
    const uint32_t prescalerMax = isDataPhase ? 32U : 512U;
    const uint32_t tseg1Min     = isDataPhase ? 1U : 2U;
    const uint32_t tseg1Max     = isDataPhase ? 32U : 256U;
    const uint32_t tseg2Min     = isDataPhase ? 1U : 2U;
    const uint32_t tseg2Max     = isDataPhase ? 16U : 128U;

    const float targetSP = isDataPhase ? 0.75f : 0.80f;

    int32_t bestError = INT32_MAX;
    uint32_t bestPrescaler = 1, bestTseg1 = tseg1Min, bestTseg2 = tseg2Min;

    for (uint32_t p = prescalerMin; p <= prescalerMax; p++) {
        uint32_t totalTq = (uint32_t)(clock / ((uint64_t)p * baud));

        uint32_t minTq = 1 + tseg1Min + tseg2Min;
        uint32_t maxTq = 1 + tseg1Max + tseg2Max;
        if (totalTq < minTq || totalTq > maxTq) {
            continue;
        }

        uint32_t actualBaud = (uint32_t)(clock / ((uint64_t)p * totalTq));
        int32_t error = (int32_t)baud - (int32_t)actualBaud;
        if (error < 0) error = -error;

        uint32_t tseg1_candidate = (uint32_t)(totalTq * targetSP) - 1;
        if (tseg1_candidate < tseg1Min) tseg1_candidate = tseg1Min;
        if (tseg1_candidate > tseg1Max) tseg1_candidate = tseg1Max;

        uint32_t tseg2_candidate = totalTq - 1 - tseg1_candidate;
        if (tseg2_candidate < tseg2Min) {
            tseg2_candidate = tseg2Min;
            tseg1_candidate = totalTq - 1 - tseg2_candidate;
            if (tseg1_candidate < tseg1Min || tseg1_candidate > tseg1Max) {
                continue;
            }
        }
        if (tseg2_candidate > tseg2Max) {
            continue;
        }

        if (error < bestError) {
            bestError     = error;
            bestPrescaler = p;
            bestTseg1     = tseg1_candidate;
            bestTseg2     = tseg2_candidate;
        }

        if (error == 0) break;
    }

    if (bestError == INT32_MAX) return false;
    if (bestError > (int32_t)(baud / 100)) return false;

    prescaler = bestPrescaler;
    tseg1     = bestTseg1;
    tseg2     = bestTseg2;

    return true;
}

// =========================================================================
// DLC 编解码
// =========================================================================

uint32_t STM32CAN::_encodeDlc(uint8_t len)
{
    switch (len) {
        case 0:  return FDCAN_DLC_BYTES_0;
        case 1:  return FDCAN_DLC_BYTES_1;
        case 2:  return FDCAN_DLC_BYTES_2;
        case 3:  return FDCAN_DLC_BYTES_3;
        case 4:  return FDCAN_DLC_BYTES_4;
        case 5:  return FDCAN_DLC_BYTES_5;
        case 6:  return FDCAN_DLC_BYTES_6;
        case 7:  return FDCAN_DLC_BYTES_7;
        case 8:  return FDCAN_DLC_BYTES_8;
        case 12: return FDCAN_DLC_BYTES_12;
        case 16: return FDCAN_DLC_BYTES_16;
        case 20: return FDCAN_DLC_BYTES_20;
        case 24: return FDCAN_DLC_BYTES_24;
        case 32: return FDCAN_DLC_BYTES_32;
        case 48: return FDCAN_DLC_BYTES_48;
        case 64: return FDCAN_DLC_BYTES_64;
        default:
            if (len <= 8)  return FDCAN_DLC_BYTES_8;
            if (len <= 12) return FDCAN_DLC_BYTES_12;
            if (len <= 16) return FDCAN_DLC_BYTES_16;
            if (len <= 20) return FDCAN_DLC_BYTES_20;
            if (len <= 24) return FDCAN_DLC_BYTES_24;
            if (len <= 32) return FDCAN_DLC_BYTES_32;
            if (len <= 48) return FDCAN_DLC_BYTES_48;
            return FDCAN_DLC_BYTES_64;
    }
}

uint8_t STM32CAN::_decodeDlc(uint32_t dlc)
{
    switch (dlc) {
        case FDCAN_DLC_BYTES_0:  return 0;
        case FDCAN_DLC_BYTES_1:  return 1;
        case FDCAN_DLC_BYTES_2:  return 2;
        case FDCAN_DLC_BYTES_3:  return 3;
        case FDCAN_DLC_BYTES_4:  return 4;
        case FDCAN_DLC_BYTES_5:  return 5;
        case FDCAN_DLC_BYTES_6:  return 6;
        case FDCAN_DLC_BYTES_7:  return 7;
        case FDCAN_DLC_BYTES_8:  return 8;
        case FDCAN_DLC_BYTES_12: return 12;
        case FDCAN_DLC_BYTES_16: return 16;
        case FDCAN_DLC_BYTES_20: return 20;
        case FDCAN_DLC_BYTES_24: return 24;
        case FDCAN_DLC_BYTES_32: return 32;
        case FDCAN_DLC_BYTES_48: return 48;
        case FDCAN_DLC_BYTES_64: return 64;
        default: return 0;
    }
}

// =========================================================================
// 时钟检测
// =========================================================================

uint32_t STM32CAN::_getFdcanClock()
{
    return HAL_RCC_GetPCLK1Freq();
}

// =========================================================================
// HAL 中断回调 (弱函数覆盖)
// =========================================================================

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;

    STM32CAN* instance = STM32CAN::_instance;
    if (instance == nullptr) return;
    if (&instance->_handle != hfdcan) return;
    if (instance->_rxCallback == nullptr) return;

    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[64] = {0};

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
        STM32CAN::Message msg;
        msg.id          = rxHeader.Identifier;
        msg.extended    = (rxHeader.IdType == FDCAN_EXTENDED_ID);
        msg.fd          = (rxHeader.FDFormat == FDCAN_FD_CAN);
        msg.brs         = (rxHeader.BitRateSwitch == FDCAN_BRS_ON);
        msg.esi         = (rxHeader.ErrorStateIndicator == FDCAN_ESI_PASSIVE);
        msg.len         = instance->_decodeDlc(rxHeader.DataLength);
        msg.timestamp   = (uint16_t)rxHeader.RxTimestamp;
        msg.filterIndex = (uint8_t)rxHeader.FilterIndex;

        for (uint8_t i = 0; i < msg.len; i++) {
            msg.data[i] = rxData[i];
        }

        instance->_rxCallback(msg);
    }
}