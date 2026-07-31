# 车体板到钢珠控制板 CAN 协议

## 1. 总线参数

- 模式：经典 CAN 2.0 数据帧
- 标识符：11 位标准 ID
- 仲裁波特率：500 kbit/s
- 字节序：多字节整数均为小端
- 车体板引脚：PA12 = CANTX，PA13 = CANRX
- 物理连接：使用扩展板 CAN_H、CAN_L 和 GND，不能直接连接两块 MCU 的 PA12/PA13

总线两端各保留一个 120 Ω 终端电阻。扩展板原理图中的 R5 已经是 120 Ω；
若又连接带终端的 USB-CAN，应确认整条总线只有两个 120 Ω 终端。

## 2. ID 分配

| 标准 ID | 方向 | 当前定义 |
|---|---|---|
| `0x180` | 车体板 -> 钢珠控制板 | WT61 三轴加速度，100 Hz |
| `0x181` | 车体板 -> 钢珠控制板 | 预留：姿态/角速度 |
| `0x182` | 车体板 -> 钢珠控制板 | 预留：车轮速度/底盘状态 |
| `0x183`~`0x187` | 车体板 -> 钢珠控制板 | 预留 |
| `0x188`~`0x18E` | 钢珠控制板 -> 车体板 | 预留：球控状态/命令 |
| `0x18F` | 双向约定 | 预留：版本与诊断 |

新增数据应优先分配新 ID，不要改变 `0x180` 的字节含义。这样旧版钢珠控制
固件仍能继续解析加速度。

## 3. `0x180` 加速度帧

经典 CAN，DLC = 8：

| 字节 | 类型 | 单位 | 含义 |
|---|---|---|---|
| 0~1 | `int16_t` LE | mg | WT61 X 轴加速度 |
| 2~3 | `int16_t` LE | mg | WT61 Y 轴加速度 |
| 4~5 | `int16_t` LE | mg | WT61 Z 轴加速度 |
| 6 | `uint8_t` | - | 发送序号，模 256 递增 |
| 7 bit0 | flag | - | `ACCEL_VALID`：至少收到过一帧正确的 `0x51` |
| 7 bit1 | flag | - | `ACCEL_FRESH`：加速度帧龄不超过配置阈值 |
| 7 bit2 | flag | - | `GYRO_FRESH` |
| 7 bit3 | flag | - | `ANGLE_FRESH` |
| 7 bit4 | reserved | - | 当前必须为 0 |
| 7 bit7~5 | `uint3` | - | 协议版本，当前为 1 |

换算：

```c
float acceleration_g = (float)acceleration_mg * 0.001f;
float acceleration_mps2 = (float)acceleration_mg * 0.00980665f;
```

`0x51` 是传感器坐标系加速度并包含重力分量。钢珠控制板必须按 WT61 的实际
安装方向确定哪一轴对应车辆前后/左右，静止时完成零偏与重力补偿。只有
`ACCEL_VALID=1` 且 `ACCEL_FRESH=1` 时才允许使用加速度前馈；否则前馈量应
平滑衰减到 0，不能保持最后一次加速度。

## 4. 钢珠控制板解析示例

```c
#include <stdbool.h>
#include <stdint.h>

#define VEHICLE_CAN_ID_ACCEL       (0x180u)
#define VEHICLE_CAN_VERSION        (1u)
#define VEHICLE_CAN_ACCEL_VALID    (1u << 0)
#define VEHICLE_CAN_ACCEL_FRESH    (1u << 1)

typedef struct
{
    float ax_mps2;
    float ay_mps2;
    float az_mps2;
    uint8_t sequence;
    bool usable;
} VehicleAcceleration;

static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] |
                     ((uint16_t)data[1] << 8u));
}

bool VehicleCan_DecodeAcceleration(uint16_t standard_id,
                                   const uint8_t *data,
                                   uint8_t dlc,
                                   VehicleAcceleration *output)
{
    uint8_t status;
    uint8_t version;

    if ((standard_id != VEHICLE_CAN_ID_ACCEL) ||
        (data == 0) || (output == 0) || (dlc != 8u)) {
        return false;
    }

    status = data[7];
    version = status >> 5u;
    if (version != VEHICLE_CAN_VERSION) {
        return false;
    }

    output->ax_mps2 = (float)read_i16_le(&data[0]) * 0.00980665f;
    output->ay_mps2 = (float)read_i16_le(&data[2]) * 0.00980665f;
    output->az_mps2 = (float)read_i16_le(&data[4]) * 0.00980665f;
    output->sequence = data[6];
    output->usable =
        ((status & (VEHICLE_CAN_ACCEL_VALID |
                    VEHICLE_CAN_ACCEL_FRESH)) ==
         (VEHICLE_CAN_ACCEL_VALID |
          VEHICLE_CAN_ACCEL_FRESH));
    return true;
}
```

钢珠控制板还应设置本地接收超时，例如连续 30~50 ms 未收到 `0x180` 时，将
`usable` 强制清零。序号跳变可用于统计丢帧，但 CAN 自带 CRC，不需要在8字节
载荷中重复增加校验和。
