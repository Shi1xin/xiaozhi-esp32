# X-Atom 开发板（Linx）

X-Atom 是 Linx 对讲设备的板级支持（board support），芯片为 ESP32-S3，双网络（Wi-Fi 默认 + ML307 4G），板内集成 ES8311 音频编解码、AXP2101 电源管理、QMI8658 惯性传感器、RKB1161 雷达、4 像素 WS2812 灯环与单按键

## 引脚映射

| 功能 | 引脚 / 参数 |
| --- | --- |
| 共享 I2C | I2C1，SDA=GPIO3，SCL=GPIO2 |
| ES8311 编解码 | I2C 设备地址 0x18（编解码 API 参数 0x30） |
| I2S0 | MCLK=6，BCLK=7，DIN=8，WS=9，DOUT=10 |
| PA 使能 | GPIO41，高电平有效，上电锁存低电平 |
| AXP2101 电源管理 | I2C 地址 0x35，芯片 ID 0x4A |
| QMI8658 IMU | I2C 地址 0x6B，WHO_AM_I=0x05；中断脚 GPIO42（仅输入） |
| RKB1161 雷达 | UART2，TX=17，RX=18，115200 8N1；数字输入 GPIO16 |
| ML307 4G | UART1，TX=43，RX=44，DTR=21（低电平唤醒） |
| 启动控制 | GPIO39 输出高电平（复现原厂固件启动状态，电气角色尚未确认） |
| LED 灯环 | GPIO40，4 像素 WS2812 GRB，RMT 驱动 |
| 按键 | GPIO5，低电平有效 |

无显示屏，`GetDisplay()` 返回内置 NoDisplay。闪存 16 MiB，PSRAM 为 8 MiB 八线 80 MHz，分区表使用 `partitions/v2/16m.csv`

## 构建

规范构建命令：

```sh
python3 scripts/build.py xatom --name xatom-linx --language zh-CN --wake-word nihaoxiaozhi
```

该变体启用 BluFi/Bluedroid 配网（设备名 `ESP_<Wi-Fi MAC 小写 12 位十六进制>`）、AFE 音频处理与设备侧 AEC、USB Serial/JTAG 控制台

## 按键与网络

| 操作 | 行为 |
| --- | --- |
| 单击 | 进入/结束对话 |
| 双击 | 重启 |
| 三击 | 切换到另一种网络并重启（Wi-Fi ↔ 4G，持久化语义与上游一致） |
| 长按 3 秒 | Wi-Fi 模式：进入 BluFi 配网，保留已有凭据；4G 模式：写入 `network.type=0` 与一次性救援标志 `xatom.provision=true` 后重启 |
| 长按 7 秒 | 关机（与 `self.xatom.shutdown` 相同序列；3 秒配网动作会先触发但不影响关机执行） |

救援启动路径：正常初始化网络回调；若此前存在已保存 SSID，则额外显式进入配网模式；若列表为空，常规启动流程已自动进入配网，不会重复初始化。救援标志在进入该路径后清除，其他 NVS 命名空间不受影响

`xatom.provision` 是本板新增的 NVS 键（命名空间 `xatom`，布尔值），仅表示一次强制配网启动

## 电源

- 空闲 60 秒进入低网络功耗档，无自动关机
- 唤醒来源：按键、有效非零雷达状态（每 1000ms 至多一次，待处理唤醒最多一个）
- 4G 省电调用 `AtModem::SetSleepMode` 虚接口：LOW_POWER 发送 `AT+MLPMCFG="sleepmode",2,0`，其他档位发送模式 0。命令经长度 1 覆盖队列进入专用控制任务执行，相同档位去重，Connected 事件后强制应用一次记录档位
- 关机仅由显式用户操作触发：PA 关断、灯环清零、GPIO39 置低、PMIC 0x10 读改写置 bit0

## MCP 工具

| 工具 | 权限 | 说明 |
| --- | --- | --- |
| `self.xatom.get_state` | 只读 | 返回 power/motion/radar/inputs 组合快照 JSON |
| `self.xatom.shutdown` | 仅用户 | 延迟 1000ms 后执行上述关机序列；已有挂起动作时返回失败 |
| `self.xatom.switch_network` | 仅用户 | 必填字符串 `network`，取值 `wifi` 或 `4g`；相同网络返回 true 不重启；非法值返回错误 |

快照语义：`available` 表示设备初始化时探测成功，`valid` 表示本次采样全部必需读取成功，`last_error`/`sample_us` 记录失败码与微秒时间戳。缺失或失效字段以 JSON `null` 表示。gyro 数据为 CTRL3=0x66 预设下的原始计数，雷达 distance/signal 为原始值，单位未经确认

## 音频

ES8311 采用 24 kHz/16bit 立体声参考输入（BCLK，ADCL+DACR，use_mclk=false），输出对应用层保持单声道：Write 内将每个单声道采样复制到交错立体声两个槽位（240 帧栈缓冲分块）。开启后 1000ms 内 Read 返回 0（非阻塞 ADC 稳定窗口）。其他开发板使用的构造函数参数不变，`input_reference` 为追加的可选参数
