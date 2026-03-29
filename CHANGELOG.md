# Electric Framework 更新简报

## 2026-03-28 更新

### 🎯 视觉控制系统优化

**核心改进：视觉控制优先级提升**
- 重构视觉接管逻辑，确保视觉模式（mode 1/2）下优先响应视觉数据
- 视觉控制时自动清零遥控器增量输入，避免控制冲突
- 添加 `RAD_TO_DEG` 宏定义，正确处理弧度到角度的转换

**代码变更：**
```c
// 视觉接管优先级最高
if (vision_recv_data->mode == 1 || vision_recv_data->mode == 2)
{
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    gimbal_cmd_send.yaw = vision_recv_data->yaw * RAD_TO_DEG;
    gimbal_cmd_send.pitch = vision_recv_data->pitch * RAD_TO_DEG;
    add_yaw = 0.0f;
    add_pitch = 0.0f;
}
```

### ⚙️ 云台配置修正

**Pitch轴电机方向调整**
- 修正哨兵机器人云台pitch轴电机和反馈方向配置
- 从 `MOTOR_DIRECTION_NORMAL` 改为 `MOTOR_DIRECTION_REVERSE`
- 从 `FEEDBACK_DIRECTION_NORMAL` 改为 `FEEDBACK_DIRECTION_REVERSE`

### 🔍 调试功能增强

**视觉数据解码日志**
- 在 `DecodeVision` 函数中添加详细调试日志
- 记录接收数据长度、SP协议头位置、CRC校验结果
- 输出视觉模式和云台角度数据，便于问题排查

**日志输出示例：**
```
[vision] DecodeVision called, recv_len=XX
[vision] Found SP header at offset X
[vision] Received: mode=X, yaw=X.XXX, pitch=X.XXX
```

---

## 影响范围

- **机器人类型：** 哨兵机器人
- **影响模块：** 视觉控制、云台控制、主控通信
- **测试建议：** 验证视觉自瞄模式下的云台响应和角度精度

## 提交信息

- **Commit:** 00b7967
- **分支:** main
- **仓库:** https://github.com/la-defense/electric_framework
