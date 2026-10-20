#include "minikame.h"

// ============================================================
// 更精确的舵机驱动参数
// ============================================================
static constexpr int MINIKAME_MIN_PULSE_US = 500;
static constexpr int MINIKAME_MAX_PULSE_US = 2500;

// 逻辑角度的合法范围（度）
static constexpr float MINIKAME_MIN_ANGLE = 0.0f;
static constexpr float MINIKAME_MAX_ANGLE = 180.0f;

// ============================================================
// MiniKame 四足机器人控制类实现
// 舵机编号与身体部位对应关系：
//   0: 前左大腿    1: 前右大腿
//   2: 前左小腿    3: 前右小腿
//   4: 后左大腿    5: 后右大腿
//   6: 后左小腿    7: 后右小腿
// ============================================================

void MiniKame::init() {
  // 舵机与开发板引脚的映射（与接线图一致）
  board_pins[0] = 12; // 舵机 S0 - 前左大腿
  board_pins[1] = 16; // 舵机 S1 - 前右大腿
  board_pins[2] = 25; // 舵机 S2 - 前左小腿
  board_pins[3] = 18; // 舵机 S3 - 前右小腿
  board_pins[4] = 13; // 舵机 S4 - 后左大腿
  board_pins[5] = 17; // 舵机 S5 - 后右大腿
  board_pins[6] = 26; // 舵机 S6 - 后左小腿
  board_pins[7] = 19; // 舵机 S7 - 后右小腿

  // 零位校准微调值（trim），用于修正舵机安装偏差
  // 实际使用时根据机械零位情况修改这些值
  trim[0] = 0; // 前左大腿
  trim[1] = 0; // 前右大腿
  trim[2] = 0; // 前左小腿
  trim[3] = 0; // 前右小腿
  trim[4] = 0; // 后左大腿
  trim[5] = 0; // 后右大腿
  trim[6] = 0; // 后左小腿
  trim[7] = 0; // 后右小腿

  // 设置舵机反向标志，默认全部不反向
  for (int i = 0; i < 8; i++) reverse[i] = false;

  // 初始化每个舵机的振荡器，并将舵机 attach 到对应引脚
  // 显式指定 500~2500us 脉宽范围：ESP32Servo 默认范围为 544~2400us，
  // 若用默认值会把 0/180 度两端截断，导致实际行程不足 180 度
  for (int i = 0; i < 8; i++) {
    oscillator[i].start();
    servo[i].attach(board_pins[i], MINIKAME_MIN_PULSE_US, MINIKAME_MAX_PULSE_US);
  }

  // 回到初始站立姿态
  home();
}

// 翻转指定舵机的运动方向
void MiniKame::reverseServo(int id) {
  if (id < 0 || id >= 8) return;
  reverse[id] = !reverse[id];
}

// ------------------------------------------------------------
// 角度 → 脉宽（更精确的线性映射）
// ------------------------------------------------------------
int MiniKame::angToUsec(float value) {
  // 严格钳位，防止超程
  if (value < MINIKAME_MIN_ANGLE) value = MINIKAME_MIN_ANGLE;
  if (value > MINIKAME_MAX_ANGLE) value = MINIKAME_MAX_ANGLE;

  // 使用浮点计算，最后四舍五入，减少量化误差
  float pulse = MINIKAME_MIN_PULSE_US +
                (value / MINIKAME_MAX_ANGLE) *
                (MINIKAME_MAX_PULSE_US - MINIKAME_MIN_PULSE_US);

  return (int)(pulse + 0.5f);
}

// ------------------------------------------------------------
// 设置单个舵机（更严谨的 trim + reverse + 钳位）
// ------------------------------------------------------------
void MiniKame::setServo(int id, float target) {
  if (id < 0 || id >= 8) return;

  // 1. 先加 trim
  float angle = target + trim[id];

  // 2. 钳位到物理安全范围
  if (angle < MINIKAME_MIN_ANGLE) angle = MINIKAME_MIN_ANGLE;
  if (angle > MINIKAME_MAX_ANGLE) angle = MINIKAME_MAX_ANGLE;

  // 3. 处理反向
  float output_angle = reverse[id] ? (MINIKAME_MAX_ANGLE - angle) : angle;

  // 4. 写入脉宽（使用精确转换）
  servo[id].writeMicroseconds(angToUsec(output_angle));

  // 5. 记录的是「逻辑角度」（未反向前），方便上层步态计算
  _servo_position[id] = target;
}

// 获取指定舵机当前记录的角度
float MiniKame::getServo(int id) {
  if (id < 0 || id >= 8) return 0;
  return _servo_position[id];
}

// ------------------------------------------------------------
// 商家动作插值：每 10ms 增加一次固定角度增量
// ------------------------------------------------------------
void MiniKame::moveServos(int time, float target[8]) {
  if (time > 10) {
    // 计算每个舵机每 10ms 的增量
    for (int i = 0; i < 8; i++)
      _increment[i] = (target[i] - _servo_position[i]) / (time / 10.0);
    _final_time = millis() + time;

    // 按 10ms 节拍逐步更新舵机位置
    while (millis() < _final_time) {
      const unsigned long partial_time = millis() + 10;
      for (int i = 0; i < 8; i++)
        setServo(i, _servo_position[i] + _increment[i]);
      while (millis() < partial_time); // 等待下一个 10ms 节拍
    }
  } else {
    // 时间太短，直接设置到目标位置
    for (int i = 0; i < 8; i++) setServo(i, target[i]);
  }

  // 最终位置对齐
  for (int i = 0; i < 8; i++) _servo_position[i] = target[i];
}

// ------------------------------------------------------------
// 商家周期动作执行器：按原相位和时间基准刷新
// ------------------------------------------------------------
void MiniKame::execute(float steps, int period[8], int amplitude[8], int offset[8], int phase[8]) {
  // 配置每个舵机对应的振荡器参数
  for (int i = 0; i < 8; i++) {
    oscillator[i].setPeriod(period[i]);
    oscillator[i].setAmplitude(amplitude[i]);
    oscillator[i].setPhase(phase[i]);
    oscillator[i].setOffset(offset[i]);
  }

  unsigned long global_time = millis();
  // 统一所有振荡器的时间基准
  for (int i = 0; i < 8; i++) oscillator[i].setTime(global_time);

  // 计算总运行时间：以 0 号舵机的周期为基准乘以步数
  _final_time = millis() + period[0] * steps;

  // 循环刷新每个舵机的位置，直到时间结束
  while (millis() < _final_time) {
    for (int i = 0; i < 8; i++) {
      setServo(i, oscillator[i].refresh());
    }
    yield(); // 让出 CPU，允许后台任务运行（如 WiFi、串口等）
  }
}

// ------------------------------------------------------------
// 基础姿态
// ------------------------------------------------------------

// 回到初始站立姿态，略微调整使四腿对称站立
void MiniKame::home() {
  int ap = 2;  // 前后大腿微调
  int hi = 2;  // 小腿微调
  int position[] = {
    90 + ap, 90 - ap,   // 前左大腿、前右大腿
    90 - hi, 90 + hi,   // 前左小腿、前右小腿
    90 - ap, 90 + ap,   // 后左大腿、后右大腿
    90 + hi, 90 - hi    // 后左小腿、后右小腿
  };
  for (int i = 0; i < 8; i++) setServo(i, position[i]);
}

// ------------------------------------------------------------
// 步态动作
// ------------------------------------------------------------

// 一种交替步行步态，手动控制左右侧腿的抬起与摆动
// steps: 步数，t: 周期（ms）
void MiniKame::walk(float steps, int t) {
  int x_amp = 15;      // 大腿前后摆动幅度
  int z_amp = 20;      // 小腿抬腿幅度
  int ap = 20;         // 大腿偏移
  int hi = 10;         // 小腿偏移
  int front_x = 12;    // 前腿前后偏移
  int period[] = {t, t, t / 2, t / 2, t, t, t / 2, t / 2};
  int amplitude[] = {x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp};
  int offset[] = {
    90 + ap - front_x,
    90 - ap + front_x,
    90 - hi,
    90 + hi,
    90 - ap - front_x,
    90 + ap + front_x,
    90 + hi,
    90 - hi
  };
  int phase[] = {90, 90, 270, 90, 270, 270, 90, 270};

  // 重置并配置振荡器
  for (int i = 0; i < 8; i++) {
    oscillator[i].reset();
    oscillator[i].setPeriod(period[i]);
    oscillator[i].setAmplitude(amplitude[i]);
    oscillator[i].setPhase(phase[i]);
    oscillator[i].setOffset(offset[i]);
  }

  _final_time = millis() + period[0] * steps;
  const unsigned long init_time = millis();
  bool side;

  // 循环执行，根据当前时间决定哪一组小腿抬起
  while (millis() < _final_time) {
    side = (int)((millis() - init_time) / (period[0] / 2)) % 2;
    setServo(0, oscillator[0].refresh());
    setServo(1, oscillator[1].refresh());
    setServo(4, oscillator[4].refresh());
    setServo(5, oscillator[5].refresh());

    if (side == 0) {
      // 一侧腿组
      setServo(3, oscillator[3].refresh());
      setServo(6, oscillator[6].refresh());
    } else {
      // 另一侧腿组
      setServo(2, oscillator[2].refresh());
      setServo(7, oscillator[7].refresh());
    }
    delay(1);
  }
}

// 前进动作
// steps: 步数，t: 周期（ms）
void MiniKame::forward(float steps, int t) {
  int x_amp = 15;  // 大腿摆动幅度
  int z_amp = 15;  // 小腿抬腿幅度
  int ap = 15;     // 大腿偏移
  int hi = 15;     // 小腿偏移
  int period[8];
  for (int i = 0; i < 8; i++) {
    period[i] = t;
  }
  int amplitude[8] = {x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp};
  int offset[8] = {
    90 - ap,
    90 + ap,
    90 - hi,
    90 + hi,
    90 + ap,
    90 - ap,
    90 + hi,
    90 - hi
  };
  // 相位设置：前后腿交替，形成前进步态
  int phase[8] = {0, 0, 90, 90, 180, 180, 90, 90};
  execute(steps, period, amplitude, offset, phase);
}

// 后退动作
// steps: 步数，t: 周期（ms）
void MiniKame::backward(float steps, int t) {
  int x_amp = 15;
  int z_amp = 15;
  int ap = 15;
  int hi = 15;
  int period[8] = {t, t, t, t, t, t, t, t};
  int amplitude[8] = {x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp};
  int offset[8] = {
    90 - ap,
    90 + ap,
    90 - hi,
    90 + hi,
    90 + ap,
    90 - ap,
    90 + hi,
    90 - hi
  };
  // 相位与前进相反，实现后退
  int phase[8] = {180, 180, 90, 90, 0, 0, 90, 90};
  execute(steps, period, amplitude, offset, phase);
}

// 左转动作
// steps: 步数，t: 周期（ms）
void MiniKame::turnL(float steps, int t) {
  int x_amp = 15;
  int z_amp = 15;
  int ap = 10;
  int hi = 23;
  int period[] = {t, t, t, t, t, t, t, t};
  int amplitude[] = {x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp};
  int offset[] = {90 - ap, 90 + ap, 90 - hi, 90 + hi, 90 + ap, 90 - ap, 90 + hi, 90 - hi};
  // 通过左右腿相位差实现左转
  int phase[] = {180, 0, 90, 90, 0, 180, 90, 90};
  execute(steps, period, amplitude, offset, phase);
}

// 右转动作
// steps: 步数，t: 周期（ms）
void MiniKame::turnR(float steps, int t) {
  int x_amp = 15;
  int z_amp = 15;
  int ap = 10;
  int hi = 23;
  int period[] = {t, t, t, t, t, t, t, t};
  int amplitude[] = {x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp};
  int offset[] = {90 - ap, 90 + ap, 90 - hi, 90 + hi, 90 + ap, 90 - ap, 90 + hi, 90 - hi};
  // 与左转相位相反
  int phase[] = {0, 180, 90, 90, 180, 0, 90, 90};
  execute(steps, period, amplitude, offset, phase);
}

// ------------------------------------------------------------
// 趣味动作
// ------------------------------------------------------------

// 打招呼动作：抬起前腿并摆动
void MiniKame::hello() {
  int a = 50;
  int b = 30;
  int c = 20;
  int d = 70;

  // 状态1：准备姿势，前腿抬起
  float state1[8] = {
    90 - a, 90, 90 + c, 90 - c,
    90 + c, 90 - c, 90 - d, 90 + d
  };

  // 状态2：前腿向一侧摆动
  float state2[8] = {
    90 - a, 90 + b, 90 + c, 90 + d,
    90 + c, 90 - c, 90 - d, 90 + d
  };

  // 状态3：前腿向另一侧摆动
  float state3[8] = {
    90 - a, 90 - b, 90 + c, 90 + d,
    90 + c, 90 - c, 90 - d, 90 + d
  };

  // 状态4：恢复站立
  float state4[8] = {90, 90, 90, 90, 90, 90, 90, 90};

  moveServos(300, state1);

  // 来回摆动 3 次
  for (int i = 0; i < 3; i++) {
    moveServos(200, state2);
    moveServos(200, state3);
  }

  delay(300);
  moveServos(200, state4);
}

// 全向行走：可控制侧向和转向比例
// steps: 步数，t: 周期，side: 方向，turn_factor: 转向因子
void MiniKame::omniWalk(float steps, int t, bool side, float turn_factor) {
  int x_amp = 15;
  int z_amp = 15;
  int ap = 15;
  int hi = 23;
  int front_x = 6 * (1 - pow(turn_factor, 2)); // 根据转向因子调整前腿偏移
  int period[] = {t, t, t, t, t, t, t, t};
  int amplitude[] = {x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp};
  int offset[] = {
    90 + ap - front_x,
    90 - ap + front_x,
    90 - hi,
    90 + hi,
    90 - ap - front_x,
    90 + ap + front_x,
    90 + hi,
    90 - hi
  };

  int phase[8];
  if (side) {
    // 右侧全向行走相位
    int phase1[]  = {0, 0, 90, 90, 180, 180, 90, 90};
    int phase2R[] = {0, 180, 90, 90, 180, 0, 90, 90};
    for (int i = 0; i < 8; i++)
      phase[i] = phase1[i] * (1 - turn_factor) + phase2R[i] * turn_factor;
  } else {
    // 左侧全向行走相位
    int phase1[]  = {0, 0, 90, 90, 180, 180, 90, 90};
    int phase2L[] = {180, 0, 90, 90, 0, 180, 90, 90};
    for (int i = 0; i < 8; i++)
      phase[i] = phase1[i] * (1 - turn_factor) + phase2L[i] * turn_factor + oscillator[i].getPhaseProgress();
  }

  execute(steps, period, amplitude, offset, phase);
}

// 月球漫步动作
void MiniKame::moonwalkL(float steps, int t) {
  int z_amp = 30;
  int hi = 15;
  int ap = 10;
  int period[] = {t, t, t, t, t, t, t, t};
  int amplitude[] = {0, 0, z_amp, z_amp, 0, 0, z_amp, z_amp};
  int offset[] = {90 - ap, 90 + ap, 90 - hi, 90 + hi, 90 + ap, 90 - ap, 90 + hi, 90 - hi};
  int phase[] = {0, 0, 0, 80, 0, 0, 160, 290};
  execute(steps, period, amplitude, offset, phase);
}

// 跳舞动作
void MiniKame::dance(float steps, int t) {
  int x_amp = 0;
  int z_amp = 30;
  int ap = 0;
  int hi = 20;
  int period[] = {t, t, t, t, t, t, t, t};
  int amplitude[] = {x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp};
  int offset[] = {90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi};
  int phase[] = {0, 0, 0, 270, 0, 0, 90, 180};
  execute(steps, period, amplitude, offset, phase);
}

// 上下起伏动作
void MiniKame::upDown(float steps, int t) {
  int x_amp = 0;
  int z_amp = 35;
  int ap = 10;
  int hi = 15;
  int front_x = 0;
  int period[8] = {t, t, t, t, t, t, t, t};
  int amplitude[8] = {x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp};
  int offset[8] = {
    90 + ap - front_x,
    90 - ap + front_x,
    90 - hi,
    90 + hi,
    90 - ap - front_x,
    90 + ap + front_x,
    90 + hi,
    90 - hi
  };
  int phase[8] = {0, 0, 90, 270, 180, 180, 270, 90};
  execute(steps, period, amplitude, offset, phase);
}

// 俯卧撑动作
void MiniKame::pushUp(float steps, int t) {
  int z_amp = 40;
  int x_amp = 45;
  int hi = 0;
  int b = 35;
  int period[8];
  for (int i = 0; i < 8; i++) {
    period[i] = t;
  }
  int amplitude[8] = {0, 0, z_amp, z_amp, 0, 0, 0, 0};
  int offset[8] = {
    90,
    90,
    90 - hi,
    90 + hi,
    90 + x_amp,
    90 - x_amp,
    90 + b,
    90 - b
  };
  int phase[8] = {0, 0, 90, -90, 0, 0, 0, 0};
  execute(steps, period, amplitude, offset, phase);
}

// 前后运动动作
void MiniKame::frontBack(float steps, int t) {
  int x_amp = 30;
  int z_amp = 25;
  int ap = 20;
  int hi = 30;
  int period[] = {t, t, t, t, t, t, t, t};
  int amplitude[] = {x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp};
  int offset[] = {90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi};
  int phase[] = {0, 180, 270, 90, 0, 180, 90, 270};
  execute(steps, period, amplitude, offset, phase);
}

// 挥手动作
void MiniKame::waveHand(float steps, int t) {
  int period[8];
  for (int i = 0; i < 8; i++) {
    period[i] = t;
  }
  int amplitude[8] = {20, 0, 0, 30, 0, 0, 0, 0};
  int offset[8] = {
    90 - 50,  // 前左大腿抬起
    90,       // 前右大腿不动
    90 + 20,  // 前左小腿
    90 + 60,  // 前右小腿
    90, 90, 90, 90
  };
  int phase[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  execute(steps, period, amplitude, offset, phase);
}

// 隐藏/趴下动作
void MiniKame::hide(float steps, int t) {
  int a = 60;
  int b = 70;
  int period[8] = {t, t, t, t, t, t, t, t};
  int amplitude[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  int offset[8] = {
    90 - a,
    90 + a,
    90 + b,
    90 - b,
    90 + a,
    90 - a,
    90 - b,
    90 + b
  };
  int phase[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  execute(steps, period, amplitude, offset, phase);
}

// 受惊动作：坐下然后跳起
void MiniKame::scared() {
  int ap = 10;
  int hi = 40;

  // 坐下姿势
  float sentado[8] = {
    90 - 15, 90 + 15, 90 - hi, 90 + hi,
    90 - 20, 90 + 20, 90 + hi, 90 - hi
  };

  // 跳起姿势
  float salto[8] = {
    90 - ap, 90 + ap, 160, 20,
    90 + ap * 3, 90 - ap * 3, 20, 160
  };

  moveServos(600, sentado);
  moveServos(1000, salto);
  delay(1000);
}

// 奔跑动作
// steps: 步数，t: 周期（ms）
void MiniKame::run(float steps, int t) {
  int x_amp = 15;
  int z_amp = 15;
  int ap = 15;
  int hi = 15;
  int front_x = 6;
  int period[] = {t, t, t, t, t, t, t, t};
  int amplitude[] = {x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp};
  int offset[] = {
    90 + ap - front_x,
    90 - ap + front_x,
    90 - hi,
    90 + hi,
    90 - ap - front_x,
    90 + ap + front_x,
    90 + hi,
    90 - hi
  };
  int phase[] = {0, 0, 90, 90, 180, 180, 90, 90};
  execute(steps, period, amplitude, offset, phase);
}
