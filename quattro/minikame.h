#ifndef minikame_h
#define minikame_h
#include <ESP32Servo.h>
#include "Octosnake.h"

// 保留原 GaitParams 接口以兼容已有代码。
// 当前已切换到商家动作中的固定参数，修改此结构不再影响动作。
struct GaitParams{
    int x_amp;    // 大腿前后摆动幅度（决定步长）
    int z_amp;    // 小腿抬腿幅度（决定抬腿高度）
    int ap;       // 大腿中心偏移
    int hi;       // 小腿中心偏移
    int front_x;  // 前腿相对后腿的前后偏移（run / omniWalk 使用）
    int turn_ap;  // 转向步态（turnL / turnR）的大腿中心偏移
    int turn_hi;  // 转向类步态（turnL / turnR / omniWalk）的小腿中心偏移
};

class MiniKame{
public:
    // 兼容保留，商家动作不读取此参数
    GaitParams gait = {15, 15, 15, 15, 6, 10, 23};

    void init();
    void setServo(int id, float target);
    void reverseServo(int id);
    float getServo(int id);
    void moveServos(int time, float target[8]);

    // 定义动作
    // 1. 方向
    void forward(float steps, int period);
    void turnL(float steps, int period);
    void home();
    void turnR(float steps, int period);
    void backward(float steps, int period);

    // 2. 手势
    void hello();
    void omniWalk(float steps, int T, bool side, float turn_factor);
    void moonwalkL(float steps, int period);
    void dance(float steps, int period);
    void upDown(float steps, int period);
    void pushUp(float steps, int period);
    void frontBack(float steps, int period);
    void waveHand(float steps, int period);
    void hide(float steps, int t);
    void scared();

    void run(float steps, int t);
    void walk(float steps, int t);

private:
    Oscillator oscillator[8];
    Servo servo[8];
    int board_pins[8];
    int trim[8];
    bool reverse[8];
    unsigned long _final_time;
    float _increment[8];
    float _servo_position[8];

    int angToUsec(float value);
    void execute(float steps, int period[8], int amplitude[8], int offset[8], int phase[8]);
};

#endif
