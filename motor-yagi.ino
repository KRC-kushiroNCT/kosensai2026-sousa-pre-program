#include <Arduino.h>
#include "Cytron_MDDS30_Serial.h"
#include <Bluepad32.h>

Cytron_MDDS30_Serial motors; // インスタンスは一つだけ

// Xboxコントローラー
ControllerPtr myController = nullptr;

// モーター出力上限（%）
const int MAX_POWER       = 25;  // 通常
const int MAX_POWER_SLOW  = 12;  // LB 押下中（精密操作）
const int MAX_POWER_BOOST = 50;  // RB 押下中（高速）

// スティックのデッドゾーン（Bluepad32 の軸は -511 ～ 512）
const int DEADZONE = 50;

// 旋回の強さ（式の L に相当。移動と旋回の効き具合のバランス）
const float L_ROT = 1.0f;

// 1ループあたりの出力変化量の上限（%）。急発進・急停止を抑える
const float RAMP_STEP = 3.0f;

// 制御周期（ms）
const int LOOP_MS = 20;

// --------------------------------------------------
// ホイール定義
//   既存の前進・横移動・旋回パターンから求めた取り付け角度
// --------------------------------------------------
struct Wheel {
  int   driverId; // MDDS30 の ID
  int   channel;  // 0 = 左ch, 1 = 右ch
  float thetaDeg; // θn
};

const int NUM_WHEELS = 4;
const Wheel WHEELS[NUM_WHEELS] = {
  {0, 0, 135.0f},
  {0, 1,  45.0f},
  {1, 0, -45.0f},
  {1, 1, 225.0f},
};

float wheelSin[NUM_WHEELS];
float wheelCos[NUM_WHEELS];

// 現在の出力（ランプ処理用）
float currentPower[NUM_WHEELS] = {0, 0, 0, 0};

// ==================================================
// モーター出力
// ==================================================
void applyMotors() {
  int out[2][2] = {{0, 0}, {0, 0}};
  for (int i = 0; i < NUM_WHEELS; i++) {
    out[WHEELS[i].driverId][WHEELS[i].channel] = (int)lroundf(currentPower[i]);
  }
  motors.control(0, out[0][0], out[0][1]);
  motors.control(1, out[1][0], out[1][1]);
}

void stopMotors() {
  for (int i = 0; i < NUM_WHEELS; i++) {
    currentPower[i] = 0;
  }
  applyMotors();
}

// ==================================================
// コントローラー接続 / 切断
// ==================================================
void onConnectedController(ControllerPtr ctl) {
  if (myController == nullptr && ctl->isGamepad()) {
    myController = ctl;

    Serial.println("[INFO] Controller connected!");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (myController == ctl) {
    myController = nullptr;

    stopMotors();

    Serial.println("[INFO] Controller disconnected!");
    Serial.println("[INFO] All motors stopped.");
  }
}

// ==================================================
// スティック値を -1.0 ～ 1.0 に変換（1軸）
// ==================================================
float readStick(int value) {
  if (abs(value) < DEADZONE) {
    return 0.0f;
  }

  // デッドゾーンの外側を 0～1 に引き伸ばして、動き出しを滑らかにする
  float v = (abs(value) - DEADZONE) / (512.0f - DEADZONE);
  v = constrain(v, 0.0f, 1.0f);
  return value < 0 ? -v : v;
}

// ==================================================
// 左スティックを円形デッドゾーンで読む（斜め入力用）
// ==================================================
void readStick2D(int rawX, int rawY, float &x, float &y) {
  float fx = rawX / 512.0f;
  float fy = rawY / 512.0f;
  float mag = sqrtf(fx * fx + fy * fy);
  float dz = DEADZONE / 512.0f;

  if (mag < dz) {
    x = 0;
    y = 0;
    return;
  }

  // デッドゾーン外を 0～1 に再マップ（方向は保持）
  float scaled = min((mag - dz) / (1.0f - dz), 1.0f);
  x = fx / mag * scaled;
  y = fy / mag * scaled;
}

// ==================================================
// 逆運動学: (vx, vy, ω) -> 各ホイール出力 (-1.0 ～ 1.0)
// ==================================================
void computeWheelSpeeds(float vx, float vy, float omega, float target[NUM_WHEELS]) {
  float maxAbs = 0.0f;

  for (int i = 0; i < NUM_WHEELS; i++) {
    // 斜め45°配置だと sin/cos が 0.707 になり、真っ直ぐ進むとき
    // 最大出力が出ないので √2 倍して補正する
    target[i] = (vx * wheelSin[i] + vy * wheelCos[i]) * M_SQRT2
              + L_ROT * omega;
    maxAbs = max(maxAbs, fabsf(target[i]));
  }

  // どれかが 1.0 を超えたら、比率を保ったまま全体を縮める
  // （比率を崩すと進行方向がずれるため）
  if (maxAbs > 1.0f) {
    for (int i = 0; i < NUM_WHEELS; i++) {
      target[i] /= maxAbs;
    }
  }
}

// ==================================================
// 出力を目標値へ少しずつ近づける
// ==================================================
float rampTo(float current, float target) {
  float diff = target - current;
  if (diff >  RAMP_STEP) return current + RAMP_STEP;
  if (diff < -RAMP_STEP) return current - RAMP_STEP;
  return target;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("ESP32 Omniwheel Controller");

  for (int i = 0; i < NUM_WHEELS; i++) {
    float rad = WHEELS[i].thetaDeg * DEG_TO_RAD;
    wheelSin[i] = sinf(rad);
    wheelCos[i] = cosf(rad);
  }

  // ID 0 として1台目を登録
  motors.begin(0, &Serial2, 9600, 5, 23);

  // ID 1 として2台目を登録
  motors.begin(1, &Serial1, 9600, 14, 21);

  stopMotors();

  BP32.setup(
    &onConnectedController,
    &onDisconnectedController
  );

  BP32.enableNewBluetoothConnections(true);

  Serial.println("[INFO] Initialization complete.");
  Serial.println("[INFO] Waiting for Xbox controller...");
}

void loop() {
  // Bluetoothの状態を更新
  BP32.update();

  // 未接続ならモーターを停止
  if (myController == nullptr ||
      !myController->isConnected()) {
    stopMotors();
    delay(LOOP_MS);
    return;
  }

  // ---- 入力 ----
  float vx, vy;
  readStick2D(myController->axisX(), myController->axisY(), vx, vy);
  vy = -vy; // スティックは上が負なので反転して「前が正」にする

  // 右スティック右 = 右旋回（時計回り）。式は反時計回りが正なので反転
  float omega = -readStick(myController->axisRX());

  // 速度モード（LB: 低速 / RB: 高速）
  int maxPower = MAX_POWER;
  if (myController->l1()) {
    maxPower = MAX_POWER_SLOW;
  } else if (myController->r1()) {
    maxPower = MAX_POWER_BOOST;
  }

  // ---- 計算 ----
  float target[NUM_WHEELS];
  computeWheelSpeeds(vx, vy, omega, target);

  for (int i = 0; i < NUM_WHEELS; i++) {
    currentPower[i] = rampTo(currentPower[i], target[i] * maxPower);
  }

  applyMotors();

  Serial.printf(
    "vx=%.2f vy=%.2f w=%.2f | %.1f %.1f %.1f %.1f\n",
    vx, vy, omega,
    currentPower[0], currentPower[1], currentPower[2], currentPower[3]
  );

  delay(LOOP_MS);
}
