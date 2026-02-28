# ビルド手順

このプロジェクトはSTM32 HALと共に使用するC/C++混合プロジェクトです。

## 📋 ファイル一覧

| ファイル | 言語 | 説明 |
|---------|------|------|
| `attitude_ekf.h` | C | C インターフェース |
| `attitude_ekf.cpp` | C++ | EKF実装（C++） |
| `attitude_ekf_example.cpp` | C++ | 使用例 |
| `konfig.h` | C/C++ | 設定 |
| `matrix.h` | C++ | 行列ライブラリ |
| `matrix.cpp` | C++ | 行列実装 |
| `ekf.h` | C++ | EKFインターフェース |
| `ekf.cpp` | C++ | EKF実装 |

## 🔨 STM32CubeIDE でのビルド

### 1. ファイルをプロジェクトに追加

STM32CubeIDEで以下のファイルをプロジェクトに追加します：

```
ekf_stm32_attitude/
├── attitude_ekf.h
├── attitude_ekf.cpp         ← C++としてコンパイル
├── konfig.h
├── matrix.h
├── matrix.cpp               ← C++としてコンパイル
├── ekf.h
└── ekf.cpp                  ← C++としてコンパイル
```

### 2. インクルードパス設定

**プロジェクト設定 → C/C++ General → Paths and Symbols**

Include paths に `ekf_stm32_attitude` ディレクトリを追加：

```
${PROJECT_DIR}/ekf_stm32_attitude
```

### 3. コンパイラ設定

**プロジェクト設定 → C/C++ Build → Settings**

- **MCU G++ Compiler → Includes**
  - `ekf_stm32_attitude` フォルダを追加

- **MCU G++ Compiler → Miscellaneous**
  - C++ standards: `-std=c++11` または `-std=c++14`

### 4. リンカ設定

通常は不要（すべてソースコンパイル）

## 📝 main.c からの使用例

```c
#include "attitude_ekf.h"

// グローバル変数
AttitudeEKF_t attitude_ekf;

int main(void) {
    // HAL初期化
    HAL_Init();
    // ... その他初期化 ...

    // EKF初期化
    if (!AttitudeEKF_Init(&attitude_ekf, SS_DT)) {
        Error_Handler();
    }

    // タイマ割り込みを開始
    HAL_TIM_Base_Start_IT(&htim3);  // 50 Hz用タイマ

    while (1) {
        // メインループ
    }
}

// タイマ割り込みコールバック（50 Hzで実行）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        // センサデータを読み取る
        float ax, ay, az, gx, gy, gz;
        ReadAccelerometer(&ax, &ay, &az);
        ReadGyroscope(&gx, &gy, &gz);

        // EKF更新
        float accel[3] = {ax, ay, az};
        float gyro[3] = {gx, gy, gz};
        AttitudeEKF_Update(&attitude_ekf, accel, gyro);

        // 推定角度を取得
        float roll = AttitudeEKF_GetRoll(&attitude_ekf);
        float pitch = AttitudeEKF_GetPitch(&attitude_ekf);
        float yaw = AttitudeEKF_GetYaw(&attitude_ekf);

        // 制御ロジック
        ControlLoop(roll, pitch, yaw);
    }
}
```

## 🔧 CMake を使う場合

### CMakeLists.txt の例

```cmake
cmake_minimum_required(VERSION 3.12)
project(stm32_attitude_ekf)

# C++ 標準
set(CMAKE_CXX_STANDARD 11)

# ソースファイル
set(SOURCES
    ekf_stm32_attitude/attitude_ekf.cpp
    ekf_stm32_attitude/matrix.cpp
    ekf_stm32_attitude/ekf.cpp
    src/main.c
    # ... その他のSTM32ファイル ...
)

# インクルードディレクトリ
set(INCLUDES
    ekf_stm32_attitude
    Inc
    # ... その他のインクルード ...
)

# ライブラリ作成
add_library(attitude_ekf ${SOURCES})
target_include_directories(attitude_ekf PUBLIC ${INCLUDES})

# 実行ファイル
add_executable(firmware ${SOURCES})
target_include_directories(firmware PRIVATE ${INCLUDES})
```

### ビルドコマンド

```bash
mkdir build
cd build
cmake ..
make
```

## ⚙️ Makefile での場合

```makefile
# コンパイラ
CXX = arm-none-eabi-g++
CC = arm-none-eabi-gcc
CXXFLAGS = -std=c++11 -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS = -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16

# ソースファイル
SOURCES = ekf_stm32_attitude/attitude_ekf.cpp \
          ekf_stm32_attitude/matrix.cpp \
          ekf_stm32_attitude/ekf.cpp

OBJECTS = $(SOURCES:.cpp=.o)

# オブジェクトファイルビルド
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -I ekf_stm32_attitude -c $< -o $@

# すべてビルド
all: $(OBJECTS)
	@echo "Object files created"
```

## 🧪 テスト用のスタンドアロンビルド

PC上でテストしたい場合（オプション）：

```bash
g++ -std=c++11 -I ekf_stm32_attitude \
    ekf_stm32_attitude/attitude_ekf.cpp \
    ekf_stm32_attitude/matrix.cpp \
    ekf_stm32_attitude/ekf.cpp \
    ekf_stm32_attitude/attitude_ekf_example.cpp \
    -o attitude_test -lm
```

## 📊 コンパイルエラーの対応

### エラー: "Matrix: No such file or directory"
→ インクルードパスに `ekf_stm32_attitude` を追加してください

### エラー: "undefined reference to `_SPEW_THE_ERROR'"
→ `attitude_ekf.cpp` がC++としてコンパイルされているか確認してください

### エラー: "conflicting declaration 'float_prec'"
→ `konfig.h` が複数回インクルードされていないか確認してください。`#ifndef KONFIG_H` ガード確認

## 💾 メモリ最適化

マトリックスサイズを削減する場合（メモリ制約がある場合）：

`konfig.h` で：
```c
#define MATRIX_MAXIMUM_SIZE     (3)  /* 必要に応じて削減可能 */
```

浮動小数点精度を削減（メモリ重視）：
```c
#define FPU_PRECISION       (PRECISION_SINGLE)  /* float (4 bytes) */
/* または */
#define FPU_PRECISION       (PRECISION_DOUBLE)  /* double (8 bytes) */
```

## 🚀 最小限の統合例

最小限のコードで統合する場合：

```c
// main.c
#include "attitude_ekf.h"

static AttitudeEKF_t ekf;

void init_attitude_filter(void) {
    AttitudeEKF_Init(&ekf, SS_DT);
}

void update_attitude(float ax, float ay, float az,
                     float gx, float gy, float gz) {
    float accel[3] = {ax, ay, az};
    float gyro[3] = {gx, gy, gz};
    AttitudeEKF_Update(&ekf, accel, gyro);
}

float get_roll(void) {
    return AttitudeEKF_GetRoll(&ekf);
}
```

## ✅ ビルド確認チェックリスト

- [ ] `attitude_ekf.cpp` が C++ コンパイラでビルドされている
- [ ] インクルードパスに `ekf_stm32_attitude` を追加
- [ ] `konfig.h` の `SYSTEM_IMPLEMENTATION` が `SYSTEM_IMPLEMENTATION_EMBEDDED_NO_PRINT` に設定
- [ ] `FPU_PRECISION` が適切に設定されている（通常は `PRECISION_SINGLE`）
- [ ] リンク時に `SPEW_THE_ERROR()` 関数が定義されている
