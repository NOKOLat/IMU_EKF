# 6DoF_EKF

外部の方が作成した6軸imuを使ったEKFです
姿勢角を計算することができます

6軸imuを使用しているので、yaw角度の推定はずれることがあります。使用する場合は角速度にして使用するとよさそうです

## セットアップ

### 設定ファイルの準備

`konfig.h` は環境固有の設定ファイルです。リポジトリをクローンした後、必ず以下の手順で設定してください：

```bash
cp konfig.h.example konfig.h
```

その後、`konfig.h` を環境に合わせて編集してください。

**Git submodule として使う場合：**

このファイルは `.gitignore` に設定されているため、各環境での変更が git に追跡されません。複数の環境で使い分ける際に便利です。

## クレジット

制作者さんや元ライブラリのリンクを貼ってあります
フォローやスターなどをつけてあげてね！

- [制作者さんのプロフィール](https://github.com/pronenewbits)

- [元のライブラリ](https://github.com/pronenewbits/Embedded_EKF_Library)

## サンプルコード

stm32環境を想定したサンプルコードです。
センサーのデータ取得などの実装は書いていないので、環境に合わせたコードを追加してください

```cpp
#include "wrapper.hpp"
#include "stdio.h"
#include "math.h"
#include <cstring>

// EKF library
#include "ekf_stm32_attitude/attitude_ekf.h"

// EKF instance
AttitudeEKF_t* attitude_ekf = nullptr;

// センサーデータ
float accel_data[3] = {0, 0, 0};  // m/s^2
float gyro_data[3] = {0, 0, 0};   // deg/s

// タイミング制御
static uint32_t last_update_tick = 0;
static const uint32_t UPDATE_PERIOD_MS = 10;  // 10ms (100Hz)

void init(){

	printf("=== System Start ===\n");

	// IMUの初期化と設定をする
    // 使うセンサーのライブラリに合わせてください

	// ----- EKF初期化 -----
	printf("[EKF] Initializing...\n");

	attitude_ekf = new AttitudeEKF_t();

	if (!AttitudeEKF_Init(attitude_ekf, SS_DT)) {
        
		printf("[EKF] ERROR: Initialization failed\n");
		while(1){

            HAL_Delay(1000);
        }
	} 
    else {

		printf("[EKF] Initialized\n");
	}

	printf("=== Initialization Complete ===\n\n");

	last_update_tick = HAL_GetTick();
}

void loop(){

	// 10ms周期で実行
	uint32_t current_tick = HAL_GetTick();
	uint32_t elapsed = current_tick - last_update_tick;

	if(elapsed < UPDATE_PERIOD_MS){

		return;
	}

	last_update_tick += UPDATE_PERIOD_MS;
	if(current_tick - last_update_tick > UPDATE_PERIOD_MS){

		last_update_tick = current_tick;
	}

	// ----- センサーデータ取得 -----
    // 使うセンサーのライブラリに合わせてください
	// float accel_data[3] 加速度[m/s^2] 
    // float gyro_data[3] 角速度[dgree/s]

	// ----- EKF更新 -----
	if(attitude_ekf != nullptr){

		// ジャイロをdeg/s -> rad/sに変換
		float gyro_rad[3];
		gyro_rad[0] = gyro_data[0] * M_PI / 180.0f;
		gyro_rad[1] = gyro_data[1] * M_PI / 180.0f;
		gyro_rad[2] = gyro_data[2] * M_PI / 180.0f;

		// EKF更新
		if(AttitudeEKF_Update(attitude_ekf, accel_data, gyro_rad)){

			// オイラー角取得
			float roll = AttitudeEKF_GetRoll(attitude_ekf);
			float pitch = AttitudeEKF_GetPitch(attitude_ekf);
			float yaw = AttitudeEKF_GetYaw(attitude_ekf);

			// 結果表示（rad -> deg変換）
			float roll_deg = roll * 180.0f / M_PI;
			float pitch_deg = pitch * 180.0f / M_PI;
			float yaw_deg = yaw * 180.0f / M_PI;

			printf("Roll: %+7.2f  Pitch: %+7.2f  Yaw: %+7.2f [deg]\n", roll_deg, pitch_deg, yaw_deg);
		}
	}
}
```

