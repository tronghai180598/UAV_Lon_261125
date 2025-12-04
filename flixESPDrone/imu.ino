// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Work with the IMU sensor

#include "lpf.h"
#include "util.h"
#include <math.h>

//#include "vector.h"                                                                                                                                                                                                                                                                                                                                                                │
#include <MPU6050.h>
#include <BMP280.h>
BMP280 bmp280;
MPU6050 imu(Wire);

Vector accBias(0.0, 0.0, 0.0);
Vector accScale(1.0, 1.0, 1.0);
Vector gyroBias;
float AccZInertial;

extern float dt;
const float SEA_LEVEL_PRESSURE_HPA = 1013.25f;
float baroAltAbs_m    = 0.0f;   // so với mực nước biển chuẩn (m) – chỉ để debug
float baroAltOffset_m = 0.0f;   // mốc "0" (bàn) (m)
float baroAltRel_m    = 0.0f;   // độ cao tương đối (m)
float baroAltRel_cm   = 0.0f;   // độ cao tương đối (cm)
// Offset sẽ được tự động tính trong những lần đọc đầu tiên
bool  baroOffsetReady = false;
int   baroOffsetCount = 0;
float baroOffsetSum   = 0.0f;

// ================= KALMAN Z,VZ ===================
// Trạng thái ước lượng
float z_kf_cm      = 0.0f;  // độ cao Kalman (cm)
float vz_kf_cm_s   = 0.0f;  // vận tốc Z Kalman (cm/s)

// Ma trận hiệp phương sai P (2x2)
float P00 = 0.0f, P01 = 0.0f;
float P10 = 0.0f, P11 = 0.0f;
// Nhiễu quá trình Q (2x2, giả sử đường chéo)
float Qpos = 0.01f;    // nhiễu trên z (cm^2)
float Qvel = 0.01f;   // nhiễu trên v (cm^2/s^2)
float Rbaro = 0.5f; // ~ (50 cm)^2, chỉnh sau cho hợp

float roll_H, pitch_H;
const float ACC_LSB_PER_G = 8192.0f;

static inline float approx_atan2_quadrant(float y, float z) {
	const float kEPS = 1e-6f;
	float ay = fabsf(y);
	float az = fabsf(z);
	if (ay < kEPS && az < kEPS) { return 0.0f; }

	float beta;
	if (ay <= az)  beta = M_PI_4 * (ay / az);
	else  beta = M_PI_4 * (2.0f - az / ay);

	float th;
	if (z >= 0.0f) th = (y >= 0.0f) ? beta : -beta;
	else th = (y >= 0.0f) ? (M_PI - beta) : (-(M_PI) + beta);
	return th;
}
void update_attitude_from_acc(Vector &acc) {
	roll_H = approx_atan2_quadrant(acc.y, acc.z)*1000.0;
	pitch_H = approx_atan2_quadrant(-acc.x, acc.z)*1000.0;
	
	float rol = roll_H  * 0.001f; // roll_H mrad -> rad
	float pit = pitch_H * 0.001f; // pitch_H mrad -> rad

	float ax = acc.x/ACC_LSB_PER_G;
	float ay = acc.y/ACC_LSB_PER_G;
	float az = acc.z/ACC_LSB_PER_G;
	AccZInertial = ((az + rol * ay - pit * ax) - 1.0f) * ONE_G;
}
static float pressureToAltitude(uint32_t pressurePa) {
    double pressure_hPa = (double)pressurePa / 100.0; // Pa -> hPa
    double z = 44330.0 * (1.0 - pow(pressure_hPa / SEA_LEVEL_PRESSURE_HPA,
                                    1.0 / 5.255));
    return (float)z; // m
}

void initKalman() {
    // trạng thái ban đầu
    z_kf_cm    = 0.0f;
    vz_kf_cm_s = 0.0f;

    // P lớn: chưa biết gì
    P00 = 10.0f;
    P01 = 0.0f;
    P10 = 0.0f;
    P11 = 10.0f;
}
void kalmanUpdate() {
    // dt theo giây, AccZInertial cm/s^2, baroAltRel_cm cm
    float dt_local = dt;
    if (dt_local <= 0.0f) return;

    float a = AccZInertial;    // cm/s^2
    float z_meas = baroAltRel_cm; // cm

    // ---------- PREDICT ----------
    // x = [z; v]
    // z_pred = z + v*dt + 0.5*a*dt^2
    // v_pred = v + a*dt
    float dt2 = dt_local * dt_local;

    float z_pred = z_kf_cm + vz_kf_cm_s * dt_local + 0.5f * a * dt2;
    float v_pred = vz_kf_cm_s + a * dt_local;

    // P_pred = F P F' + Q với
    // F = [1 dt; 0 1]
    float Pp00 = P00 + dt_local*(P10 + P01) + dt2*P11 + Qpos;
    float Pp01 = P01 + dt_local*P11;
    float Pp10 = P10 + dt_local*P11;
    float Pp11 = P11 + Qvel;

    // ---------- UPDATE ----------
    // H = [1 0]; R = Rbaro
    // S = H P H' + R = Pp00 + R
    float S = Pp00 + Rbaro;
    if (S < 1e-6f) S = 1e-6f; // tránh chia 0

    // K = P H' / S => K = [Kz; Kv]
    float Kz = Pp00 / S;
    float Kv = Pp10 / S;

    // innovation
    float y = z_meas - z_pred;

    // x_new = x_pred + K y
    z_kf_cm    = z_pred + Kz * y;
    vz_kf_cm_s = v_pred + Kv * y;

    // P_new = (I - K H) P_pred
    // I - K H = [[1-Kz, 0], [-Kv, 1]]
    float P00_new = (1.0f - Kz) * Pp00;
    float P01_new = (1.0f - Kz) * Pp01;
    float P10_new = Pp10 - Kv * Pp00;
    float P11_new = Pp11 - Kv * Pp01;

    // ép lại đối xứng (numerical)
    float P01_sym = 0.5f * (P01_new + P10_new);
    P00 = P00_new;
    P01 = P01_sym;
    P10 = P01_sym;
    P11 = P11_new;
}

void setupIMU() {
	print("Setup IMU\n");
	Wire.begin(); 
	imu.begin();
  bmp280.begin();
	configureIMU();
	calibrateGyroOnce();
	initKalman();
}

void configureIMU() {
	imu.setAccelRange(imu.ACCEL_RANGE_4G);
	imu.setGyroRange(imu.GYRO_RANGE_2000DPS);
	imu.setDLPF(imu.DLPF_MAX);
	imu.setRate(imu.RATE_1KHZ_APPROX);
}

float kGr = 1.0f;
#define to_mRad (1.065264436f)
void readIMU() {
	if(!imu.getIntDataReadyStatus()) return;
	int16_t ax, ay, az, gx, gy, gz;
	imu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

    // --- BARO ---
    uint32_t pressure = bmp280.getPressure();      // Pa
    float baroAltAbs_m    = pressureToAltitude(pressure); // m
    // Tự động "zero" độ cao trong những lần đọc đầu tiên
    if (!baroOffsetReady) {
        baroOffsetSum   += baroAltAbs_m;
        baroOffsetCount++;

        if (baroOffsetCount >= 100) { // lấy trung bình 100 mẫu đầu
            baroAltOffset_m = baroOffsetSum / (float)baroOffsetCount;
            baroOffsetReady = true;
            print("Baro offset set to %.2f m\n", baroAltOffset_m);
        }

        // Trong thời gian offset chưa sẵn sàng, coi như độ cao tương đối = 0
        baroAltRel_m  = 0.0f;
        baroAltRel_cm = 0.0f;
    } else {
        baroAltRel_m  = baroAltAbs_m - baroAltOffset_m;
        baroAltRel_cm = baroAltRel_m;
    }

	Vector lacc = Vector(ax,ay,az);
	lacc = (lacc - accBias) / accScale;
	rotateIMU(lacc);
	acc += (lacc - acc)*0.02;
	gyro = Vector(gx, gy, gz) - gyroBias; 
	gyro = gyro * to_mRad * kGr;
	rotateIMU(gyro);
	update_attitude_from_acc(acc); 
	kalmanUpdate();
}

void rotateIMU(Vector& data) {
	// Rotate from LFD to FLU
	// NOTE: In case of using other IMU orientation, change this line:
	data = Vector(data.x, data.y, data.z);
	// Axes orientation for various boards: https://github.com/okalachev/flixperiph#imu-axes-orientation
}

void calibrateAccel(){}
void calibrateGyroOnce() {
	Vector rawGyro; int i = 100;
	while(!imu.getIntDataReadyStatus()) {
		i--; if(i==0) return;
		delay(1);
	}

	int16_t ax, ay, az, gx, gy, gz;
	imu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
	gyroBias = Vector(gx, gy, gz);
	for(i=0; i<100; i++) { // 100 samples
		if(imu.getIntDataReadyStatus()) {
			imu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
			rawGyro = Vector(gx, gy, gz);
			gyroBias += (rawGyro - gyroBias) / 50.0;
		}
		delay(1);
	}
	for(int i=0; i<1000; i++) { // 1000 samples
		if(imu.getIntDataReadyStatus()) {
			imu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
			rawGyro = Vector(gx, gy, gz);
			gyroBias += (rawGyro - gyroBias) / 500.0;
		}
		delay(1);
	}
}
void printIMUCalibration() {
	print("gyro bias: %f %f %f\n", gyroBias.x, gyroBias.y, gyroBias.z);
	print("accel bias: %f %f %f\n", accBias.x, accBias.y, accBias.z);
	print("accel scale: %f %f %f\n", accScale.x, accScale.y, accScale.z);
}

void printIMUInfo() {
	imu.status() ? print("status: ERROR %d\n", imu.status()) : print("status: OK\n");
	print("model: %s\n", imu.getModel());
	print("who am I: 0x%02X\n", imu.whoAmI());
	print("rate: %.0f\n", loopRate);
	print("gyro: %f %f %f\n", gyro.x, gyro.y, gyro.z);
	print("acc: %f %f %f\n", acc.x, acc.y, acc.z);
	print("Altitude (REL) / VelZ: %.2f %.2f \n", z_kf_cm, vz_kf_cm_s);
	print("roll pitch: %f %f\n", roll_H, pitch_H);
	imu.waitForData();
	Vector rawGyro, rawAcc;
	imu.getGyro(rawGyro.x, rawGyro.y, rawGyro.z);
	imu.getAccel(rawAcc.x, rawAcc.y, rawAcc.z);
	print("raw gyro: %f %f %f\n", rawGyro.x, rawGyro.y, rawGyro.z);
	print("raw acc: %f %f %f\n", rawAcc.x, rawAcc.y, rawAcc.z);
}
