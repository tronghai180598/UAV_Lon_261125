// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Work with the IMU sensor

#include "lpf.h"
#include "util.h"
//#include "vector.h"                                                                                                                                                                                                                                                                                                                                                                │
#include <MPU6050.h>
MPU6050 imu(Wire);

Vector accBias(0.0, 0.0, 0.0);
Vector accScale(1.0, 1.0, 1.0);
Vector gyroBias;

float roll_H, pitch_H;
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
	roll_H = approx_atan2_quadrant(acc.y, acc.z)*1000;
	pitch_H = approx_atan2_quadrant(-acc.x, acc.z)*1000;
}

void setupIMU() {
	print("Setup IMU\n");
	imu.begin();
	configureIMU();
	calibrateGyroOnce();
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
	Vector lacc = Vector(ax,ay,az);
	lacc = (lacc - accBias) / accScale;
	rotateIMU(lacc);
	acc += (lacc - acc)*0.02;
	gyro = Vector(gx, gy, gz) - gyroBias; 
	gyro = gyro * to_mRad * kGr;
	rotateIMU(gyro);
	update_attitude_from_acc(acc); 
}

void rotateIMU(Vector& data) {
	// Rotate from LFD to FLU
	// NOTE: In case of using other IMU orientation, change this line:
	data = Vector(data.x, data.y, data.z);
	// Axes orientation for various boards: https://github.com/okalachev/flixperiph#imu-axes-orientation
}

void calibrateGyroOnce() {
	Vector rawGyro; int i = 100;
	while(!imu.getIntDataReadyStatus()) {
		i--; if(i==0) return;
		delay(1);
	}

	int16_t ax, ay, az, gx, gy, gz;
	imu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
	gyroBias = Vector(gx, gy, gz);
	for(i=0; i<100; i++) { // 1000 samples
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

void calibrateAccel() {
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
	print("torg: %f %f %f\n", torqueTarget.x, torqueTarget.y, torqueTarget.z);
	print("roll pitch: %f %f\n", roll_H, pitch_H);
	imu.waitForData();
	Vector rawGyro, rawAcc;
	imu.getGyro(rawGyro.x, rawGyro.y, rawGyro.z);
	imu.getAccel(rawAcc.x, rawAcc.y, rawAcc.z);
	print("raw gyro: %f %f %f\n", rawGyro.x, rawGyro.y, rawGyro.z);
	print("raw acc: %f %f %f\n", rawAcc.x, rawAcc.y, rawAcc.z);
}
