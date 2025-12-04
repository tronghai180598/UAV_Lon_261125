// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Flight control

#include "vector.h"
#include "quaternion.h"
#include "pid.h"
#include "lpf.h"
#include "util.h"
#include "KrenCtrl.hpp"

#define PITCHRATE_P 0.05
#define PITCHRATE_I 0.2
#define PITCHRATE_D 0.001
#define PITCHRATE_I_LIM 0.3
#define ROLLRATE_P PITCHRATE_P
#define ROLLRATE_I PITCHRATE_I
#define ROLLRATE_D PITCHRATE_D
#define ROLLRATE_I_LIM PITCHRATE_I_LIM
#define YAWRATE_P 0.3
#define YAWRATE_I 0.0
#define YAWRATE_D 0.0
#define YAWRATE_I_LIM 0.3
#define ROLL_P 6
#define ROLL_I 0
#define ROLL_D 0
#define PITCH_P ROLL_P
#define PITCH_I ROLL_I
#define PITCH_D ROLL_D
#define YAW_P 3

#define YAWRATE_MAX radians(10)
#define TILT_MAX radians(30)
#define RATES_D_LPF_ALPHA 0.2 // cutoff frequency ~ 40 Hz

enum NoFlg{
	NoRoll = 1, NoPitch, NoYaw,
};
const int MANUAL = 0, ACRO = 1, STAB = 2, AUTO = 3; // flight modes
int mode = STAB;
bool armed = false;

PID rollRatePID(ROLLRATE_P, ROLLRATE_I, ROLLRATE_D, ROLLRATE_I_LIM, RATES_D_LPF_ALPHA);
PID pitchRatePID(PITCHRATE_P, PITCHRATE_I, PITCHRATE_D, PITCHRATE_I_LIM, RATES_D_LPF_ALPHA);
PID yawRatePID(YAWRATE_P, YAWRATE_I, YAWRATE_D);
PID rollPID(ROLL_P, ROLL_I, ROLL_D);
PID pitchPID(PITCH_P, PITCH_I, PITCH_D);
PID yawPID(YAW_P, 0, 0);

PID altPID(0.2, 0.0, 0.0); // altitude outer loop PID
PID velPID(0.08, 0.01, 0.008);
#define ALT_SP_MIN_M     0.00f
#define ALT_SP_MAX_M     2.00f
float tiltMax = TILT_MAX;

Quaternion attitudeTarget;
Vector ratesTarget;
Vector ratesExtra; // feedforward rates
Vector torqueTarget;
float thrustTarget;

KrenCtrl pdpiRoll;// = KrenCtrl();
KrenCtrl pdpiPitch; // = KrenCtrl();
float rollSp_mrad  = 0.0f;
float pitchSp_mrad = 0.0f;
extern const int MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT;
extern float controlRoll, controlPitch, controlThrottle, controlYaw, controlMode, roll_H, pitch_H, dt, vz_kf_cm_s, z_kf_cm;
#define RATES_LFP_ALPHA 0.2 // cutoff frequency ~ 40 Hz
float pich = 0.0f;
float rll = 0.0f;
void control() {
	interpretControls();
	//controlAltitude();
	controlAttitude();
	controlTorque();
}

void interpretControls() {
	mode = STAB;
//	thrustTarget = controlThrottle;
//	rollSp_mrad = controlRoll* tiltMax*1000.0f; // to mrad
//	pitchSp_mrad = controlPitch* tiltMax*1000.0f; // to mrad
}
void controlAltitude(){
    if (!armed) {
        thrustTarget = 0.0f;
        return;
    }
    if (controlThrottle < 0.05f) {
        thrustTarget = controlThrottle;
        altPID.reset();
        velPID.reset();
        return;
    }
    float spCmd = ALT_SP_MIN_M + (ALT_SP_MAX_M - ALT_SP_MIN_M) * constrain(controlThrottle, 0.0f, 1.0f);
    float altError = spCmd - z_kf_cm;          // cm -> m
    float altOutput = altPID.update(altError);     // m/s

    float velError = altOutput - vz_kf_cm_s;   // m/s
    float velOutput = velPID.update(velError);     // thrust delta
    const float hoverThrust = 0.5f; // chỉnh theo thực tế
    thrustTarget = constrain(hoverThrust + velOutput, 0.0f, 1.0f);
}
void controlAttitude(){
	static LowPassFilter<Vector> ratesFilter(RATES_LFP_ALPHA);
	rates = ratesFilter.update(gyro);

	//failsafe();
	torqueTarget.x = pdpiRoll.updateCtrl(dt, rollSp_mrad, roll_H, rates.x) / 1000.0f;
	torqueTarget.y = pdpiPitch.updateCtrl(dt, pitchSp_mrad, pitch_H, rates.y) / 1000.0f;
	float yawRateSp = -controlYaw * YAWRATE_MAX;
	torqueTarget.z = yawRatePID.update(yawRateSp - rates.z);
}

uint8_t bnRll=0, bnPtch=0, bnYaw=0, bManualMtr = 0;
void DisableCnl(int cnl, int val){	
	if(cnl == NoRoll) {
		bnRll = val;
		if (!val) {
			pdpiRoll.reset();    // Reset roll khi bật
			pdpiPitch.reset();   // ⚠️ RESET tất cả (để an toàn)
		}
	}
	else if(cnl == NoPitch) {
		bnPtch = val;
		if (!val) {
			pdpiRoll.reset();    // ⚠️ RESET tất cả (để an toàn)
			pdpiPitch.reset();   // Reset pitch khi bật
		}
	}
	else if(cnl == NoYaw) bnYaw = val;
	else {
		bnRll = bnPtch = bnYaw = val;
		if (!val) {
			pdpiRoll.reset();    // Reset tất cả kênh
			pdpiPitch.reset();
		}
	}
	print("Channel %d set to %d - Controllers reset\n", cnl, val);
}
void controlTorque() {
	if (!torqueTarget.valid()) return; // skip torque control

	if (!armed) {
		memset(motors, 0, sizeof(motors)); // stop motors if disarmed
		return;
	}

	if (bManualMtr) return;

	motors[MOTOR_FRONT_LEFT] = thrustTarget + (bnRll?0:torqueTarget.x) - (bnPtch?0:torqueTarget.y) + (bnYaw?0:torqueTarget.z);
	motors[MOTOR_FRONT_RIGHT] = thrustTarget - (bnRll?0:torqueTarget.x) - (bnPtch?0:torqueTarget.y) - (bnYaw?0:torqueTarget.z);
	motors[MOTOR_REAR_LEFT] = thrustTarget + (bnRll?0:torqueTarget.x) + (bnPtch?0:torqueTarget.y) - (bnYaw?0:torqueTarget.z);
	motors[MOTOR_REAR_RIGHT] = thrustTarget - (bnRll?0:torqueTarget.x) + (bnPtch?0:torqueTarget.y) + (bnYaw?0:torqueTarget.z);

	motors[0] = constrain(motors[0], 0, 1);
	motors[1] = constrain(motors[1], 0, 1);
	motors[2] = constrain(motors[2], 0, 1);
	motors[3] = constrain(motors[3], 0, 1);
}

const char* getModeName() {
	switch (mode) {
		case MANUAL: return "MANUAL";
		case ACRO: return "ACRO";
		case STAB: return "STAB";
		case AUTO: return "AUTO";
		default: return "UNKNOWN";
	}
}
