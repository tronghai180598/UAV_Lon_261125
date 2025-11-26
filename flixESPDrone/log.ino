// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// In-RAM logging

#include "vector.h"
#include "util.h"

#define LOG_RATE 100
#define LOG_DURATION 5
#define LOG_SIZE LOG_DURATION * LOG_RATE

Vector attitudeEuler;
Vector attitudeTargetEuler;
extern float roll_H;
extern float pitch_H;

struct LogEntry {
	const char *name;
	float *value;
};

LogEntry logEntries[] = {
	{"t", &t},
	{"gyro.x", &gyro.x},
	{"gyro.y", &gyro.y},
	{"roll_H", &roll_H},
	{"pitch_H", &pitch_H},
//	{"gyro.z", &gyro.z},

};

const int logColumns = sizeof(logEntries) / sizeof(logEntries[0]);
float logBuffer[LOG_SIZE][logColumns];

void prepareLogData() {
	attitudeEuler = attitude.toEuler();
	attitudeTargetEuler = attitudeTarget.toEuler();
}

int glog = 0; int lcntr;
void logData() {
	if(glog == 1){
		if(!(lcntr > 0)){
			print("%d %d %d\n", int(roll_H), int(gyro.x),  int(torqueTarget.x*1000) );		
			lcntr = 50;
		}
		lcntr--;
		return;
	}
	if(glog == 2){
		if(!(lcntr > 0)){
			print("%d %d %d\n", int(pitch_H), int(gyro.y),  int(torqueTarget.y*1000) );		
			lcntr = 50;
		}
		lcntr--;
		return;
	}
	if (!armed) return;
	static int logPointer = 0;
	static Rate period(LOG_RATE);
	if (!period) return;

	prepareLogData();

	for (int i = 0; i < logColumns; i++) {
		logBuffer[logPointer][i] = *logEntries[i].value;
	}

	logPointer++;
	if (logPointer >= LOG_SIZE) {
		logPointer = 0;
	}
}

void dumpLog() {
	// Print header
	for (int i = 0; i < logColumns; i++) {
		print("%s%s", logEntries[i].name, i < logColumns - 1 ? "," : "\n");
	}
	// Print data
	for (int i = 0; i < LOG_SIZE; i++) {
		if (logBuffer[i][0] == 0) continue; // skip empty records
		for (int j = 0; j < logColumns; j++) {
			print("%g%s", logBuffer[i][j], j < logColumns - 1 ? "," : "\n");
		}
	}
}
