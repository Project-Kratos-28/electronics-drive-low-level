#include "PID.h"

PID::PID(float kp, float ki, float kd, float outputMin, float outputMax)
    : kp_(kp), ki_(ki), kd_(kd),
      setpoint_(0.0f), integral_(0.0f), prevMeasurement_(0.0f),
      outMin_(outputMin), outMax_(outputMax),
      firstRun_(true), wrapEnabled_(false), wrapRange_(0.0f)
{
}

void PID::setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::setOutputLimits(float outMin, float outMax) {
    outMin_ = outMin;
    outMax_ = outMax;
}

void PID::setSetpoint(float setpoint) {
    setpoint_ = setpoint;
}

float PID::getSetpoint() const {
    return setpoint_;
}

void PID::reset() {
    integral_ = 0.0f;
    prevMeasurement_ = 0.0f;
    firstRun_ = true;
}

void PID::enableWrap(float range) {
    wrapEnabled_ = true;
    wrapRange_ = range;
}

void PID::disableWrap() {
    wrapEnabled_ = false;
}

float PID::computeError(float measurement) const {
    float error = setpoint_ - measurement;

    if (wrapEnabled_ && wrapRange_ > 0.0f) {
        // Fold error into [-range/2, +range/2] so a 350 deg -> 10 deg move
        // is a 20 deg step, not a 340 deg one.
        float half = wrapRange_ * 0.5f;
        while (error > half)  error -= wrapRange_;
        while (error < -half) error += wrapRange_;
    }

    return error;
}

float PID::compute(float measurement, float dt) {
    if (dt <= 0.0f) {
        dt = 1e-3f; // guard against div-by-zero or bad timing on first call
    }

    float error = computeError(measurement);

    // --- Proportional ---
    float pTerm = kp_ * error;

    // --- Integral ---
    integral_ += error * dt;
    float iTerm = ki_ * integral_;

    // --- Derivative on measurement, not on error ---
    // Avoids a derivative "kick" every time you change the setpoint.
    float dTerm = 0.0f;
    if (!firstRun_) {
        float dMeasurement = measurement - prevMeasurement_;
        if (wrapEnabled_ && wrapRange_ > 0.0f) {
            float half = wrapRange_ * 0.5f;
            while (dMeasurement > half)  dMeasurement -= wrapRange_;
            while (dMeasurement < -half) dMeasurement += wrapRange_;
        }
        dTerm = -kd_ * (dMeasurement / dt);
    }

    float output = pTerm + iTerm + dTerm;

    // --- Clamp + anti-windup ---
    // If we saturate, undo the integral step we just took so it doesn't
    // keep climbing while the motor is already maxed out.
    if (output > outMax_) {
        output = outMax_;
        if (ki_ != 0.0f) integral_ -= error * dt;
    } else if (output < outMin_) {
        output = outMin_;
        if (ki_ != 0.0f) integral_ -= error * dt;
    }

    prevMeasurement_ = measurement;
    firstRun_ = false;

    return output;
}
