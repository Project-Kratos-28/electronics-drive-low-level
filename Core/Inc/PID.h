#ifndef PID_H_
#define PID_H_

/*
 * Generic PID controller.
 * Used internally by both PositionMotor and SpeedMotor - not tied to any
 * specific hardware, so it's easy to unit-test on your PC before flashing.
 */
class PID {
public:
    PID(float kp, float ki, float kd, float outputMin, float outputMax);

    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float outMin, float outMax);
    void setSetpoint(float setpoint);
    float getSetpoint() const;

    void reset();

    // For circular/wrapping feedback, e.g. an absolute encoder that reports
    // 0-360 deg or 0-4095 counts and wraps around. Disabled by default -
    // only PositionMotor turns this on.
    void enableWrap(float range);
    void disableWrap();

    // measurement: current process value (position or speed)
    // dt: elapsed time in seconds since the previous call
    // returns: controller output, clamped to [outputMin, outputMax]
    float compute(float measurement, float dt);

private:
    float kp_;
    float ki_;
    float kd_;

    float setpoint_;
    float integral_;
    float prevMeasurement_;

    float outMin_;
    float outMax_;

    bool firstRun_;
    bool wrapEnabled_;
    float wrapRange_;

    float computeError(float measurement) const;
};

#endif /* PID_H_ */
