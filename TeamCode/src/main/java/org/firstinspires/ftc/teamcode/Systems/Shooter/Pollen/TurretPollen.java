package org.firstinspires.ftc.teamcode.Systems.Shooter.Pollen;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.seramitae.ftc.hardware.Servo.CRServoEx;

@Config
public class TurretPollen {

    private final CRServoEx s1, s2;
    private final DcMotorEx encoder;

    public static double rpt = 0.0017;

    public static double kp = 0.007;
    public static double kd = 0.0;
    public static double kf = 0.0;

    public static double sp = 0.01;
    public static double sd = 0.0001;
    public static double sf = 0.0;

    public static double pidfSwitch = 50;

    public static double dMin = 110;
    public static double dMax = 110;

    public static double readyTolerance = 30;

    public static double error = 0;
    public static double power = 0;
    public static double manualPower = 0;
    public static double t = 0;

    private boolean enabled = true;
    private boolean manualMode = false;

    private double lastError = 0;
    private long lastTime = 0;

    public TurretPollen(HardwareMap hwMap) {

        s1 = new CRServoEx(hwMap, "ServoTuretaPollen1");
        s2 = new CRServoEx(hwMap, "ServoTuretaPollen2");

        s1.reverse(); // TODO verify direction
        s2.forward(); // TODO verify direction

        encoder = hwMap.get(DcMotorEx.class, "EncoderPollen");

        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setPower(0);

        t = encoder.getCurrentPosition();

        resetPID();
    }

    public void periodic() {

        if (!enabled) {
            stopServos();
            resetPID();
            return;
        }

        if (manualMode) {

            double p = clamp(manualPower, -1, 1);

            if ((p > 0 && getYaw() >= getMaxYaw()) ||
                    (p < 0 && getYaw() <= getMinYaw())) {
                p = 0;
            }

            setServoPower(p);
            resetPID();
            return;
        }

        error = t - getTurret();

        double pGain;
        double dGain;
        double fGain;

        if (Math.abs(error) > pidfSwitch) {
            pGain = kp;
            dGain = kd;
            fGain = kf;
        } else {
            pGain = sp;
            dGain = sd;
            fGain = sf;
        }

        long now = System.nanoTime();

        double dt = lastTime == 0
                ? 0
                : (now - lastTime) / 1e9;

        double derivative = 0;

        if (dt > 0) {
            derivative = (error - lastError) / dt;
        }

        power = pGain * error
                + dGain * derivative
                + fGain * Math.signum(error);

        power = clamp(power, -1, 1);

        if ((power > 0 && getYaw() >= getMaxYaw()) ||
                (power < 0 && getYaw() <= getMinYaw())) {
            power = 0;
        }

        setServoPower(power);

        lastError = error;
        lastTime = now;
    }

    private void setServoPower(double p) {
        p = clamp(p, -1, 1);

        s1.setPower(p);
        s2.setPower(p);

        power = p;
    }

    private void stopServos() {
        s1.stop();
        s2.stop();
        power = 0;
    }

    private void resetPID() {
        lastError = 0;
        lastTime = 0;
    }

    public void manual(double p) {
        manualMode = true;
        manualPower = clamp(p, -1, 1);
    }

    public void automatic() {
        manualMode = false;
        manualPower = 0;
        t = getTurret();
        resetPID();
    }

    public void on() {
        enabled = true;
        t = getTurret();
        resetPID();
    }

    public void off() {
        enabled = false;
        stopServos();
        resetPID();
    }

    public double getTurret() {
        return encoder.getCurrentPosition();
    }

    public double getYaw() {
        return getTurret() * rpt;
    }

    private double getMaxYaw() {
        return Math.toRadians(dMax);
    }

    private double getMinYaw() {
        return -Math.toRadians(dMin);
    }

    public void setYaw(double radians) {

        radians = normalizeAngle(radians);

        radians = clamp(
                radians,
                getMinYaw(),
                getMaxYaw()
        );

        setTurretTarget(radians / rpt);
    }

    public void addYaw(double radians) {
        setYaw(getYaw() + radians);
    }

    public void face(Pose targetPose, Pose robotPose) {

        double dx = targetPose.x() - robotPose.x();
        double dy = targetPose.y() - robotPose.y();

        double angleToTarget = Math.atan2(dy, dx);

        double turretAngle = normalizeAngle(
                angleToTarget - robotPose.heading()
        );

        setYaw(turretAngle);
    }

    public void resetTurret() {

        stopServos();

        encoder.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        encoder.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        t = 0;
        error = 0;

        resetPID();
    }

    private void setTurretTarget(double ticks) {

        double yaw = ticks * rpt;

        yaw = clamp(
                yaw,
                getMinYaw(),
                getMaxYaw()
        );

        t = yaw / rpt;
    }

    public double getTarget() {
        return t;
    }

    public double getError() {
        return t - getTurret();
    }

    public boolean isReady() {
        return enabled
                && !manualMode
                && Math.abs(getError()) < readyTolerance;
    }

    public boolean isManual() {
        return manualMode;
    }

    public boolean isEnabled() {
        return enabled;
    }

    private static double clamp(
            double value,
            double min,
            double max
    ) {
        return Math.max(min, Math.min(max, value));
    }

    public static double normalizeAngle(double angle) {

        angle %= 2.0 * Math.PI;

        if (angle <= -Math.PI) {
            angle += 2.0 * Math.PI;
        }

        if (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }

        return angle;
    }
}