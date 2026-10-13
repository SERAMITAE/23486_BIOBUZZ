package org.firstinspires.ftc.teamcode.Systems.Shooter.Nectar;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ShooterNectar {

    private final DcMotorEx motor;
    private final Servo Gate;
    private final Servo Hood;

    public static double targetVelocity = 0;
    private boolean activated = false;

    public static double kP = 0.01;
    public static double kS = 0.07;
    public static double kV = 0.000365;

    public static int velocityTolerance = 25;

    public static double CLOSE = 630;
    public static double FAR = 1725;
    public static double AUTOFAR = 1530;
    public static double AUTOCLOSE = 630;

    public static double OPENGATEPOS = 0.15;
    public static double CLOSEGATE = 0.0;

    private double lastError = 0;
    private boolean isGettingUpToSpeed = true;
    private boolean onceShot = false;

    public ShooterNectar(HardwareMap hardwareMap) {

        motor = hardwareMap.get(
                DcMotorEx.class,
                "ShooterNectar"
        );

        Hood = hardwareMap.get(
                Servo.class,
                "HoodNectar"
        );

        Gate = hardwareMap.get(
                Servo.class,
                "GateNectar"
        );

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        closegate();
    }

    public void setTarget(double velocity) {
        targetVelocity = velocity;
        activated = true;
        isGettingUpToSpeed = true;
        onceShot = false;
    }

    public void off() {
        activated = false;
        targetVelocity = 0;
        motor.setPower(0);
        closegate();
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public double getTarget() {
        return targetVelocity;
    }

    public void periodic() {

        if (!activated) {
            motor.setPower(0);
            return;
        }

        double currentVelocity = getVelocity();
        double error = targetVelocity - currentVelocity;

        double power =
                kP * error
                        + kV * targetVelocity
                        + kS * Math.signum(targetVelocity);

        power = Math.max(-1, Math.min(1, power));

        motor.setPower(power);

        if (isGettingUpToSpeed && atTarget()) {
            isGettingUpToSpeed = false;
        }

        lastError = error;
    }

    public boolean hasShot() {

        if (!activated || isGettingUpToSpeed) {
            return false;
        }

        double currentError = targetVelocity - getVelocity();
        double derivative = currentError - lastError;

        if (atTarget()) {
            onceShot = false;
            return false;
        }

        if (derivative > 0
                && currentError > 160
                && !onceShot) {

            onceShot = true;
            return true;
        }

        return false;
    }

    public boolean atTarget() {
        return activated
                && Math.abs(targetVelocity - getVelocity())
                <= velocityTolerance;
    }

    public void close() {
        setTarget(CLOSE);
    }

    public void shoot(double velo, double pos) {
        setTarget(velo);
        opengate();
        Hood.setPosition(pos);
    }

    public void AUTOCLOSE() {
        setTarget(AUTOCLOSE);
    }

    public void far() {
        setTarget(FAR);
    }

    public void autofar() {
        setTarget(AUTOFAR);
    }

    public void opengate() {
        Gate.setPosition(OPENGATEPOS);
    }

    public void closegate() {
        Gate.setPosition(CLOSEGATE);
    }
}