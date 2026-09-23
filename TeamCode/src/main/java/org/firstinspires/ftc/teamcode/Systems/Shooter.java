package org.firstinspires.ftc.teamcode.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.seramitae.ftc.controller.PIDController;
import org.seramitae.ftc.hardware.Motor.Motor;
import org.seramitae.ftc.hardware.Motor.MotorEx;

@Config
public class Shooter {

    private final MotorEx motorLeft;
    private final MotorEx motorRight;

    private final PIDController controller;

    public static double targetVelocity = 0;

    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double kS = 0.07;
    public static double kV = 0.000365;

    public static int velocityTolerance = 25;

    public static double CLOSE = 630;
    public static double FAR = 1725;
    public static double AUTOFAR = 1530;
    public static double AUTOCLOSE = 630;

    private boolean activated = false;
    private boolean isGettingUpToSpeed = true;
    private boolean onceShot = false;

    private double lastError = 0;

    public Shooter(HardwareMap hardwareMap) {

        motorLeft = new MotorEx(
                hardwareMap,
                "m2"
        );

        motorRight = new MotorEx(
                hardwareMap,
                "m1"
        );

        motorLeft.forward();
        motorRight.reverse();
        motorRight.reverse();

        motorLeft.brake();
        motorRight.brake();

        motorLeft.setRunMode(
                Motor.RunMode.RawPower
        );

        motorRight.setRunMode(
                Motor.RunMode.RawPower
        );

        controller = new PIDController(
                kP,
                kI,
                kD
        );

        controller.setTolerance(
                velocityTolerance
        );
    }

    public void setTarget(double velocity) {

        targetVelocity = velocity;

        controller.setSetPoint(
                velocity
        );

        controller.reset();

        activated = true;
        isGettingUpToSpeed = true;
        onceShot = false;
    }

    public void off() {

        activated = false;

        targetVelocity = 0;

        controller.reset();

        motorLeft.stop();
        motorRight.stop();
    }

    public void periodic() {

        if (!activated) {

            motorLeft.stop();
            motorRight.stop();

            return;
        }

        controller.setPID(
                kP,
                kI,
                kD
        );

        controller.setTolerance(
                velocityTolerance
        );

        double currentVelocity =
                getVelocity();

        double pid =
                controller.calculate(
                        currentVelocity
                );

        double feedforward =
                kS * Math.signum(targetVelocity)
                        + kV * targetVelocity;

        double power =
                clamp(
                        pid + feedforward,
                        -1,
                        1
                );

        motorLeft.setPower(power);
        motorRight.setPower(power);

        if (isGettingUpToSpeed
                && atTarget()) {

            isGettingUpToSpeed = false;
        }

        lastError =
                targetVelocity
                        - currentVelocity;
    }

    public boolean atTarget() {

        return Math.abs(
                targetVelocity
                        - getVelocity()
        ) <= velocityTolerance;
    }

    public boolean hasShot() {

        if (atTarget()
                || isGettingUpToSpeed) {

            onceShot = false;

            return false;
        }

        double currentError =
                targetVelocity
                        - getVelocity();

        double derivative =
                currentError
                        - lastError;

        if (derivative > 0
                && currentError > 50
                && !onceShot) {

            onceShot = true;

            return true;
        }

        return false;
    }

    public void shoot(double velocity) {
        setTarget(velocity);
    }

    public void close() {
        setTarget(CLOSE);
    }

    public void far() {
        setTarget(FAR);
    }

    public void autoClose() {
        setTarget(AUTOCLOSE);
    }

    public void autoFar() {
        setTarget(AUTOFAR);
    }

    public double getVelocity() {
        return Math.abs(
                motorLeft.getVelocity()
        );
    }

    public double getRightVelocity() {
        return Math.abs(
                motorRight.getVelocity()
        );
    }

    public double getAverageVelocity() {

        return (
                motorRight.getVelocity()
        ) ;
    }

    public double getTarget() {
        return targetVelocity;
    }

    public double getError() {

        return targetVelocity
                - getVelocity();
    }

    public boolean isActivated() {
        return activated;
    }

    public MotorEx getLeftMotor() {
        return motorLeft;
    }

    public MotorEx getRightMotor() {
        return motorRight;
    }

    public PIDController getController() {
        return controller;
    }

    private double clamp(
            double value,
            double min,
            double max
    ) {

        return Math.max(
                min,
                Math.min(
                        max,
                        value
                )
        );
    }
}