package org.firstinspires.ftc.teamcode.Systems.Swerve;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DiffySwerve {

    public enum ModuleSide {
        LEFT,
        RIGHT
    }

    private final DcMotorEx motorTop;
    private final DcMotorEx motorBottom;

    private final ModuleSide side;

    public static double MOTOR_TICKS_PER_REV = 28.0; // TODO: change to your motor encoder ticks

    public static double MODULE_GEAR_RATIO =
            (60.0 / 11.0) *
                    (48.0 / 15.0) *
                    (82.0 / 22.0) *
                    2.0; // TODO: change to your module gear ratio

    public static double WHEEL_GEAR_RATIO =
            (60.0 / 11.0) *
                    (48.0 / 15.0) *
                    (82.0 / 22.0) *
                    (14.0 / 60.0); // TODO: change to your wheel gear ratio

    public static double WHEEL_DIAMETER_CM = 7.62; // TODO: change to your wheel diameter

    public static double MODULE_DISTANCE_FROM_CENTER_CM = 9.0; // TODO: change to your module position

    public static double ALLOWED_ANGLE_ERROR = 5.0; // TODO: tune
    public static double MAX_ROTATION_POWER_ANGLE = 60.0; // TODO: tune
    public static double ROTATION_ADVANTAGE = 1.0; // TODO: tune
    public static double MAX_MOTOR_POWER = 0.7; // TODO: tune

    public static double REVERSE_THRESHOLD = 90.0; // TODO: tune

    public static double TICKS_PER_MODULE_REV =
            MOTOR_TICKS_PER_REV * MODULE_GEAR_RATIO;

    public static double DEGREES_PER_TICK =
            360.0 / TICKS_PER_MODULE_REV;

    public static double TICKS_PER_WHEEL_REV =
            MOTOR_TICKS_PER_REV * WHEEL_GEAR_RATIO;

    public static double WHEEL_CIRCUMFERENCE_CM =
            Math.PI * WHEEL_DIAMETER_CM;

    public static double CM_PER_TICK =
            WHEEL_CIRCUMFERENCE_CM / TICKS_PER_WHEEL_REV;


    private boolean reversed = false;

    private double lastTopEncoder;
    private double lastBottomEncoder;

    private double distanceTraveled = 0;


    public DiffySwerve(HardwareMap hardwareMap, ModuleSide side) {

        this.side = side;

        if (side == ModuleSide.RIGHT) {

            motorTop = hardwareMap.get(
                    DcMotorEx.class,
                    "rightTopMotor" // TODO: change hardware name
            );

            motorBottom = hardwareMap.get(
                    DcMotorEx.class,
                    "rightBottomMotor" // TODO: change hardware name
            );

        } else {

            motorTop = hardwareMap.get(
                    DcMotorEx.class,
                    "leftTopMotor" // TODO: change hardware name
            );

            motorBottom = hardwareMap.get(
                    DcMotorEx.class,
                    "leftBottomMotor" // TODO: change hardware name
            );
        }


        motorTop.setDirection(DcMotorSimple.Direction.FORWARD); // TODO: check direction
        motorBottom.setDirection(DcMotorSimple.Direction.FORWARD); // TODO: check direction


        motorTop.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        motorBottom.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );


        motorTop.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        motorBottom.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );


        lastTopEncoder = motorTop.getCurrentPosition();
        lastBottomEncoder = motorBottom.getCurrentPosition();
    }


    public void setTarget(double movePower, double targetAngle) {

        double currentAngle = getCurrentAngle();

        double error = normalizeAngle(
                targetAngle - currentAngle
        );


        if (Math.abs(error) > REVERSE_THRESHOLD) {

            reversed = true;

            movePower *= -1;

            targetAngle = normalizeAngle(
                    targetAngle + 180
            );

            error = normalizeAngle(
                    targetAngle - currentAngle
            );

        } else {

            reversed = false;
        }


        double pivotPower = calculatePivotPower(error);

        setMotorPowers(
                movePower,
                pivotPower
        );
    }


    private double calculatePivotPower(double error) {

        if (Math.abs(error) < ALLOWED_ANGLE_ERROR) {
            return 0;
        }

        double power =
                Math.abs(error) /
                        MAX_ROTATION_POWER_ANGLE;

        power = Math.min(
                power,
                ROTATION_ADVANTAGE
        );

        return Math.signum(error) * power;
    }


    private void setMotorPowers(double move, double pivot) {

        double topPower = move + pivot;
        double bottomPower = move - pivot; // TODO: check differential signs

        double max = Math.max(
                Math.abs(topPower),
                Math.abs(bottomPower)
        );

        if (max > MAX_MOTOR_POWER) {

            double scale = MAX_MOTOR_POWER / max;

            topPower *= scale;
            bottomPower *= scale;
        }

        motorTop.setPower(topPower);
        motorBottom.setPower(bottomPower);
    }


    public double getCurrentAngle() {

        double top = motorTop.getCurrentPosition();
        double bottom = motorBottom.getCurrentPosition();

        double ticks = top + bottom; // TODO: check encoder combination

        double angle = ticks * DEGREES_PER_TICK;

        return normalizeAngle(angle);
    }


    public void updateTracking() {

        double currentTop = motorTop.getCurrentPosition();
        double currentBottom = motorBottom.getCurrentPosition();

        double topChange =
                currentTop - lastTopEncoder;

        double bottomChange =
                currentBottom - lastBottomEncoder;

        double wheelTicks =
                (topChange - bottomChange) / 2.0; // TODO: check encoder signs

        double deltaDistance =
                wheelTicks * CM_PER_TICK;

        if (reversed) {
            distanceTraveled -= deltaDistance;
        } else {
            distanceTraveled += deltaDistance;
        }

        lastTopEncoder = currentTop;
        lastBottomEncoder = currentBottom;
    }


    public double getDistanceTraveled() {
        return distanceTraveled;
    }


    public void resetDistanceTraveled() {

        distanceTraveled = 0;

        lastTopEncoder = motorTop.getCurrentPosition();
        lastBottomEncoder = motorBottom.getCurrentPosition();
    }


    public void resetEncoders() {

        motorTop.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        motorBottom.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        motorTop.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        motorBottom.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        resetDistanceTraveled();
    }


    private double normalizeAngle(double angle) {

        while (angle > 180) {
            angle -= 360;
        }

        while (angle <= -180) {
            angle += 360;
        }

        return angle;
    }


    public boolean isReversed() {
        return reversed;
    }


    public ModuleSide getSide() {
        return side;
    }
}