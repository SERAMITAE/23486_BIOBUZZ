package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.seramitae.ftc.drive.MecanumDrive;
import org.seramitae.ftc.hardware.Motor.MotorEx;

public class MeccanumChassis {

    private final MotorEx frontLeft;
    private final MotorEx frontRight;
    private final MotorEx backLeft;
    private final MotorEx backRight;

    private final MecanumDrive drive;

    private double normalSpeed = 1.0;
    private double slowSpeed = 0.4;

    private double deadzone = 0.05;

    private boolean previousY = false;
    private boolean previousOptions = false;

    public MeccanumChassis(HardwareMap hardwareMap) {

        frontLeft =
                new MotorEx(
                        hardwareMap,
                        "FL"
                );

        frontRight =
                new MotorEx(
                        hardwareMap,
                        "FR"
                );

        backLeft =
                new MotorEx(
                        hardwareMap,
                        "BL"
                );

        backRight =
                new MotorEx(
                        hardwareMap,
                        "BR"
                );

        frontLeft.setInverted(true);
        backLeft.setInverted(true);

        frontLeft.brake(); backLeft.brake(); backRight.brake(); frontRight.brake();

        drive =
                new MecanumDrive(
                        frontLeft,
                        frontRight,
                        backLeft,
                        backRight,
                        hardwareMap,
                        "imu"
                );

        drive.initializeIMU(
                RevHubOrientationOnRobot
                        .LogoFacingDirection.LEFT,

                RevHubOrientationOnRobot
                        .UsbFacingDirection.UP
        );

        drive
                .brake()
                .fieldCentric()
                .setSpeedMultiplier(
                        normalSpeed
                );
    }

    public void update(Gamepad gamepad) {

        handleDriveMode(gamepad);
        handleHeadingReset(gamepad);
        handleSpeed(gamepad);

        drive(gamepad);

        previousY = gamepad.y;
        previousOptions = gamepad.options;
    }

    public void drive(Gamepad gamepad) {

        double forward =
                applyDeadzone(
                        -gamepad.left_stick_y
                );

        double strafe =
                applyDeadzone(
                        gamepad.left_stick_x
                );

        double turn =
                applyDeadzone(
                        -gamepad.right_stick_x
                );

        drive.drive(
                forward,
                strafe,
                turn
        );
    }

    public void drive(
            double forward,
            double strafe,
            double turn
    ) {

        drive.drive(
                applyDeadzone(forward),
                applyDeadzone(strafe),
                applyDeadzone(turn)
        );
    }

    public void driveRobotCentric(
            Gamepad gamepad
    ) {

        drive.driveRobotCentric(
                applyDeadzone(
                        -gamepad.left_stick_y
                ),
                applyDeadzone(
                        gamepad.left_stick_x
                ),
                applyDeadzone(
                        gamepad.right_stick_x
                )
        );
    }

    public void driveFieldCentric(
            Gamepad gamepad
    ) {

        drive.driveFieldCentric(
                applyDeadzone(
                        -gamepad.left_stick_y
                ),
                applyDeadzone(
                        gamepad.left_stick_x
                ),
                applyDeadzone(
                        gamepad.right_stick_x
                )
        );
    }

    private void handleDriveMode(
            Gamepad gamepad
    ) {

        if (gamepad.y && !previousY) {
            drive.toggleDriveMode();
        }
    }

    private void handleHeadingReset(
            Gamepad gamepad
    ) {

        if (gamepad.options
                && !previousOptions) {

            drive.resetHeading();
        }
    }

    private void handleSpeed(
            Gamepad gamepad
    ) {

        if (gamepad.left_bumper) {

            drive.setSpeedMultiplier(
                    slowSpeed
            );

        } else {

            drive.setSpeedMultiplier(
                    normalSpeed
            );
        }
    }

    public void fieldCentric() {
        drive.fieldCentric();
    }

    public void robotCentric() {
        drive.robotCentric();
    }

    public void toggleDriveMode() {
        drive.toggleDriveMode();
    }

    public void resetHeading() {
        drive.resetHeading();
    }

    public void resetYaw() {
        drive.resetYaw();
    }

    public void slowMode() {

        drive.setSpeedMultiplier(
                slowSpeed
        );
    }

    public void normalMode() {

        drive.setSpeedMultiplier(
                normalSpeed
        );
    }

    public void setSlowMode(
            boolean enabled
    ) {

        drive.setSpeedMultiplier(
                enabled
                        ? slowSpeed
                        : normalSpeed
        );
    }

    public void setNormalSpeed(
            double speed
    ) {

        normalSpeed =
                clamp(
                        speed,
                        0,
                        1
                );
    }

    public void setSlowSpeed(
            double speed
    ) {

        slowSpeed =
                clamp(
                        speed,
                        0,
                        1
                );
    }

    public double getNormalSpeed() {
        return normalSpeed;
    }

    public double getSlowSpeed() {
        return slowSpeed;
    }

    public double getCurrentSpeed() {
        return drive.getSpeedMultiplier();
    }

    public void setDeadzone(
            double deadzone
    ) {

        this.deadzone =
                clamp(
                        deadzone,
                        0,
                        1
                );
    }

    public double getDeadzone() {
        return deadzone;
    }

    public double getHeading() {
        return drive.getHeadingDegrees();
    }

    public boolean isFieldCentric() {
        return drive.isFieldCentric();
    }

    public MecanumDrive.DriveMode
    getDriveMode() {

        return drive.getDriveMode();
    }

    public void stop() {
        drive.stop();
    }

    public void brake() {
        drive.brake();
    }

    public void coast() {
        drive.coast();
    }

    public MotorEx getFrontLeft() {
        return frontLeft;
    }

    public MotorEx getFrontRight() {
        return frontRight;
    }

    public MotorEx getBackLeft() {
        return backLeft;
    }

    public MotorEx getBackRight() {
        return backRight;
    }

    public MecanumDrive getDrive() {
        return drive;
    }

    private double applyDeadzone(
            double value
    ) {

        if (Math.abs(value) < deadzone) {
            return 0;
        }

        return value;
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