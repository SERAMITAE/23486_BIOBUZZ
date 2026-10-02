package org.firstinspires.ftc.teamcode.OpModes.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Swerve.DiffySwerve;


@TeleOp(name = "Diff Swerve Test", group = "Test")
public class TestSwerve extends OpMode {

    private DiffySwerve leftPod;
    private DiffySwerve rightPod;

    private double targetAngle = 0;

    @Override
    public void init() {

        leftPod = new DiffySwerve(
                hardwareMap,
                DiffySwerve.ModuleSide.LEFT
        );

        rightPod = new DiffySwerve(
                hardwareMap,
                DiffySwerve.ModuleSide.RIGHT
        );

        telemetry.update();
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up) {
            targetAngle = 0;
        }

        if (gamepad1.dpad_right) {
            targetAngle = 90;
        }

        if (gamepad1.dpad_down) {
            targetAngle = 180;
        }

        if (gamepad1.dpad_left) {
            targetAngle = -90;
        }


        double movePower = -gamepad1.left_stick_y;

        if (Math.abs(movePower) < 0.05) {
            movePower = 0;
        }


        leftPod.setTarget(movePower, targetAngle);
        rightPod.setTarget(movePower, targetAngle);

        leftPod.updateTracking();
        rightPod.updateTracking();


        telemetry.addData("Target Angle", targetAngle);

        telemetry.addData(
                "Left Angle",
                leftPod.getCurrentAngle()
        );

        telemetry.addData(
                "Right Angle",
                rightPod.getCurrentAngle()
        );

        telemetry.addData(
                "Left Reversed",
                leftPod.isReversed()
        );

        telemetry.addData(
                "Right Reversed",
                rightPod.isReversed()
        );

        telemetry.addData(
                "Left Distance",
                leftPod.getDistanceTraveled()
        );

        telemetry.addData(
                "Right Distance",
                rightPod.getDistanceTraveled()
        );

        telemetry.update();
    }
}