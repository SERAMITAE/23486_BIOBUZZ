package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.seramitae.ftc.hardware.Servo.CRServoEx;

@Config
@TeleOp(name = "Turret RPT Calibration", group = "Test")
public class TurretRPTCalibration extends OpMode {

    private CRServoEx s1, s2;
    private DcMotorEx encoder;

    public static double speed = 0.2;
    public static double calibrationAngle = 90;

    private int zeroTicks = 0;
    private double calculatedRPT = 0;

    private boolean lastA, lastB;
    private boolean lastUp, lastDown, lastLeft, lastRight;

    @Override
    public void init() {

        s1 = new CRServoEx(hardwareMap, "ServoTuretaPollen1");
        s2 = new CRServoEx(hardwareMap, "ServoTuretaPollen2");

        s1.reverse(); // TODO verify direction
        s2.forward(); // TODO verify direction

        encoder = hardwareMap.get(
                DcMotorEx.class,
                "EncoderPollen"
        );

        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setPower(0);

        zeroTicks = encoder.getCurrentPosition();

        telemetry.update();
    }

    @Override
    public void loop() {

        double input = gamepad1.right_stick_x;

        if (Math.abs(input) < 0.05) {
            input = 0;
        }

        double power = Math.max(
                -speed,
                Math.min(speed, input * speed)
        );

        s1.setPower(power);
        s2.setPower(power);

        if (gamepad1.a && !lastA) {
            zeroTicks = encoder.getCurrentPosition();
            calculatedRPT = 0;
        }

        if (gamepad1.dpad_up && !lastUp) {
            calibrationAngle += 5;
        }

        if (gamepad1.dpad_down && !lastDown) {
            calibrationAngle -= 5;
        }

        if (gamepad1.dpad_right && !lastRight) {
            calibrationAngle += 1;
        }

        if (gamepad1.dpad_left && !lastLeft) {
            calibrationAngle -= 1;
        }

        calibrationAngle = Math.max(
                1,
                Math.min(110, calibrationAngle)
        );

        int currentTicks = encoder.getCurrentPosition();
        int deltaTicks = currentTicks - zeroTicks;

        if (gamepad1.b && !lastB && deltaTicks != 0) {
            calculatedRPT =
                    Math.toRadians(calibrationAngle)
                            / Math.abs(deltaTicks);
        }

        telemetry.addData("Encoder", currentTicks);
        telemetry.addData("Zero Ticks", zeroTicks);
        telemetry.addData("Delta Ticks", deltaTicks);

        telemetry.addData("Calibration Angle",
                "%.1f deg", calibrationAngle);

        telemetry.addData("Calculated RPT",
                "%.9f", calculatedRPT);

        telemetry.addData("Encoder Direction",
                deltaTicks > 0 ? "POSITIVE" :
                        deltaTicks < 0 ? "NEGATIVE" : "ZERO");

        telemetry.addLine();
        telemetry.addLine("A = Set Zero");
        telemetry.addLine("B = Calculate RPT");
        telemetry.addLine("DPad = Change Angle");
        telemetry.addLine("Right Stick = Rotate");

        telemetry.update();

        lastA = gamepad1.a;
        lastB = gamepad1.b;

        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
    }

    @Override
    public void stop() {
        s1.stop();
        s2.stop();
    }
}