package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="DistanceSensorTest", group="Test")
public class DigitalChannelTest extends OpMode {

    public DigitalChannel d;

    int Balls = 0;

    public boolean detected = false;
    public boolean lastDetected = false;

    @Override
    public void init() {
        d = hardwareMap.get(DigitalChannel.class, "distanceSensor");
        d.setMode(DigitalChannel.Mode.INPUT);

        lastDetected = d.getState();
    }

    @Override
    public void loop() {

        detected = d.getState();

        if (detected && !lastDetected) {
            Balls++;
        }

        lastDetected = detected;

        telemetry.addData("BALLS", Balls);
        telemetry.addData("Detected", detected);
        telemetry.update();
    }
}