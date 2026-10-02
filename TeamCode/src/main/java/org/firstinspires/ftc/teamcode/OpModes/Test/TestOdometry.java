package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.seramitae.ftc.hardware.Motor.Motor;
import org.seramitae.ftc.hardware.Motor.MotorEx;


@TeleOp(name = "TestOdometry" , group = "Test")
public class TestOdometry extends OpMode {

    MotorEx motor;

    @Override
    public void init()
    {
        motor = new MotorEx(hardwareMap , "FL");
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.resetEncoder();
    }
    public void loop()
    {
        double pos = motor.getCurrentPosition();
        telemetry.addData("Position" , pos);
    }
}
