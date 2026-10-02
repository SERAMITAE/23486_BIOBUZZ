package org.firstinspires.ftc.teamcode.Systems.Shooter;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.seramitae.ftc.controller.feedforward.SimpleMotorFeedforward;
import org.seramitae.ftc.hardware.Motor.Motor;
import org.seramitae.ftc.hardware.Motor.MotorEx;

public class ShooterPollen {
    MotorEx motor;
    SimpleMotorFeedforward controller;
    public double VELOCITY = 0;
    double kv = 0 , ks = 0  , ka = 0;
    double power;
    public ShooterPollen(HardwareMap HwMap) {
        motor = new MotorEx(HwMap, "MotorPollen");
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor.setRunMode(Motor.RunMode.VelocityControl);
        controller = new SimpleMotorFeedforward(ks, kv, ka);
    }
    public void periodic()
    {
        controller.calculate(VELOCITY);
    }
}
