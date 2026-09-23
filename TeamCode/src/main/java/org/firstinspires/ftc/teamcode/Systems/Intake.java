package org.firstinspires.ftc.teamcode.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.seramitae.ftc.hardware.Motor.Motor;
import org.seramitae.ftc.hardware.Motor.MotorEx;


@Config
public class Intake {
    MotorEx intake;
    public static double INPOWER = 1, OUTPOWER = -1;
    public Intake(HardwareMap h)
    {
        intake = new MotorEx(h , "intake" , 103.8 , 1620);
        intake.forward();
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.brake();
    }
    public void SpinIn()
    {
        intake.setPower(INPOWER);
    }
    public void SpinOut()
    {
        intake.setPower(OUTPOWER);
    }
    public void Stop()
    {
        intake.setPower(0);
    }
}
