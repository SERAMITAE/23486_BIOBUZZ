package org.firstinspires.ftc.teamcode.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.seramitae.ftc.hardware.Motor.Motor;
import org.seramitae.ftc.hardware.Motor.MotorEx;


@Config
public class Intake {
    MotorEx intake;
    DigitalChannel p;
    public static double INPOWER = 1, OUTPOWER = -1;
    public Intake(HardwareMap h)
    {
        intake = new MotorEx(h , "intake" , 103.8 , 1620);
        p = h.get(DigitalChannel.class, "Balls");
        p.setMode(DigitalChannel.Mode.INPUT);
        intake.forward();
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.brake();
    }
    public void SpinIn() {intake.setPower(INPOWER);}
    public void SpinOut()
    {
        intake.setPower(OUTPOWER);
    }
    public void getDetection() {p.getState();}
    public void Stop()
    {
        intake.setPower(0);
    }
}
