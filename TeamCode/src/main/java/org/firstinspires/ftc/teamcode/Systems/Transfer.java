package org.firstinspires.ftc.teamcode.Systems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.seramitae.ftc.hardware.Motor.Motor;
import org.seramitae.ftc.hardware.Motor.MotorEx;

@Config
public class Transfer {
    MotorEx m ;
    public static double INPOWER , OUTPOWER;
    public Transfer(HardwareMap H)
    {
        m = new MotorEx(H , "transfer");
        m.setRunMode(Motor.RunMode.RawPower);
        m.brake();
    }

    public void SpinIn()
    {
        m.setPower(INPOWER);
    }
    public void SpinOut()
    {
        m.setPower(OUTPOWER);
    }
    public void Stop()
    {
        m.setPower(0);
    }
}
