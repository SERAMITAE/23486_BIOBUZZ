package org.firstinspires.ftc.teamcode.Systems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.seramitae.ftc.hardware.Servo.CRServoEx;

@Config
public class Spindexer {
    CRServoEx s;
    public Spindexer(HardwareMap HwMap)
    {
        s  = new CRServoEx(HwMap , "spindexer");
        s.reverse();
        s.setPower(0);
        s.forward();
    }
    public void SpinIn()
    {
        s.setPower(1);
    }
    public void SpinOut()
    {
        s.setPower(-1);
    }
    public void Stop()
    {
        s.setPower(0);
    }
}