package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.seramitae.ftc.hardware.Motor.MotorEx;
import org.seramitae.ftc.hardware.Servo.ServoEx;

public class CoaxialSwerveChassis {
    MotorEx FL , FR , BL , BR;
    ServoEx FLS , FRS , BLS , BRS;


    public CoaxialSwerveChassis(HardwareMap h)
    {
        FL = new MotorEx(h , "FL", 112 , 6000);
        FR = new MotorEx(h , "FR" , 112 , 6000);
        BL = new MotorEx(h , "BL" , 112 , 6000);
        BR = new MotorEx(h , "BR" , 112 , 6000);

        FLS = new ServoEx(h , "FLS");
        FRS = new ServoEx(h , "FRS");
        BLS = new ServoEx(h , "BLS");
        BRS = new ServoEx(h , "BRS");

        FL.reverse(); BL.reverse();
    }
}
