package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.seramitae.ftc.hardware.Lights.RGBLight;
import org.seramitae.ftc.hardware.Motor.Motor;
import org.seramitae.ftc.hardware.Motor.MotorEx;
import org.seramitae.ftc.hardware.Servo.CRServoEx;


import java.util.List;

@TeleOp(name = "Library Test" , group = "Test")
public class LibraryTest extends OpMode {

    MotorEx m1;
    CRServoEx s1;
    RGBLight light;
    ElapsedTime looptime;
    double looptimeMs = 0;
    String isStalled = "";

    @Override
    public void init()
    {
        //TEST MOTOR
        m1 = new MotorEx(hardwareMap , "intake" , 103.8 , 1620);
        m1.forward();
        m1.brake();
        m1.setRunMode(Motor.RunMode.RawPower);
        //TEST SERVO
        s1 = new CRServoEx(hardwareMap , "spindexer ");
        looptime = new ElapsedTime();
        light.green();
    }
    @Override
    public void start()
    {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        looptime.reset();
    }
    @Override
    public void loop()
    {
        looptimeMs = looptime.milliseconds();
        looptime.reset();

        if(gamepad1.aWasPressed())
        {
            m1.setPower(.5);
            s1.setPower(1);
            light.blue();
        }

        if(gamepad1.bWasPressed())
        {
            m1.setPower(0);
            s1.setPower(0);
            light.green();
        }

        telemetry.addData("LoopTime" , looptime);
        telemetry.addData("Velocity", m1.getVelocity());
        telemetry.addData("Caching Tolerance" , m1.getCachingTolerance());
        telemetry.addData("Current(AMPS)" , m1.getCurrent(CurrentUnit.AMPS));
        if(m1.isStalled(1 , 50 ))
        {
            isStalled = "true";
        }
        else isStalled = "false";
        telemetry.addData("isStalled" , isStalled);
        telemetry.update();
    }
}
