package org.firstinspires.ftc.teamcode.OpModes.Tele;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Systems.MeccanumChassis;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.seramitae.ftc.hardware.Lights.RGBLight;
import org.seramitae.ftc.hardware.Motor.Motor;
import org.seramitae.ftc.hardware.Motor.MotorEx;
import org.seramitae.ftc.hardware.Servo.CRServoEx;

import java.util.List;

@TeleOp(name = "teleOp", group = "Test")
public class TeleOP extends OpMode {

    MotorEx m1;
    CRServoEx s1;
    RGBLight light;

    MeccanumChassis drive;
    Shooter shooter;

    ElapsedTime looptime;

    List<LynxModule> hubs;

    @Override
    public void init() {

        m1 = new MotorEx(
                hardwareMap,
                "intake",
                103.8,
                1620
        );

        m1.forward();
        m1.brake();

        m1.setRunMode(
                Motor.RunMode.RawPower
        );
        s1 = new CRServoEx(
                hardwareMap,
                "spindexer"
        );
        s1.setDirection(DcMotorSimple.Direction.REVERSE);
        light = new RGBLight(
                hardwareMap,
                "light"
        );

        light.green();

        drive = new MeccanumChassis(
                hardwareMap
        );

        shooter = new Shooter(
                hardwareMap
        );

        looptime = new ElapsedTime();

        hubs = hardwareMap.getAll(
                LynxModule.class
        );
    }

    @Override
    public void start() {

        hubs.forEach(
                hub -> hub.setBulkCachingMode(
                        LynxModule.BulkCachingMode.MANUAL
                )
        );

        looptime.reset();
    }

    @Override
    public void loop() {

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        looptime.reset();

        drive.update(gamepad1);

        if (gamepad1.yWasPressed()) {
            shooter.shoot(3000);
        }

        if (gamepad1.bWasPressed()) {
            shooter.off();
            m1.setPower(0);
            s1.setPower(0);
        }
        if(gamepad1.left_bumper)
        {
            m1.setPower(1);
            s1.setPower(1);
        }
        if(gamepad1.right_bumper)
        {
            m1.setPower(-1);
        }
        if(gamepad1.optionsWasPressed())
        {
            drive.resetHeading();
        }
        shooter.periodic();
        telemetry.addData("velo" , shooter.getAverageVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {

        drive.stop();
        shooter.off();

        m1.stop();
        s1.stop();

        light.off();
    }
}