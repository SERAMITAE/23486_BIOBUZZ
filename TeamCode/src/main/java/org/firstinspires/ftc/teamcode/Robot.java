package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Shooter.Nectar.ShooterNectar;
import org.firstinspires.ftc.teamcode.Systems.Shooter.Nectar.TurretNectar;
import org.firstinspires.ftc.teamcode.Systems.Shooter.Pollen.ShooterPollen;
import org.firstinspires.ftc.teamcode.Systems.Shooter.Pollen.TurretPollen;
import org.firstinspires.ftc.teamcode.Systems.Transfer;
import org.firstinspires.ftc.teamcode.pedro.Constants;


@Config
public class Robot {
    private final Intake i;
    private final Transfer t;
    private final ShooterPollen sp;
    private final ShooterNectar sn;
    private final Follower f;
    private final TurretNectar tn;
    private final TurretPollen tp;
    private final Pose startPose = PoseFactory.degrees().of(0 , 0 , 180);
    private final Pose HivePose1 = PoseFactory.degrees().of(85 , 57 , 0);
    private final Pose HivePose2 = PoseFactory.degrees().of(56 , 57 , 0);
    public static double PVELO = 100 , NVELO = 100 , PPOS = .5 , NPOS = 0.5;
    int state = 0;
    public Robot(HardwareMap h)
    {
        i = new Intake(h);
        t = new Transfer(h);
        sp = new ShooterPollen(h);
        sn = new ShooterNectar(h);
        f = Constants.create(h);
        tn = new TurretNectar(h);
        tp = new TurretPollen(h);
        f.setPose(startPose);
        tn.on();
        tp.on();
    }
    public void periodic() {

        f.update();

        Pose robotPose = f.pose();

        Pose targetPose = robotPose.x() >= 70
                ? HivePose2
                : HivePose1;

        tp.face(targetPose, robotPose);
        tn.face(targetPose, robotPose);

        tp.periodic();
        tn.periodic();

        sp.periodic();
        sn.periodic();
    }
    public void intakeSpinIn()
    {
        i.SpinIn();t.SpinIn();sn.closegate();sp.closegate();
    }
    public void intakeSpinOut()
    {
        i.SpinOut();t.SpinOut();
    }
    public void Shoot()
    {
        switch (state)
        {
            case 0:
                sp.shoot(PVELO ,PPOS);
                sn.shoot(NVELO, NPOS);
                sn.opengate(); sp.opengate();
                i.Stop();
                t.Stop();
            case 1:
                i.SpinIn();t.SpinIn();
        }
    }
    public void StopEverything()
    {
        state = 0;
    }
}
