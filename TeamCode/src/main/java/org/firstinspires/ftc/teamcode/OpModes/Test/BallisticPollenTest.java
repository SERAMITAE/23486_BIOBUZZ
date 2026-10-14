
package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.Systems.Shooter.Pollen.ShooterPollen;
import org.firstinspires.ftc.teamcode.Systems.Shooter.Pollen.TurretPollen;

@Config
@TeleOp(name = "Ballistic Pollen Test", group = "Test")
public class BallisticPollenTest extends OpMode {

    private ShooterPollen shooter;
    private TurretPollen turret;
    private Follower follower;
    private Servo hood;

    private final ElapsedTime readyTimer = new ElapsedTime();

    public static double START_X_CM = 0; // TODO change
    public static double START_Y_CM = 0; // TODO change
    public static double START_HEADING_DEG = 0; // TODO change

    public static double TARGET_X_CM = 0; // TODO change
    public static double TARGET_Y_CM = 0; // TODO change
    public static double TARGET_HEIGHT_CM = 67; // TODO change

    public static double TURRET_FORWARD_CM = 0; // TODO change
    public static double TURRET_LEFT_CM = 0; // TODO change
    public static double SHOOTER_HEIGHT_CM = 0; // TODO change

    public static double MIN_TPS = 0; // TODO change
    public static double MAX_TPS = 0; // TODO change

    public static double HOOD_MIN_POS = 0; // TODO change
    public static double HOOD_MAX_POS = 0; // TODO change

    public static double FIXED_HOOD_POS = 0; // TODO change
    public static double FIXED_ANGLE_DEG = 0; // TODO change

    public static double ANGLE_AT_MIN_POS = 0; // TODO change
    public static double ANGLE_AT_MAX_POS = 0; // TODO change

    public static double TPS_PER_CM_S = 0; // TODO change
    public static double TPS_OFFSET = 0; // TODO change

    public static double VELOCITY_UPDATE_TOLERANCE = 5;
    public static double READY_TIME_MS = 200;
    public static double DRIVE_SPEED = 0.6;

    private static final double G = 981.0;
    private static final double CM_PER_INCH = 2.54;

    private boolean enabled = false;
    private boolean automaticHood = false;
    private boolean lastX = false;
    private boolean hasVelocityTarget = false;

    private double distanceCm = 0;
    private double heightDifferenceCm = 0;

    private static class Shot {

        double yaw;
        double angle;
        double hoodPos;
        double velocityTPS;
        double ballSpeedCmS;
        double flightTime;

        Shot(
                double yaw,
                double angle,
                double hoodPos,
                double velocityTPS,
                double ballSpeedCmS,
                double flightTime
        ) {
            this.yaw = yaw;
            this.angle = angle;
            this.hoodPos = hoodPos;
            this.velocityTPS = velocityTPS;
            this.ballSpeedCmS = ballSpeedCmS;
            this.flightTime = flightTime;
        }
    }

    @Override
    public void init() {

        shooter = new ShooterPollen(hardwareMap);
        turret = new TurretPollen(hardwareMap);

        hood = hardwareMap.get(
                Servo.class,
                "HoodPollen"
        );

        follower = Constants.create(hardwareMap);

        follower.setPose(new Pose(
                START_X_CM / CM_PER_INCH,
                START_Y_CM / CM_PER_INCH,
                Math.toRadians(START_HEADING_DEG)
        ));

        turret.on();
        turret.automatic();

        hood.setPosition(clampHood(FIXED_HOOD_POS));
        shooter.closegate();

        readyTimer.reset();
    }

    @Override
    public void loop() {

        follower.manual(
                -gamepad1.left_stick_y * DRIVE_SPEED,
                gamepad1.left_stick_x * DRIVE_SPEED,
                gamepad1.right_stick_x * DRIVE_SPEED
        );

        follower.update();

        if (gamepad1.x && !lastX) {
            automaticHood = !automaticHood;
            readyTimer.reset();
        }

        lastX = gamepad1.x;

        if (gamepad1.a) {
            enabled = true;
        }

        if (gamepad1.b) {
            enabled = false;
        }

        Shot shot = calculateShot(follower.pose());

        if (enabled && shot != null) {

            turret.setYaw(shot.yaw);

            hood.setPosition(
                    clampHood(shot.hoodPos)
            );

            if (!hasVelocityTarget ||
                    Math.abs(shooter.getTarget() - shot.velocityTPS)
                            > VELOCITY_UPDATE_TOLERANCE) {

                shooter.setTarget(shot.velocityTPS);
                hasVelocityTarget = true;
                readyTimer.reset();
            }

        } else {

            shooter.off();
            hasVelocityTarget = false;
            readyTimer.reset();
        }

        turret.periodic();
        shooter.periodic();

        boolean stationary =
                Math.abs(gamepad1.left_stick_x) < 0.05 &&
                        Math.abs(gamepad1.left_stick_y) < 0.05 &&
                        Math.abs(gamepad1.right_stick_x) < 0.05;

        boolean ready =
                enabled &&
                        shot != null &&
                        shooter.atTarget() &&
                        turret.isReady() &&
                        stationary;

        if (!ready) {
            readyTimer.reset();
        }

        boolean canFire =
                ready &&
                        readyTimer.milliseconds() >= READY_TIME_MS;

        if (gamepad1.right_bumper && canFire) {
            shooter.opengate();
        } else {
            shooter.closegate();
        }

        telemetry.addData(
                "Mode",
                automaticHood ? "VELOCITY + HOOD" : "VELOCITY ONLY"
        );

        telemetry.addData("Enabled", enabled);
        telemetry.addData("Distance CM", distanceCm);
        telemetry.addData("Height Difference CM", heightDifferenceCm);

        telemetry.addData(
                "Robot X CM",
                follower.pose().x() * CM_PER_INCH
        );

        telemetry.addData(
                "Robot Y CM",
                follower.pose().y() * CM_PER_INCH
        );

        telemetry.addData(
                "Flywheel Current TPS",
                shooter.getVelocity()
        );

        telemetry.addData(
                "Flywheel Target TPS",
                shooter.getTarget()
        );

        telemetry.addData("Flywheel Ready", shooter.atTarget());
        telemetry.addData("Turret Ready", turret.isReady());
        telemetry.addData("Ready To Fire", canFire);

        if (shot != null) {

            telemetry.addData(
                    "Grade Tureta",
                    Math.toDegrees(shot.yaw)
            );

            telemetry.addData(
                    "Unghi arncare",
                    shot.angle
            );

            telemetry.addData(
                    "Hood Pos",
                    shot.hoodPos
            );

            telemetry.addData(
                    "Ball Speed cm/s",
                    shot.ballSpeedCmS
            );

            telemetry.addData(
                    "Flight time s",
                    shot.flightTime
            );

        } else {

            telemetry.addData("NU" , "MERGE");
        }

        telemetry.update();
    }

    private Shot calculateShot(Pose robot) {

        double heading = robot.heading();

        double robotX = robot.x() * CM_PER_INCH;
        double robotY = robot.y() * CM_PER_INCH;

        double turretX =
                robotX +
                        TURRET_FORWARD_CM * Math.cos(heading) -
                        TURRET_LEFT_CM * Math.sin(heading);

        double turretY =
                robotY +
                        TURRET_FORWARD_CM * Math.sin(heading) +
                        TURRET_LEFT_CM * Math.cos(heading);

        double dx = TARGET_X_CM - turretX;
        double dy = TARGET_Y_CM - turretY;

        distanceCm = Math.hypot(dx, dy);

        heightDifferenceCm =
                TARGET_HEIGHT_CM - SHOOTER_HEIGHT_CM;

        double yaw =
                Math.atan2(dy, dx) - heading;

        yaw = TurretPollen.normalizeAngle(yaw);

        if (Math.toDegrees(yaw) > TurretPollen.dMax ||
                Math.toDegrees(yaw) < -TurretPollen.dMin) {
            return null;
        }

        if (!automaticHood) {

            return solve(
                    distanceCm,
                    heightDifferenceCm,
                    yaw,
                    FIXED_ANGLE_DEG,
                    clampHood(FIXED_HOOD_POS)
            );
        }

        double minAngle = Math.min(
                ANGLE_AT_MIN_POS,
                ANGLE_AT_MAX_POS
        );

        double maxAngle = Math.max(
                ANGLE_AT_MIN_POS,
                ANGLE_AT_MAX_POS
        );

        double angleRange =
                ANGLE_AT_MAX_POS - ANGLE_AT_MIN_POS;

        if (Math.abs(angleRange) < 0.001) {
            return null;
        }

        double minHood = getMinHood();
        double maxHood = getMaxHood();

        Shot best = null;

        for (double angle = minAngle;
             angle <= maxAngle + 0.001;
             angle += 1.0) {

            double hoodPos =
                    minHood +
                            (angle - ANGLE_AT_MIN_POS) /
                                    angleRange *
                                    (maxHood - minHood);

            Shot candidate = solve(
                    distanceCm,
                    heightDifferenceCm,
                    yaw,
                    angle,
                    hoodPos
            );

            if (candidate == null) {
                continue;
            }

            if (best == null ||
                    candidate.velocityTPS < best.velocityTPS) {

                best = candidate;
            }
        }

        return best;
    }

    private Shot solve(
            double distance,
            double height,
            double yaw,
            double angleDeg,
            double hoodPos
    ) {

        if (distance < 1.0 ||
                angleDeg <= 0 ||
                angleDeg >= 90) {
            return null;
        }

        double angle = Math.toRadians(angleDeg);

        double cos = Math.cos(angle);
        double sin = Math.sin(angle);

        double denominator =
                2.0 * cos * cos *
                        (distance * Math.tan(angle) - height);

        if (denominator <= 0) {
            return null;
        }

        double ballSpeed = Math.sqrt(
                G * distance * distance / denominator
        );

        double flightTime =
                distance / (ballSpeed * cos);

        double impactVerticalVelocity =
                ballSpeed * sin - G * flightTime;

        if (impactVerticalVelocity >= 0) {
            return null;
        }

        double velocityTPS =
                TPS_PER_CM_S * ballSpeed + TPS_OFFSET;

        if (!Double.isFinite(velocityTPS) ||
                velocityTPS < MIN_TPS ||
                velocityTPS > MAX_TPS) {
            return null;
        }

        return new Shot(
                yaw,
                angleDeg,
                clampHood(hoodPos),
                velocityTPS,
                ballSpeed,
                flightTime
        );
    }

    private double getMinHood() {
        return Math.max(
                0,
                Math.min(1, Math.min(HOOD_MIN_POS, HOOD_MAX_POS))
        );
    }

    private double getMaxHood() {
        return Math.max(
                0,
                Math.min(1, Math.max(HOOD_MIN_POS, HOOD_MAX_POS))
        );
    }

    private double clampHood(double position) {

        return Math.max(
                getMinHood(),
                Math.min(getMaxHood(), position)
        );
    }

    @Override
    public void stop() {

        shooter.off();
        turret.off();
        shooter.closegate();
    }
}
