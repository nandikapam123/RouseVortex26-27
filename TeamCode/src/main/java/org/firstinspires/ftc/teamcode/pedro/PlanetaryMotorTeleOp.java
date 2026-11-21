package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Main driver TeleOp. Field-centric drive, shooter, intake, flower ramp, hood, and turret.
 *
 * Gamepad 1 (PlayStation names / Xbox names):
 *   Left stick          drive (field-centric)
 *   Right stick X       turn
 *   R2 / RT (hold)      shoot: opens the servo blocker so pollen reaches the flywheel
 *                       (the flywheel spins the whole match)
 *   L2 / LT (hold)      intake flowers
 *   Circle / B (hold)   reverse intake
 *   R1 / RB             flower ramp up/down
 *   L1 / LB             slow mode on/off (70% max speed, for parking)
 *   Square / X          turret auto-aim on/off (toggle off/on to restart if aim pauses at a limit)
 *   D-pad up/down       turret Hive cell: up = raised cell, down = lower cell
 *   D-pad right/left    hood angle up/down
 *   Options / Start     auto-drive to the closest shooting spot (move a stick to cancel)
 *   Triangle / Y        shooter power +0.01
 *   Cross / A           shooter power -0.01
 *   Share / Back        reset field-centric "forward" to the way the robot faces now
 *
 * Turret buttons come from TurretController.updateTeleOp. During INIT (Gamepad 1):
 * A marks turret calibration, X redoes it, D-pad left/right picks the alliance,
 * B confirms the practice start pose.
 */
@TeleOp(name = "Driver TeleOp")
public class PlanetaryMotorTeleOp extends OpMode {
    // Flywheel ("pm") + gate ("blocker") live in Shooter; intake ("intake") + flower ramp
    // ("lowerIntake") live in Intake. Auton uses the same classes, so names and positions match.
    private double MOTOR_POWER = 0.7;
    private static final double INTAKE_POWER = 0.8;

    // ---------- Drive ----------
    private static final double SLOW_SPEED = 0.7;        // slow mode = 30% slower than max
    // Field heading the driver treats as "forward" on the stick. 0 deg = the way the robot
    // faces at the Auton / practice start pose, away from the alliance wall.
    // Share/Back resets this during the match.
    private static final double DRIVER_FORWARD_HEADING = Math.toRadians(0);
    private double driverForward = DRIVER_FORWARD_HEADING;
    private boolean slowMode = false;

    // ---------- Hood servo (angle) ----------
    private static final String SERVO_NAME = "hood";     // must match the hardware config name
    private static final double SERVO_RANGE_DEG = 300;   // servo's full range: 0.0 = 0°, 1.0 = 300°
    private double servoPos = 0.5;                       // 0.0 to 1.0, start in the middle

    // ---------- Flower ramp (flips between UP and DOWN) ----------
    private boolean lowerDown = false;

    private Shooter shooter;
    private Intake intake;
    private Servo servo;
    private Follower follower;
    private TurretController turret;
    private boolean autoDriving = false;
    private final PoseFactory p = PoseFactory.degrees(); // lets you write headings in degrees

    // PLACEHOLDERS, replace with real x y & headings measured on the field
    private final Pose[] shootPoses = {
            p.of(39, 118, 315),
            p.of(39, 22, 50)
    };

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap); // flywheel off, gate closed
        intake = new Intake(hardwareMap);   // intake off, flower ramp up
        lowerDown = false;

        servo = hardwareMap.get(Servo.class, SERVO_NAME);
        servo.setPosition(servoPos);

        follower = Constants.create(hardwareMap);
        turret = new TurretController(hardwareMap);
        // Starts from Auton's end pose if Auton just ran, otherwise the practice start pose.
        turret.seedTeleOpPose(follower);
    }

    @Override
    public void init_loop() {
        // Gamepad 1: alliance (D-pad L/R), confirm practice pose (B), turret calibration (A, X redo).
        turret.teleOpInit(gamepad1, telemetry);
        telemetry.update();
    }

    @Override
    public void start() {
        turret.startTeleOp();
        driverForward = DRIVER_FORWARD_HEADING;
        slowMode = false;
        autoDriving = false;
    }

    @Override
    public void loop() {
        // ---------- Drive ----------
        if (gamepad1.leftBumperWasPressed()) slowMode = !slowMode;
        if (gamepad1.backWasPressed()) driverForward = follower.pose().heading();

        if (gamepad1.startWasPressed()) {
            follower.hold(closestShootPose());
            autoDriving = true;
        }

        double forward = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        boolean touchingSticks = Math.abs(forward) > 0.15 || Math.abs(lateral) > 0.15
                || Math.abs(turn) > 0.15;
        if (touchingSticks) autoDriving = false; // driver takes over

        if (!autoDriving) {
            double speed = slowMode ? SLOW_SPEED : 1.0;
            DrivePowers powers = ManualDrive.fieldCentric(
                    forward * speed,
                    lateral * speed,
                    turn * speed,
                    follower.pose().heading() - driverForward
            );
            follower.manual(powers);
        }
        follower.update();

        // ---------- Intake: L2 in, Circle reverse (reverse wins if both held) ----------
        if (gamepad1.b) {
            intake.setPower(-INTAKE_POWER);
        } else if (gamepad1.left_trigger > 0.2) {
            intake.setPower(INTAKE_POWER);
        } else {
            intake.setPower(0);
        }

        // ---------- Flower ramp: R1 flips UP/DOWN ----------
        if (gamepad1.rightBumperWasPressed()) lowerDown = !lowerDown;
        intake.setRampDown(lowerDown);
        double lowerDeg = lowerDown ? Intake.RAMP_DOWN_DEG : Intake.RAMP_UP_DEG;

        // ---------- Shooter: flywheel always spins, hold R2 to open the blocker ----------
        if (gamepad1.yWasPressed()) MOTOR_POWER += 0.01;
        if (gamepad1.aWasPressed()) MOTOR_POWER -= 0.01;
        MOTOR_POWER = Math.max(0, Math.min(1, MOTOR_POWER));
        shooter.setFlywheelPower(MOTOR_POWER);
        boolean shooting = gamepad1.right_trigger > 0.2;
        shooter.setGateOpen(shooting);

        // ---------- Hood: D-pad right/left (D-pad up/down belongs to the turret) ----------
        if (gamepad1.dpadRightWasPressed()) servoPos += 0.01;
        if (gamepad1.dpadLeftWasPressed())  servoPos -= 0.01;
        servoPos = Math.max(0, Math.min(1, servoPos));   // servo only accepts 0.0 to 1.0
        servo.setPosition(servoPos);

        // ---------- Turret: Square toggles auto-aim, D-pad up/down picks the cell ----------
        Pose pose = follower.pose();
        turret.updateTeleOp(pose, gamepad1, telemetry);

        // ---------- Telemetry ----------
        telemetry.addData("Flywheel power", "%.2f", MOTOR_POWER);
        telemetry.addData("Blocker", shooting ? "OPEN (shooting)" : "CLOSED");
        telemetry.addData("Speed", slowMode ? "SLOW (70%)" : "FULL");
        telemetry.addData("Flower ramp", "%s (%.0f°)", lowerDown ? "DOWN" : "UP", lowerDeg);
        // commanded position, not measured (servos don't report back)
        telemetry.addData("Hood", "%.2f (%.1f°)", servoPos, servoPos * SERVO_RANGE_DEG);
        telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Auto-drive", autoDriving);
        Pose target = closestShootPose();
        telemetry.addData("Nearest spot", "x=%.1f y=%.1f", target.x(), target.y());
        telemetry.addLine("R2 shoot | L2 intake | O reverse | R1 ramp | L1 slow | □ turret aim | D-pad U/D cell | D-pad L/R hood | Options auto-drive | △/X power | Share reset fwd");
        telemetry.update();
    }

    private Pose closestShootPose() {
        Pose current = follower.pose();
        Pose best = shootPoses[0];
        double bestDist = Double.MAX_VALUE;

        for (Pose spot : shootPoses) {
            double dist = Math.hypot(spot.x() - current.x(), spot.y() - current.y());
            if (dist < bestDist) {
                bestDist = dist;
                best = spot;
            }
        }
        return best;
    }

    @Override
    public void stop() {
        if (shooter != null) shooter.stop();
        if (intake != null) intake.stop();
        if (turret != null) turret.stop();
    }
}