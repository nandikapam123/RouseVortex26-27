package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.math.Pose;
import com.pedropathing.api.PoseFactory;



/** A: shooter on/off, B/Y: power, Dpad U/D: servo angle, X: auto-drive to closest shooting spot, RT/LT: intake in/out, RB: lower intake up/down. */
@TeleOp(name = "Planetary Motor - A Toggle", group = "Motor Tests")
public class PlanetaryMotorTeleOp extends OpMode {
    private static final String MOTOR_NAME = "pm";
    private static final String intake = "intake";
    private double MOTOR_POWER = 0.7;

    // ---------- Servo (angle) ----------
    private static final String SERVO_NAME = "hood";     // must match the hardware config name
    private static final double SERVO_RANGE_DEG = 300;   // servo's full range: 0.0 = 0°, 1.0 = 300°
    private double servoPos = 0.5;                       // 0.0 to 1.0, start in the middle

    // ---------- Lower intake servo (flips between UP and DOWN) ----------
    private static final String LOWER_INTAKE_NAME = "lowerIntake"; // must match the hardware config name
    private static final double LOWER_RANGE_DEG = 300;  // servo's full range: 0.0 = 0°, 1.0 = 300°
    private static final double LOWER_UP_DEG = 0;       // stowed
    private static final double LOWER_DOWN_DEG = 100;   // down to grab pollen
    private boolean lowerDown = false;

    private DcMotor shoot;
    private Servo lowerIntake;
    private DcMotor INTAKE;
    private Servo servo;
    private boolean running;
    private boolean previousA;
    private Follower follower;
    private boolean autoAiming = false;
    private final PoseFactory p = PoseFactory.degrees(); // lets you write headings in degrees
    private final Pose START_POSE = p.of(72, 8, 90);   //change this to where out bot starts at init

    // PLACEHOLDERS, replace with real x y & headings measured on the field
    private final Pose[] shootPoses = {
            p.of(39, 118, 315),
            p.of(39,22,50 )
//            p.of(24, 120, 135),
//            p.of(30, 114, 135),
//            p.of(14, 116, 125),
//            p.of(28, 128, 145),
    };
    @Override
    public void init() {
        shoot = hardwareMap.get(DcMotor.class, MOTOR_NAME);
        INTAKE = hardwareMap.get(DcMotor.class, intake);
        servo = hardwareMap.get(Servo.class, SERVO_NAME);


        shoot.setPower(0);
        shoot.setDirection(DcMotor.Direction.FORWARD);
        shoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        running = false;

        servo.setPosition(servoPos);

        lowerIntake = hardwareMap.get(Servo.class, LOWER_INTAKE_NAME);
        lowerIntake.setPosition(LOWER_UP_DEG / LOWER_RANGE_DEG); // start stowed

        follower = Constants.create(hardwareMap);
        follower.setPose(START_POSE);

        telemetry.addLine("Ready. Place bot at START_POSE, then press START.");
        telemetry.update();
    }

    @Override
    public void start() {
        running = false;
        previousA = gamepad1.a;
    }

    @Override
    public void loop() {

        // Hold RT to intake, hold LT to reverse (clear jams / spit out extras)
        if (gamepad1.right_trigger > 0.2) {
            INTAKE.setPower(0.8);
        } else if (gamepad1.left_trigger > 0.2) {
            INTAKE.setPower(-0.8);
        } else {
            INTAKE.setPower(0);
        }

        if (gamepad1.xWasPressed()) {
            follower.hold(closestShootPose());
            autoAiming = true;
        }

        boolean touchingSticks = Math.abs(gamepad1.left_stick_x) > 0.15
                || Math.abs(gamepad1.left_stick_y) > 0.15
                || Math.abs(gamepad1.right_stick_x) > 0.15;
        if (touchingSticks) autoAiming = false; // driver takes over

        if (!autoAiming) {
            follower.manual(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
        follower.update();

        boolean currentA = gamepad1.a;
        // toggled once per press, even if A stays held down.

        if (gamepad1.bWasPressed()){
            MOTOR_POWER += 0.01;
        }
        if (gamepad1.yWasPressed()){
            MOTOR_POWER -= 0.01;
        }

        // Dpad up/down: adjust servo angle
        if (gamepad1.dpadUpWasPressed())   servoPos += 0.01;
        if (gamepad1.dpadDownWasPressed()) servoPos -= 0.01;
        servoPos = Math.max(0, Math.min(1, servoPos));   // servo only accepts 0.0 to 1.0
        servo.setPosition(servoPos);

        // RB: flip lower intake between UP and DOWN
        if (gamepad1.rightBumperWasPressed()) lowerDown = !lowerDown;
        double lowerDeg = lowerDown ? LOWER_DOWN_DEG : LOWER_UP_DEG;
        lowerIntake.setPosition(lowerDeg / LOWER_RANGE_DEG); // degrees -> 0.0-1.0

        if (currentA && !previousA) {
            running = !running;
        }
        previousA = currentA;

        shoot.setPower(running ? MOTOR_POWER : 0);
        telemetry.addData("Motor", running ? "ON" : "OFF");
        telemetry.addData("Power", running ? MOTOR_POWER : 0);
        // commanded position, not measured (servos don't report back)
        telemetry.addData("Servo pos", "%.2f", servoPos);
        telemetry.addData("Servo angle", "%.1f°", servoPos * SERVO_RANGE_DEG);
        telemetry.addData("Lower intake", "%s (%.0f°)", lowerDown ? "DOWN" : "UP", lowerDeg);
        telemetry.addLine("A: shooter | B/Y: power | Dpad U/D: angle | X: auto-aim | RT/LT: intake | RB: lower intake");
        Pose pose = follower.pose();
        telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Auto-aim", autoAiming);
        // shows which spot it chose:
        Pose target = closestShootPose();
        telemetry.addData("Nearest spot", "x=%.1f y=%.1f", target.x(), target.y());
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
        running = false;
        if (shoot != null) {
            shoot.setPower(0);
        }
        if (INTAKE != null) {
            INTAKE.setPower(0);
        }
    }
}