package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.pedropathing.math.Pose;
import com.pedropathing.api.PoseFactory;



/** A: shooter on/off, B/Y: power, X: auto-drive to closest shooting spot. */
@TeleOp(name = "Planetary Motor - A Toggle", group = "Motor Tests")
public class PlanetaryMotorTeleOp extends OpMode {
    private static final String MOTOR_NAME = "pm";
    private double MOTOR_POWER = 0.7;

    private DcMotor motor;
    private boolean running;
    private boolean previousA;
    private Follower follower;
    private boolean autoAiming = false;
    private final PoseFactory p = PoseFactory.degrees(); // lets you write headings in degrees
    private final Pose START_POSE = p.of(72, 72, 90);   //change this to where out bot starts at init

    // PLACEHOLDERS, replace with real x y & headings measured on the field
    private final Pose[] shootPoses = {
            p.of(18, 126, 135),
            p.of(24, 120, 135),
            p.of(30, 114, 135),
            p.of(14, 116, 125),
            p.of(28, 128, 145),
    };
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, MOTOR_NAME);
        motor.setPower(0);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        running = false;

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

        if (currentA && !previousA) {
            running = !running;
        }
        previousA = currentA;

        motor.setPower(running ? MOTOR_POWER : 0);
        telemetry.addData("Motor", running ? "ON" : "OFF");
        telemetry.addData("Power", running ? MOTOR_POWER : 0);
        telemetry.addLine("A: shooter on/off | B/Y: power | X: auto-aim");
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
        if (motor != null) {
            motor.setPower(0);
        }
    }
}
