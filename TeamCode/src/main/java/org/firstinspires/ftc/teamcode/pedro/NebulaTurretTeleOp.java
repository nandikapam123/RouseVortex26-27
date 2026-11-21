package org.firstinspires.ftc.teamcode.pedro;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Nebula Turret", group = "Nebula")
public class NebulaTurretTeleOp extends LinearOpMode {
    // Use the name of the motor port whose encoder socket is wired to the REV encoder.
    // If it shares the turret motor's port, give both constants the same name.
    private static final String ACTUATOR_NAME = "turret";
    private static final String ENCODER_PORT_NAME = "turretEncoder";
    // Set to -1 if the direction check reports reversed motion on the assembled robot.
    private static final int POWER_DIRECTION = 1;

    private static final String PINPOINT_NAME = "pinpoint";
    // Measure from the turret pivot to the center of each tracking wheel.
    // The forward pod's lateral offset is positive to the left.
    // The strafe pod's longitudinal offset is positive toward the robot's front.
    private static final double FORWARD_POD_LEFT_IN = Double.NaN;
    private static final double STRAFE_POD_FORWARD_IN = Double.NaN;
    // Confirm the pod type and that the counts increase going forward (X) and left (Y).
    private static final GoBildaOdometryPods POD_MODEL = GoBildaOdometryPods.goBILDA_4_BAR_POD;
    private static final EncoderDirection X_DIRECTION = EncoderDirection.FORWARD;
    private static final EncoderDirection Y_DIRECTION = EncoderDirection.FORWARD;

    // Place the robot at this known pose before pressing INIT. Keep it still while the IMU calibrates.
    // With the default pose, +X is forward from the start and +Y is left from the start.
    // For another field frame, enter the turret pivot's field X/Y and chassis heading in that frame.
    private static final Pose2D START_POSE = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    // Enter the raised cell opening's horizontal center in the same field frame, in inches.
    // Set both slots if the driver needs to switch cells after the HIVE tips.
    private static final double[][] CELL_TARGETS_IN = {
            {Double.NaN, Double.NaN}, // Slot A, D-pad left
            {Double.NaN, Double.NaN}  // Slot B, D-pad right
    };

    private static final double STICK_DEADBAND = 0.12;
    private static final double SLEW_DEGREES_PER_SECOND = 45.0;
    private static final double MAX_CLEARANCE_DEGREES = 280.0;
    private static final String[] CALIBRATION_STEPS = {
            "With the turret unpowered, point it straight ahead. Press A.",
            "Move left by hand. Stop with slack in every wire and clear of obstructions. Press A.",
            "Move right by hand. Stop with slack in every wire and clear of obstructions. Press A."
    };

    @Override
    public void runOpMode() {
        final Turret turret;
        try {
            turret = new Turret(hardwareMap, ACTUATOR_NAME, ENCODER_PORT_NAME, POWER_DIRECTION);
        } catch (IllegalArgumentException e) {
            telemetry.addLine(e.getMessage());
            telemetry.update();
            return;
        }

        try {
            boolean offsetsSet = Double.isFinite(FORWARD_POD_LEFT_IN) && Double.isFinite(STRAFE_POD_FORWARD_IN);
            GoBildaPinpointDriver pinpoint = offsetsSet
                    ? hardwareMap.tryGet(GoBildaPinpointDriver.class, PINPOINT_NAME) : null;
            if (pinpoint != null) {
                pinpoint.setOffsets(FORWARD_POD_LEFT_IN, STRAFE_POD_FORWARD_IN, DistanceUnit.INCH);
                pinpoint.setEncoderResolution(POD_MODEL);
                pinpoint.setEncoderDirections(X_DIRECTION, Y_DIRECTION);
                pinpoint.resetPosAndIMU();
            }
            String odometrySetup = offsetsSet ? "Configure the Pinpoint as " + PINPOINT_NAME + "."
                    : "Enter the two measured Pinpoint pod offsets in the code.";
            boolean startPoseSet = false;
            int step = 0;
            int[] marks = new int[3];
            boolean previousA = false;
            String calibrationError = "";
            while (opModeInInit()) {
                if (pinpoint != null) {
                    pinpoint.update();
                    DeviceStatus status = pinpoint.getDeviceStatus();
                    if (!startPoseSet && status == DeviceStatus.READY) {
                        pinpoint.setPosition(START_POSE);
                        startPoseSet = true;
                    }
                    telemetry.addData("Pinpoint", status);
                } else {
                    telemetry.addData("Auto aim setup", odometrySetup);
                }
                int ticks = turret.readTicks();
                if (step < marks.length && gamepad1.a && !previousA) {
                    marks[step++] = ticks;
                    if (step == marks.length && !turret.calibrate(marks[0], marks[1], marks[2])) {
                        step = 0;
                        calibrationError = "Check that the encoder moves and leave room for the 8-degree buffer on both sides.";
                    } else {
                        calibrationError = "";
                    }
                }
                previousA = gamepad1.a;
                telemetry.addData("Encoder ticks", ticks);
                if (step < marks.length) {
                    telemetry.addLine(CALIBRATION_STEPS[step]);
                    telemetry.addLine("Watch the wire bundle through the entire sweep for pulling or snagging.");
                    if (step > 0) {
                        double distance = Math.abs(Turret.ticksToDegrees(ticks - marks[0]));
                        telemetry.addData("From center", "%.1f deg (%.0f deg max)", distance, MAX_CLEARANCE_DEGREES);
                    }
                    if (!calibrationError.isEmpty()) {
                        telemetry.addLine(calibrationError);
                    }
                } else {
                    telemetry.addData("Soft limits", "%.1f to %.1f deg", turret.minAngle, turret.maxAngle);
                    telemetry.addLine("Return the turret near center by hand, then press START.");
                }
                telemetry.update();
                idle();
            }

            if (isStopRequested()) {
                return;
            }
            if (turret.calibrated) {
                turret.finishCalibration();
            }
            double target = 0.0;
            long lastLoop = System.nanoTime();
            boolean wasEnabled = false;
            boolean wasAutoAiming = false;
            int cellSlot = 0;
            while (opModeIsActive()) {
                Pose2D pose = null;
                if (pinpoint != null) {
                    pinpoint.update();
                    DeviceStatus status = pinpoint.getDeviceStatus();
                    if (startPoseSet && status == DeviceStatus.READY) {
                        pose = pinpoint.getPosition();
                        telemetry.addData("Pivot X/Y/heading", "%.1f in, %.1f in, %.1f deg",
                                pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH),
                                pose.getHeading(AngleUnit.DEGREES));
                    }
                    telemetry.addData("Pinpoint", status);
                }
                long now = System.nanoTime();
                double elapsed = Math.min((now - lastLoop) * 1e-9, 0.05);
                lastLoop = now;
                if (!turret.calibrated) {
                    telemetry.addLine("Stop and reinitialize to complete calibration.");
                } else {
                    double angle = turret.angleDegrees();
                    boolean enabled = gamepad1.right_bumper && turret.fault.isEmpty();
                    double stick = gamepad1.right_stick_x;
                    boolean manualInput = gamepad1.y || Math.abs(stick) > STICK_DEADBAND;
                    boolean autoAiming = enabled && gamepad1.left_bumper && !manualInput;
                    int selectedSlot = gamepad1.dpad_left ? 0 : gamepad1.dpad_right ? 1 : cellSlot;
                    double[] cell = CELL_TARGETS_IN[selectedSlot];
                    if (autoAiming) {
                        boolean trackingSameCell = wasAutoAiming && selectedSlot == cellSlot;
                        target = turret.aimAtCell(pose, cell[0], cell[1],
                                trackingSameCell ? target : angle, trackingSameCell);
                    } else if (enabled) {
                        if (!wasEnabled || wasAutoAiming) {
                            target = angle;
                        }
                        if (gamepad1.y) {
                            target = 0.0;
                        } else if (manualInput) {
                            target += stick * SLEW_DEGREES_PER_SECOND * elapsed;
                        }
                        target = turret.clampTarget(target);
                    }
                    boolean hasTarget = Double.isFinite(target);
                    turret.update(target, angle, now, enabled && hasTarget);
                    wasEnabled = enabled;
                    wasAutoAiming = autoAiming;
                    cellSlot = selectedSlot;
                    telemetry.addData("Angle", "%.1f deg", angle);
                    if (enabled && hasTarget) {
                        telemetry.addData("Target angle", "%.1f deg", target);
                    }
                    telemetry.addData("Soft limits", "%.1f to %.1f deg", turret.minAngle, turret.maxAngle);
                    telemetry.addLine("Hold RB to enable. Right stick pans. Y centers.");
                    telemetry.addLine("Hold LB with RB to aim. D-pad left/right selects cell A/B.");
                    telemetry.addData("Selected cell", cellSlot == 0 ? "A" : "B");
                    if (autoAiming && !hasTarget) {
                        if (pinpoint == null) {
                            telemetry.addData("Auto aim unavailable", odometrySetup);
                        } else if (!startPoseSet) {
                            telemetry.addLine("Pinpoint wasn't ready during INIT. Restart at the known pose.");
                        } else if (!Double.isFinite(cell[0]) || !Double.isFinite(cell[1])) {
                            telemetry.addLine("Enter the selected cell's X and Y in the code.");
                        } else {
                            telemetry.addLine("Auto aim paused. Check Pinpoint and turret range, then release LB and retry.");
                        }
                    }
                    if (!turret.fault.isEmpty()) {
                        telemetry.addData("Stopped", turret.fault);
                    }
                }
                telemetry.update();
                idle();
            }
        } finally {
            turret.stop();
        }
    }

    private static final class Turret {
        // V1 CAD shows a 10T encoder pinion and 44T turret gears.
        // Check the ratio on the assembled turret. A 10-degree turn should give about 1,001 ticks.
        private static final double TICKS_PER_DEGREE = 8192.0 * (44.0 / 10.0) / 360.0;
        // Keep powered targets inside the cable clearance verified during INIT.
        private static final double LIMIT_MARGIN_DEGREES = 8.0;
        private static final double SLOWDOWN_ZONE_DEGREES = 15.0;
        private static final double POSITION_TOLERANCE_DEGREES = 1.0;
        private static final double KP = 0.018;
        private static final double MIN_POWER = 0.06;
        private static final double MAX_POWER = 0.22;
        private static final double MIN_PROGRESS_DEGREES = 0.5;
        private static final long NO_PROGRESS_NANOS = 500_000_000L;

        private final DcMotor encoder;
        private final DcMotor motor;
        private final CRServo servo;
        private final int powerDirection;
        private int zeroTicks;
        private int encoderDirection;
        private double leftCableLimit;
        private double rightCableLimit;
        private double minAngle;
        private double maxAngle;
        private boolean calibrated;
        private String fault = "";
        private int monitoredDirection;
        private double progressAngle;
        private long progressTime;

        Turret(HardwareMap hardware, String actuatorName, String encoderName, int powerDirection) {
            motor = hardware.tryGet(DcMotor.class, actuatorName);
            servo = motor == null ? hardware.tryGet(CRServo.class, actuatorName) : null;
            if (motor == null && servo == null) {
                throw new IllegalArgumentException("Configure turret as a DC motor or continuous-rotation servo.");
            }
            this.powerDirection = powerDirection;
            stop();
            encoder = hardware.tryGet(DcMotor.class, encoderName);
            if (encoder == null) {
                throw new IllegalArgumentException("Configure the REV encoder's motor port as " + encoderName + ".");
            }
            if (motor != null) {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }

        int readTicks() {
            return encoder.getCurrentPosition();
        }

        static double ticksToDegrees(int delta) {
            return delta / TICKS_PER_DEGREE;
        }

        boolean calibrate(int centerTicks, int leftTicks, int rightTicks) {
            zeroTicks = centerTicks;
            int leftDelta = leftTicks - centerTicks;
            int rightDelta = rightTicks - centerTicks;
            encoderDirection = rightDelta > 0 ? 1 : -1;
            leftCableLimit = Math.max(-MAX_CLEARANCE_DEGREES, encoderDirection * ticksToDegrees(leftDelta));
            rightCableLimit = Math.min(MAX_CLEARANCE_DEGREES, encoderDirection * ticksToDegrees(rightDelta));
            minAngle = leftCableLimit + LIMIT_MARGIN_DEGREES;
            maxAngle = rightCableLimit - LIMIT_MARGIN_DEGREES;
            calibrated = minAngle < -POSITION_TOLERANCE_DEGREES
                    && maxAngle > POSITION_TOLERANCE_DEGREES;
            return calibrated;
        }

        double angleDegrees() {
            return encoderDirection * ticksToDegrees(readTicks() - zeroTicks);
        }

        void finishCalibration() {
            if (motor != null) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }

        double clampTarget(double degrees) {
            return Range.clip(degrees, minAngle, maxAngle);
        }

        double aimAtCell(Pose2D pose, double fieldX, double fieldY, double reference, boolean tracking) {
            if (pose == null || !Double.isFinite(reference)) {
                return Double.NaN;
            }
            double dx = fieldX - pose.getX(DistanceUnit.INCH);
            double dy = fieldY - pose.getY(DistanceUnit.INCH);
            double heading = pose.getHeading(AngleUnit.DEGREES);
            double range = Math.hypot(dx, dy);
            if (!Double.isFinite(range) || range < 1e-6 || !Double.isFinite(heading)) {
                return Double.NaN;
            }

            // Pinpoint headings are positive left. The calibrated turret angles are positive right.
            double bearing = heading - Math.toDegrees(Math.atan2(dy, dx));
            double turns = Math.rint((reference - bearing) / 360.0);
            if (!tracking) {
                double firstLegalTurn = Math.ceil((minAngle - bearing) / 360.0);
                double lastLegalTurn = Math.floor((maxAngle - bearing) / 360.0);
                if (firstLegalTurn > lastLegalTurn) {
                    return Double.NaN;
                }
                turns = Range.clip(turns, firstLegalTurn, lastLegalTurn);
            }
            double candidate = bearing + 360.0 * turns;
            // When tracking, pause at the travel limit. Releasing LB allows a new approach.
            return candidate >= minAngle && candidate <= maxAngle ? candidate : Double.NaN;
        }

        void update(double target, double angle, long now, boolean enabled) {
            if (!fault.isEmpty()) {
                stop();
                return;
            }
            if (angle < leftCableLimit || angle > rightCableLimit) {
                fail("Turret passed a cable limit. Check the wires and reinitialize.");
                return;
            }
            if (!enabled) {
                stop();
                return;
            }

            double error = clampTarget(target) - angle;
            if (Math.abs(error) <= POSITION_TOLERANCE_DEGREES) {
                stop();
                return;
            }

            int direction = error > 0 ? 1 : -1;
            if (direction != monitoredDirection) {
                monitoredDirection = direction;
                progressAngle = angle;
                progressTime = now;
            }
            double progress = direction * (angle - progressAngle);
            if (progress < -MIN_PROGRESS_DEGREES) {
                fail("Reversed motion. Check actuator direction and POWER_DIRECTION.");
                return;
            }
            if (progress >= MIN_PROGRESS_DEGREES) {
                progressAngle = angle;
                progressTime = now;
            } else if (now - progressTime >= NO_PROGRESS_NANOS) {
                fail("No encoder progress. Check wiring and mechanical clearance.");
                return;
            }
            double remainingTravel = direction > 0 ? maxAngle - angle : angle - minAngle;
            double powerLimit = Range.clip(
                    MAX_POWER * remainingTravel / SLOWDOWN_ZONE_DEGREES, MIN_POWER, MAX_POWER);
            double magnitude = Range.clip(KP * Math.abs(error), MIN_POWER, powerLimit);
            setPower(magnitude * direction * powerDirection);
        }

        private void setPower(double power) {
            if (motor != null) {
                motor.setPower(power);
            } else {
                servo.setPower(power);
            }
        }

        private void fail(String reason) {
            fault = reason;
            stop();
        }

        void stop() {
            setPower(0.0);
            monitoredDirection = 0;
        }
    }
}
