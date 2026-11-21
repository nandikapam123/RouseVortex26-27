package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Field-centric auto-aiming turret with cable-safe limits and optional homing.
 *
 * Gear train:
 *   CR servo -> 44T gear "A"
 *   A meshes 10T on the encoder   -> encoder turns 44/10 = 4.4x per turret turn
 *   A meshes 44T on the turret    -> servo : turret = 1 : 1
 *   Encoder and turret spin the same direction (each mesh flips once).
 *
 * Angle convention: radians, 0 = turret facing intake/front, CCW positive
 * (same as Pedro heading). Measured cable clearance is capped at ±280°.
 */
public class Turret {

    // ===================== Gear train / sensors =====================
    public static double ENCODER_CPR = 8192;                       // REV Through Bore quadrature
    public static double ENCODER_REVS_PER_TURRET_REV = 44.0 / 10.0;
    private static final double TICKS_PER_RAD =
            ENCODER_CPR * ENCODER_REVS_PER_TURRET_REV / (2 * Math.PI);

    public static boolean ENCODER_REVERSED = false;  // flip so turret CCW = angle goes UP
    public static boolean SERVO_REVERSED   = false;  // flip so +power = turret CCW

    // ===================== Cable limits =====================
    public static final double HARD_LIMIT = Math.toRadians(280);
    public static final double SAFETY_MARGIN = Math.toRadians(8);
    public static double SLOWDOWN_ZONE = Math.toRadians(25);   // power ramps down near a limit
    public static double MIN_LIMIT_POWER = 0.08;

    // ===================== Control gains (tune!) =====================
    public static double kP = 1.2;     // power per rad of error
    public static double kD = 0.03;    // power per rad/s of (target speed - turret speed)
    public static double kV = 0.10;    // feedforward: power per rad/s of target motion
    public static double kS = 0.06;    // static friction / servo deadband
    public static double MAX_POWER = 0.22;
    public static double TOLERANCE = Math.toRadians(1.5);
    public static double RATE_FILTER = 0.3;

    // ===================== Mounting (YOUR convention, millimeters) =====================
    // + forward / - back,  + right / - left   (converted to Pedro inches below)
    public static double TURRET_OFFSET_X_MM = -95.9;
    public static double TURRET_OFFSET_Y_MM = -57.4;

    // ===================== Homing (optional switch) =====================
    public static boolean USE_HOME_SWITCH = true;      // false = trust the alignment mark only
    public static String  HOME_SWITCH_NAME = "turretHome";
    public static boolean SWITCH_ACTIVE_LOW = true;    // REV magnetic limit switch reads LOW at magnet
    public static double  HOME_SWITCH_ANGLE = 0;       // turret angle where switch trips (approaching CCW)
    public static double  HOMING_WINDOW = Math.toRadians(30);  // search ±30° around the mark
    public static double  HOMING_FAST = 0.30, HOMING_SLOW = 0.12;
    public static double  HOMING_TIMEOUT = 3.0;        // seconds

    public enum Mode { HOMING, TRACK_GOAL, HOLD_ANGLE, MANUAL, OFF }
    private enum HomeStep { BACK_OFF, CREEP }

    private final CRServo servo;
    private final DcMotorEx encoder;
    private DigitalChannel homeSwitch = null;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime homingTimer = new ElapsedTime();

    private Mode mode = Mode.HOLD_ANGLE;
    private Mode afterHoming = Mode.HOLD_ANGLE;
    private HomeStep homeStep = HomeStep.BACK_OFF;
    private boolean homingFailed = false;

    private double zeroTicks;
    private int encoderSign;
    private double lowerClearance = -HARD_LIMIT;
    private double upperClearance = HARD_LIMIT;
    private double goalX = 0, goalY = 0;
    private double holdTarget = 0, manualPower = 0;

    private double target = 0, lastTarget = Double.NaN;
    private double lastAngle, targetRate = 0, lastPower = 0;
    private boolean goalJumped = false;
    private boolean aimLimitReached = false;
    private String fault = "";
    private long progressTime;
    private double progressAngle;
    private int powerSign;
    private static final long NO_PROGRESS_NS = 500_000_000L;
    private static final double MIN_PROGRESS = Math.toRadians(0.5);

    /**
     * @param encoderName name of the Control Hub MOTOR PORT whose encoder jack the
     *                    Through Bore is plugged into.
     */
    public Turret(HardwareMap hw, String servoName, String encoderName) {
        servo = hw.get(CRServo.class, servoName);
        servo.setPower(0);
        servo.setDirection(SERVO_REVERSED ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        encoder = hw.get(DcMotorEx.class, encoderName);

        if (USE_HOME_SWITCH) {
            homeSwitch = hw.tryGet(DigitalChannel.class, HOME_SWITCH_NAME);
            if (homeSwitch != null) homeSwitch.setMode(DigitalChannel.Mode.INPUT);
        }

        // Reuse Auto's zero if we have one; otherwise assume the turret is on the center mark.
        zeroTicks = RobotState.turretZeroValid ? RobotState.turretZeroTicks
                : encoder.getCurrentPosition();
        encoderSign = RobotState.turretZeroValid ? RobotState.turretEncoderSign
                : ENCODER_REVERSED ? -1 : 1;
        if (hasCableCalibration()) {
            lowerClearance = RobotState.turretLowerClearance;
            upperClearance = RobotState.turretUpperClearance;
        }
        lastAngle = getAngle();
        timer.reset();
    }

    // ===================== Commands =====================
    public void setGoal(double x, double y) {
        if (Math.hypot(x - goalX, y - goalY) > 0.01) goalJumped = true;   // e.g. hive switched
        goalX = x; goalY = y;
    }
    public void setGoal(double[] xy) { setGoal(xy[0], xy[1]); }

    public void trackGoal() { setMode(Mode.TRACK_GOAL); }
    public void holdAngle(double rad) { holdTarget = rad; setMode(Mode.HOLD_ANGLE); }
    public void center() { holdAngle(0); }
    public void manual(double power) { manualPower = power; mode = Mode.MANUAL; resetHistory(); } // cancels homing
    public void off() {
        mode = Mode.OFF;
        powerSign = 0;
        setPower(0);
    }

    public int encoderTicks() { return encoder.getCurrentPosition(); }

    public double distanceFromMark(int markTicks) {
        return Math.abs((encoderTicks() - markTicks) / TICKS_PER_RAD);
    }

    public boolean hasCableCalibration() {
        return RobotState.turretZeroValid && Double.isFinite(RobotState.turretLowerClearance)
                && Double.isFinite(RobotState.turretUpperClearance);
    }

    public void clearCableCalibration() {
        off();
        RobotState.turretZeroValid = false;
        RobotState.turretLowerClearance = Double.NaN;
        RobotState.turretUpperClearance = Double.NaN;
    }

    public boolean calibrateCableClearance(int center, int left, int right) {
        int leftDelta = left - center;
        int rightDelta = right - center;
        int sign = leftDelta > 0 ? 1 : -1;
        double upper = Math.min(HARD_LIMIT, sign * leftDelta / TICKS_PER_RAD);
        double lower = Math.max(-HARD_LIMIT, sign * rightDelta / TICKS_PER_RAD);
        if (lower + SAFETY_MARGIN >= -TOLERANCE || upper - SAFETY_MARGIN <= TOLERANCE) {
            return false;
        }
        zeroTicks = center;
        encoderSign = sign;
        lowerClearance = RobotState.turretLowerClearance = lower;
        upperClearance = RobotState.turretUpperClearance = upper;
        acceptZero();
        lastAngle = getAngle();
        resetHistory();
        return true;
    }

    /** Find the exact zero with the switch. With no switch, just accepts the mark as zero. */
    public void home() {
        if (homeSwitch == null) { acceptZero(); return; }
        if (mode != Mode.HOMING) afterHoming = (mode == Mode.TRACK_GOAL) ? Mode.TRACK_GOAL : Mode.HOLD_ANGLE;
        mode = Mode.HOMING;
        homeStep = HomeStep.BACK_OFF;
        homingFailed = false;
        homingTimer.reset();
    }

    /** Only call when the turret is physically on the center mark. */
    public void zeroHere() {
        zeroTicks = encoder.getCurrentPosition();
        acceptZero();
        resetHistory();
    }

    // ===================== State =====================
    public double getAngle() {
        return encoderSign * (encoderTicks() - zeroTicks) / TICKS_PER_RAD;
    }
    public double getTarget()     { return target; }
    public double getPower()      { return lastPower; }
    public Mode getMode()         { return mode; }
    public String getFault()     { return fault; }
    public boolean isHomed()      { return RobotState.turretZeroValid; }
    public boolean homingFailed() { return homingFailed; }
    public boolean hasHomeSwitch(){ return homeSwitch != null; }
    public boolean isSwitchPressed() {
        return homeSwitch != null && (SWITCH_ACTIVE_LOW != homeSwitch.getState());
    }

    /** Aimed, not swinging around, not homing -> safe to shoot. */
    public boolean isOnTarget() {
        return mode == Mode.TRACK_GOAL && fault.isEmpty() && Math.abs(target - getAngle()) < TOLERANCE;
    }

    // ===================== Main loop (call every loop, after follower.update()) =====================
    /**
     * Pass Pedro's pose as plain numbers, e.g.
     *   turret.update(pose.getX(), pose.getY(), pose.getHeading());
     * x/y in inches (Pedro field coords), heading in radians.
     */
    public void update(double robotX, double robotY, double robotHeading) {
        double dt = Math.max(timer.seconds(), 1e-3);
        timer.reset();
        double current = getAngle();
        double velocity = (current - lastAngle) / dt;
        lastAngle = current;

        if (current < lowerClearance || current > upperClearance) {
            fault = "Past a marked cable limit. Check the wires before restarting.";
        }
        if (!fault.isEmpty()) {
            off();
            return;
        }
        switch (mode) {
            case OFF:     setPower(0); return;
            case MANUAL:  setPower(limitAware(manualPower, current)); return;
            case HOMING:  updateHoming(current); return;
            case HOLD_ANGLE:
                target = Range.clip(holdTarget, lowerTargetLimit(), upperTargetLimit());
                break;
            case TRACK_GOAL:
                target = chooseBranch(robotRelativeAngleToGoal(robotX, robotY, robotHeading), current);
                break;
        }

        if (!Double.isFinite(target)) {
            targetRate = 0;
            powerSign = 0;
            setPower(0);
            return;
        }
        // Zero feedforward when acquiring a new target or switching cells.
        boolean jumped = Double.isNaN(lastTarget) || goalJumped;
        double rawRate = jumped ? 0 : (target - lastTarget) / dt;
        targetRate = jumped ? 0 : targetRate + RATE_FILTER * (rawRate - targetRate);
        lastTarget = target;
        goalJumped = false;

        double error = target - current;
        if (mode == Mode.HOLD_ANGLE && Math.abs(error) <= TOLERANCE) {
            powerSign = 0;
            setPower(0);
            return;
        }
        double power = kP * error + kD * (targetRate - velocity) + kV * targetRate;
        if (Math.abs(error) > TOLERANCE) power += kS * Math.signum(error);
        power = Range.clip(power, -MAX_POWER, MAX_POWER);

        driveWithEncoderCheck(limitAware(power, current), current);
    }

    // ===================== Aiming math =====================
    private double robotRelativeAngleToGoal(double x, double y, double h) {
        double fwd  =  TURRET_OFFSET_X_MM / 25.4;   // Pedro robot frame: +x forward
        double left = -TURRET_OFFSET_Y_MM / 25.4;   // Pedro robot frame: +y LEFT (yours: - = left)
        double tx = x + fwd * Math.cos(h) - left * Math.sin(h);
        double ty = y + fwd * Math.sin(h) + left * Math.cos(h);
        double distance = Math.hypot(goalX - tx, goalY - ty);
        if (!Double.isFinite(distance) || distance < 1e-6 || !Double.isFinite(h)) {
            return Double.NaN;
        }
        double fieldAngle = Math.atan2(goalY - ty, goalX - tx);
        return fieldAngle - h;
    }

    // ===================== Cable management =====================
    private double lowerTargetLimit() { return lowerClearance + SAFETY_MARGIN; }
    private double upperTargetLimit() { return upperClearance - SAFETY_MARGIN; }

    /**
     * Acquire the nearest reachable angle, then follow that angle continuously through
     * the heading wrap. Pause at a cable limit until the angle comes back into range.
     */
    private double chooseBranch(double bearing, double current) {
        double hi = upperTargetLimit(), lo = lowerTargetLimit();
        boolean acquire = Double.isNaN(lastTarget) || goalJumped;
        if (aimLimitReached && !acquire) return Double.NaN;
        double reference = acquire ? current : lastTarget;
        double turn = 2 * Math.PI;
        double turns = Math.rint((reference - bearing) / turn);
        if (acquire) {
            double first = Math.ceil((lo - bearing) / turn);
            double last = Math.floor((hi - bearing) / turn);
            if (first > last) return Double.NaN;
            turns = Range.clip(turns, first, last);
        }
        double candidate = bearing + turns * turn;
        if (candidate >= lo && candidate <= hi) {
            aimLimitReached = false;
            return candidate;
        }
        aimLimitReached = !acquire && Double.isFinite(candidate);
        return Double.NaN;
    }

    /** Slow down toward the buffered target limits. */
    private double limitAware(double power, double current) {
        if (power == 0) return 0;
        double room = power > 0 ? upperTargetLimit() - current : current - lowerTargetLimit();
        if (room <= 0) return 0;
        if (room < SLOWDOWN_ZONE) {
            double cap = Math.max(MIN_LIMIT_POWER, MAX_POWER * room / SLOWDOWN_ZONE);
            power = Range.clip(power, -cap, cap);
        }
        return power;
    }

    private void driveWithEncoderCheck(double power, double current) {
        int direction = (int) Math.signum(power);
        long now = System.nanoTime();
        if (direction != powerSign || direction == 0) {
            powerSign = direction;
            progressTime = now;
            progressAngle = current;
        } else {
            double progress = direction * (current - progressAngle);
            if (progress < -MIN_PROGRESS) {
                fault = "Turret moved the wrong way. Check servo direction.";
                off();
                return;
            }
            if (progress >= MIN_PROGRESS) {
                progressTime = now;
                progressAngle = current;
            } else if (now - progressTime >= NO_PROGRESS_NS && Math.abs(target - current) > TOLERANCE) {
                fault = "No turret encoder movement. Check wiring and clearance.";
                off();
                return;
            }
        }
        setPower(power);
    }

    // ===================== Homing =====================
    /**
     * Turret starts roughly on the center mark. Back off to -30°, then creep CCW until the
     * switch trips (always approaching from the same side = repeatable). Never leaves ±30°,
     * so a broken switch can't wind the cables. Fails safe to the mark-based zero.
     */
    private void updateHoming(double current) {
        if (homingTimer.seconds() > HOMING_TIMEOUT) { finishHoming(false); return; }

        if (homeStep == HomeStep.BACK_OFF) {
            double err = -HOMING_WINDOW - current;
            if (Math.abs(err) < Math.toRadians(2)) { homeStep = HomeStep.CREEP; setPower(0); return; }
            setPower(Range.clip(kP * err + kS * Math.signum(err), -HOMING_FAST, HOMING_FAST));
            return;
        }

        // CREEP
        if (isSwitchPressed()) {
            zeroTicks = encoderTicks() - encoderSign * HOME_SWITCH_ANGLE * TICKS_PER_RAD;
            finishHoming(true);
            return;
        }
        if (current > HOMING_WINDOW) { finishHoming(false); return; }
        setPower(HOMING_SLOW);
    }

    private void finishHoming(boolean ok) {
        homingFailed = !ok;
        if (ok) acceptZero();          // failed -> keep mark-based zero, retry next OpMode
        setPower(0);
        lastAngle = getAngle();
        mode = afterHoming;
        resetHistory();
    }

    // ===================== Helpers =====================
    private void setMode(Mode m) {
        if (mode == Mode.HOMING) { afterHoming = m; return; }   // finish homing first
        if (mode != m) resetHistory();
        mode = m;
    }
    private void acceptZero() {
        RobotState.turretZeroTicks = zeroTicks;
        RobotState.turretEncoderSign = encoderSign;
        RobotState.turretZeroValid = true;
    }
    private void resetHistory() {
        lastTarget = Double.NaN;
        targetRate = 0;
        powerSign = 0;
        aimLimitReached = false;
    }
    private void setPower(double p) { lastPower = p; servo.setPower(p); }

}
