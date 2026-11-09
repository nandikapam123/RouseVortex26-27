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
 * (same as Pedro heading). Range is HARD-limited to ±180° for the wires.
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
    public static double HARD_LIMIT    = Math.toRadians(180);  // NEVER beyond this
    public static double SAFETY_MARGIN = Math.toRadians(5);    // aim targets stay within ±175°
    public static double SLOWDOWN_ZONE = Math.toRadians(25);   // power ramps down near a limit
    public static double MIN_LIMIT_POWER = 0.08;

    // ===================== Control gains (tune!) =====================
    public static double kP = 1.2;     // power per rad of error
    public static double kD = 0.03;    // power per rad/s of (target speed - turret speed)
    public static double kV = 0.10;    // feedforward: power per rad/s of target motion
    public static double kS = 0.06;    // static friction / servo deadband
    public static double MAX_POWER = 1.0;
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
    private double goalX = 0, goalY = 0;
    private double holdTarget = 0, manualPower = 0;

    private double target = 0, lastTarget = Double.NaN;
    private double lastAngle, targetRate = 0, lastPower = 0;
    private boolean goalJumped = false, branchJumped = false, unwinding = false;

    /**
     * @param encoderName name of the Control Hub MOTOR PORT whose encoder jack the
     *                    Through Bore is plugged into.
     */
    public Turret(HardwareMap hw, String servoName, String encoderName) {
        servo = hw.get(CRServo.class, servoName);
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
    public void off() { mode = Mode.OFF; }

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
        double rad = (encoder.getCurrentPosition() - zeroTicks) / TICKS_PER_RAD;
        return ENCODER_REVERSED ? -rad : rad;
    }
    public double getTarget()     { return target; }
    public double getPower()      { return lastPower; }
    public Mode getMode()         { return mode; }
    public boolean isUnwinding()  { return unwinding; }
    public boolean isHomed()      { return RobotState.turretZeroValid; }
    public boolean homingFailed() { return homingFailed; }
    public boolean hasHomeSwitch(){ return homeSwitch != null; }
    public boolean isSwitchPressed() {
        return homeSwitch != null && (SWITCH_ACTIVE_LOW != homeSwitch.getState());
    }

    /** Aimed, not swinging around, not homing -> safe to shoot. */
    public boolean isOnTarget() {
        return mode == Mode.TRACK_GOAL && !unwinding && Math.abs(target - getAngle()) < TOLERANCE;
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

        switch (mode) {
            case OFF:     setPower(0); return;
            case MANUAL:  setPower(limitAware(manualPower, current)); return;
            case HOMING:  updateHoming(current); return;
            case HOLD_ANGLE:
                target = Range.clip(holdTarget, -usable(), usable());
                break;
            case TRACK_GOAL:
                target = chooseBranch(robotRelativeAngleToGoal(robotX, robotY, robotHeading), current);
                break;
        }

        // Feedforward: how fast the aim point is moving. Zeroed on jumps (hive switch / unwind).
        boolean jumped = Double.isNaN(lastTarget) || goalJumped || branchJumped;
        double rawRate = jumped ? 0 : (target - lastTarget) / dt;
        targetRate = jumped ? 0 : targetRate + RATE_FILTER * (rawRate - targetRate);
        lastTarget = target;
        goalJumped = branchJumped = false;

        double error = target - current;
        double power = kP * error + kD * (targetRate - velocity) + kV * targetRate;
        if (Math.abs(error) > TOLERANCE) power += kS * Math.signum(error);
        power = Range.clip(power, -MAX_POWER, MAX_POWER);

        if (unwinding && Math.abs(error) < TOLERANCE) unwinding = false;
        setPower(limitAware(power, current));
    }

    // ===================== Aiming math =====================
    private double robotRelativeAngleToGoal(double x, double y, double h) {
        double fwd  =  TURRET_OFFSET_X_MM / 25.4;   // Pedro robot frame: +x forward
        double left = -TURRET_OFFSET_Y_MM / 25.4;   // Pedro robot frame: +y LEFT (yours: - = left)
        double tx = x + fwd * Math.cos(h) - left * Math.sin(h);
        double ty = y + fwd * Math.sin(h) + left * Math.cos(h);
        double fieldAngle = Math.atan2(goalY - ty, goalX - tx);
        return wrap(fieldAngle - h);
    }

    // ===================== Cable management =====================
    private double usable() { return HARD_LIMIT - SAFETY_MARGIN; }

    /**
     * Each aim direction can be written as a, a±360°, ... Keep only copies inside the
     * usable range and take the one closest to the current aim. With ±175° usable there
     * is a 10° blind spot directly behind: there we WAIT at the limit we're already on,
     * and only swing the long way (unwind) once the goal comes out the other side.
     */
    private double chooseBranch(double wrapped, double current) {
        double hi = usable(), lo = -hi;
        double ref = Double.isNaN(lastTarget) ? current : lastTarget;

        double best = Double.NaN, bestDist = Double.POSITIVE_INFINITY;
        for (int k = -2; k <= 2; k++) {
            double c = wrapped + k * 2 * Math.PI;
            if (c < lo || c > hi) continue;
            double d = Math.abs(c - ref);
            if (d < bestDist) { bestDist = d; best = c; }
        }
        if (Double.isNaN(best)) best = (ref >= 0) ? hi : lo;   // blind spot: stay on our side

        if (!Double.isNaN(lastTarget) && Math.abs(best - lastTarget) > Math.PI) {
            unwinding = true;
            branchJumped = true;
        }
        return best;
    }

    /** Never push past ±180°, and slow down as we approach it. Applies to every mode. */
    private double limitAware(double power, double current) {
        if (power == 0) return 0;
        double room = power > 0 ? HARD_LIMIT - current : current + HARD_LIMIT;
        if (room <= 0) return 0;
        if (room < SLOWDOWN_ZONE) {
            double cap = Math.max(MIN_LIMIT_POWER, MAX_POWER * room / SLOWDOWN_ZONE);
            power = Range.clip(power, -cap, cap);
        }
        return power;
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
            double sign = ENCODER_REVERSED ? -1 : 1;
            zeroTicks = encoder.getCurrentPosition() - sign * HOME_SWITCH_ANGLE * TICKS_PER_RAD;
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
        RobotState.turretZeroValid = true;
    }
    private void resetHistory() { lastTarget = Double.NaN; targetRate = 0; unwinding = false; }
    private void setPower(double p) { lastPower = p; servo.setPower(p); }

    private static double wrap(double a) {
        while (a > Math.PI)   a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }
}