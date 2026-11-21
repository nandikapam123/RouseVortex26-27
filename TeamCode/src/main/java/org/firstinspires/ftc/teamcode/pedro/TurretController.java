package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.HiveTargets;
import org.firstinspires.ftc.teamcode.subsystems.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class TurretController {
    private static final String SERVO_NAME = "turret";
    private static final String ENCODER_NAME = "turretEncoder";

    private final Turret turret;
    private final GoBildaPinpointDriver pinpoint;
    private final HiveTargets hives = new HiveTargets();
    private final TurretCalibration calibration = new TurretCalibration();
    private boolean previousX;
    private boolean fromAuto;
    private boolean knownFieldPose;
    private boolean aimEnabled = true;
    private Boolean upperCell;

    public TurretController(HardwareMap hardwareMap) {
        Turret configuredTurret;
        try {
            configuredTurret = new Turret(hardwareMap, SERVO_NAME, ENCODER_NAME);
            configuredTurret.off();
        } catch (IllegalArgumentException e) {
            configuredTurret = null;
        }
        turret = configuredTurret;
        // Pedro owns the updates and settings for this same Pinpoint.
        pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
    }

    public void calibrateInInit(Gamepad gamepad, Telemetry telemetry) {
        calibration.update(turret, gamepad, telemetry);
        previousX = gamepad.x;
    }

    void centerAtStart() {
        if (turret == null) return;
        if (!turret.hasCableCalibration()) {
            turret.restoreCableCalibrationFromSwitch();
        }
        if (canCenter()) {
            turret.center();
        }
    }

    boolean canCenter() {
        return turret != null && turret.hasCableCalibration() && turret.getFault().isEmpty();
    }

    boolean isCentered() {
        return canCenter() && turret.getMode() == Turret.Mode.HOLD_ANGLE
                && Math.abs(turret.getAngle()) <= Turret.TOLERANCE;
    }

    public void updateHold(Telemetry telemetry) {
        if (turret != null && turret.hasCableCalibration()) {
            turret.update(0, 0, 0);
        }
        showTurret(telemetry);
    }

    public void seedTeleOpPose(Follower follower) {
        Pose end = storePose.autonomousEndPose;
        fromAuto = end != null;
        knownFieldPose = fromAuto;
        follower.setPose(fromAuto ? end : storePose.PRACTICE_START_POSE);
        follower.update();
    }

    public void teleOpInit(Gamepad gamepad, Telemetry telemetry) {
        if (gamepad.dpad_left) RobotState.alliance = HiveTargets.Alliance.RED;
        if (gamepad.dpad_right) RobotState.alliance = HiveTargets.Alliance.BLUE;
        if (gamepad.b) knownFieldPose = true;
        telemetry.addData("Alliance (D-pad left/right)", RobotState.alliance);
        if (!knownFieldPose) {
            Pose start = storePose.PRACTICE_START_POSE;
            telemetry.addData("Practice pose X/Y/heading", "%.1f in / %.1f in / %.0f deg",
                    start.x(), start.y(), Math.toDegrees(start.heading()));
            telemetry.addLine("Place the robot at this pose, then press B to enable field aiming.");
        } else {
            telemetry.addData("Starting pose", fromAuto ? "From Auton" : "Practice pose confirmed");
        }
        calibrateInInit(gamepad, telemetry);
    }

    public void startTeleOp() {
        if (fromAuto) storePose.autonomousEndPose = null;
    }

    public void updateTeleOp(Pose pose, Gamepad gamepad, Telemetry telemetry) {
        if (gamepad.x && !previousX) aimEnabled = !aimEnabled;
        previousX = gamepad.x;
        if (gamepad.dpad_up) upperCell = true;
        if (gamepad.dpad_down) upperCell = false;
        if (turret != null && turret.hasCableCalibration()) {
            if (aimEnabled && knownFieldPose && canUsePose(pose)) {
                double[] cell = upperCell == null
                        ? hives.select(pose.x(), pose.y(), RobotState.alliance)
                        : hives.cell(upperCell, RobotState.alliance);
                turret.setGoal(cell);
                turret.trackGoal();
                telemetry.addData("Hive target", "%.1f in / %.1f in", cell[0], cell[1]);
            } else {
                turret.off();
            }
            turret.update(pose.x(), pose.y(), pose.heading());
        }
        telemetry.addData("Auto-aim (X toggles)", aimEnabled ? "On" : "Off");
        if (!knownFieldPose) {
            telemetry.addLine("Auto-aim needs Auton's end pose or a confirmed practice pose.");
        } else if (aimEnabled && !canUsePose(pose)) {
            telemetry.addLine("Auto-aim paused while Pinpoint is unavailable.");
        }
        telemetry.addLine("D-pad up/down selects the raised Hive cell.");
        showTurret(telemetry);
    }

    public boolean canUsePose(Pose pose) {
        return pinpoint != null && pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
                && pose != null && Double.isFinite(pose.x())
                && Double.isFinite(pose.y()) && Double.isFinite(pose.heading());
    }

    public void stop() {
        if (turret != null) turret.off();
    }

    private void showTurret(Telemetry telemetry) {
        if (turret == null || !turret.hasCableCalibration()) {
            telemetry.addLine("Turret unavailable. Check hardware and complete INIT calibration.");
        } else {
            telemetry.addData("Turret", "%s at %.1f deg", turret.getMode(), Math.toDegrees(turret.getAngle()));
            if (!turret.getFault().isEmpty()) {
                telemetry.addData("Turret stopped", turret.getFault());
            } else if (turret.getMode() == Turret.Mode.TRACK_GOAL && Double.isNaN(turret.getTarget())) {
                telemetry.addLine("Aim paused. Toggle X off/on or select the other cell to reacquire.");
            }
        }
    }
}
