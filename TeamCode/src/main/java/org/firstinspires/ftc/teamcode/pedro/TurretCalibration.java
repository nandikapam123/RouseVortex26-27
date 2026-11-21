package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

final class TurretCalibration {
    private static final String[] CALIBRATION_STEPS = {
            "Point the turret forward at its center mark and press A.",
            "Turn left by hand while every wire has slack, then press A.",
            "Turn right by hand while every wire has slack, then press A."
    };

    private final int[] marks = new int[3];
    private int markCount;
    private boolean previousA;
    private boolean previousX;
    private boolean invalidMarks;
    private boolean centerSwitchVerified;

    void update(Turret turret, Gamepad gamepad, Telemetry telemetry) {
        if (turret == null) {
            telemetry.addLine("Configure turret as a CR servo and turretEncoder as its encoder motor port.");
            return;
        }
        if (gamepad.x && !previousX) {
            turret.clearCableCalibration();
            markCount = 0;
            invalidMarks = false;
        }
        if (!turret.hasCableCalibration()) {
            turret.restoreCableCalibrationFromSwitch();
        }
        if (!turret.hasCableCalibration() && gamepad.a && !previousA) {
            if (turret.hasSavedCableCalibration()) {
                if (!turret.hasVerifiedCenterSwitch()) {
                    invalidMarks = !turret.restoreCableCalibrationAtCenter();
                }
            } else {
                // Trust the switch only when it is active at center and inactive at both ends.
                if (markCount == 0) {
                    centerSwitchVerified = turret.isSwitchPressed();
                } else {
                    centerSwitchVerified &= !turret.isSwitchPressed();
                }
                marks[markCount++] = turret.encoderTicks();
                if (markCount == marks.length) {
                    invalidMarks = !turret.calibrateCableClearance(marks[0], marks[1], marks[2], centerSwitchVerified);
                    if (invalidMarks) markCount = 0;
                }
            }
        }
        previousA = gamepad.a;
        previousX = gamepad.x;

        if (turret.hasCableCalibration()) {
            telemetry.addData("Turret ready (deg)", "%.1f", Math.toDegrees(turret.getAngle()));
            telemetry.addData("Cable limits", turret.hasSavedCableCalibration() ? "Saved" : "This session only");
            telemetry.addLine("X clears the limits for a full recalibration.");
        } else if (turret.hasSavedCableCalibration()) {
            if (turret.hasVerifiedCenterSwitch()) {
                telemetry.addLine("Turret waiting for its center switch. Park at center before powering on.");
            } else {
                telemetry.addLine("Park the turret at the center mark before powering on, then confirm with A.");
            }
            telemetry.addLine("Press X for a full calibration if wiring or turret hardware changed.");
        } else {
            telemetry.addLine(CALIBRATION_STEPS[markCount]);
            if (markCount > 0) {
                telemetry.addData("Distance from center", "%.1f deg (%.0f max)",
                        Math.toDegrees(turret.distanceFromMark(marks[0])), Math.toDegrees(Turret.HARD_LIMIT));
            }
            if (invalidMarks) telemetry.addLine("Mark a cable-safe position on both sides of center.");
        }
    }
}
