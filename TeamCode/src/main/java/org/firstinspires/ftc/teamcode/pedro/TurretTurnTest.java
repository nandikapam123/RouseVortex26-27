package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Turret 90 Degree Test", group = "Motor Tests")
public class TurretTurnTest extends OpMode {
    private final ElapsedTime moveTimer = new ElapsedTime();
    private TurretCalibration calibration;
    private Turret turret;
    private double targetDegrees;
    private double timeToTarget;
    private boolean moving;

    @Override
    public void init() {
        try {
            turret = new Turret(hardwareMap, "turret", "turretEncoder");
            turret.off();
        } catch (IllegalArgumentException e) {
            turret = null;
        }
        calibration = new TurretCalibration();
        targetDegrees = Double.NaN;
        timeToTarget = Double.NaN;
        moving = false;
    }

    @Override
    public void init_loop() {
        calibration.update(turret, gamepad1, telemetry);
        telemetry.update();
    }

    @Override
    public void loop() {
        if (turret == null || !turret.hasCableCalibration()) {
            telemetry.addLine("Check turret hardware and complete INIT calibration.");
            telemetry.update();
            return;
        }

        double previousTarget = targetDegrees;
        if (gamepad1.y) {
            targetDegrees = 0;
        } else if (gamepad1.dpad_left) {
            targetDegrees = 90;
        } else if (gamepad1.dpad_right) {
            targetDegrees = -90;
        }
        boolean selected = Double.isFinite(targetDegrees);
        boolean changedTarget = selected && targetDegrees != previousTarget;
        double target = Math.toRadians(targetDegrees);
        boolean reachable = target >= RobotState.turretLowerClearance + Turret.SAFETY_MARGIN
                && target <= RobotState.turretUpperClearance - Turret.SAFETY_MARGIN;
        boolean enabled = gamepad1.right_bumper && reachable && turret.getFault().isEmpty();

        if (changedTarget) {
            timeToTarget = Double.NaN;
        }
        if (enabled) {
            if (!moving || changedTarget) {
                moveTimer.reset();
                timeToTarget = Double.NaN;
            }
            turret.holdAngle(target);
        } else {
            turret.off();
        }
        turret.update(0, 0, 0);
        moving = enabled && turret.getFault().isEmpty();

        double angle = turret.getAngle();
        telemetry.addData("Actual angle", "%.1f deg", Math.toDegrees(angle));
        telemetry.addData("Servo power", "%.2f", turret.getPower());
        if (selected) {
            double error = target - angle;
            telemetry.addData("Target / error", "%.1f / %.1f deg", targetDegrees, Math.toDegrees(error));
            if (!reachable) {
                telemetry.addLine("Selected angle is outside the calibrated cable range.");
            } else if (moving && Double.isNaN(timeToTarget) && Math.abs(error) <= Turret.TOLERANCE) {
                timeToTarget = moveTimer.seconds();
            }
            if (Double.isFinite(timeToTarget)) {
                telemetry.addData("Time to reach target", "%.2f s", timeToTarget);
            }
        }
        if (!turret.getFault().isEmpty()) {
            telemetry.addData("Turret stopped", turret.getFault());
        }
        telemetry.addLine("D-pad left/right selects 90 deg. Y selects center. Hold RB to move.");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (turret != null) turret.off();
    }
}