package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

/**
 * Flywheel + servo gate ("blocker"). Shared by Auton and the driver TeleOp so both use the
 * same hardware names and gate positions.
 *
 * Flywheel: goBILDA 5000 Series 12VDC motor (8mm REX pinion), config name "pm".
 * Gate:     goBILDA 2000-0025-0002 Dual Mode Servo (Torque) in default (position) mode,
 *           config name "blocker". 300 degrees of travel with a 500-2500 us signal.
 */
public class Shooter {
    public static final String FLYWHEEL_NAME = "pm";
    public static final String GATE_NAME = "blocker";

    public static final double AUTO_FLYWHEEL_POWER = 0.56;

    // Gate angles, measured from servo position 0. Closed holds pollen back; open feeds the flywheel.
    public static final double GATE_RANGE_DEG = 300;
    public static final double GATE_CLOSED_DEG = 90;
    public static final double GATE_OPEN_DEG = 150;

    // How long the gate stays open in Auton to let every loaded ball through.
    private static final int FOUR_BALLS_MS = 1500;
    private static final int LOADED_BALLS_MS = 1500;

    private final DcMotor motor;
    private final Servo gate;

    public Shooter(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, FLYWHEEL_NAME);
        motor.setPower(0);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gate = hardwareMap.get(Servo.class, GATE_NAME);
        useFullServoRange(gate);
        setGateOpen(false); // start closed
    }

    /** Lets a goBILDA 2000 Series servo use its full 300 degrees, so the degree values above are real. */
    static void useFullServoRange(Servo servo) {
        if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
    }

    // ===================== Direct control (TeleOp) =====================
    public void setFlywheelPower(double power) {
        motor.setPower(power);
    }

    public void setGateOpen(boolean open) {
        gate.setPosition((open ? GATE_OPEN_DEG : GATE_CLOSED_DEG) / GATE_RANGE_DEG);
    }

    // ===================== Commands (Auton) =====================
    /** Start the flywheel and leave it running so it builds speed before the first shot. */
    public Command spinUp() {
        return instant(() -> setFlywheelPower(AUTO_FLYWHEEL_POWER));
    }

    public Command openGate() {
        return instant(() -> setGateOpen(true));
    }

    public Command closeGate() {
        return instant(() -> setGateOpen(false));
    }

    public Command shootFour() {
        return shootFor(FOUR_BALLS_MS);
    }

    public Command shootLoadedBalls() {
        return shootFor(LOADED_BALLS_MS);
    }

    /** Flywheel stays on; this only opens the gate, waits, and closes it. */
    private Command shootFor(int durationMs) {
        return sequential(
                openGate(),
                waitMs(durationMs),
                closeGate()
        );
    }

    public Command stopCommand() {
        return instant(this::stop);
    }

    /** Flywheel off and gate closed. */
    public void stop() {
        motor.setPower(0);
        setGateOpen(false);
    }
}