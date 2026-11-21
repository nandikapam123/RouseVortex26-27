package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static com.pedropathing.ivy.commands.Commands.instant;

/**
 * Intake/transfer motor + flower intake servo ("flower ramp"). Shared by Auton and the
 * driver TeleOp so both use the same hardware names and positions.
 *
 * Motor: goBILDA 5203 Yellow Jacket (1620 RPM), config name "intake". It runs both the
 *        intake and the transfer; positive power pulls pollen in.
 * Servo: goBILDA 2000-0025-0002 Dual Mode Servo (Torque), config name "lowerIntake".
 *        0 deg = straight up (stowed), RAMP_DOWN_DEG = down to collect.
 */
public class Intake {
    public static final String MOTOR_NAME = "intake";
    public static final String RAMP_NAME = "lowerIntake";

    public static final double FULL_POWER = 1.0;
    // PLACEHOLDER: "slowed down" speed between pickups and shots. Tune on the robot.
    public static final double SLOW_POWER = 0.3;

    public static final double RAMP_RANGE_DEG = 300;
    public static final double RAMP_UP_DEG = 0;
    // Allowed range is 100-150 deg. Start at 100 and increase until it reaches the floor.
    public static final double RAMP_DOWN_DEG = 100;

    private final DcMotor motor;
    private final Servo ramp;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, MOTOR_NAME);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ramp = hardwareMap.get(Servo.class, RAMP_NAME);
        Shooter.useFullServoRange(ramp);
        setRampDown(false); // start straight up
    }

    // ===================== Direct control (TeleOp) =====================
    public void setPower(double power) {
        motor.setPower(power);
    }

    public void setRampDown(boolean down) {
        ramp.setPosition((down ? RAMP_DOWN_DEG : RAMP_UP_DEG) / RAMP_RANGE_DEG);
    }

    // ===================== Commands (Auton) =====================
    public Command full() {
        return instant(() -> setPower(FULL_POWER));
    }

    public Command slow() {
        return instant(() -> setPower(SLOW_POWER));
    }

    public Command rampDown() {
        return instant(() -> setRampDown(true));
    }

    public Command rampUp() {
        return instant(() -> setRampDown(false));
    }

    public Command stopCommand() {
        return instant(this::stop);
    }

    public void stop() {
        motor.setPower(0);
    }
}