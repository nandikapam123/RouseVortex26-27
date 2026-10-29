package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/** Press A on controller 1 to toggle a goBILDA DC motor on or off. */
@TeleOp(name = "Planetary Motor - A Toggle", group = "Motor Tests")
public class PlanetaryMotorTeleOp extends OpMode {
    // Match this name exactly in the Robot Controller hardware configuration.
    private static final String MOTOR_NAME = "pm";
    // Fraction of full power, not a target RPM.
    private static final double MOTOR_POWER = 1.0;

    private DcMotor motor;
    private boolean running;
    private boolean previousA;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, MOTOR_NAME);
        motor.setPower(0);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        running = false;
        telemetry.addLine("Ready. Press START, then A to toggle the motor.");
        telemetry.update();
    }

    @Override
    public void start() {
        running = false;
        previousA = gamepad1.a;
        motor.setPower(0);
    }

    @Override
    public void loop() {
        boolean currentA = gamepad1.a;
        // Toggle once per press, even if A stays held down.
        if (currentA && !previousA) {
            running = !running;
        }
        previousA = currentA;

        motor.setPower(running ? MOTOR_POWER : 0);
        telemetry.addData("Motor", running ? "ON" : "OFF");
        telemetry.addData("Power", running ? MOTOR_POWER : 0);
        telemetry.addLine("Controller 1: A toggles ON/OFF.");
        telemetry.update();
    }

    @Override
    public void stop() {
        running = false;
        if (motor != null) {
            motor.setPower(0);
        }
    }
}
