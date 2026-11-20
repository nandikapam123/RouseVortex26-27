package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

public class Shooter {
    private static final double SHOOTER_POWER = 0.68;
    private static final int FOUR_BALLS_MS = 1500;
    private static final int LOADED_BALLS_MS = 1500;

    private final DcMotor motor;

    public Shooter(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "pm");
        motor.setPower(0);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Command shootFour() {
        return runFor(FOUR_BALLS_MS);
    }

    public Command shootLoadedBalls() {
        return runFor(LOADED_BALLS_MS);
    }

    private Command runFor(int durationMs) {
        return sequential(
                instant(() -> motor.setPower(SHOOTER_POWER)),
                waitMs(durationMs),
                instant(this::stop)
        );
    }

    public void stop() {
        motor.setPower(0);
    }
}
