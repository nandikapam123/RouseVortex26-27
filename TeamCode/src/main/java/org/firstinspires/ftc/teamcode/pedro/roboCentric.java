package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "roboCentric")
public class roboCentric extends OpMode {

    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void loop() {
        follower.manual(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        follower.update();
    }
}