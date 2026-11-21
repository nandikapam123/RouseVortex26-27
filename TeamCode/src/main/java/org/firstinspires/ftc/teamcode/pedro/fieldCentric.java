package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Satvik Stuff")
public class fieldCentric extends OpMode {
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        Pose startPose = (storePose.autonomousEndPose != null)
                ? storePose.autonomousEndPose
                : new Pose(0, 0, 0);
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        DrivePowers powers = ManualDrive.fieldCentric(
                forward,
                lateral,
                turn,
                follower.pose().heading()
        );

        follower.manual(powers);
        follower.update();

        Pose currentPose = follower.pose();
        telemetry.addData("X", currentPose.x());
        telemetry.addData("Y", currentPose.y());
        telemetry.addData("Heading", Math.toDegrees(currentPose.heading()));
        telemetry.addData("Mode", follower.mode());
        telemetry.addData("Forward", forward);
        telemetry.addData("Lateral", lateral);
        telemetry.addData("Turn", turn);
        telemetry.update();
    }
}

