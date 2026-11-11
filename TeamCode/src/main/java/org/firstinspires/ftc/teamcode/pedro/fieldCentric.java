package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;


@TeleOp(name="Satvik Stuff")
public class fieldCentric extends OpMode {
    private Follower follower;


    @Override
    public void init() {

        // Initialize Pedro Pathing
        follower = Constants.create(hardwareMap);

        // TODO: Set the robot's actual starting pose
        // This example assumes the robot starts at (56, 9)
        follower.setPose(storePose.autonomousEndPose);

        follower.update();
    }

    @Override
    public void loop() {

        // Read joystick inputs
        double forward = gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // Convert joystick inputs into field-centric powers
        DrivePowers powers = ManualDrive.fieldCentric(
                forward,
                lateral,
                turn,
                follower.pose().heading()
        );

        // Send powers to the drivetrain
        follower.manual(powers);

        // Update Pedro localization and drivetrain
        follower.update();

        // Display robot position
        Pose currentPose = follower.pose();

        telemetry.addData("X", currentPose.x());
        telemetry.addData("Y", currentPose.y());

        telemetry.addData(
                "Heading",
                Math.toDegrees(currentPose.heading())
        );

        telemetry.addData("Forward", forward);
        telemetry.addData("Lateral", lateral);
        telemetry.addData("Turn", turn);

        telemetry.update();
    }
}
