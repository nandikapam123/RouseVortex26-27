
package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.math.Pose;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.Command;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;

import static com.pedropathing.ivy.commands.Commands.waitSeconds;

@Autonomous(name = "Auton")
public class Auton extends OpMode {

    private Follower follower;
    private final AutonPaths paths = new AutonPaths();

    private final Pose startPose = paths.getStartPose();

    private Command autoRoutine() {
        return sequential(
                waitSeconds(1.5),

                follow(follower, paths.redLoadingZoneapproach()),
                follow(follower, paths.path2()),
                follow(follower, paths.path3()),
                follow(follower, paths.path4()),
                follow(follower, paths.path5()),
                follow(follower, paths.path6()),
                follow(follower, paths.path7()),
                follow(follower, paths.path8())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading",
                Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());

        telemetry.update();
    }

    @Override
    public void stop() {
        storePose.autonomousEndPose = follower.pose();
    }
}
