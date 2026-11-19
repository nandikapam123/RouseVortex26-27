
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
import static com.pedropathing.ivy.commands.Commands.waitMs;

@Autonomous(name = "Auton")
public class Auton extends OpMode {

    private Follower follower;

    private final Paths paths = new Paths();

    private final Pose startPose = paths.getStartPose();


    private Command autoRoutine() {
        return sequential(

                // Initial wait
                waitMs(1500),

                // Path 1: Loading Zone Approach
                follow(follower, paths.loadingZoneApproach()),
                waitMs(750),

                // Path 2: Move Toward Upper Field
                follow(follower, paths.moveToUpperScoringArea()),
                waitMs(1500),

                // Path 3: Upper Scoring Position
                follow(follower, paths.moveToUpperScoringPosition()),
                waitMs(750),

                // Path 4: Return From Upper Position
                follow(follower, paths.returnFromUpperScoringPosition()),
                waitMs(1500),

                // Path 5: Move Toward Lower Field
                follow(follower, paths.moveToLowerField()),
                waitMs(750),

                // Path 6: Move to Pickup Position
                follow(follower, paths.moveToLowerPickup()),
                waitMs(1500),

                // Path 7: Return Toward Upper Field
                follow(follower, paths.returnTowardUpperField())

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

        telemetry.addData(
                "Heading",
                Math.toDegrees(follower.pose().heading())
        );

        telemetry.addData("Follower Mode", follower.mode());

        telemetry.update();
    }


    @Override
    public void stop() {
        storePose.autonomousEndPose = follower.pose();
    }
}
