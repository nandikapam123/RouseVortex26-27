package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "Auton")
public class Auton extends OpMode {
    private static final int PICKUP_PAUSE_MS = 750;

    private Follower follower;
    private Shooter shooter;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.0275, 8, 270);
    private final Pose redGarden = poseFactory.of(7.3513, 7.9587, 270);

    private final Pose nearcameraFlowerStart = poseFactory.of(7.3513, 7.9587, 72.6473);
    private final Pose nearcameraFlower = poseFactory.of(46.2163, 132.3369, 72.6473);

    private final Pose redwallFlowerStart = poseFactory.of(46.2163, 132.3369, 244.0549);
    private final Pose redwallFlower = poseFactory.of(8.1832, 54.1675, 244.0549);

    private final Pose backtoRedHivelaneStart = poseFactory.of(8.1832, 54.1675, 135.9089);
    private final Pose backtoRedHivelane = poseFactory.of(56.2034, 7.6472, 135.9089);

    private final Pose redLoadingZoneapproachStart = poseFactory.of(56.2034, 7.6472, 298.0913);
    private final Pose redLoadingZoneapproach = poseFactory.of(9.4184, 95.2997, 298.0913);

    private Path redGarden() {
        return Paths.line(start, redGarden).linear(start, redGarden);
    }

    private Path nearcameraFlower() {
        return Paths.line(nearcameraFlowerStart, nearcameraFlower)
                .linear(nearcameraFlowerStart, nearcameraFlower);
    }

    private Path redwallFlower() {
        return Paths.line(redwallFlowerStart, redwallFlower)
                .linear(redwallFlowerStart, redwallFlower);
    }

    private Path backtoRedHivelane() {
        return Paths.line(backtoRedHivelaneStart, backtoRedHivelane)
                .linear(backtoRedHivelaneStart, backtoRedHivelane);
    }

    private Path redLoadingZoneapproach() {
        return Paths.line(redLoadingZoneapproachStart, redLoadingZoneapproach)
                .linear(redLoadingZoneapproachStart, redLoadingZoneapproach);
    }

    private Command autoRoutine() {
        return sequential(
                shooter.shootFour(),
                follow(follower, redGarden()),
                waitMs(PICKUP_PAUSE_MS),
                follow(follower, nearcameraFlower()),
                shooter.shootLoadedBalls(),
                follow(follower, redwallFlower()),
                waitMs(PICKUP_PAUSE_MS),
                follow(follower, backtoRedHivelane()),
                shooter.shootLoadedBalls(),
                follow(follower, redLoadingZoneapproach())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        shooter = new Shooter(hardwareMap);
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
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
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (shooter != null) {
            shooter.stop();
        }

        if (follower != null) {
            storePose.autonomousEndPose = follower.pose();
        }
    }
}
