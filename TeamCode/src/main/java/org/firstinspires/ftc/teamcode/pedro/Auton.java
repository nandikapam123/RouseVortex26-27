package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.subsystems.HiveTargets;
import org.firstinspires.ftc.teamcode.subsystems.RobotState;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "Auton")
public class Auton extends OpMode {
    private static final int PICKUP_PAUSE_MS = 750;

    private final Paths paths = new Paths();
    private Follower follower;
    private Shooter shooter;
    private TurretController turret;
    private boolean started;

    private Command autoRoutine() {
        return sequential(
                follow(follower, paths.moveToOpeningShotPosition()),
                shooter.shootFour(),
                follow(follower, paths.loadingZoneApproach()),
                waitMs(PICKUP_PAUSE_MS),
                follow(follower, paths.moveToUpperScoringArea()),
                follow(follower, paths.moveToUpperScoringPosition()),
                shooter.shootLoadedBalls(),
                follow(follower, paths.returnFromUpperScoringPosition()),
                follow(follower, paths.moveToLowerField()),
                follow(follower, paths.moveToLowerPickup()),
                waitMs(PICKUP_PAUSE_MS),
                follow(follower, paths.returnTowardUpperField()),
                shooter.shootLoadedBalls()
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        storePose.autonomousEndPose = null;
        RobotState.alliance = HiveTargets.Alliance.RED;
        shooter = new Shooter(hardwareMap);
        follower = Constants.create(hardwareMap);
        follower.setPose(paths.getStartPose());
        follower.update();
        turret = new TurretController(hardwareMap);
    }

    @Override
    public void init_loop() {
        turret.calibrateInInit(gamepad1, telemetry);
        telemetry.update();
    }

    @Override
    public void start() {
        started = true;
        turret.holdAtStart();
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        turret.updateHold(telemetry);
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (turret != null) {
            turret.stop();
        }
        if (shooter != null) {
            shooter.stop();
        }

        if (started && follower != null && turret.canUsePose(follower.pose())) {
            storePose.autonomousEndPose = follower.pose();
        }
    }
}
