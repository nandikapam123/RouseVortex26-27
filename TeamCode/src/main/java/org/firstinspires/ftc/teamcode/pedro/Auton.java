package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.HiveTargets;
import org.firstinspires.ftc.teamcode.subsystems.RobotState;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "Auton")
public class Auton extends OpMode {
    private static final int PICKUP_PAUSE_MS = 750;
    private static final int TURRET_SETTLE_MS = 150;
    private static final int TURRET_CENTER_TIMEOUT_MS = 2500;

    private final Paths paths = new Paths();
    private final ElapsedTime turretCenterTimer = new ElapsedTime();
    private final ElapsedTime turretSettledTimer = new ElapsedTime();
    private Follower follower;
    private Shooter shooter;
    private TurretController turret;
    private boolean started;
    private boolean turretCenteringFailed;

    private Command autoRoutine() {
        return sequential(
                waitUntil(this::finishTurretCentering),
                follow(follower, paths.moveToOpeningShotPosition()),
                shootWhenCentered(shooter.shootFour()),
                follow(follower, paths.loadingZoneApproach()),
                waitMs(PICKUP_PAUSE_MS),
                follow(follower, paths.moveToUpperScoringArea()),
                follow(follower, paths.moveToUpperScoringPosition()),
                shootWhenCentered(shooter.shootLoadedBalls()),
                follow(follower, paths.returnFromUpperScoringPosition()),
                follow(follower, paths.moveToLowerField()),
                follow(follower, paths.moveToLowerPickup()),
                waitMs(PICKUP_PAUSE_MS),
                follow(follower, paths.returnTowardUpperField()),
                shootWhenCentered(shooter.shootLoadedBalls())
        );
    }

    private boolean finishTurretCentering() {
        if (!turret.isCentered()) {
            turretSettledTimer.reset();
        } else if (turretSettledTimer.milliseconds() >= TURRET_SETTLE_MS) {
            return true;
        }
        if (!turret.canCenter() || turretCenterTimer.milliseconds() >= TURRET_CENTER_TIMEOUT_MS) {
            turret.stop();
            turretCenteringFailed = true;
            return true;
        }
        return false;
    }

    private Command shootWhenCentered(Command shot) {
        return conditional(turret::isCentered, shot, waitMs(0));
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
        turretCenteringFailed = false;
        turret.centerAtStart();
        turretCenterTimer.reset();
        turretSettledTimer.reset();
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        turret.updateHold(telemetry);
        Scheduler.execute();

        if (turretCenteringFailed) {
            telemetry.addLine("Turret centering failed. Running the drive route with shooting disabled.");
        }
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
