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

/**
 * Red-side autonomous on the ourPaths route.
 *
 * INIT: gate closed (90 deg), flower intake straight up (0 deg), turret calibration on Gamepad 1.
 * The flywheel spins from START until the last shot. Every mechanism and the drivetrain
 * are stopped by STOP_ALL_SECONDS so nothing is moving when AUTO ends.
 */
@Autonomous(name = "Auton")
public class Auton extends OpMode {
    private static final int PICKUP_PAUSE_MS = 750;
    private static final int TURRET_SETTLE_MS = 150;
    private static final int TURRET_CENTER_TIMEOUT_MS = 2500;
    private static final double STOP_ALL_SECONDS = 28.0;

    private final ourPaths paths = new ourPaths();
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime turretCenterTimer = new ElapsedTime();
    private final ElapsedTime turretSettledTimer = new ElapsedTime();
    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private TurretController turret;
    private boolean started;
    private boolean stoppedForTime;
    private boolean turretCenteringFailed;

    private Command autoRoutine() {
        return sequential(
                waitUntil(this::finishTurretCentering),

                // Build flywheel speed now and keep it on. Transfer at full power.
                shooter.spinUp(),
                intake.full(),

                // Path 1: opening shot
                follow(follower, paths.moveToOpeningShotPosition()),
                shootWhenCentered(shooter.shootFour()),
                intake.slow(),

                // Path 2 / 2.5: loading zone pickup
                follow(follower, paths.loadingZoneApproach()),
                intake.full(),
                follow(follower, paths.loadingZoneApproach2()),
                waitMs(PICKUP_PAUSE_MS),
                intake.slow(),

                // Path 3: upper scoring area shot
                follow(follower, paths.moveToUpperScoringArea()),
                intake.full(),
                shootWhenCentered(shooter.shootLoadedBalls()),
                intake.slow(),

                // Path 4: upper flower pickup, flower intake down
                intake.rampDown(),
                follow(follower, paths.moveToUpperFlower()),
                intake.full(),
                waitMs(PICKUP_PAUSE_MS),
                intake.slow(),

                // Path 5: back to the upper shot
                follow(follower, paths.returnFromUpperScoringPosition()),
                intake.full(),
                shootWhenCentered(shooter.shootLoadedBalls()),
                intake.slow(),

                // Path 6: lower field pickup, then flower intake back up
                follow(follower, paths.moveToLowerField()),
                intake.full(),
                waitMs(PICKUP_PAUSE_MS),
                intake.slow(),
                intake.rampUp(),

                // Path 7: final shot. Then stop everything except the drivetrain.
                follow(follower, paths.moveToShoot()),
                intake.full(),
                shootWhenCentered(shooter.shootLoadedBalls()),
                shooter.stopCommand(),
                intake.stopCommand(),

                // Path 8: park
                follow(follower, paths.returnTowardUpperFieldPark())
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

    /** Only opens the gate when the turret is pointing forward; otherwise the gate stays shut. */
    private Command shootWhenCentered(Command shot) {
        return conditional(turret::isCentered, shot, waitMs(0));
    }

    @Override
    public void init() {
        Scheduler.reset();
        storePose.autonomousEndPose = null;
        RobotState.alliance = HiveTargets.Alliance.RED;
        shooter = new Shooter(hardwareMap); // gate closed
        intake = new Intake(hardwareMap);   // flower intake up
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
        stoppedForTime = false;
        turretCenteringFailed = false;
        autoTimer.reset();
        turret.centerAtStart();
        turretCenterTimer.reset();
        turretSettledTimer.reset();
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        if (!stoppedForTime && autoTimer.seconds() >= STOP_ALL_SECONDS) {
            stopEverything();
        }
        follower.update();
        if (!stoppedForTime) {
            turret.updateHold(telemetry);
            Scheduler.execute();
        }

        if (stoppedForTime) {
            telemetry.addLine("Stopped at " + STOP_ALL_SECONDS + " s so nothing moves at the end of AUTO.");
        }
        if (turretCenteringFailed) {
            telemetry.addLine("Turret centering failed. Running the drive route with shooting disabled.");
        }
        telemetry.addData("Time", "%.1f s", autoTimer.seconds());
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    /** Ends the routine early: drivetrain, flywheel, transfer and turret all stop. */
    private void stopEverything() {
        stoppedForTime = true;
        follower.manual(0, 0, 0);
        shooter.stop();
        intake.stop();
        turret.stop();
    }

    @Override
    public void stop() {
        if (turret != null) turret.stop();
        if (shooter != null) shooter.stop();
        if (intake != null) intake.stop();

        if (started && follower != null && turret.canUsePose(follower.pose())) {
            storePose.autonomousEndPose = follower.pose();
        }
    }
}