package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Static fields survive between OpModes (Auto -> TeleOp) as long as the robot app
 * isn't restarted. The Control Hub also keeps encoder counts between OpModes while
 * powered, so a turret zero found in Auto is still correct in TeleOp.
 */
public class RobotState {
    public static double turretZeroTicks = 0;
    public static boolean turretZeroValid = false;

    public static HiveTargets.Alliance alliance = HiveTargets.Alliance.RED;

    // End-of-auto pose, used as the TeleOp start pose (inches, inches, radians)
    public static boolean hasLastPose = false;
    public static double lastX = 0, lastY = 0, lastHeading = 0;

    public static void savePose(double x, double y, double heading) {
        lastX = x; lastY = y; lastHeading = heading;
        hasLastPose = true;
    }
}