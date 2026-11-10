package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Picks which hive to aim at.
 *
 * Everything is written ONCE for red. For blue, the robot pose is mirrored into the
 * "red frame", the red rule picks a hive, and that hive is mirrored back to blue.
 * So both alliances share one rule; you only choose how the field is mirrored.
 */
public class HiveTargets {

    public enum Alliance { RED, BLUE }

    /** How the blue half relates to the red half of the field (Pedro coords, 0-141.5). */
    public enum Mirror {
        ACROSS_X_CENTER,  // (x, y) -> (141.5 - x, y)     left/right mirror
        ACROSS_Y_CENTER,  // (x, y) -> (x, 141.5 - y)     top/bottom mirror
        ROTATE_180,       // (x, y) -> (141.5 - x, 141.5 - y) point symmetry
        NONE              // blue uses the exact same coordinates
    }

    // ---- RED hives (inches, Pedro coordinates) ----
    public static double RED_UPPER_X = 57.75, RED_UPPER_Y = 89.75;   // used when robot y > SPLIT_Y
    public static double RED_LOWER_X = 57.75, RED_LOWER_Y = 52.75;   // used when robot y < SPLIT_Y

    public static double SPLIT_Y = 70.75;
    /** Dead band around SPLIT_Y so the target doesn't flicker while driving along the line. */
    public static double SPLIT_HYSTERESIS = 1.5;

    public static Mirror BLUE_MIRROR = Mirror.ACROSS_X_CENTER;   // VERIFY against your field
    // Visualizer center is (70.75, 70.75), so the symmetric coordinate span is 141.5.
    public static double FIELD_SIZE = 141.5;

    private boolean useUpper = true;
    private boolean initialized = false;

    /**
     * @param robotX robot x in inches (e.g. pose.getX())
     * @param robotY robot y in inches (e.g. pose.getY())
     * @return {x, y} of the hive to aim at.
     */
    public double[] select(double robotX, double robotY, Alliance alliance) {
        double[] r = {robotX, robotY};
        if (alliance == Alliance.BLUE) r = mirror(r);          // robot -> red frame

        double y = r[1];
        if (!initialized) { useUpper = y >= SPLIT_Y; initialized = true; }
        else if (y > SPLIT_Y + SPLIT_HYSTERESIS) useUpper = true;
        else if (y < SPLIT_Y - SPLIT_HYSTERESIS) useUpper = false;

        double[] hive = useUpper ? new double[]{RED_UPPER_X, RED_UPPER_Y}
                : new double[]{RED_LOWER_X, RED_LOWER_Y};
        return alliance == Alliance.BLUE ? mirror(hive) : hive;   // red frame -> blue
    }

    public boolean isUpper() { return useUpper; }

    /** Every mirror here is its own inverse, so the same function goes both directions. */
    private static double[] mirror(double[] p) {
        switch (BLUE_MIRROR) {
            case ACROSS_X_CENTER: return new double[]{FIELD_SIZE - p[0], p[1]};
            case ACROSS_Y_CENTER: return new double[]{p[0], FIELD_SIZE - p[1]};
            case ROTATE_180:      return new double[]{FIELD_SIZE - p[0], FIELD_SIZE - p[1]};
            default:              return new double[]{p[0], p[1]};
        }
    }
}