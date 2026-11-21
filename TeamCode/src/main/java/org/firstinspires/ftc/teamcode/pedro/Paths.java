
package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class Paths {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Starting position
    private final Pose start = poseFactory.of(9, 24, 0);

    // Path 1: Opening shot position
    private final Pose openingShotPosition =
            poseFactory.of(22.6906, 24.2599, 45);

    // Path 2: Loading Zone Approach
    private final Pose loadingZoneApproach =
            poseFactory.of(9.0914, 10.0043, 270);
    private final Pose loadingZoneApproachControl =
            poseFactory.of(8.1737, 19.9278, 0);

    // Path 3: Move Toward Upper Field
    private final Pose upperScoringStart =
            poseFactory.of(9.0914, 10.0043, 270);
    private final Pose upperScoringEnd =
            poseFactory.of(34.9201, 118.699, 305);
    private final Pose upperScoringControl =
            poseFactory.of(32.5957, 53.3174, 0);

    // Path 4: Upper Scoring Position
    private final Pose upperPositionStart =
            poseFactory.of(34.9201, 118.699, 180);
    private final Pose upperPositionEnd =
            poseFactory.of(46.7164, 129.7888, 90);
    private final Pose upperPositionControl =
            poseFactory.of(49.4532, 113.0268, 0);

    // Path 5: Return From Upper Position
    private final Pose returnPoint =
            poseFactory.of(35.1293, 118.8885, 90);

    // Path 6: Move Toward Lower Field
    private final Pose lowerFieldStart =
            poseFactory.of(35.1293, 118.8885, 90);
    private final Pose lowerFieldEnd =
            poseFactory.of(12.4316, 47.1085, 180);
    private final Pose lowerFieldControl =
            poseFactory.of(32.7288, 71.8783, 0);

    // Path 7: Move to Pickup Position
    private final Pose pickupStart =
            poseFactory.of(12.4316, 47.1085, 180);
    private final Pose pickupEnd =
            poseFactory.of(16.8406, 22.8398, 45);

    // Path 8: Return Toward Upper Field
    private final Pose finalStart =
            poseFactory.of(16.8406, 22.8398, 45);
    private final Pose finalEnd =
            poseFactory.of(9.7842, 95.3286, 0);
    private final Pose finalControl =
            poseFactory.of(33.8992, 64.9164, 0);


    public Pose getStartPose() {
        return start;
    }

    // Path 1
    public Path moveToOpeningShotPosition() {
        return com.pedropathing.api.Paths
                .line(start, openingShotPosition)
                .linear(start, openingShotPosition);
    }

    // Path 2
    public Path loadingZoneApproach() {
        return com.pedropathing.api.Paths
                .curve(openingShotPosition, loadingZoneApproachControl, loadingZoneApproach)
                .constant(loadingZoneApproach);
    }

    // Path 3
    public Path moveToUpperScoringArea() {
        return com.pedropathing.api.Paths
                .curve(upperScoringStart, upperScoringControl, upperScoringEnd)
                .linear(upperScoringStart, upperScoringEnd);
    }

    // Path 4
    public Path moveToUpperScoringPosition() {
        return com.pedropathing.api.Paths
                .curve(upperPositionStart, upperPositionControl, upperPositionEnd)
                .linear(upperPositionStart, upperPositionEnd);
    }

    // Path 5
    public Path returnFromUpperScoringPosition() {
        return com.pedropathing.api.Paths
                .line(upperPositionEnd, returnPoint)
                .constant(returnPoint);
    }

    // Path 6
    public Path moveToLowerField() {
        return com.pedropathing.api.Paths
                .curve(lowerFieldStart, lowerFieldControl, lowerFieldEnd)
                .linear(lowerFieldStart, lowerFieldEnd);
    }

    // Path 7
    public Path moveToLowerPickup() {
        return com.pedropathing.api.Paths
                .line(pickupStart, pickupEnd)
                .linear(pickupStart, pickupEnd);
    }

    // Path 8
    public Path returnTowardUpperField() {
        return com.pedropathing.api.Paths
                .curve(finalStart, finalControl, finalEnd)
                .linear(finalStart, finalEnd);
    }
}
