
package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class AutonPaths {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(9, 14, 270);
    private final Pose redLoadingZoneapproach = poseFactory.of(9, 9.9735, 270);

    private final Pose point2Start = poseFactory.of(9, 9.9735, 0);
    private final Pose point2 = poseFactory.of(34.9201, 118.699, 180);

    private final Pose point3Start = poseFactory.of(34.9201, 118.699, 180);
    private final Pose point3 = poseFactory.of(46.4558, 128.697, 90);

    private final Pose point4 = poseFactory.of(46.7104, 130.836, 90);
    private final Pose point5 = poseFactory.of(35.1293, 118.8885, 90);

    private final Pose point6Start = poseFactory.of(35.1293, 118.8885, 90);
    private final Pose point6 = poseFactory.of(10.957, 47.1085, 180);

    private final Pose point7 = poseFactory.of(16.8406, 22.8398, 103.6276);
    private final Pose point8 = poseFactory.of(9.7842, 95.3286, -84.4401);

    public Pose getStartPose() {
        return start;
    }

    public Path redLoadingZoneapproach() {
        return com.pedropathing.api.Paths.line(start, redLoadingZoneapproach)
                .constant(redLoadingZoneapproach);
    }

    public Path path2() {
        return com.pedropathing.api.Paths.line(point2Start, point2)
                .linear(point2Start, point2);
    }

    public Path path3() {
        return com.pedropathing.api.Paths.line(point3Start, point3)
                .linear(point3Start, point3);
    }

    public Path path4() {
        return com.pedropathing.api.Paths.line(point3, point4)
                .constant(point4);
    }

    public Path path5() {
        return com.pedropathing.api.Paths.line(point4, point5)
                .constant(point5);
    }

    public Path path6() {
        return com.pedropathing.api.Paths.line(point6Start, point6)
                .linear(point6Start, point6);
    }

    public Path path7() {
        return com.pedropathing.api.Paths.line(point6, point7)
                .reverseTangent();
    }

    public Path path8() {
        return com.pedropathing.api.Paths.line(point7, point8)
                .reverseTangent();
    }
}
