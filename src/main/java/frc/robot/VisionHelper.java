package frc.robot;

import edu.wpi.first.math.geometry.*;

public class VisionHelper {
    public static Pose2d correctCameraOffset(Pose2d cameraPose) {
        Transform2d offset = new Transform2d(new Translation2d(-0.15, 0.0), new Rotation2d());
        return cameraPose.plus(offset);
    }
}