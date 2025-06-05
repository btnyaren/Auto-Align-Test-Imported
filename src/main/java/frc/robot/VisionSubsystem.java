package frc.robot;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.VisionHelper;


public class VisionSubsystem {
    private final PhotonCamera camera = new PhotonCamera("photonvision");
    private final PhotonPoseEstimator estimator;
    private Pose2d lastKnownPose = new Pose2d();

    public VisionSubsystem(PhotonPoseEstimator estimator) {
        this.estimator = estimator;
    }

    public Pose2d getCorrectedPose() {
        Optional<EstimatedRobotPose> result = estimator.update(null);
        if (result.isPresent()) {
            lastKnownPose = VisionHelper.correctCameraOffset(result.get().estimatedPose.toPose2d());
        }
        return lastKnownPose;
    }

    public double getCorrectedX() {
        return getCorrectedPose().getTranslation().getX();
    }

    public double getCorrectedY() {
        return getCorrectedPose().getTranslation().getY();
    }

    public double getRotationDegrees() {
        return getCorrectedPose().getRotation().getDegrees();
    }
}