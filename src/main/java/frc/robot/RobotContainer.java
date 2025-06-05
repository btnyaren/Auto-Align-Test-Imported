package frc.robot;


import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// Kendi swerve module sınıfınızı import edin
// import your.package.SwerveModule;

public class RobotContainer {
    private final XboxController controller = new XboxController(0);

    private final PhotonCamera camera = new PhotonCamera("photonvision");
    private final Transform3d robotToCamera = new Transform3d(
        new Translation3d(0.15, 0.0, 0.25),
        new Rotation3d(0.0, 0.0, 0.0)
    );
    private final PhotonPoseEstimator estimator = new PhotonPoseEstimator(
        Constants.FieldConstants.aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, robotToCamera
    );

    private final VisionSubsystem vision = new VisionSubsystem(estimator);



    //Swerve Kinematics ve Drive Subsystem Eklenecek
    private final DriveSubsystem drive = new DriveSubsystem(null, null);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        new JoystickButton(controller, XboxController.Button.kB.value).whileTrue(
            new AlignToAprilTagCommand(
                () -> vision.getCorrectedX(),
                () -> vision.getCorrectedY(),
                () -> vision.getRotationDegrees(),
                speeds -> drive.drive(speeds)
            )
        );
    }
}