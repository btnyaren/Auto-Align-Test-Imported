package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveSubsystem {
    private final SwerveDriveKinematics kinematics;
    private final SwerveModuleState[] swerveModules;
    private static final double MAX_SPEED = 4.0; // örnek değer m/s

    public DriveSubsystem(SwerveDriveKinematics kinematics, SwerveModuleState[] modules) {
        this.kinematics = kinematics;
        this.swerveModules = modules;
    }


    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED);

        /* for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setDesiredState(states[i]);
        } */
    }

    public void stop() {
        drive(new ChassisSpeeds(0, 0, 0));
    }
}