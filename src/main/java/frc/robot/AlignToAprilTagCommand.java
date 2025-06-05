package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;
import java.util.function.Consumer;

public class AlignToAprilTagCommand extends Command {
    private final PIDController xController = new PIDController(1.2, 0.0, 0.05);
    private final PIDController yController = new PIDController(1.2, 0.0, 0.05);
    private final PIDController rotController = new PIDController(0.5, 0.0, 0.02);

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier headingSupplier;
    private final Consumer<ChassisSpeeds> output;

    public AlignToAprilTagCommand(
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        DoubleSupplier headingSupplier,
        Consumer<ChassisSpeeds> output
    ) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.headingSupplier = headingSupplier;
        this.output = output;

        xController.setTolerance(0.03);
        yController.setTolerance(0.02);
        rotController.setTolerance(Units.degreesToRadians(2));
    }

    @Override
    public void execute() {
        double xSpeed = xController.calculate(xSupplier.getAsDouble(), 0.03);
        double ySpeed = yController.calculate(ySupplier.getAsDouble(), 0.0);
        double rotSpeed = rotController.calculate(Units.degreesToRadians(headingSupplier.getAsDouble()), 0.0);

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rotSpeed,
            Rotation2d.fromDegrees(headingSupplier.getAsDouble())
        );

        output.accept(speeds);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint()
            && yController.atSetpoint()
            && rotController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        output.accept(new ChassisSpeeds(0, 0, 0));
    }
}
