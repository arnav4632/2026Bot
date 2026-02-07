package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.*;

/**
 * Command that points the robot toward the hub while still allowing the
 * driver to translate the robot using the left stick. The driver's rotation
 * input is ignored and replaced with a P-controlled rotational rate that
 * turns the robot to face the hub (based on alliance-specific hub coordinates
 * in {@link Constants.Shooter} and our currently estimated pose from {@link CommandSwerveDrivetrain#getEstimatedPose()}).
 */
public class PointAtHub extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier vxSupplier;
    private final DoubleSupplier vySupplier;
    private final double hubX;
    private final double hubY;

    // Maximum angular speed (match value used in RobotContainer)
    private final double maxAngularRateRadPerSec = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    

    // Proportional gain for heading controller (rad/s per rad of error)
    private static final double kP_angle = 3.0;

    public PointAtHub(CommandSwerveDrivetrain drivetrain, DoubleSupplier vxSupplier, DoubleSupplier vySupplier) {
        this.drivetrain = drivetrain;
        this.vxSupplier = vxSupplier;
        this.vySupplier = vySupplier;

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance == Alliance.Red) {
            this.hubX = Constants.Shooter.redGoalX;
            this.hubY = Constants.Shooter.redGoalY;
        } else {
            this.hubX = Constants.Shooter.blueGoalX;
            this.hubY = Constants.Shooter.blueGoalY;
        }
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // Current robot pose
        Pose2d pose = drivetrain.getEstimatedPose();

        // Vector from robot to hub 
    double dx = hubX - pose.getX();
    double dy = hubY - pose.getY();

        // Desired heading (angle to hub)
        double desiredYaw = Math.atan2(dy, dx);
        double currentYaw = pose.getRotation().getRadians();

        // Compute smallest angle error (-PI .. PI) to see which way to turn
        double angleError = desiredYaw - currentYaw;
        while (angleError > Math.PI) angleError -= 2.0 * Math.PI;
        while (angleError < -Math.PI) angleError += 2.0 * Math.PI;

        // P-controller for rotational rate
        double rotRate = kP_angle * angleError;
        // Saturate
        if (rotRate > maxAngularRateRadPerSec) rotRate = maxAngularRateRadPerSec;
        if (rotRate < -maxAngularRateRadPerSec) rotRate = -maxAngularRateRadPerSec;

    // Translation supplied by RobotContainer (already processed: deadband, slew, scaling)
    double vx = vxSupplier.getAsDouble();
    double vy = vySupplier.getAsDouble();

        // Build and apply field-centric request
        var req = new SwerveRequest.FieldCentric()
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(rotRate)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        drivetrain.setControl(req);
    }

    @Override
    public boolean isFinished() {
        return false; // default command-style: never finishes on its own
    }

    @Override
    public void end(boolean interrupted) {
        // Stop commanding speeds when command ends
        drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds());
    }
}



