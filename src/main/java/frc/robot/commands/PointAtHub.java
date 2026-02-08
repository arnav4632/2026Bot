package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.DoubleSupplier;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.MatchInfo;

import static edu.wpi.first.units.Units.*;

public class PointAtHub extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier vxSupplier;
    private final DoubleSupplier vySupplier;
    private final CommandXboxController driverController;
    private double hubX;
    private double hubY;

    // Reuse the request object to improve performance (avoid garbage collection)
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Maximum angular speed
    private final double maxAngularRateRadPerSec = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Proportional gain for heading controller
    private static final double kP_angle = 3.0;

    public PointAtHub(CommandSwerveDrivetrain drivetrain, CommandXboxController driverController, DoubleSupplier vxSupplier, DoubleSupplier vySupplier) {
        this.drivetrain = drivetrain;
        this.driverController = driverController;
        this.vxSupplier = vxSupplier;
        this.vySupplier = vySupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        MatchInfo.getInstance().ensureInitialized();
        var optAlliance = MatchInfo.getInstance().getOwnAlliance();

        if (optAlliance.isPresent()) {
            Alliance alliance = optAlliance.get();
            if (alliance == Alliance.Red) {
                this.hubX = Constants.Shooter.redGoalX;
                this.hubY = Constants.Shooter.redGoalY;
            } else {
                this.hubX = Constants.Shooter.blueGoalX;
                this.hubY = Constants.Shooter.blueGoalY;
            }

            Pose2d pose = drivetrain.getEstimatedPose();
            double x = pose.getX();
            double blueBoundary = Constants.Shooter.blueXBoundary;
            double redBoundary = Constants.Shooter.redXBoundary;

            boolean inNeutral = (x > blueBoundary && x < redBoundary);
            boolean inWrongZone = (alliance == Alliance.Blue && x >= redBoundary) || (alliance == Alliance.Red && x <= blueBoundary);

            if (inNeutral || inWrongZone) {
                rumbleDriver(0.5);
            }
        } else {
            // Fallback: choose closest hub
            Pose2d pose = drivetrain.getEstimatedPose();
            double bx = Constants.Shooter.blueGoalX;
            double by = Constants.Shooter.blueGoalY;
            double rx = Constants.Shooter.redGoalX;
            double ry = Constants.Shooter.redGoalY;

            double distToBlue = Math.hypot(pose.getX() - bx, pose.getY() - by);
            double distToRed = Math.hypot(pose.getX() - rx, pose.getY() - ry);

            if (distToRed < distToBlue) {
                this.hubX = rx;
                this.hubY = ry;
            } else {
                this.hubX = bx;
                this.hubY = by;
            }
        }
    }

    private void rumbleDriver(double seconds) {
        var hid = driverController.getHID();
        Commands.sequence(
            Commands.runOnce(() -> { hid.setRumble(RumbleType.kLeftRumble, 1.0); hid.setRumble(RumbleType.kRightRumble, 1.0); }),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> { hid.setRumble(RumbleType.kLeftRumble, 0.0); hid.setRumble(RumbleType.kRightRumble, 0.0); })
        ).schedule();
    }

    @Override
    public void execute() {
        Pose2d pose = drivetrain.getEstimatedPose();

        // 1. Get Inputs
        double vx = vxSupplier.getAsDouble();
        double vy = vySupplier.getAsDouble();

        // 2. Calculate vector to hub
        double rx = hubX - pose.getX();
        double ry = hubY - pose.getY();
        double distSq = rx * rx + ry * ry;

        // Calculate Angular Feedforward (compensating for lateral translation)
        // if you're translating laterally, you need to rotate to keep facing the hub.
        //this will help to predict rather than just react. results in smoother rotation and better tracking, especially at higher speeds.
        // Formula: FF = (ry * vx - rx * vy) / radius^2
        double omegaFF = 0;
        omegaFF = (ry * vx - rx * vy) / distSq;

        // Calculate PID Feedback(compensate for heading error)
        double desiredYaw = Math.atan2(ry, rx);
        double currentYaw = pose.getRotation().getRadians();
        double angleError = MathUtil.angleModulus(desiredYaw - currentYaw);
        
        double pidOutput = kP_angle * angleError;

        // 5. Combine and Saturate
        double totalRotRate = pidOutput + omegaFF;
        totalRotRate = MathUtil.clamp(totalRotRate, -maxAngularRateRadPerSec, maxAngularRateRadPerSec);

        // 6. Apply Request
        drivetrain.setControl(driveRequest
            .withVelocityX(vx)
            .withVelocityY(vy)
            .withRotationalRate(totalRotRate));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds());
    }
}