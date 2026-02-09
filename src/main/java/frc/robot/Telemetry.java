package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;

public class Telemetry {
    private static Telemetry s_instance;
    private DoubleArrayLogEntry m_visionPoseLog;
    private DoubleArrayLogEntry m_visionStdDevLog;
    private DoubleLogEntry m_visionTimestampLog;
    private DoubleLogEntry m_batteryLog;
    // SysId logging entries
    private DoubleArrayLogEntry m_sysIdTranslationLog;
    private DoubleArrayLogEntry m_sysIdRotationLog;
    public static Telemetry getInstance() {
        if (s_instance == null) s_instance = new Telemetry();
        return s_instance;
    }

    public Telemetry() {
        if (Constants.Drive.log) {
            var log = DataLogManager.getLog(); //starts the log and gets the instance
            m_visionPoseLog = new DoubleArrayLogEntry(log, "Vision/Pose");
            m_visionStdDevLog = new DoubleArrayLogEntry(log, "Vision/StdDevs");
            m_visionTimestampLog = new DoubleLogEntry(log, "Vision/Timestamp");
            //tell the log to record everything in NetworkTables:
            DataLogManager.logNetworkTables(true); 
            m_batteryLog = new DoubleLogEntry(log, "System/BatteryVoltage");
            // SysId compatible logs: append arrays matching (voltage, position, velocity)
            // Translation: [voltage, position_meters, velocity_mps]
            m_sysIdTranslationLog = new DoubleArrayLogEntry(log, "SysId/Translation");
            // Rotation: [rotational_rate_as_voltage, pigeon_yaw_deg, pigeon_yaw_rate_deg_per_sec]
            m_sysIdRotationLog = new DoubleArrayLogEntry(log, "SysId/Rotation");
        }
    }

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();
    /** * Telemeterize the swerve drive state. 
     * Since NT logging is enabled, simply setting the NT values handles the file logging too.
     */
    public void telemeterize(SwerveDriveState state) {
        m_batteryLog.append(edu.wpi.first.wpilibj.RobotController.getBatteryVoltage());
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);
    }

    /**
     * SysId translation to WPILog.
     * The array layout is: [voltage, position_meters, velocity_mps].
     */
    public void logSysIdTranslation(double voltage, double positionMeters, double velocityMetersPerSec) {
        if (m_sysIdTranslationLog == null) return;
        m_sysIdTranslationLog.append(new double[] { voltage, positionMeters, velocityMetersPerSec });
    }

    /**
     * Log SysId rotation data to WPILog.
     * The array layout is: [rotational_rate_as_voltage, pigeon_yaw_degrees, pigeon_yaw_rate_deg_per_sec].
     * docs for sysID rotation say: set "voltage" = rotational_rate, "position" = pigeon_yaw, "velocity" = pigeon_yaw_rate
     * and scale position and velocity by pi/180 when importing.
     */
    public void logSysIdRotation(double rotationalRateAsVoltage, double pigeonYawDegrees, double pigeonYawRateDegPerSec) {
        if (m_sysIdRotationLog == null) return;
        m_sysIdRotationLog.append(new double[] { rotationalRateAsVoltage, pigeonYawDegrees, pigeonYawRateDegPerSec });
    }

    //log vision measurements
    public void logVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> visionStdDevs) {
        // Convert seconds to microseconds for the DataLog timestamp
        long timestampMicro = (long) (timestampSeconds * 1e6);

        //Pose: [X, Y, yaw]
        m_visionPoseLog.append(new double[] {
            visionPose.getX(), 
            visionPose.getY(), 
            visionPose.getRotation().getDegrees()
        }, timestampMicro);

        //std devs: [X, Y, Theta]
        if (visionStdDevs != null) {
            m_visionStdDevLog.append(new double[] {
                visionStdDevs.get(0, 0),
                visionStdDevs.get(1, 0),
                visionStdDevs.get(2, 0)
            }, timestampMicro);
        }

        m_visionTimestampLog.append(timestampSeconds, timestampMicro);
    }
}