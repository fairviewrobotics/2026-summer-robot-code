package frc.robot.subsystems;

import com.studica.frc.AHRS;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SwerveConstants;
import frc.robot.utils.SwerveModuleConfig;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {

    private final SwerveModule frontLeft = new SwerveModule(SwerveConstants.FRONT_LEFT_CONFIG);
    private final SwerveModule frontRight = new SwerveModule(SwerveConstants.FRONT_RIGHT_CONFIG);
    private final SwerveModule backLeft = new SwerveModule(SwerveConstants.BACK_LEFT_CONFIG);
    private final SwerveModule backRight = new SwerveModule(SwerveConstants.BACK_RIGHT_CONFIG);
    private Field2d field = new Field2d();

    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    private SimDeviceSim gyroSim;
    private SimDouble gyroYawSim;

    public Swerve() {
        initializePreferences();

        gyroSim = new SimDeviceSim("navX-Sensor", gyro.getPort());
        if (gyroSim != null) {
            gyroYawSim = gyroSim.getDouble("Yaw");
        }
    }

    private final SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(
                    SwerveConstants.swerveDriveKinematics,
                    gyro.getRotation2d(),
                    new SwerveModulePosition[] {
                            frontLeft.getPosition(),
                            frontRight.getPosition(),
                            backLeft.getPosition(),
                            backRight.getPosition()
                    },
                    Pose2d.kZero,
                    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                    VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    public void drive(double xVel, double yVel, double omega) {
        var swerveModuleStates =
                SwerveConstants.swerveDriveKinematics.toSwerveModuleStates(
                        ChassisSpeeds.discretize(
                                ChassisSpeeds.fromFieldRelativeSpeeds(
                                        xVel, yVel, omega, getPose().getRotation()),
                                0.02
                        )
                );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    private void initializePreferences() {
        Preferences.initDouble("Swerve/DriveP", SwerveConstants.DRIVE_P);
        Preferences.initDouble("Swerve/DriveD", SwerveConstants.DRIVE_D);
        Preferences.initDouble("Swerve/DriveKV", SwerveConstants.DRIVE_KV);
        Preferences.initDouble("Swerve/TurningP", SwerveConstants.TURNING_P);
        Preferences.initDouble("Swerve/TurningD", SwerveConstants.TURNING_D);
    }

    public void updatePreferences() {
        frontLeft.refreshPreferences();
        frontRight.refreshPreferences();
        backLeft.refreshPreferences();
        backRight.refreshPreferences();
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3,N1> stdDevs) {
        poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
    }

    public ChassisSpeeds getFieldVelocity() {
        ChassisSpeeds robotRelativeSpeeds = SwerveConstants.swerveDriveKinematics.toChassisSpeeds(
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        );

        return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, gyro.getRotation2d());
    }

    public void zeroGyro() {
        gyro.reset();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    @Override
    public void periodic() {
        poseEstimator.update(
                gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                }
        );

        Logger.recordOutput("Swerve/Pose", getPose());
        SmartDashboard.putData("Swerve/Field", field);
        field.setRobotPose(poseEstimator.getEstimatedPosition());
        Logger.recordOutput("Swerve/ModuleStates",
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState());
    }

    @Override
    public void simulationPeriodic() {
        ChassisSpeeds chassisSpeeds = SwerveConstants.swerveDriveKinematics.toChassisSpeeds(
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        );

        double dt = 0.02; 
        double angleDeltaDegrees = Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond * dt);

        if (gyroYawSim != null) {
            gyroYawSim.set(gyroYawSim.get() - angleDeltaDegrees);
        }
    }
}