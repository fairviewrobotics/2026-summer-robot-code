package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;

public class Vision extends SubsystemBase {

    Swerve swerve;

    public Vision(Swerve swerve) {
        this.swerve = swerve;
    }

    private double calculateStdDev(double avgDistance, int tagCount) {
        double xyStdDev = Preferences.getDouble("Vision/XYStdDev", VisionConstants.BASE_XY_STD_DEV);

        xyStdDev *= Math.pow(avgDistance, 2);

        if (tagCount > 1) {
            xyStdDev /= (tagCount * 1.5);
        }

        return xyStdDev;
    }

    public void updatePose() {

        double[] botpose = NetworkTableInstance.getDefault()
                .getTable("limelight")
                .getEntry("botpose_wpiblue")
                .getDoubleArray(new double[0]);

        if (botpose.length > 0 && botpose[7] >= 1) {
            double x = botpose[0];
            double y = botpose[1];
            double timestamp = Timer.getFPGATimestamp() - (botpose[6] / 1000.0);

            Pose2d robotPose = new Pose2d(x, y, Rotation2d.fromDegrees(botpose[5]));

            double avgDistance = botpose[9];
            double xyStdDev = calculateStdDev(avgDistance, (int)botpose[7]);
            double thetaStdDev = calculateStdDev(avgDistance, (int)botpose[8]);

            swerve.addVisionMeasurement(
                    robotPose,
                    timestamp,
                    VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
            );
        }
    }

    public void initializePreferences() {
        Preferences.initDouble("Vision/XYStdDev", VisionConstants.BASE_XY_STD_DEV);
        Preferences.initDouble("Vision/ThetaStdDev", VisionConstants.BASE_THETA_STD_DEV);
    }

    @Override
    public void periodic() {
        updatePose();
    }

}
