package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.utils.MathUtils;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Vision2 extends SubsystemBase {
    private final Swerve swerve;
    private final PhotonCamera[] cameras;
    private Pose3d[] cameraPoses;
    private AprilTagFieldLayout fieldLayout;
    private static Vision instance;

    public Vision2 (Swerve swerve)
    {
        this.swerve = swerve;
        this.cameras = new PhotonCamera[] {new PhotonCamera("back_cam")};
        this.cameraPoses = getAdjustedCameraPoses();

        try{
            this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        } catch(Exception e){
            DriverStation.reportError("Failed to load AprilTagFieldLayout!", e.getStackTrace());
        }
        Preferences.initDouble("Vision/BASE_XY_STD_DEV", VisionConstants.BASE_VISION_XY_STD_DEV);
        Preferences.initDouble("Vision/BASE_THETA_STD_DEV", VisionConstants.BASE_VISION_THETA_STD_DEV);
        Preferences.initDouble("Vision/MAX_Z_ERROR", VisionConstants.MAX_Z_ERROR);
        Preferences.initDouble("Vision/SINGLE_TAG_DISTRUST_COEFFICIENT", VisionConstants.SINGLE_TAG_DISTRUST_COEFFICIENT);
        Preferences.initDouble("Vision/MAX_AMBIGUITY", VisionConstants.TAG_AMBIGUITY_TOLERANCE);
        Preferences.initDouble("Vision/MAX_ACCEPTABLE_TAG_RANGE", VisionConstants.MAX_ACCEPTABLE_TAG_RANGE);
    }

    private Pose3d[] getAdjustedCameraPoses() {
            return new Pose3d[] {
                    new Pose3d(
                            Units.inchesToMeters(VisionConstants.BACK_CAM_POSE_X),
                            Units.inchesToMeters(VisionConstants.BACK_CAM_POSE_Y),
                            Units.inchesToMeters(VisionConstants.BACK_CAM_POSE_Z),
                            new Rotation3d(
                                    Units.degreesToRadians(VisionConstants.BACK_CAM_POSE_ROLL),
                                    Units.degreesToRadians(VisionConstants.BACK_CAM_POSE_PITCH),
                                    Units.degreesToRadians(VisionConstants.BACK_CAM_POSE_YAW)))

        };
    }


    @Override
    public void periodic() {
        if (fieldLayout == null) return;
        updatePose();
    }

    public void updatePose() {
        this.cameraPoses = getAdjustedCameraPoses();
        for (int cameraIndex = 0; cameraIndex < cameraPoses.length; cameraIndex++) {
            List<PhotonPipelineResult> results = cameras[cameraIndex].getAllUnreadResults();
            if (results.isEmpty()) {
                continue;
            }
            PhotonPipelineResult latestResult = results.get(results.size() - 1);
            if (!latestResult.hasTargets()) {
                continue;
            }
            double timestamp = latestResult.getTimestampSeconds();
            boolean useMultitag = latestResult.multitagResult.isPresent();

            if (useMultitag) {
                Pose3d cameraPoseEsitmation = MathUtil.getPose3dFromTransform3d(
                        latestResult.getMultiTagResult().isEmpty())
                )
            }


        }
    }
}

