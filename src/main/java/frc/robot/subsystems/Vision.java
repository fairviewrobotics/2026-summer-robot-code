package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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

public class Vision extends SubsystemBase {
    private final Swerve swerve;
    private final PhotonCamera[] cameras;
    private Pose3d[] cameraPoses;
    private AprilTagFieldLayout fieldLayout;
    private static Vision instance;

    public static Vision getInstance(Swerve swerve) {
        if (instance == null) {
            instance = new Vision(swerve);
        }
        return instance;
    }

    public Vision(Swerve swerve) {
        this.swerve = swerve;
        this.cameras = new PhotonCamera[] { new PhotonCamera("back_cam") };
        this.cameraPoses = getAdjustedCameraPoses();

        try {
            this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        } catch (Exception e) {
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
                Pose3d cameraPoseEstimation = MathUtils.getPose3dFromTransform3d(
                        latestResult.getMultiTagResult().get().estimatedPose.best);
                List<Pose3d> tagPoses = new ArrayList<>();
                double totalDistance = 0.0;
                for (int id : latestResult.getMultiTagResult().get().fiducialIDsUsed) {
                    if (fieldLayout.getTagPose(id).isEmpty()) {
                        continue;
                    }
                    Pose3d tagPose = fieldLayout.getTagPose(id).get();
                    double distance = tagPose.getTranslation().getDistance(cameraPoseEstimation.getTranslation());
                    if (distance < Preferences.getDouble("Vision/MAX_ACCEPTABLE_TAG_RANGE", VisionConstants.MAX_ACCEPTABLE_TAG_RANGE)) {
                        tagPoses.add(tagPose);
                        totalDistance += distance;
                    } else {
                        return;
                    }
                }
                if (tagPoses.isEmpty()) {
                    continue;
                }
                if (!latestResult.targets.isEmpty() && latestResult.targets.get(0).getPoseAmbiguity() > Preferences.getDouble("Vision/MAX_AMBIGUITY", VisionConstants.TAG_AMBIGUITY_TOLERANCE)) {
                    continue;
                }
                Pose2d robotPoseEstimation = cameraPoseEstimation
                        .transformBy(MathUtils.getTransform3dFromPose3d(cameraPoses[cameraIndex]).inverse())
                        .toPose2d();
                Pose3d robotPoseEstimation3d = cameraPoseEstimation
                        .transformBy(MathUtils.getTransform3dFromPose3d(cameraPoses[cameraIndex]).inverse());

                if (robotPoseEstimation.getX() < -FieldConstants.FIELD_BORDER_MARGIN_METERS || robotPoseEstimation.getX() > FieldConstants.FIELD_LENGTH_METERS + FieldConstants.FIELD_BORDER_MARGIN_METERS || robotPoseEstimation.getY() < -FieldConstants.FIELD_BORDER_MARGIN_METERS || robotPoseEstimation.getY() > FieldConstants.FIELD_WIDTH_METERS + FieldConstants.FIELD_BORDER_MARGIN_METERS) {
                    return;
                }
                if (robotPoseEstimation3d.getZ() > Preferences.getDouble("Vision/MAX_Z_ERROR", VisionConstants.MAX_Z_ERROR)) {
                    return;
                }
                double avgDistance = totalDistance / tagPoses.size();
                double xyStdDev = VisionConstants.BASE_VISION_XY_STD_DEV * (Math.pow(avgDistance, 2.0) / tagPoses.size());
                double thetaStdDev = VisionConstants.BASE_VISION_THETA_STD_DEV * (Math.pow(avgDistance, 2.0) / tagPoses.size());
                swerve.addVisionMeasurement(
                        robotPoseEstimation, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
            } else {
                PhotonTrackedTarget target = latestResult.targets.get(0);
                if (fieldLayout.getTagPose(target.getFiducialId()).isEmpty()) {
                    continue;
                }
                Pose3d singleTagPose = fieldLayout.getTagPose(target.getFiducialId()).get();
                Pose3d cameraPose = singleTagPose.transformBy(target.getBestCameraToTarget().inverse());
                Pose3d robotPoseEstimation = cameraPose
                        .transformBy(MathUtils.getTransform3dFromPose3d(cameraPoses[cameraIndex]).inverse());
                double distance = singleTagPose.getTranslation().getDistance(cameraPose.getTranslation());
                Logger.recordOutput("Vision/Distance to tag", distance);

                boolean rejectPose = Preferences.getDouble("Vision/MAX_ACCEPTABLE_TAG_RANGE", VisionConstants.MAX_ACCEPTABLE_TAG_RANGE) < distance || robotPoseEstimation.getX() < -FieldConstants.FIELD_BORDER_MARGIN_METERS || robotPoseEstimation.getX() > FieldConstants.FIELD_LENGTH_METERS + FieldConstants.FIELD_BORDER_MARGIN_METERS || robotPoseEstimation.getY() < -FieldConstants.FIELD_BORDER_MARGIN_METERS || robotPoseEstimation.getY() > FieldConstants.FIELD_WIDTH_METERS + FieldConstants.FIELD_BORDER_MARGIN_METERS || robotPoseEstimation.getZ() > VisionConstants.MAX_Z_ERROR;
                if (rejectPose) {
                    return;
                }
                double xyStdDev = Preferences.getDouble("Vision/BASE_XY_STD_DEV", VisionConstants.BASE_VISION_XY_STD_DEV) * Math.pow(distance, 2.0);
                double thetaStdDev = VisionConstants.SINGLE_TAG_DISTRUST_COEFFICIENT * VisionConstants.BASE_VISION_THETA_STD_DEV * Math.pow(distance, 2.0);
                swerve.addVisionMeasurement(
                        robotPoseEstimation.toPose2d(), timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
            }
        }
    }

    public Pose2d getRobotPose() {
        return swerve.getPose();
    }

    public void resetPose(Pose2d pose) {
        swerve.resetOdometry(pose);
    }
}
