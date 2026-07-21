package frc.robot.autonomous.routines;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimAtTarget;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.DriveToPointContinuous;
import frc.robot.commands.IntakeCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.AllianceFlipUtil;

public class DoubleSwipeOverTrench extends SequentialCommandGroup {
    public DoubleSwipeOverTrench(Swerve swerve, Shooter shooter, Hood hood) {
        setName("BOX TEST");
        addCommands(
                Commands.defer(() -> {
                    return new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                Pose2d startPose = AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT);
                                swerve.resetOdometry(startPose);
                            }),
                            new DriveToPointContinuous(swerve, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TRANSITION_PICKUP), SwerveConstants.MAX_SPEED),
                            new ParallelDeadlineGroup(
                                    new DriveToPointContinuous(swerve, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TRANSITION_DROPOFF), SwerveConstants.MAX_SPEED),
                                    new IntakeCommand()
                            ),
                            new DriveToPointContinuous(swerve, AllianceFlipUtil.apply(FieldConstants.BLUE_BUMP_LEFT_PASSAGE_POINT), SwerveConstants.MAX_SPEED),
                            new DriveToPoint(swerve, AllianceFlipUtil.apply(FieldConstants.BLUE_BUMP_LEFT_SHOOT_POINT), SwerveConstants.MAX_SPEED),
                            new AimAtTarget(shooter, hood, swerve, () -> AllianceFlipUtil.apply(FieldConstants.BLUE_HUB_POSE3D.toPose2d()), () -> 0, () -> 0),
                            new DriveToPointContinuous(swerve, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT), SwerveConstants.MAX_SPEED),
                            new DriveToPointContinuous(swerve, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_SECOND_SWIPE_START), SwerveConstants.MAX_SPEED),
                            new ParallelDeadlineGroup(
                                    new IntakeCommand(),
                                    new DriveToPointContinuous(swerve, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_SECOND_SWIPE_END), SwerveConstants.MAX_SPEED)
                            ),
                            new DriveToPointContinuous(swerve, AllianceFlipUtil.apply(FieldConstants.BLUE_BUMP_LEFT_PASSAGE_POINT), SwerveConstants.MAX_SPEED),
                            new DriveToPoint(swerve, AllianceFlipUtil.apply(FieldConstants.BLUE_BUMP_LEFT_SHOOT_POINT), SwerveConstants.MAX_SPEED),
                            new AimAtTarget(shooter, hood, swerve, () -> AllianceFlipUtil.apply(FieldConstants.BLUE_HUB_POSE3D.toPose2d()), () -> 0, () -> 0)
                    );
                }, Set.of(swerve, shooter, hood))
        );
    }
}