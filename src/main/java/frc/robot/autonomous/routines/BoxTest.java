package frc.robot.autonomous.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPoint;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.AllianceFlipUtil;

import java.util.Set;

public class BoxTest extends SequentialCommandGroup {
    public BoxTest(Swerve swerve) {
        setName("BOX TEST");
        addCommands(
                Commands.defer(() -> {
                    return new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                Pose2d startPose = AllianceFlipUtil.apply(FieldConstants.START_POINT);
                                swerve.resetOdometry(startPose);
                            }),
                            new DriveToPoint(swerve, AllianceFlipUtil.apply(FieldConstants.TOP_RIGHT_POINT), 0.75),
                            new DriveToPoint(swerve, AllianceFlipUtil.apply(FieldConstants.BOTTOM_RIGHT_POINT), 0.75),
                            new DriveToPoint(swerve, AllianceFlipUtil.apply(FieldConstants.BOTTOM_LEFT_POINT), 0.75),
                            new DriveToPoint(swerve, AllianceFlipUtil.apply(FieldConstants.START_POINT), 0.75)
                    );

                }, Set.of(swerve))
        );
    }
}