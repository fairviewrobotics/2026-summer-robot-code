package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.AimAtTarget;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.DriveToPointContinuous;
import frc.robot.commands.IntakeCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class BadDoubleSwiple extends SequentialCommandGroup {

    public BadDoubleSwiple(Swerve swerve, Hood hood, Shooter shooter, Intake intake) {
        addCommands(
            new InstantCommand(() -> swerve.resetOdometry(FieldConstants.BLUE_TRENCH_LEFT)),
            new DriveToPointContinuous(swerve, FieldConstants.BLUE_TRENCH_LEFT_INTAKE_START, SwerveConstants.MAX_SPEED),
            new ParallelDeadlineGroup(
                new DriveToPoint(swerve, FieldConstants.BLUE_TRENCH_LEFT_INTAKE_END, 1.0),
                new IntakeCommand(intake)
            ),
            new DriveToPoint(swerve, FieldConstants.BLUE_BUMP_LEFT_RETURN, 1.0),
            new DriveToPoint(swerve, FieldConstants.BLUE_LEFT_SHOOT_POSE, 1.0),

            new ParallelDeadlineGroup(
                new WaitCommand(4.0),
                new AimAtTarget(shooter, hood, swerve, FieldConstants.BLUE_HUB_POSE3D::toPose2d, () -> 0.0, () -> 0.0),
                new Hopper(hopper)
            ),

            new DriveToPointContinuous(swerve, FieldConstants.BLUE_BUMP_LEFT_DEPART, 1.0),
            new DriveToPoint(swerve, FieldConstants.BLUE_BUMP_LEFT_RETURN, 1.0),
            new DriveToPoint(swerve, FieldConstants.BLUE_LEFT_SHOOT_POSE, 1.0),

            new ParallelDeadlineGroup(
                new WaitCommand(4.0),
                new AimAtTarget(shooter, hood, swerve, FieldConstants.BLUE_HUB_POSE3D::toPose2d, () -> 0.0, () -> 0.0),
                new Hopper(hopper)
            )

        );
    }

}
