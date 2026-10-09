// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import choreo.auto.AutoChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.SuperSecretMissileTech;
import frc.robot.commands.*;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.*;
import frc.robot.utils.AllianceFlipUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer
{

    final CommandPS5Controller primary_controller = new CommandPS5Controller(0);
    final CommandXboxController secondary_controller = new CommandXboxController(1);
    private final Swerve swerve = new Swerve();
    private final Shooter shooter = new Shooter();
    private final Kicker kicker = new Kicker();
    private final Hood hood = new Hood();
    private final Intake intake = new Intake();
    private final Hopper hopper = new Hopper();
    private final SuperSecretMissileTech superSecretMissileTech;
    private final Vision vision = new Vision(swerve);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        NamedCommands.registerCommand("AimAtTarget", new AimAtTarget(shooter, hood, swerve, kicker, hopper, intake, () -> AllianceFlipUtil.apply(FieldConstants.BLUE_HUB_POSE3D.toPose2d()), () -> 0.0, () -> 0.0));
        NamedCommands.registerCommand("Intake", new IntakeCommand(intake));
        NamedCommands.registerCommand("IntakeDeploy", new IntakeDeployVoltageCommand(intake,4.0).withTimeout(2.0));
        NamedCommands.registerCommand("ResetGyro", new InstantCommand(swerve::resetGyro));


        superSecretMissileTech = new SuperSecretMissileTech(swerve, hood, shooter, intake, hopper);

        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
    }


    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
     * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
     */

    private void configureBindings()
    {
        swerve.setDefaultCommand(
                new Drive(
                        swerve,
                        () -> -primary_controller.getLeftY(),
                        () -> -primary_controller.getLeftX(),
                        () -> -primary_controller.getRightX()
                )
        );
        primary_controller.options().onTrue(new InstantCommand(swerve::zeroGyro));
        primary_controller.L2().whileTrue(new IntakeRollerVoltageCommand(intake, 6));
        primary_controller.R2().whileTrue(new AimAtTarget(shooter, hood, swerve, kicker, hopper, intake, () -> AllianceFlipUtil.apply(FieldConstants.BLUE_HUB_POSE3D.toPose2d()), () -> 0.0, () -> 0.0));
        primary_controller.R3().whileTrue(new RunCommand(swerve::xWheels, swerve));
        secondary_controller.rightStick().onTrue(new RefreshPreferences(swerve, intake, shooter, hood, kicker));
        secondary_controller.leftStick().onTrue(new InstantCommand(intake::resetDeployMotor, intake));
        secondary_controller.b().whileTrue(new IntakeDeployVoltageCommand(intake,6.0));
        secondary_controller.a().whileTrue(new AgitateIntakeCommand(intake));
        secondary_controller.leftBumper().whileTrue(new IntakeRollerVoltageCommand(intake, -12.0));
        secondary_controller.rightBumper().whileTrue(new ShooterPreferencesCommand(shooter));
        secondary_controller.x().whileTrue(new IntakeDeployVoltageCommand(intake,-6.0));
        secondary_controller.y().whileTrue(new KickerWithRPM(kicker));
        secondary_controller.rightTrigger().whileTrue(new HoodPreferencesCommand(hood));

    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command getAutonomousCommand()
    {
        return superSecretMissileTech.getSelected();
    }

}