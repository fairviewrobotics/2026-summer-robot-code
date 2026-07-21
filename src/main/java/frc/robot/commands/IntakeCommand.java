package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {

    private final Intake intake;

    public IntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setDeployMotor(IntakeConstants.INTAKE_DEPLOY_POSITION);
        intake.setIntakeRollerMotorVoltage(4.0);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setDeployMotor(0.0);
        intake.setIntakeRollerMotorVoltage(0.0);
    }

}
