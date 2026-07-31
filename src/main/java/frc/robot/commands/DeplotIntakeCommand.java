package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class DeplotIntakeCommand extends Command {
    private final Intake intake;

    public DeplotIntakeCommand(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute(){
        intake.setDeployMotor(IntakeConstants.INTAKE_DEPLOY_POSITION);
    }

    @Override
    public void end(boolean interrupted){
        intake.setDeployMotor(0);
    }

}
