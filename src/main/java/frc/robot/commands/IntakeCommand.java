package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {

    private final Intake intake;
    private final Timer timer = new Timer();

    public IntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if(timer.get() < 3) {
            intake.setDeployMotorVoltage(6);
        }
        else{
            intake.setDeployMotorVoltage(0);
        }
        intake.setIntakeRollerMotorVoltage(10.0);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        intake.setDeployMotorVoltage(0.0);
        intake.setIntakeRollerMotorVoltage(0.0);
    }

}
