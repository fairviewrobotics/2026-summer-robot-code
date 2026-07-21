package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeDeployPreferences extends Command {
    private final Intake intake;

    public IntakeDeployPreferences(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void execute() { intake.setDeployMotor(Preferences.getDouble("Deploy/TARGET_ANGLE", 0.0)); }

    @Override
    public void end(boolean interrupted) { intake.setDeployMotor(0.0); }
}
