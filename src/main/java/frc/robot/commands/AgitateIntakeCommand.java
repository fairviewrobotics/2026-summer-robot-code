package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import org.littletonrobotics.junction.Logger;

public class AgitateIntakeCommand extends Command {
    private final Intake intake;
    private final Timer timer = new Timer();
    private final double defaultAmplitude;
    private final double defaultFrequency;

    public AgitateIntakeCommand(Intake intake, double defaultAmplitude, double defaultFrequency) {
        this.intake = intake;
        this.defaultAmplitude = defaultAmplitude;
        this.defaultFrequency = defaultFrequency;
        addRequirements(intake);

        Preferences.initDouble("AgitateIntake/AmplitudeVolts", defaultAmplitude);
        Preferences.initDouble("AgitateIntake/FrequencyHz", defaultFrequency);
    }

    public AgitateIntakeCommand(Intake intake) {
        this(intake, 3.0, 2.0);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double amplitude = Preferences.getDouble("AgitateIntake/AmplitudeVolts", defaultAmplitude);
        double frequency = Preferences.getDouble("AgitateIntake/FrequencyHz", defaultFrequency);

        double elapsedTime = timer.get();
        double voltage = amplitude * Math.sin(2.0 * Math.PI * frequency * elapsedTime);

        intake.setDeployMotorVoltage(voltage);

        Logger.recordOutput("AgitateIntake/Voltage", voltage);
        Logger.recordOutput("AgitateIntake/Amplitude", amplitude);
        Logger.recordOutput("AgitateIntake/Frequency", frequency);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        intake.setDeployMotorVoltage(0.0);
    }
}
