package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

public class HopperTestCommand extends Command {
    private final Hopper hopper;
    private final double voltage;
    public HopperTestCommand(Hopper hopper, double voltage){
        this.hopper = hopper;
        this.voltage = voltage;
    }

    @Override
    public void execute() {
        hopper.setHopperRightMotorVoltage(voltage);
        hopper.setLeftHopperMotorVoltage(voltage);
    }

    @Override
    public void end(boolean interrupted) {
        hopper.setHopperRightMotorVoltage(0.0);
        hopper.setLeftHopperMotorVoltage(0.0);
    }
}
