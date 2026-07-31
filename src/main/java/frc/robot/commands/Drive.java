package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class Drive extends Command {
    private final Swerve swerve;
    private final DoubleSupplier xVel, yVel, omega;

    public Drive(
            Swerve swerve,
            DoubleSupplier xVel,
            DoubleSupplier yVel,
            DoubleSupplier omega
    ) {
        this.swerve = swerve;
        this.xVel = xVel;
        this.yVel = yVel;
        this.omega = omega;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double xSpeed = xVel.getAsDouble();
        double ySpeed = yVel.getAsDouble();
        double turningSpeed = omega.getAsDouble();

        xSpeed = MathUtil.applyDeadband(xSpeed, 0.1);
        ySpeed = MathUtil.applyDeadband(ySpeed, 0.1);
        turningSpeed = MathUtil.applyDeadband(turningSpeed, 0.1);

        // MoSim scaling
        double xVel = xSpeed * SwerveConstants.MAX_SPEED * ((double) 3 /5);
        double yVel = ySpeed * SwerveConstants.MAX_SPEED * ((double) 3 /5);
        double turningVel = turningSpeed * (Math.PI * 2) * 0.5;

        swerve.drive(xVel, yVel, turningVel);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0);
    }
}