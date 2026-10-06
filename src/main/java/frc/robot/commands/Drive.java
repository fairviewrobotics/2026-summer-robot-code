package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.AllianceFlipUtil;

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
        double xSpeed = Math.pow(MathUtil.applyDeadband(xVel.getAsDouble(), 0.1), 3);
        double ySpeed = Math.pow(MathUtil.applyDeadband(yVel.getAsDouble(), 0.1), 3);
        double turningSpeed = MathUtil.applyDeadband(omega.getAsDouble(), 0.1);

        // Scaling
        double xVelocity = xSpeed * SwerveConstants.MAX_SPEED * 0.6;
        double yVelocity = ySpeed * SwerveConstants.MAX_SPEED * 0.6;
        double turningVelocity = turningSpeed * (Math.PI * 2) * 0.5;
//
//        if (AllianceFlipUtil.shouldFlip()) {
//            xVelocity = -xVelocity;
//            yVelocity = -yVelocity;
//        }

        swerve.drive(xVelocity, yVelocity, turningVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0);
    }
}