package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.routines.BadDoubleSwiple;
import frc.robot.autonomous.routines.BoxTest;
import frc.robot.autonomous.routines.DoubleSwipeOverTrench;
import frc.robot.subsystems.*;

public class SuperSecretMissileTech {

    private final SendableChooser<Command> superSecretMissileTech;

    public SuperSecretMissileTech(Swerve swerve, Hood hood, Shooter shooter, Intake intake, Hopper hopper) {
        if (AutoBuilder.isConfigured()) {
            superSecretMissileTech = AutoBuilder.buildAutoChooser();
        } else {
            superSecretMissileTech = new SendableChooser<>();
        }
        superSecretMissileTech.setDefaultOption("NOTHING", new SequentialCommandGroup());
        superSecretMissileTech.addOption("BOX TEST", new BoxTest(swerve));
        superSecretMissileTech.addOption("BAD DOUBLE SWIPE", new BadDoubleSwiple(swerve, hood, shooter, intake, hopper));
        superSecretMissileTech.addOption("DOUBLE SWIPE", new DoubleSwipeOverTrench(swerve, shooter, hood, intake));
        SmartDashboard.putData("Autonomous Selector", superSecretMissileTech);
    }

    public Command getSelected() {
        return superSecretMissileTech.getSelected();
    }

}