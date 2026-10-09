package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
            superSecretMissileTech.setDefaultOption("NOTHING", new SequentialCommandGroup());
        }
        superSecretMissileTech.addOption("NOTHING", new SequentialCommandGroup());
        superSecretMissileTech.addOption("LET CHOPPED DOUBLE SWIPE", new PathPlannerAuto("Left Chopped Double Swipe", false));
        superSecretMissileTech.addOption("RIGHT CHOPPED DOUBLE SWIPE", new PathPlannerAuto("Left Chopped Double Swipe", true));
        superSecretMissileTech.addOption("BOX TEST", new BoxTest(swerve));
        SmartDashboard.putData("Autonomous Selector", superSecretMissileTech);
    }

    public Command getSelected() {
        return superSecretMissileTech.getSelected();
    }

}