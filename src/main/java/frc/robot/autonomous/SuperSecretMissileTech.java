package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.routines.BadDoubleSwiple;
import frc.robot.autonomous.routines.BoxTest;
import frc.robot.autonomous.routines.DoubleSwipeOverTrench;
import frc.robot.subsystems.*;


public class SuperSecretMissileTech {

    private final SendableChooser<SequentialCommandGroup> superSecretMissileTech = new SendableChooser<>();

    public SuperSecretMissileTech(Swerve swerve, Hood hood, Shooter shooter, Intake intake, Hopper hopper) {
        superSecretMissileTech.setDefaultOption("NOTHING", new SequentialCommandGroup());
        superSecretMissileTech.addOption("BOX TEST", new BoxTest(swerve));
        superSecretMissileTech.addOption("BAD DOUBLE SWIPE", new BadDoubleSwiple(swerve, hood, shooter, intake, hopper));
        superSecretMissileTech.addOption("DOUBLE SWIPE", new DoubleSwipeOverTrench(swerve, shooter, hood, intake));
        SmartDashboard.putData("Autonomous Selector", superSecretMissileTech);
    }

    public SequentialCommandGroup getSelected() {
        return superSecretMissileTech.getSelected();
    }

}