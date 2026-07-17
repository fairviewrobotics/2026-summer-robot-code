package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.routines.ArcTestAuto;
import frc.robot.autonomous.routines.BoxTest;
import frc.robot.subsystems.Swerve;


public class SuperSecretMissileTech {

    private final SendableChooser<SequentialCommandGroup> superSecretMissileTech = new SendableChooser<>();

    public SuperSecretMissileTech(Swerve swerve) {
        superSecretMissileTech.setDefaultOption("NOTHING", new SequentialCommandGroup());
        superSecretMissileTech.addOption("BOX TEST", new BoxTest(swerve));
        superSecretMissileTech.addOption("ARC TEST AUTO", new ArcTestAuto(swerve));
        SmartDashboard.putData("Autonomous Selector", superSecretMissileTech);
    }

    public SequentialCommandGroup getSelected() {
        return superSecretMissileTech.getSelected();
    }

}