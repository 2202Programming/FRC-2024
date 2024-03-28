package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.EjectNote;
import frc.robot.commands.Intake.IntakeSequence;
import frc.robot.commands.Shooter.ShooterSequence;
import frc.robot.commands.Shooter.ShooterServoSequence;
import frc.robot.commands.Swerve.RotateTo;
import frc.robot.commands.Swerve.RotateUntilSeeTags;
/*
 * Place commands named in PathPlaner autos here.
 */
public class RegisteredCommands {

    public static SendableChooser<Command> RegisterCommands() {
        SendableChooser<Command> autoChooser;

        // NamedCommands for use in PathPlanner scripts.
        NamedCommands.registerCommand("pickup", new IntakeSequence(true));
        NamedCommands.registerCommand("eject", new EjectNote());
        if (RobotContainer.getRobotSpecs().getRobotNameString().equals("CompetitionBotAlpha2024")) {// Just for alpha
            NamedCommands.registerCommand("shoot", new ShooterSequence(true, 3500.0));
            NamedCommands.registerCommand("angle_shoot",
                    new SequentialCommandGroup(new RotateTo(), new ShooterSequence(3200.0)));
        } else {
            NamedCommands.registerCommand("shoot", new ShooterServoSequence(true));
            // TODO: CHANGE THE TAG BASED ON ALLIANCE
            NamedCommands.registerCommand("angle_shoot",
                    new SequentialCommandGroup(new RotateUntilSeeTags(), new ShooterServoSequence(true)));
            NamedCommands.registerCommand("RotateTo", new RotateUntilSeeTags());
        }
        autoChooser = AutoBuilder.buildAutoChooser();
        // select our auto
        SmartDashboard.putData("Auto Chooser", autoChooser);
        return autoChooser;
    }
}