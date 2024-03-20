package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Intake.IntakeSequence;
import frc.robot.commands.Shooter.ShooterServoSequence;
import frc.robot.commands.Swerve.RotateTo;
import frc.robot.commands.auto.AutoShooting;
import frc.robot.commands.auto.AutoShooting.ShootingTarget;

/*
 * Place commands named in PathPlaner autos here.
 */
public class RegisteredCommands {

    public static SendableChooser<Command> RegisterCommands() {
        SendableChooser<Command> autoChooser;
        
        // NamedCommands for use in PathPlanner scripts.
        NamedCommands.registerCommand("pickup", new IntakeSequence(true));
        NamedCommands.registerCommand("shoot", new ShooterServoSequence(46.0, 3000.0));
        NamedCommands.registerCommand("angle_shoot", new AutoShooting(ShootingTarget.Speaker, 38, 3200.0));
        NamedCommands.registerCommand("RotateTo", new RotateTo());
        autoChooser = AutoBuilder.buildAutoChooser(); 
        // select our auto
        SmartDashboard.putData("Auto Chooser", autoChooser);
        return autoChooser;
    }
}