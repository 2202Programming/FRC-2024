// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Shooter.RPMShooter;
import frc.robot.commands.Swerve.AllianceAwareGyroReset;
import frc.robot.commands.Swerve.FieldCentricDrive;
import frc.robot.commands.Swerve.RobotCentricDrive;
import frc.robot.subsystems.BlinkyLights.BlinkyLightUser;
import frc.robot.subsystems.Sensors.Limelight_Subsystem;
import frc.robot.subsystems.Sensors.Sensors_Subsystem;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.util.RobotSpecs;
import frc.robot.Constants.CAN;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements BlinkyLightUser {

  // enum for bindings add when needed
  public enum Bindings {
    DriveTest, Shooter_test
    Comptition
  }

  // The robot's subsystems and commands are defined here...
  static RobotContainer rc;
  public final RobotSpecs robotSpecs;

  // Subsystems use locally or in RC() reference
  //public final PneumaticsControl pneumatics;
  public final HID_Xbox_Subsystem dc;
  public final Limelight_Subsystem limelight;
  public final Sensors_Subsystem sensors;
  public final SwerveDrivetrain drivetrain;
  //public final BlinkyLights lights;
  //public final Intake intake;
  //public final Shooter shooter;

  // singleton accessor for robot public sub-systems
  public static RobotContainer RC() {
    return rc;
  }

  public static Subsystem getSubSys(String name) {
    return rc.robotSpecs.mySubsystemConfig.getSubsystem(name);
  }
  
  public static <T> Object getSubSys(Class<? extends Subsystem> clz) {
    return rc.robotSpecs.mySubsystemConfig.getSubsystem(clz.getSimpleName().toUpperCase());
  }
  
  public static boolean hasSubsystem(Class<? extends Subsystem> clz) {
    return rc.robotSpecs.mySubsystemConfig.hasSubsystem(clz);
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    RobotContainer.rc = this;
    robotSpecs = new RobotSpecs();
    robotSpecs.mySubsystemConfig.constructAll();
    
    // Quiet some of the noise
    DriverStation.silenceJoystickConnectionWarning(true);

    // get subsystem vars as needed
    drivetrain =(SwerveDrivetrain) getSubSys("DRIVETRAIN");
    dc = (HID_Xbox_Subsystem) getSubSys("DC");
    limelight = (Limelight_Subsystem) getSubSys("LIMELIGHT");
    sensors = (Sensors_Subsystem) getSubSys("SENSORS");

    /* Set the commands below */
    configureBindings(Bindings.Shooter_test); // Change this to swich between bindings
    if (drivetrain != null) {
      drivetrain.setDefaultCommand(new FieldCentricDrive(drivetrain));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  private void configureBindings(Bindings bindings) {
    CommandXboxController driver = dc.Driver();
    CommandXboxController operator = dc.Operator();

    switch (bindings) {
      case DriveTest:
        driver.leftTrigger().whileTrue(new RobotCentricDrive(drivetrain, dc));
        driver.b().onTrue(new AllianceAwareGyroReset(false));

        // This appears to break if initial pose is too close to path start pose
        // (zero-length path?)
        driver.a().onTrue(new SequentialCommandGroup(
            new InstantCommand(RobotContainer.RC().drivetrain::printPose),
            AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("test_1m"),
                new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720))),
            new InstantCommand(RobotContainer.RC().drivetrain::printPose)));

        driver.x().onTrue(new SequentialCommandGroup(
            new InstantCommand(RobotContainer.RC().drivetrain::printPose),
            AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(1.73, 5.38), new Rotation2d(0.0)),
                new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720))),
            new InstantCommand(RobotContainer.RC().drivetrain::printPose)));

        // driver.x().whileTrue(new Lights(BlinkyLights.GREEN));
        // driver.leftBumper().whileTrue(new Lights(BlinkyLights.RED));
        // driver.y().whileTrue(new Lights(BlinkyLights.WHITE));
      
      case Shooter_test:
        if(shooter != null){
          shooter.setDefaultCommand(new RPMShooter(operator, shooter));
        }
        driver.b().onTrue(new InstantCommand(() -> {shooter.cycleShootingMode();}));
    }
        // break; fall through since these are placeholders on CompBot

      case Comptition:
        // TODO: replace Print/Dummy with real commands when known - ER
        driver.rightTrigger().whileTrue(new DummyShooterCmd());
        driver.leftTrigger().onTrue(new PrintCommand("PlaceholderCMD: Align with shooter"));

      default:
        break;
    }
    configureOperator(bindings);
  }

  private void configureOperator(Bindings bindings) {
    CommandXboxController operator = dc.Operator();

    switch (bindings) {
      // all the same for now since they are placeholders -- fall through ok
      default:
      case DriveTest:
      case Comptition:
        operator.rightBumper().onTrue(new PrintCommand("PlaceholderCMD: Intake Motor On"));
        
        //TODO mentor pls check if right syntax!!
        operator.x().whileTrue(new PrintCommand("PlaceholderCMD: Intake Deploy"));
        operator.x().whileFalse(new PrintCommand("PlaceholderCMD: Intake Retract"));

        //Drive team mentioned that they want climber buttons on switchboard but i need to find that syntax -ER
        //WIP THESE BINDINGS ARE NOT AT ALL FINAL
        operator.povUp().onTrue(new PrintCommand("PlaceholderCMD: Climber UP"));
        operator.povDown().onTrue(new PrintCommand("PlaceholderCMD: Climber Down"));

        break;
    }
  }
}
