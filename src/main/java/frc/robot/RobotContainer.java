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
import frc.robot.commands.utility.DummyShooterCmd;
import frc.robot.subsystems.BlinkyLights.BlinkyLightUser;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Sensors.Limelight_Subsystem;
import frc.robot.subsystems.Sensors.Sensors_Subsystem;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.util.RobotSpecs;

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
    DriveTest, Shooter_test, Comptition
  }

  // The robot's subsystems and commands are defined here...
  static RobotContainer rc;
  public final RobotSpecs robotSpecs;

  // Subsystems use locally or in RC() reference
  // public final PneumaticsControl pneumatics;
  public final HID_Xbox_Subsystem dc;
  public final Limelight_Subsystem limelight;
  public final Sensors_Subsystem sensors;
  public final SwerveDrivetrain drivetrain;
  // public final BlinkyLights lights;
  // public final Intake intake;
  // public final Shooter shooter;

  // singleton accessor for robot public sub-systems
  //TODO make this @Deprecated
  public static RobotContainer RC() {
    return rc;
  }

  // The following methods are unchecked, but the SystemConfig class does
  // check the types. 
  // Use the string name when there are multiple instance of the subsystem
  @SuppressWarnings("unchecked")
  public static <T> T getSubsystem(String name) {
    return (T) rc.robotSpecs.mySubsystemConfig.getSubsystem(name);
  }

  // Use this when there is only one instance of the Subsystem
  @SuppressWarnings("unchecked")
  public static <T extends Subsystem> T getSubsystem(Class<T> clz) {
    return (T) rc.robotSpecs.mySubsystemConfig.getSubsystem(clz);
  }

  // Use this form when the RobotContainer object is NOT a Subsystem
  @SuppressWarnings("unchecked")
  public static <T> T getObject(String name) {
    return (T) rc.robotSpecs.mySubsystemConfig.getObject(name);
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
    drivetrain = (SwerveDrivetrain) getSubsystem("DRIVETRAIN");
    dc = getSubsystem("DC");
    limelight = getSubsystem(Limelight_Subsystem.class);
    sensors = getSubsystem(Sensors_Subsystem.class);

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

  private
   void configureBindings(Bindings bindings) {
    CommandXboxController driver = dc.Driver();

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
        break;

      case Comptition:
        // TODO: replace Print/Dummy with real commands when known - ER
        driver.rightTrigger().whileTrue(new DummyShooterCmd());
        driver.leftTrigger().onTrue(new PrintCommand("PlaceholderCMD: Align with shooter"));
        break;

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

        // TODO mentor pls check if right syntax!!
        operator.x().whileTrue(new PrintCommand("PlaceholderCMD: Intake Deploy"));
        operator.x().whileFalse(new PrintCommand("PlaceholderCMD: Intake Retract"));

        // Drive team mentioned that they want climber buttons on switchboard but i need
        // to find that syntax -ER
        // WIP THESE BINDINGS ARE NOT AT ALL FINAL
        operator.povUp().onTrue(new PrintCommand("PlaceholderCMD: Climber UP"));
        operator.povDown().onTrue(new PrintCommand("PlaceholderCMD: Climber Down"));

        break;
      case Shooter_test:
        var shooter = getSubsystem(Shooter.class);
        if (shooter != null) {
          shooter.setDefaultCommand(new RPMShooter(operator));
          operator.b().onTrue(new InstantCommand(() -> {
            shooter.cycleShootingMode();
          }));
        }
        break;
    }
  }
}
