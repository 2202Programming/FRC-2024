// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.PDPMonitorCmd;
import frc.robot.commands.RandomLightsCmd;
import frc.robot.commands.Intake.AngleCalibration;
import frc.robot.commands.Intake.CalibratePos;
import frc.robot.commands.Intake.EjectNote;
import frc.robot.commands.Intake.IntakeSequence;
import frc.robot.commands.Intake.IntakeTest;
import frc.robot.commands.Intake.MoveToAnglePos;
import frc.robot.commands.Shooter.PneumaticsSequence;
import frc.robot.commands.Shooter.RPMShooter;
import frc.robot.commands.Shooter.ShooterSequence;
import frc.robot.commands.Swerve.AllianceAwareGyroReset;
import frc.robot.commands.Swerve.FaceToTag;
import frc.robot.commands.Swerve.FieldCentricDrive;
//todo re-enable after testing import frc.robot.commands.Swerve.FieldCentricDrive;
import frc.robot.commands.Swerve.RobotCentricDrive;
import frc.robot.commands.auto.AutoShooting;
import frc.robot.commands.auto.AutoShooting.ShootingTarget;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
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
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  // enum for bindings add when needed
  public enum Bindings {
    Competition,
    DriveTest, Shooter_test, IntakeTesting, auto_shooter_test
  }

  // The robot's subsystems and commands are defined here...
  static RobotContainer rc;
  final RobotSpecs robotSpecs;
  final HID_Xbox_Subsystem dc;
  final SwerveDrivetrain drivetrain;

  // singleton accessor for robot public sub-systems
  @Deprecated
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

  // Use this when there is only one instance of the Subsystem - preferred
  @SuppressWarnings("unchecked")
  public static <T extends Subsystem> T getSubsystem(Class<T> clz) {
    return (T) rc.robotSpecs.mySubsystemConfig.getSubsystem(clz);
  }

  // Use this when there is only one instance of the Subsystem and can deal with
  // nulls
  // in the context. It bypasses NPE checks. Know what you are doing.
  @SuppressWarnings("unchecked")
  public static <T extends Subsystem> T getSubsystemOrNull(Class<T> clz) {
    return (T) rc.robotSpecs.mySubsystemConfig.getObjectOrNull(clz.getSimpleName());
  }

  // Use this form when the RobotContainer object is NOT a Subsystem
  @SuppressWarnings("unchecked")
  public static <T> T getObject(String name) {
    return (T) rc.robotSpecs.mySubsystemConfig.getObject(name);
  }

  // Use this form when the RobotContainer object is NOT a Subsystem, and you can
  // deal with nulls
  @SuppressWarnings("unchecked")
  public static <T> T getObjectOrNull(String name) {
    return (T) rc.robotSpecs.mySubsystemConfig.getObjectOrNull(name);
  }

  public static boolean hasSubsystem(Class<? extends Subsystem> clz) {
    return rc.robotSpecs.mySubsystemConfig.hasSubsystem(clz);
  }

  public static RobotSpecs getRobotSpecs() {
    return rc.robotSpecs;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    RobotContainer.rc = this;
    robotSpecs = new RobotSpecs();
    robotSpecs.mySubsystemConfig.constructAll();

    // Testing, but also to drive the drivers nuts...
    Command random_lights = new RandomLightsCmd();
    random_lights.schedule();

    // Quiet some of the noise
    DriverStation.silenceJoystickConnectionWarning(true);

    // get subsystem vars as needed for bindings
    drivetrain = getSubsystem(SwerveDrivetrain.class);
    dc = getSubsystem("DC");

    /* Set the commands below */
    configureBindings(Bindings.Competition); // Change this to switch between bindings
    if (drivetrain != null) {
      drivetrain.setDefaultCommand(new FieldCentricDrive());
    }

    autoChooser = AutoBuilder.buildAutoChooser();

    NamedCommands.registerCommand("pickup", new IntakeSequence(false));
    NamedCommands.registerCommand("shoot", new ShooterSequence(1000.0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configureBindings(Bindings bindings) {
    CommandXboxController driver = dc.Driver();

    switch (bindings) {
      case DriveTest:
        driver.leftBumper().whileTrue(new RobotCentricDrive(drivetrain, dc));
        driver.b().onTrue(new AllianceAwareGyroReset(false));

        // Start any watcher commands
        new PDPMonitorCmd(); // auto scheduled, runs when disabled
        driver.leftTrigger().onTrue(new ShooterSequence(true, 1200.0));
        // This appears to break if initial pose is too close to path start pose
        // (zero-length path?)
        driver.a().onTrue(new SequentialCommandGroup(
            new InstantCommand(RobotContainer.RC().drivetrain::printPose),
            AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("test_1m"),
                new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720))),
            new InstantCommand(RobotContainer.RC().drivetrain::printPose)));

      driver.b().onTrue(new SequentialCommandGroup(
        new InstantCommand(RobotContainer.RC().drivetrain::printPose),
        AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("red1"), 
          new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720))),
        new InstantCommand(RobotContainer.RC().drivetrain::printPose)));
       

        driver.x().onTrue(new SequentialCommandGroup(
            new InstantCommand(RobotContainer.RC().drivetrain::printPose),
            AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(1.73, 5.38), new Rotation2d(0.0)),
                new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720))),
            new InstantCommand(RobotContainer.RC().drivetrain::printPose)));
          break;

      case Competition:

        driver.leftBumper().whileTrue(new RobotCentricDrive(drivetrain, dc));
        driver.y().onTrue(new AllianceAwareGyroReset(false));

        // Start any watcher commands
        new PDPMonitorCmd(); // auto scheduled, runs when disabled
        break;

        // i dont like that test commands and bindings are in here but we need them ig --er
      case IntakeTesting:
        driver.rightBumper().onTrue(new IntakeSequence(false));
        driver.povUp().onTrue(new ShooterSequence(true, 2000.0));
        driver.povRight().onTrue(new ShooterSequence(true, 1200.0));
        driver.povDown().whileTrue(new ShooterSequence(3200.0)); // RPM
        driver.leftBumper().whileTrue(new PneumaticsSequence());
        driver.x().whileTrue(new AngleCalibration(5.0));
        driver.y().whileTrue(new AngleCalibration(-5.0));
        // driver.leftBumper().whileTrue(new IntakeCalibrateForwardPos());
        driver.b().whileTrue(new IntakeTest(0.35)); //% speed
        // driver.leftBumper().whileTrue(new TransferTest(30.0));
        driver.rightTrigger().onTrue(new MoveToAnglePos(Intake.TravelUp, Intake.TravelUp));
        driver.leftTrigger().onTrue(new MoveToAnglePos(Intake.TravelDown, Intake.TravelDown));
        // driver.rightTrigger().onTrue(new AnglePos(50.0));
        driver.a().onTrue(new CalibratePos(0.0));
        break;
      
      case auto_shooter_test:
        driver.a().onTrue(new FaceToTag(4));
        driver.povDown().onTrue(new AutoShooting(ShootingTarget.Speaker));
        driver.povUp().onTrue(new AutoShooting(ShootingTarget.Trap));
        driver.povRight().onTrue(new AutoShooting(ShootingTarget.Amp));
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
      case Competition:

       // operator.rightBumper().onTrue(new PrintCommand("PlaceholderCMD: Intake Motor On"));
        operator.a().whileTrue(new IntakeSequence(false));
        operator.b().whileTrue(new EjectNote());
        //BELOW 3 PIT ALIGNMENT OF INTAKE (Emergency driver calibration)

        operator.povUp().whileTrue(new AngleCalibration(8.0));
        operator.povDown().whileTrue(new AngleCalibration(-8.0));
        operator.x().onTrue(new CalibratePos(0.0));
        operator.rightBumper().onTrue(new ShooterSequence(true, 2000.0)); //speaker close
        operator.leftTrigger().onTrue(new ShooterSequence(true, 800.0)); //amp - NO WORK RN
        operator.rightTrigger().onTrue(new ShooterSequence(3500.0)); // speaker far - NO WORK RN
        
        /* TODO climber bindings, commented out for sussex -- er
         *  Drive team mentioned that they want climber buttons on switchboard but i need 
         * to find that syntax -ER
         * WIP THESE BINDINGS ARE NOT AT ALL FINAL
         * operator.povUp().onTrue(new PrintCommand("PlaceholderCMD: Climber UP"));
           operator.povDown().onTrue(new PrintCommand("PlaceholderCMD: Climber Down")); 
         */
      
        
        break;
      case Shooter_test:
        var shooter = getSubsystem(Shooter.class);
        if (shooter != null) {
          shooter.setDefaultCommand(new RPMShooter());
        }
        break;
    }
  }
}
