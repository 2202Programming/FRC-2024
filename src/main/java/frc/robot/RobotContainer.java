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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Transfer_Constants.NoteCommandedLocation;
import frc.robot.commands.PDPMonitorCmd;
import frc.robot.commands.RandomLightsCmd;
import frc.robot.commands.Intake.AngleCalibration;
import frc.robot.commands.Intake.EjectNote;
import frc.robot.commands.Intake.InAmp;
import frc.robot.commands.Intake.InIntake;
import frc.robot.commands.Intake.IntakeSequence;
import frc.robot.commands.Intake.IntakeSwap;
import frc.robot.commands.Intake.IntakeTest;
import frc.robot.commands.Intake.MoveToAnglePos;
import frc.robot.commands.Intake.SetNoteLocation;
import frc.robot.commands.Shooter.PneumaticsSequence;
import frc.robot.commands.Shooter.RPMShooter;
import frc.robot.commands.Shooter.ShooterSequence;
import frc.robot.commands.Swerve.AllianceAwareGyroReset;
import frc.robot.commands.Swerve.FaceToTag;
import frc.robot.commands.Swerve.FieldCentricDrive;
import frc.robot.commands.Swerve.RobotCentricDrive;
import frc.robot.commands.Swerve.RotateTo;
import frc.robot.commands.auto.AutoShooting;
import frc.robot.commands.auto.AutoShooting.ShootingTarget;
import frc.robot.commands.auto.TurnFaceShootAuto;
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
  
  // Use this form when the RobotContainer object is NOT a Subsystem, and you can deal with nulls
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

    // Set binding to Competition (or your mode for testing)
    Bindings bindings = Bindings.IntakeTesting;    

    // Testing, but also to drive the drivers nuts...
    Command random_lights = new RandomLightsCmd();
    random_lights.schedule();

    // Quiet some of the noise
    DriverStation.silenceJoystickConnectionWarning(true);

    // get subsystem vars as needed for bindings 
    drivetrain = getSubsystem(SwerveDrivetrain.class);
    dc = getSubsystem("DC");

    /* Set the commands below */
    configureBindings(bindings); // Change this to switch between bindings
    if (drivetrain != null) {
      drivetrain.setDefaultCommand(new FieldCentricDrive());
    }

    //NamedCommands for use in PathPlanner scripts.
    NamedCommands.registerCommand("pickup", new IntakeSequence(true));
    NamedCommands.registerCommand("shoot", new ParallelCommandGroup(new ShooterSequence(true,2000), new WaitCommand(2.0)));
    NamedCommands.registerCommand("angle_shoot", new AutoShooting(ShootingTarget.Speaker));
    NamedCommands.registerCommand("RotateTo", new RotateTo());
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    //make some noise if we are not on Competion bindings
    if (bindings != Bindings.Competition) {
      System.out.println(
        "*****************************************************************************\n"+
        "Warning: Not using competition bindings, using: " +bindings.toString()+
        "\n*****************************************************************************\n");
    }
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

      //This appears to break if initial pose is too close to path start pose (zero-length path?)
      driver.x().onTrue(new SequentialCommandGroup(
        new InstantCommand(RobotContainer.RC().drivetrain::printPose),
        AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("blue1"), 
          new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720))),
        new InstantCommand(RobotContainer.RC().drivetrain::printPose)));

      driver.b().onTrue(new SequentialCommandGroup(
        new InstantCommand(RobotContainer.RC().drivetrain::printPose),
        AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("red1"), 
          new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720))),
        new InstantCommand(RobotContainer.RC().drivetrain::printPose)));
       
        // Start any watcher commands
        new PDPMonitorCmd(); // auto scheduled, runs when disabled
        driver.leftTrigger().onTrue(new ShooterSequence(true, 1200.0));
        // This appears to break if initial pose is too close to path start pose
        // (zero-length path?)
        driver.a().onTrue(new SequentialCommandGroup(
            new InstantCommand(drivetrain::printPose),
            AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("test_1m"),
                new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720))),
            new InstantCommand(RobotContainer.RC().drivetrain::printPose)));

        driver.x().onTrue(new SequentialCommandGroup(
            new InstantCommand(drivetrain::printPose),
            AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(1.73, 5.38), new Rotation2d(0.0)),
                new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720))),
            new InstantCommand(drivetrain::printPose)));
          break;

      case Competition:

        driver.leftBumper().whileTrue(new RobotCentricDrive(drivetrain, dc));
        driver.y().onTrue(new AllianceAwareGyroReset(false));

        // Start any watcher commands
        new PDPMonitorCmd(); // auto scheduled, runs when disabled
        break;

        // MR LAUFENBERG WHY DO WE NEED ALL OF THIS IN MASTER ITS SO CLUNKY - ER
      case IntakeTesting:
        driver.rightBumper().whileTrue(new IntakeSequence(false));
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
        //driver.a().onTrue(new CalibratePos(0.0));
        break;
      
      case auto_shooter_test:
        driver.a().onTrue(new FaceToTag(4));
        driver.povDown().onTrue(new AutoShooting(ShootingTarget.Speaker));
        driver.povUp().onTrue(new AutoShooting(ShootingTarget.Trap));
        driver.povRight().onTrue(new AutoShooting(ShootingTarget.Amp));
        driver.povLeft().onTrue(new TurnFaceShootAuto(4));
        driver.x().onTrue(new RotateTo());
        break;

      
      default:
        break;
    }

    // Sideboard buttons (Default/Competition)
    var sideboard = dc.SwitchBoard();
    
    // TODO replace with actual climber cmds one they are merged - er
    sideboard.sw21().onTrue(new PrintCommand("PlaceholderCMD: Climber UP"));
    sideboard.sw22().onTrue(new PrintCommand("PlaceholderCMD: Climber Down"));

    configureOperator(bindings);
  }

  private void configureOperator(Bindings bindings) {
    CommandXboxController operator = dc.Operator();

    switch (bindings) {

      default:
      case DriveTest:
        break;
      case Competition:

      // REAL COMPETITION BINDINGS. DO NOT UNDER ANY CIRCUMSTANCES TOUCH W/O CONSULTING ME. - xoxo ER
      	operator.a().whileTrue(new IntakeSequence(true));
        operator.b().whileTrue(new EjectNote()); // eject note from intake
        operator.x().whileTrue(new InIntake()); // works ---> seq for stay in intake for amp shoot
        operator.y().whileTrue(new IntakeSequence(true));

        operator.povUp().onTrue(new AngleCalibration(-25.0));// intake calibrate

        operator.rightBumper().onTrue(new ShooterSequence(true, 2000.0)); // speaker close

        operator.leftTrigger().whileTrue(new InAmp()); // shoot into amp bc noah's strength is not in naming conventions
        operator.rightTrigger().onTrue(new ShooterSequence(3500.0)); // speaker far 
        
        break;
       
      case Shooter_test:
        var shooter = getSubsystem(Shooter.class);
        if (shooter != null) {
          shooter.setDefaultCommand(new RPMShooter());
        }
        break;

        case IntakeTesting:
          operator.a().onTrue(new IntakeSequence(true)); //works for both modes
          operator.b().onTrue(new MoveToAnglePos(Intake.DownPos, 60.0)); 
          operator.x().onTrue(new InIntake());
          operator.y().whileTrue(new IntakeTest(0.5)); //%

          operator.povDown().onTrue(new AngleCalibration(15.0));  // good for alpha 
          operator.povUp().onTrue(new AngleCalibration(-15.0)); // not needed, calibrate with up
          
          operator.povRight().onTrue(new IntakeSwap());
          operator.povLeft().whileTrue(new EjectNote());
  
          // operator.rightBumper().whileTrue(new InIntake()); //works ---> seq for stay in intake
          // operator.leftTrigger().whileTrue(new InAmp()); //works ---> into amp seq
          // operator.povRight().whileTrue(new IntakeTest(0.35)); 
        
          operator.rightBumper().onTrue(new ShooterSequence(true, 2000.0)); //speaker close
          operator.leftTrigger().onTrue(new ShooterSequence(true, 800.0)); //amp - NO WORK RN
          operator.rightTrigger().onTrue(new ShooterSequence(3500.0)); // speaker far - NO WORK RN
          break;
    }
  }
}
