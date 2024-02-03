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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Swerve.AllianceAwareGyroReset;
import frc.robot.commands.Swerve.FieldCentricDrive;
import frc.robot.commands.Swerve.RobotCentricDrive;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.BlinkyLights.BlinkyLightUser;
import frc.robot.subsystems.PneumaticsControl;
import frc.robot.subsystems.Shooter;
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
    DriveTest
  }

  // The robot's subsystems and commands are defined here...
  static RobotContainer rc;
  public final RobotSpecs robotSpecs;

  // Subsystems
  public final PowerDistribution pdp;
  public final PneumaticsControlModule pcm1;
  public final PneumaticsControlModule pcm2;
  public final PneumaticsControl pneumatics;
  public final HID_Xbox_Subsystem dc;
  public final Limelight_Subsystem limelight;
  public final Sensors_Subsystem sensors;
  public final SwerveDrivetrain drivetrain;
  public final BlinkyLights lights;
  public final Intake intake;
  public final Shooter shooter;

  // singleton accessor for robot public sub-systems
  public static RobotContainer RC() {
    return rc;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    RobotContainer.rc = this;
    // Quiet some of the noise
    DriverStation.silenceJoystickConnectionWarning(true);

    robotSpecs = new RobotSpecs();
    lights = new BlinkyLights();
    dc = new HID_Xbox_Subsystem(0.3, 0.9, 0.05);
    sensors = new Sensors_Subsystem();
    pdp = new PowerDistribution( 0, ModuleType.kRev );
    pdp.clearStickyFaults();

    //TODO may need to put into subsystemconfig() object
    pcm1 = new PneumaticsControlModule(CAN.PCM1);
    pcm2 = new PneumaticsControlModule(CAN.PCM2);

    //Use SubsystemConfig to figure out if our current bot has subsytem before trying to initialize it
    pneumatics = (robotSpecs.getSubsystemConfig().HAS_ANALOG_PNEUMATICS) ? new PneumaticsControl() : null;
    limelight = (robotSpecs.getSubsystemConfig().HAS_LIMELIGHT) ? new Limelight_Subsystem() : null;
    drivetrain = (robotSpecs.getSubsystemConfig().HAS_DRIVETRAIN) ? new SwerveDrivetrain() : null;
    intake = (robotSpecs.getSubsystemConfig().HAS_INTAKE) ? new Intake() : null;
    shooter = (robotSpecs.getSubsystemConfig().HAS_SHOOTER) ? new Shooter() : null;
    
    /* Set the commands below */
    configureBindings(Bindings.DriveTest); // Change this to swich between bindings
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

    }
  }
}
