// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.PneumaticsControl;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Lights;
import frc.robot.commands.Swerve.AllianceAwareGyroReset;
import frc.robot.commands.Swerve.FieldCentricDrive;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.BlinkyLights.BlinkyLightUser;
import frc.robot.commands.Swerve.RobotCentricDrive;
import frc.robot.subsystems.Sensors.Sensors_Subsystem;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.util.RobotSpecs;
import frc.robot.commands.Lights;

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
  private final Limelight_Subsystem m_limelight = new Limelight_Subsystem();
  static RobotContainer rc;
  public final RobotSpecs robotSpecs;

  // Subsystems
  public final PneumaticsControl pneumatics;
  public final HID_Xbox_Subsystem dc;
  public final Limelight_Subsystem limelight;
  public final Sensors_Subsystem sensors;
  public final SwerveDrivetrain drivetrain;
  public final BlinkyLights lights;

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
    // Construct sub-systems based on robot Name Specs
    switch (robotSpecs.myRobotName) {
      case CompetitionBot2023:
        pneumatics = null;
        limelight = new Limelight_Subsystem();
        sensors = new Sensors_Subsystem();
        drivetrain = new SwerveDrivetrain();
        break;

      case SwerveBot:
        pneumatics = null;
        limelight = new Limelight_Subsystem();
        sensors = new Sensors_Subsystem();
        drivetrain = new SwerveDrivetrain();
        break;

      case CompetitionBot2024:
        limelight = new Limelight_Subsystem();
        sensors = new Sensors_Subsystem();
        drivetrain = new SwerveDrivetrain();
        pneumatics = new PneumaticsControl();
        pneumatics.compressor_on();
        break;

      case ChadBot:
        pneumatics = null;
        limelight = new Limelight_Subsystem();
        sensors = new Sensors_Subsystem();
        drivetrain = new SwerveDrivetrain();
        break;

      case BotOnBoard: // fall through
      case UnknownBot: // fall through
      default:
        pneumatics = null;
        limelight = null;
        sensors = null;
        drivetrain = null;
        break;
    }
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

    switch (bindings){
      case DriveTest:
      driver.leftTrigger().whileTrue(new RobotCentricDrive(drivetrain, dc));
      driver.b().onTrue(new AllianceAwareGyroReset(false));


      driver.x().whileTrue(new Lights(BlinkyLights.GREEN));
      driver.leftBumper().whileTrue(new Lights(BlinkyLights.RED));
      driver.y().whileTrue(new Lights(BlinkyLights.WHITE));



  }
}
}

