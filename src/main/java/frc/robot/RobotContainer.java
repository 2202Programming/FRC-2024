// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swerve.FieldCentricDrive;
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
public class RobotContainer {
  static RobotContainer rc;
  public final RobotSpecs robotSpecs;

  // Subsystems
  public final HID_Xbox_Subsystem dc;
  public final Limelight_Subsystem limelight;
  public final Sensors_Subsystem sensors;
  public final SwerveDrivetrain drivetrain;

  // singleton accessor for robot public sub-systems
  public static RobotContainer RC() {
    return rc;
  }

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    RobotContainer.rc = this;
    robotSpecs = new RobotSpecs();
    dc = new HID_Xbox_Subsystem(0.3, 0.9, 0.05);
    configureBindings();
    // Construct sub-systems based on robot Name Specs
    switch (robotSpecs.myRobotName) {
      case CompetitionBot2023:
        limelight = new Limelight_Subsystem();
        sensors = new Sensors_Subsystem();
        drivetrain = new SwerveDrivetrain();
        break;

      case SwerveBot:
        limelight = new Limelight_Subsystem();
        sensors = new Sensors_Subsystem();
        drivetrain = new SwerveDrivetrain();
        break;

      case ChadBot:
        limelight = new Limelight_Subsystem();
        sensors = new Sensors_Subsystem();
        drivetrain = new SwerveDrivetrain();
        break;

      case BotOnBoard: // fall through
      case UnknownBot: // fall through
      default:
        limelight = null;
        sensors = null;
        drivetrain = null;
        break;
    }

    if (drivetrain != null) {
      drivetrain.setDefaultCommand(new FieldCentricDrive(drivetrain));
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // // An example command will be run in autonomous
  // return Autos.exampleAuto(m_exampleSubsystem);
  // }
}
