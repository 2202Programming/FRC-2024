package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Constants.CAN;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Sensors.Limelight_Subsystem;
import frc.robot.subsystems.Sensors.Sensors_Subsystem;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.util.SubsystemConfig;

/*
 * The Subsystems and object in Configs will be created by RobotContainer 
 * when the robot's serial number is read by RobotSpecs. 
 * 
 * All construction is deferred until inside RobotContainer so be careful
 * with any 'new' operators, they should be wrapped in a lambda. For an example
 * see PDP below.
 */
public class Configs {

  // Subsystems and other hardware on 2024 Robot
  public static final SubsystemConfig comp2024AlphaBotSubsystemConfig = new SubsystemConfig()
      // deferred construction via Supplier<Object> lambda
      .add(PowerDistribution.class, "PDP", () -> {
        var pdp = new PowerDistribution(CAN.PDP, ModuleType.kRev);
        pdp.clearStickyFaults();
        return pdp;
      })
      .add(BlinkyLights.class, "LIGHTS", () -> {
        return new BlinkyLights();
      })
      .add(PneumaticsControlModule.class, "PCM1", () -> {
        return new PneumaticsControlModule(CAN.PCM1);
      })
      .add(HID_Xbox_Subsystem.class, "DC", () -> {
        return new HID_Xbox_Subsystem(0.3, 0.9, 0.05);
      })
      .add(Sensors_Subsystem.class)
      .add(Limelight_Subsystem.class)
      .add(SwerveDrivetrain.class, "DRIVETRAIN") // must be after LL and Sensors
      .add(Intake.class)
      .add(Shooter.class)
      .add(Transfer.class);

   public static final SubsystemConfig comp2024BetaBotSubsystemConfig = new SubsystemConfig()
      // deferred construction via Supplier<Object> lambda
      .add(PowerDistribution.class, "PDP", () -> {
        var pdp = new PowerDistribution(CAN.PDP, ModuleType.kRev);
        pdp.clearStickyFaults();
        return pdp;
      })
      .add(BlinkyLights.class, "LIGHTS", () -> {
        return new BlinkyLights();
      })
      .add(HID_Xbox_Subsystem.class, "DC", () -> {
        return new HID_Xbox_Subsystem(0.3, 0.9, 0.05);
      })
      .add(Sensors_Subsystem.class)
      .add(Limelight_Subsystem.class)
      .add(SwerveDrivetrain.class, "DRIVETRAIN") // must be after LL and Sensors
      .add(Intake.class)
      .add(Shooter.class)
      .add(Transfer.class);
      //TODO: ADD CLIMBER

  // Subsystems and hardware on Tim 2.0
  public static final SubsystemConfig swerveBotSubsystemConfig = new SubsystemConfig()
      .add(Sensors_Subsystem.class)
      .add(Limelight_Subsystem.class)
      .add(SwerveDrivetrain.class, "DRIVETRAIN")
      .add(HID_Xbox_Subsystem.class, "DC", () -> {
        return new HID_Xbox_Subsystem(0.3, 0.9, 0.05);
      });
    
  // Chad's subsystems and objects
  public static final SubsystemConfig chadBotSubsystemConfig = new SubsystemConfig()
      .add(Sensors_Subsystem.class)
      .add(Limelight_Subsystem.class)
      .add(Intake.class)
      .add(Shooter.class)
      .add(Transfer.class)
      // Check if there is a class for climber
      .add(SwerveDrivetrain.class, "DRIVETRAIN")
      .add(HID_Xbox_Subsystem.class, "DC", () -> {
        return new HID_Xbox_Subsystem(0.3, 0.9, 0.05);
      });
  public static final SubsystemConfig doofBotSubsystemConfig = new SubsystemConfig()
      .add(Sensors_Subsystem.class)
      .add(Limelight_Subsystem.class)
      .add(Intake.class)
      .add(SwerveDrivetrain.class, "DRIVETRAIN")
      .add(HID_Xbox_Subsystem.class, "DC", () -> {
        return new HID_Xbox_Subsystem(0.3, 0.9, 0.05);
      });

  public static final SubsystemConfig botOnBoardSystemConfig = new SubsystemConfig()
  // .add(null) useful stuff for bot on boards
  ;
}
