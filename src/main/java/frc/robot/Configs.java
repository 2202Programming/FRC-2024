package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Constants.CAN;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Sensors.Limelight_Subsystem;
import frc.robot.subsystems.Sensors.Sensors_Subsystem;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.util.SubsystemConfig;

public class Configs {

  // Subsystems and other hardware on 2024 Robot
  public static final SubsystemConfig comp2024BotSubsystemConfig = new SubsystemConfig()
      // deferred construction via Supplier<Object> lambda
      .add(PowerDistribution.class, "PDP", () -> {
        var pdp = new PowerDistribution(CAN.PDP, ModuleType.kRev);
        pdp.clearStickyFaults();
        return pdp;
      })
      .add(BlinkyLights.class, "LIGHTS", () -> {
        return new BlinkyLights();
      })
      .add(Sensors_Subsystem.class, "SENSORS")
      .add(HID_Xbox_Subsystem.class, "DC", () -> {
        return new HID_Xbox_Subsystem(0.3, 0.9, 0.05);
      })
      .add(PneumaticsControlModule.class, "PCM1", () -> {
        return new PneumaticsControlModule(CAN.PCM1);
      })
      .add(PneumaticsControlModule.class, "PCM2", () -> {
        return new PneumaticsControlModule(CAN.PCM2);
      })
      .add(Limelight_Subsystem.class, "LIMELIGHT")
      .add(Intake.class)
      .add(Shooter.class)
      .add(SwerveDrivetrain.class, "DRIVETRAIN");  // must be after LL and Sensors

  // Subsystems and hardware on Tim 2.0
  public static final SubsystemConfig swerveBotSubsystemConfig = new SubsystemConfig();

  public static final SubsystemConfig chadBotSubsystemConfig = new SubsystemConfig();
  
  public static final SubsystemConfig doofBotSubsystemConfig = new SubsystemConfig();
  
  public static final SubsystemConfig botOnBoardSystemConfig = new SubsystemConfig();

}
