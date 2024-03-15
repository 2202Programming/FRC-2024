package frc.robot.commands.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterServo;

/*
 * Calibrates the shooter by movint towards the lowest angle (zero extension)
 * and stopping when the servo stops moving.
 */
public class CalibrateSetZero extends Command {
    private final ShooterServo shooterServo;

  //Lower the shooter servo until it stops, then set relative encoder to zero.
  public CalibrateSetZero() {
    shooterServo = RobotContainer.getSubsystem(ShooterServo.class);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterServo.setExtensionPosition(0.0);
  }
}