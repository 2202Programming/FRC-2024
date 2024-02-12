// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter; 

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.BlinkyLights.BlinkyLightUser;

public class ShooterToggle extends BlinkyLightUser {
  /** Creates a new ShooterToggle. */
  public final Shooter shooter;
  public final Intake intake;
  public final Transfer transfer;
  public final int DONE_COUNT = 10;
  int count_lightgate;
  final boolean pneumatics = false; // YES FOR SUSSEX NO AFTER???
  final int DELAY = 10; // figure out this number
  int count;
  boolean RPM_dropped;

  //final Intake intake; //TODO: When merge, check for hasNote - Probably move to subsystem
  public ShooterToggle() {
    this.shooter = RobotContainer.getSubsystem(Shooter.class);
    this.intake = RobotContainer.getSubsystem(Intake.class);
    this.transfer = RobotContainer.getSubsystem(Transfer.class);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RPM_dropped = false;
    count_lightgate = 0;
    count = 0;
   if(transfer.hasNote()){
    shooter.setRPM(0.5, 0.5); //make these constants (0.5 placeholder)
  }
  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.isAtRPM() && intake.angleAtSetpoint()){
      transfer.transferMotorOn();
    }
  }
      /*
     * Control the blinkylights based on our Note possession.
     *
     * Green when we have it, red otherwise
     */
    @Override
    public Color8Bit colorProvider() {
        // make sure not is safely in our possession before going back
        return (count >= DONE_COUNT) ? BlinkyLights.GREEN : BlinkyLights.RED;
    };

    @Override
    public boolean requestBlink() {
        return false; // we want solid lights
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setRPM(0, 0);
    // shooter.setTransferOff(); once merged (might be transfer.transferMotorOff())
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    count_lightgate = (transfer.hasNote()) ? count++ : 0; // count frames with note
    if(Math.abs(shooter.getRPM() - shooter.getDesiredRPM()) > 100){
      RPM_dropped = true;
    } if(RPM_dropped){
      count++;
    } if(count > DELAY){
      return true;
    } else {
      return false;
    }
  }
}
