// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.Shooter_Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.BlinkyLights.BlinkyLightUser;
/**
 * Button pressed starts command
 * Make sure we detect a note
 * Turn the shooter motors on
 * Once our shooter motors reach the proper speed, turn on the transfer motors
 * Once the lightgate no longer detects the note, wait x amount of frames
 * After x amount of frames, turn the blinky lights red, then turn everything off (command ended)
 */

public class ShooterSequence extends BlinkyLightUser {
  /** Creates a new ShooterSequence. */
  final Shooter shooter;
  final Transfer transfer;
  double count;  
  Phase phase;
  final double DONE_COUNT = 20; //placeholder
  public enum Phase{
    HasNote,ShooterMotorOn,TransferMotorOn,Finished;
  }

  public ShooterSequence() {
    this.shooter = RobotContainer.getSubsystem(Shooter.class);
    this.transfer = RobotContainer.getSubsystem(Transfer.class);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    phase = Phase.HasNote;
  }
    public Color8Bit colorProvider() {
    // make sure not is safely in our possession before going back
    return (count > DONE_COUNT) ? BlinkyLights.RED : BlinkyLights.GREEN;
  };

  @Override
  public boolean requestBlink() {
    return false; // we want solid lights
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (phase){
      case HasNote:
      if(transfer.hasNote()){
        shooter.setRPM(Shooter_Constants.ShooterDefaultSpeed, Shooter_Constants.ShooterDefaultSpeed); //placeholder
        phase = Phase.ShooterMotorOn;
      }
      break;
      case ShooterMotorOn:
      if(shooter.isAtRPM(100)){
        transfer.transferMtrOn();
        phase = Phase.TransferMotorOn;
      }
      break;
      case TransferMotorOn:
      if(!transfer.hasNote()){
        count++;
      }
      if(count > DONE_COUNT){
        phase = Phase.Finished;
      }
      break;
      case Finished:
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        transfer.transferMtrOff();
        shooter.setRPM(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(phase == Phase.Finished){
      return true;
    }
    return false;
  }
}
