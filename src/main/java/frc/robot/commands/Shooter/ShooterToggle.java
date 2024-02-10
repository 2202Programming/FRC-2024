// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter; 

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShooterToggle extends Command {
  /** Creates a new ShooterToggle. */
  public final Shooter shooter;
  final boolean pneumatics = false; // YES FOR SUSSEX NO AFTER???
  final int DELAY = 10; // figure out this number
  int count = 0;
  boolean RPM_dropped;

  //final Intake intake; //TODO: When merge, check for hasNote - Probably move to subsystem
  public ShooterToggle() {
    this.shooter = RobotContainer.getSubsystem(Shooter.class);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RPM_dropped = false;
   // if(shooter.hasNote()){
    shooter.setRPM(0.5, 0.5); //make these constants (0.5 placeholder)
 // } when this is added to subsystem

  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.isAtRPM()){
      // shooter.transferMotorOn(); TODO: Once merge, comment this in
    }
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
