// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Transfer;

public class TransferTest extends Command {
  /** Creates a new TransferTest. */
  final Transfer transfer;
  final double speed; 

  public TransferTest(double speed) {
    this.transfer = RobotContainer.getSubsystem(Transfer.class);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    transfer.setHasNote(false);
    transfer.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transfer.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return transfer.hasNote();
    //return false;
  }
}
