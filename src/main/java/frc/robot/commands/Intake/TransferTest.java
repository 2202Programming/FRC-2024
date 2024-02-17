// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Transfer;

public class TransferTest extends Command {
  /** Creates a new TransferTest. */
  public final Transfer transfer;
  public TransferTest() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.transfer = RobotContainer.getSubsystem(Transfer.class);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    transfer.transferMtrOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transfer.transferMtrOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
