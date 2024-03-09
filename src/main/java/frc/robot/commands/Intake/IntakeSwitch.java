// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;

public class IntakeSwitch extends Command {
  final Intake intake;
  final Transfer transfer;
  boolean noteToIntake; // go to intake

  /**
   * Two different buttons for swap
   */
  public IntakeSwitch(boolean noteToIntake) {
    intake = RobotContainer.getSubsystem(Intake.class);
    transfer = RobotContainer.getSubsystem(Transfer.class);
    this.noteToIntake = noteToIntake;
    addRequirements(intake, transfer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (noteToIntake) {
      transfer.setSpeed(-35.0);// not %
      intake.setIntakeSpeed(-0.8); // %
    } else {
      transfer.setSpeed(35.0);
      intake.setIntakeSpeed(0.8); // %
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO Still need a counter to deal with stopping at the correct spot
    // See IntakeSwap... best to modify that command to handle these cases.
    // Merge this cmd with Intake swap, then compare with SetNoteLocation...
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeSpeed(0.0);
    transfer.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (noteToIntake) {
      if (intake.hasNote()) {
        return true;
      }
    } else if (!noteToIntake) {
      if (transfer.hasNote()) {
        return true;
      }
    }
    return false;
  }
}