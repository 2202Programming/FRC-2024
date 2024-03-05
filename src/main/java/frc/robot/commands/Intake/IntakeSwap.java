// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;
/*
 * One button toggle based on current note location
 */
public class IntakeSwap extends Command {
  final Intake intake;
  final Transfer transfer;
  boolean intakee; //true = transfer --> intake, false = intake --> transfer
  BooleanSupplier target;
  int count;
  int DONE_COUNT;
  /** Creates a new IntakeSwitch. */
  public IntakeSwap() {
    this.intake = RobotContainer.getSubsystem(Intake.class);
    this.transfer = RobotContainer.getSubsystem(Transfer.class);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(transfer.hasNote()){
      target=intake::hasNote;
      DONE_COUNT = 10; //Counter for transfer --> intake
        intakee = true;
    }
    else if(intake.hasNote()){
      target=transfer::hasNote;
      DONE_COUNT = 5; //Counter for intake --> transfer
        intakee= false;
    }
        if(intakee){
      transfer.setSpeed(-35.0);//not %
      intake.setIntakeSpeed(-0.8); //%
    }
    else if(!intakee){
      transfer.setSpeed(35.0);
      intake.setIntakeSpeed(0.8); //%
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(target.getAsBoolean()){
      count++;
    }

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
    return count > DONE_COUNT;
  }
}