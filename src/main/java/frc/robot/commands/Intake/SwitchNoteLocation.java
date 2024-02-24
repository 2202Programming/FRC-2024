// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Notes
 * 
 * 1st run transfer and intake backwards
 * 2nd check if lightgates are triggered, specificaly the intake one
 * 3rd we want the lightgate to be triggered once, then return a false
 * 4th end command
 */

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;

import frc.robot.Constants.Transfer_Constants.noteCommandedLocation;


public class SwitchNoteLocation extends Command {
  
  final Intake intake;
  final Transfer transfer;
  boolean firstGate = false;


  /** Creates a new transferToIntake. */
  public SwitchNoteLocation() {
    this.intake = RobotContainer.getSubsystem(Intake.class);
    this.transfer = RobotContainer.getSubsystem(Transfer.class);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(transfer.commandedLocation == noteCommandedLocation.transfer) {
      transfer.commandedLocation = noteCommandedLocation.intake;
      Command command = new SwitchNoteLocation();
      command.schedule();
    } else if (transfer.commandedLocation == noteCommandedLocation.intake) {
      transfer.commandedLocation = noteCommandedLocation.transfer;
    }

    SmartDashboard.putString("Commanded Note Location", transfer.commandedLocation.toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (transfer.commandedLocation == noteCommandedLocation.transfer) {
      if (!intake.has_Note() && !intake.has_Had_Note()) {
        intake.setMaxVelocity(60.0); // DEG/S
        intake.setAngleSetpoint(100.0); //'down' position
        intake.setIntakeSpeed(-0.8); //% of motor power converted to RPM (-1 -> 1)
        transfer.setSpeed(-35.0); //cm/s
      } else if (intake.has_Had_Note() && !firstGate) {
        transfer.setHasNote(false);
        firstGate = true;
      } else if (intake.has_Had_Note() && firstGate){
        end(true);
      }
    } else if (transfer.commandedLocation != noteCommandedLocation.transfer) {
        if (!transfer.hasNote() && !firstGate) {
          intake.setMaxVelocity(60.0); // DEG/S
          intake.setAngleSetpoint(100.0); //'down' position
          intake.setIntakeSpeed(0.8); //% of motor power converted to RPM (-1 -> 1)
          transfer.setSpeed(35.0); //cm/s
      } else if (transfer.hasNote()) {
        //transfer.setHasNote(false);
        firstGate = true;
      } else if (!transfer.hasNote() && firstGate){
        end(true);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setMaxVelocity(120.0); // DEG/S
    intake.setAngleSetpoint(0.0); //'up' position
    intake.setIntakeSpeed(0.0); 
    transfer.setSpeed(0.0);
    transfer.setHasNote(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
