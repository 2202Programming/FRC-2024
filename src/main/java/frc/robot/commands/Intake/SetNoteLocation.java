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

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;

import frc.robot.Constants.Transfer_Constants.NoteCommandedLocation;

/*
 * 
 * SwtichNoteLocation(loc) - move note to given location
 * 
 * If there is no note, bails quick and clean
 * 
 * Where there is a note, 
 * 
 */
public class SetNoteLocation extends Command {

  Intake intake = RobotContainer.getSubsystem(Intake.class);
  Transfer transfer = RobotContainer.getSubsystem(Transfer.class);
  boolean firstGate = false;

  NoteCommandedLocation commandedLocation;   // arg to cmd or computed in init if toggling
  boolean isDone;                            // can't call end(), this flag captures decisions

  // speeds and target to do the transfer to
  BooleanSupplier targetHasNote;
  double intake_speed;
  double transfer_speed;
  boolean intakeUpAtEnd = false;

  /** Creates a new transferToIntake. */
  public SetNoteLocation(NoteCommandedLocation commandedLocation) {
    this.commandedLocation = commandedLocation;
    addRequirements(intake, transfer);
  }

  public SetNoteLocation(NoteCommandedLocation commandedLocation, boolean intakeUpAtEnd) {
    this(commandedLocation);
    this.intakeUpAtEnd = intakeUpAtEnd;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    boolean isNoteInTransfer = transfer.hasNote();
    boolean isNoteInIntake = intake.hasNote();
    isDone = false;

    SmartDashboard.putString("Commanded Note Location", commandedLocation.toString());

    if (!isIntakeAngleReady()) {
      System.out.println(
          "Transfer not continuing because the angles are not to our very specifically determined standards. --The Angle Master");
      isDone = true;
    }

    if (!isNoteInIntake && !isNoteInTransfer) {
      System.out.println(
          "The transfer is not continuing because neither subsystem reports owning a note. The robot will now self-destruct. --The Lightmaster");
     isDone = true;
    }

    if (isNoteInIntake && isNoteInTransfer) {
      System.out.println(
          "Apparently our robot is strong enough to either rip a note in half or tear a hole in space-time. --The Void(?)master");
     isDone = true;
    }

    // nothing more to do if any of the above guards are true
    if (isDone) return;

    // handle swap, so our commanded location is correct
    if (commandedLocation == NoteCommandedLocation.Swap) {
      if (isNoteInTransfer) {
        commandedLocation = NoteCommandedLocation.Intake;     
      } else {
        commandedLocation = NoteCommandedLocation.Transfer;
      }
    }
    // now set motor speeds and targetHasNote supplier based on target destination
    switch (commandedLocation) {
      case Intake:
        targetHasNote = intake::hasNote;        
        intake_speed = -0.8;   // [% pwr]  neg for intake <-- transfer
        transfer_speed = -35.0; // [cm/s]
        
        transfer.setHasNote(false);
        break;

      case Transfer:
        targetHasNote = transfer::hasNote;
        intake_speed = 0.8;   // [% pwr]  positive for intake --> transfer
        transfer_speed = 35.0; // [cm/s]
        
        intake.setHasNote(false);
      break;
    
      case Swap:
      default:
        //nothing - shouldn't be set at this point
      }
      // start your motors 
      intake.setIntakeSpeed(intake_speed);
      transfer.setSpeed(transfer_speed);
    }


  /*
   * makes sure intake is at a good angle to perform a transfer
   */
  private boolean isIntakeAngleReady() {
    double currentAngle = intake.getAnglePosition();
    // absolute value of the difference between 2.5(ish) degrees
    return intake.angleAtSetpoint()  && (Math.abs(currentAngle - Intake.DownPos) < 2.5); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // watch our targe subsystem to say they have the note
    isDone = (isDone) ? true : targetHasNote.getAsBoolean();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // turn off our motors
    intake.setIntakeSpeed(0.0);
    transfer.setSpeed(0.0);
    
    if (interrupted || !intakeUpAtEnd) return;

    //return intake to up position
    intake.setMaxVelocity(Intake.TravelUp); // DEG/S
    intake.setAngleSetpoint(Intake.UpPos); // 'up' position    
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
