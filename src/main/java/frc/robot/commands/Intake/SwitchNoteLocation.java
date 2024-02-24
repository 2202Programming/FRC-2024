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

  Intake intake = RobotContainer.getSubsystem(Intake.class);
  Transfer transfer = RobotContainer.getSubsystem(Transfer.class);
  boolean firstGate = false;

  noteLocationState pastState;
  noteLocationState currentState; // if needed

  noteCommandedLocation commandedLocation;

  // if this is called without a direction, it's an implicit toggle
  public SwitchNoteLocation() {
    // where is the note now?
  }

  /** Creates a new transferToIntake. */
  public SwitchNoteLocation(noteCommandedLocation commandedLocation) {
    this.commandedLocation = commandedLocation;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    boolean isNoteInTransfer = transfer.hasNote();
    boolean isNoteInIntake = intake.has_Note(); // change
    SmartDashboard.putString("Commanded Note Location", transfer.commandedLocation.toString());

    if (!isIntakeAngleReady()) {
      System.out.println(
          "Transfer not continuing because the angles are not to our very specifically determined standards. --The Angle Master");
      end(true);
    }

    if (!isNoteInIntake && !isNoteInTransfer) {
      System.out.println(
          "The transfer is not continuing because the lightgates are not detecting any notes. The robot will now self-destruct. --The Lightmaster");
      end(true);
    }

    if (isNoteInIntake && isNoteInTransfer) {
      System.out.println(
          "Apparently our robot is strong enough to either rip a note in half or tear a hole in space-time. --The Void(?)master");
      end(true);
    }

    if (isNoteInTransfer) {
      transfer.commandedLocation = noteCommandedLocation.intake;
    } else {
      transfer.commandedLocation = noteCommandedLocation.transfer;
    }
  }

  private boolean isIntakeAngleReady() {
    double currentAngle = intake.getAnglePosition();
    // absolute value of the difference between 2.5(ish) degrees
    return Math.abs(currentAngle - 100.0) < 2.5; // TODO: put into constants maybe; 2.5 is tolerance; units: degrees;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (hasNoteArrived()) {
      System.out.println("The note has arrived at the correct location :)");
      end(false);
    }

    if (transfer.commandedLocation == noteCommandedLocation.transfer) {
      intake.setIntakeSpeed(-0.8); // % of motor power converted to RPM (-1 -> 1)
      transfer.setSpeed(-35.0); // cm/s
    } else {
      intake.setIntakeSpeed(0.8);
      transfer.setSpeed(35.0);
    }
  }

  private boolean hasNoteArrived() {
    if(transfer.commandedLocation == noteCommandedLocation.transfer) {
      
      switch(currentState) {
        //TODO: create timer that when exceeded causes an error; set state to unknown; if timer expires, maybe note went back?;
        case empty:
        if(transfer.hasNote()) {
          currentState = noteLocationState.front;
        }
          return false;
        case front:
          if(!transfer.hasNote()) {
            System.out.println("Note is at the 'front' state.");
            currentState = noteLocationState.hole;
          }
          return false;
        case hole:
          if(transfer.hasNote()) {
            System.out.println("Note is at the 'hole' state.");
            currentState = noteLocationState.back;
          }
          return true; //Note has arrived when it reaches "the hole"
        case back:
          if(!transfer.hasNote()) {
            System.out.println("Note is at the 'back' state.");
            currentState = noteLocationState.past;
          }
          return false;
        case past:
        System.out.println("Note is at the 'past' state.");
          return true;
        case unknown:
          System.out.println("Note is currently traversing through an unknown dimension.");
          end(true);
          
        default:
          return false;
      }
    } else {

      switch(currentState) {
        //TODO: create timer that when exceeded causes an error; set state to unknown; if timer expires, maybe note went back?;
        case empty:
        if(intake.has_Note()) {
          System.out.println("Note is at the 'empty' state.");
          currentState = noteLocationState.front;
        }
          return false;
        case front:
          if(!intake.has_Note()) {
            System.out.println("Note is at the 'front' state.");
            currentState = noteLocationState.hole;
          }
          return false;
        case hole:
          if(intake.has_Note()) {
            System.out.println("Note is at the 'hole' state.");
            currentState = noteLocationState.back;
          }
          return false;
        case back:
          if(!intake.has_Note()) {
            System.out.println("Note is at the 'back' state.");
            currentState = noteLocationState.past;
          }
          return false;
        case past:
        System.out.println("Note is at the 'past' state.");
          return true;
        case unknown:
          System.out.println("Note is currently traversing through an unknown dimension.");
          end(true);
          
        default:
          return false;
      }
    }
  }

  private enum noteLocationState {
    empty, front, hole, back, past, unknown
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setMaxVelocity(120.0); // DEG/S
    intake.setAngleSetpoint(0.0); // 'up' position
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
