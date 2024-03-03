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

/* NR Comments no idea if correct but based on my knowledge for below
 * Intake should be the most complicated, not transfer - Transfer will always be in the pos direction since the note can only come from the intake,
 * whereas on the Intake, the note can come from either the transfer or the ground (intake)
 * Also, one lightgate is on transfer whilst one is on intake, not one on each
 * To clarify, negative direction is towards outside the robot via the intake (technically towards intake could be coming in)
 * To clarify, we do stop at the transfer during movement - As in, if we plan to shoot, we will be at the transfer until the wheel speeds are on and ready to shoot
 * Additionally, we hopefully won't have to stop to go from transfer to intake, but yes, we may have to if driver decides to switch (or we eject, but that should be smooth motion)
 * Do we need to read the motor speed? Or just the direction? Speed seems extraneous (though i may be missing something idk)
 * also explain falling/rising edge via limelight pls lol i think ik how it works via lightgate idk with limelight tho
 */

/*
 * 
 * SwtichNoteLocation(loc) - move note to given location
 * 
 * If there is no note, bails quick and clean
 * 
 * I apologise for the word vomit here - this is just me thinking through the problem - BG
 * 
 * On the robot, there are 4 states: {"OUTSIDE", "INTAKE", "TRANSFER", "SHOOTER"}
 * Outside - the state where the note is not in contact with the robot in any way. 
 * Intake - the state where the intake section of the robot is in control of the note.
 * Transfer (the most complicated one) - the state where the transfer as well as the 2 lighgates positioned there are a part of the robot.
 * Shooter - the stage where the note is queued to shoot or is being shot.
 * 
 * There are also 5 pieces of information to determine where the note is on the robot:
 * Lightgate - the only absolute information we have. Also the most annoying to program. 0 - lightgate is unbroken; 1 - lightgate is broken
 * Rising + FallingEdge - the instantaneous moment (one frame I think) when the lightgate goes from an unbroken to a broken state. Rising edge: unbroken to broken; Falling edge: broken to unbroken.
 * Where note is - where the note is currently. What we are trying to determine (most likely) from all this information.
 * Where note was - Where the note was, deterimed by using all of this information to draw a plausible reference.
 * Motor direction - positive direction is note going in the direction of the shooter, negative dirtection is the note going towards the intake (helps us determine where note is/was)
 * 
 * Using the information above, we can consider two points.
 * - There are only 3 ways that the robot can interact with "outside": intaking, shooting through the intake, and shooting through the shooter
 * - We will never want to stop at the transfer. It is merely a checkpoint where we can gather more information and a way for the note to pass through to the shooter/intake.
 * 
 * Therefore, we can consider 6 different solutions, depending on what is happening on the robot, detailed by section 1 of the following flowchart: https://lucid.app/lucidchart/09ee3b97-e8e8-46e4-a413-eb67215a929d/edit?invitationId=inv_b0701249-a09c-43cd-ad6d-d95d58985b11
 * 
 * Here's where all the data from above really comes together. Let's start with the easiest one:
 * 1. Shooter or intake to outside - the shooter does it's shoot command or the intake does its intake command.
 *   CHECKS: Read the motor speed/direction to see that the note was shot.
 * 2. Outside to intake - we intake a note.
 *   CHECKS: Read the motor speed + direction, as well as know that our previous note state as read by all data points was previously 0.
 * 3. Intake to transfer - we run the motors a set speed for a set amount of time until some of the data points above are flagged.
 *   CHECKS: Check for a RisingEdge on the limelight, the motor direction, and the lightgate.
 * 4. Transfer to Shooter - run the motors until the checks are declared true
 *   CHECKS: Check for a FallingEdge on the limelight and check motor direction/
 * 5. Shooter to Transfer - run the motors until the checks are declared true.
 *   CHECKS: Check for a risingedge and check motor direction/speed
 * 6. Transfer to Intake - run the motors until the checks are declared true
 *   CHECKS: Check for a fallingedge on the LG and check motor speed/direction
 * 
 * All of these can be easily checked through two main data points: Rising/Falling Edges and motor direction. Coupling these with previous data points we've collected, such as where the note was, then we can draw some extremely strong and accurate conclusions in the code about where the note is.
 * 
 * --Ben G. :)
 */


public class NoteLocationHandler extends Command {

  Intake intake = RobotContainer.getSubsystem(Intake.class);
  Transfer transfer = RobotContainer.getSubsystem(Transfer.class);
  boolean firstGate = false;

  NoteCommandedLocation commandedLocation;   // arg to cmd or computed in init if toggling
  boolean isDone;                            // can't call end(), this flag captures decisions

  // speeds and target to do the transfer to
  BooleanSupplier targetHasNote;
  double intake_speed;
  double transfer_speed;

  /** Creates a new transferToIntake. */
  public NoteLocationHandler(NoteCommandedLocation commandedLocation) {
    this.commandedLocation = commandedLocation;
    addRequirements(intake, transfer);
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
          "********The Transfer is not continuing because the intake angle is not ready.");
      isDone = true;
    }

    if (!isNoteInIntake && !isNoteInTransfer) {
      System.out.println(
          "*********The transfer is not continuing because neither the shooter nor the intake has a note.");
      isDone = true;
    }

    if (isNoteInIntake && isNoteInTransfer) {
      System.out.println(
          "*********FATAL: Both the intake and transfer have a note.");
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
    //NOTE: we can extend this switchcase to the long comment above - ben g
    switch (commandedLocation) {
      case Intake:
        targetHasNote = intake::hasNote;        
        intake_speed = -0.8;   // [% pwr]  neg for intake <-- transfer
        transfer_speed = -35.0; // [cm/s]
        intake.setHoldNote(true); // tell transfer to watch for Note
        transfer.setHasNote(false);
        break;

      case Transfer:
        //going intake --> transfer 
        targetHasNote = transfer::hasNote;
        intake_speed = 0.8;   // [% pwr]
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
   * TODO: Consider moving to subsystem API
   */
  private boolean isIntakeAngleReady() {
    double currentAngle = intake.getAnglePosition();
    // absolute value of the difference between 2.5(ish) degrees
    return Math.abs(currentAngle - Intake.DownPos) < 10;
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
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
