// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotContainer;
import frc.robot.Constants.Intake_Constants;
import frc.robot.Constants.Transfer_Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.BlinkyLights.BlinkyLightUser;

/**
 * Driver presses button
 * Set intake angle to floor, turn on intake and transfer motors
 * Wait until lightgate detects note
 * Turn off intake
 * Wait x amount of time after lightgate detects note
 * shut off transfer motor, bring intake to movement pos, and turn blinky lights
 * to green!!
 * 
 */

public class IntakeSequence extends BlinkyLightUser {
  public final int DONE_COUNT = 0;
  int count;
  final Intake intake;
  final Transfer transfer;

  public enum Phase {
    IntakeDown("IntakeDown"),
    WaitingForNote("WaitingForNote"),
    NoteDetected("NoteDetected"),
    Finished("Finished");

    String name;

    private Phase(String name) {
      this.name = name;
    }

    public String toString() {
      return name;
    }

  }

  Phase phase;

  /** Creates a new IntakeSequence. */
  public IntakeSequence() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = RobotContainer.getSubsystem(Intake.class);
    this.transfer = RobotContainer.getSubsystem(Transfer.class);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    phase = Phase.IntakeDown;
  }

  public Color8Bit colorProvider() {
    // make sure not is safely in our possession before going back
    return (count > DONE_COUNT) ? BlinkyLights.GREEN : BlinkyLights.RED;
  };

  @Override
  public boolean requestBlink() {
    return false; // we want solid lights
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (phase) {
      case IntakeDown:
        intake.setAnglePosition(Intake_Constants.AngleFloorPos);
        intake.setIntakeSpeed(Intake_Constants.IntakeMotorDefault);
        transfer.setSpeed(Transfer_Constants.TRANSFER_MOTOR_ON);
        phase = Phase.WaitingForNote;
        break;
      case WaitingForNote:
        if (transfer.hasNote()) {
          intake.setIntakeSpeed(0.0);
          phase = Phase.NoteDetected;
        }
        break;
      case NoteDetected:
        count++;
        if (count > DONE_COUNT) {
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
    transfer.setSpeed(0.0);
    intake.setIntakeSpeed(0.0);
    intake.setAnglePosition(Intake_Constants.DrivingPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (phase == Phase.Finished) {
      return true;
    }
    return false;
  }

}
