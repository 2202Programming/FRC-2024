// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.Intake_Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.BlinkyLights.BlinkyLightUser;

public class IntakeSequence extends BlinkyLightUser {
  public final int DONE_COUNT = 0;
  int count;
  boolean detected_note;
  final Intake intake;
  final Transfer transfer;
  public enum Phase{
    Stopped("Stopped"),
    IntakeDown("IntakeDown"),
    WaitingForNote("WaitingForNote"),
    NoteDetected("NoteDetected");

    String name;

    private Phase(String name){
      this.name = name;
    }

    public String toString(){
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
    detected_note = false;
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
    switch(phase){
      case Stopped:
      intake.setAnglePosition(Intake_Constants.TransferPosition);
      break;
      case IntakeDown:
      intake.setAnglePosition(Intake_Constants.AngleMotorDefault);
      intake.setIntakeSpeed(Intake_Constants.IntakeMotorDefault);
      transfer.transferMotorOn();
      phase = Phase.WaitingForNote;
      break;
      case WaitingForNote:
      if(transfer.hasNote()){
        detected_note = true;
        intake.setIntakeSpeed(0);
        phase = Phase.NoteDetected;
      }
      break;
      case NoteDetected:
        count++;
        if(count > DONE_COUNT){
          transfer.transferMotorOff();
          intake.setAnglePosition(Intake_Constants.TransferPosition);
        }
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
}
