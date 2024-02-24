// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;

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

public class InIntake extends Command {

  final Intake intake;
  final Transfer transfer;
  final Shooter shooter;
  final double FIRST_COUNT = 100; //frames
  final double SECOND_COUNT = 100;
  double count_one;
  double count_two;

  public enum Phase {
    IntakeDown("IntakeDown"),
    WaitingForNote("WaitingForNote"),
    Eject("Eject"),
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
  public InIntake() {
    count_one = 0;
    count_two = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = RobotContainer.getSubsystem(Intake.class);
    this.transfer = RobotContainer.getSubsystem(Transfer.class);
    this.shooter = RobotContainer.getSubsystem("SHOOTER");
    addRequirements(intake, transfer, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    phase = Phase.IntakeDown;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (phase) {
      case IntakeDown:
        shooter.retract();
        intake.setMaxVelocity(60.0);
        intake.setAngleSetpoint(100.0);
        intake.setIntakeSpeed(0.8); // %
        transfer.setSpeed(35.0);
        phase = Phase.WaitingForNote;
        break;
      case WaitingForNote:
        if (transfer.hasNote()) {
            count_one++;
        }
        if(count_one > FIRST_COUNT){
          transfer.setSpeed(-35.0);
          phase = Phase.Eject;
          }
        break;
        case Eject:
          if(!transfer.hasNote()){
            count_two++;
          }
        if(count_two > SECOND_COUNT){
            intake.setMaxVelocity(120.0);
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
    intake.setAngleSetpoint(0.0);
    intake.setHasNote(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase == Phase.Finished;

  }
}
