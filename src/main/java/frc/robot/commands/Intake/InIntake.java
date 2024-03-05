// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

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
  final double DONE_COUNT = 1;
  double count;

  public enum Phase {
    IntakeDown("IntakeDown"),
    WaitingForNote("WaitingForNote"),
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
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = RobotContainer.getSubsystem(Intake.class);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    phase = Phase.IntakeDown;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (phase) {
      case IntakeDown:
        intake.setMaxVelocity(60.0);
        intake.setAngleSetpoint(100.0);
        intake.setIntakeSpeed(0.8); // %
        phase = Phase.WaitingForNote;
        break;
      case WaitingForNote:
        if (intake.hasNote()) {
           phase = Phase.Finished;
        }
        if(count >= DONE_COUNT){
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
    intake.setIntakeSpeed(0.0);
    intake.setAngleSetpoint(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase == Phase.Finished;

  }
}
