// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * Driver presses button
 * Set intake angle to floor, turn on intake motors
 * Wait until lightgate detects note
 * Turn off intake
 * bring intake to shooting pos, and turn blinky lights
 * to green!!
 * 
 */

public class NoteInIntake extends Command {

  final Intake intake;
  final Shooter shooter;
  final double FIRST_COUNT = 100; //frames
  final double SECOND_COUNT = 100;
  double count_one;
  double count_two;

  public enum Phase {
    //TODO: maybe remove; might not need in future
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
  public NoteInIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = RobotContainer.getSubsystem(Intake.class);
    this.shooter = RobotContainer.getSubsystem(Shooter.class);
    addRequirements(intake, shooter);
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
        phase = Phase.WaitingForNote;
        break;
      case WaitingForNote:
        if (intake.has_Note()) {
            intake.setIntakeSpeed(0.0);
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
    intake.setMaxVelocity(120.0);
    intake.setAngleSetpoint(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase == Phase.Finished;

  }
}
