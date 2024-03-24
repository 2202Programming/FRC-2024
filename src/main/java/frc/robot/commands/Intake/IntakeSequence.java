// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterServo;
import frc.robot.subsystems.Transfer;
import frc.robot.util.RobotSpecs.RobotNames;

/**
 * Driver presses button
 * Set intake angle to floor, turn on intake and transfer motors
 * Wait until lightgate detects note
 * Turn off intake
 * Wait x amount of time after lightgate detects note
 * shut off transfer motor, bring intake to movement pos, and turn blinky lights
 * to green!!
 */

public class IntakeSequence extends Command {
  public final static int DONE_COUNT = 10;

  final Intake intake;
  final Transfer transfer;
  final Shooter shooter;
  boolean stay_down;
  int count;

  public enum Phase {
    IntakeDown, WaitingForNote, Finished, HaveNote
  }

  Phase phase;
  final double DownAngle;
  boolean saw_note;

  // Alpha bot doesn't transfer if the pnumatics shooter is up
  final boolean must_retract_shooter;
  /*
   * stay_down = true for No defense rapid shoot
   */
  public IntakeSequence(boolean stay_down) {
    this.stay_down = stay_down;
    this.intake = RobotContainer.getSubsystem(Intake.class);
    this.transfer = RobotContainer.getSubsystem(Transfer.class);
    this.shooter = RobotContainer.getSubsystemOrNull(Shooter.class);

    //check Shooter type to know if we must retract, true for standard Shooter
    must_retract_shooter = !(shooter instanceof ShooterServo);

    // Select down angle based on which bot we have
    DownAngle = (RobotContainer.getRobotSpecs().myRobotName == RobotNames.CompetitionBotAlpha2024) ?
      91.0 : 105.0;

    addRequirements(intake, transfer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (must_retract_shooter)
          shooter.retract();
    count = 0;
    phase = Phase.IntakeDown;
    saw_note = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (phase) {
      case IntakeDown:
        intake.setMaxVelocity(Intake.TravelDown);
        intake.setAngleSetpoint(DownAngle);
        intake.setIntakeSpeed(Intake.RollerMaxSpeed); // [cm/s]
        transfer.setSpeed(50.0);  //[cm/s]
        phase = Phase.WaitingForNote;
        break;

      case WaitingForNote:
        saw_note = transfer.senseNote();
        phase = saw_note ? Phase.HaveNote : Phase.WaitingForNote;
        break;
      case HaveNote:        
        if (++count >= DONE_COUNT) {
          intake.setMaxVelocity(Intake.TravelUp);
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
    // TODO: edge case, the sequential doesn't cancel
    // TODO: Why did the intake angle go back up even when there is no command to
    // TODO: edge case #2 - If the driver releases button as intake is coming up, it
    // will go down before coming back up again
    if (interrupted) {
      // Creates a command to continue going down until we get to the bottom before
      // moving back up, to minimize belt slippage on Alpha
      System.out.println("Interrupted intakeSequence");
      var cmd = new SequentialCommandGroup();
      if (saw_note && count < DONE_COUNT) {
        //Need to finish the transfer before we do anything else
        cmd.addCommands(new FinishIntakeSequence(count, stay_down));
      }
      if (!intake.angleAtSetpoint()) {
        cmd.addCommands(new MoveToAnglePos(Intake.DownPos, Intake.TravelDown));
      }
      cmd.addCommands(new MoveToAnglePos(Intake.UpPos, Intake.TravelUp));
      cmd.addRequirements(intake);
      cmd.schedule();
    }
    
    if (!stay_down && !interrupted) {
      intake.setMaxVelocity(Intake.TravelUp);
      intake.setAngleSetpoint(Intake.UpPos);
    }
    // turn off rollers, if not finished they get turned on again
    transfer.setSpeed(0.0);
    intake.setIntakeSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase == Phase.Finished;

  }

}
