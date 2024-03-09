// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
 */

public class IntakeSequence extends Command {

  final Intake intake;
  final Transfer transfer;
  final Shooter shooter;
  boolean stay_down;

  public enum Phase {IntakeDown, WaitingForNote, Finished  }
  Phase phase;

  /*
   * stay_down = true for No defense rapid shoot
   */
  public IntakeSequence(boolean stay_down) {
    this.stay_down = stay_down;
    this.intake = RobotContainer.getSubsystem(Intake.class);
    this.transfer = RobotContainer.getSubsystem(Transfer.class);
    this.shooter = RobotContainer.getSubsystem("SHOOTER");
    addRequirements(intake, transfer, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    phase = Phase.IntakeDown;
    System.out.println("***Init IntakeSequence....***");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (phase) {
      case IntakeDown:
        System.out.println("***IntakeSequence:IntakeDown....***");
        shooter.retract();
        intake.setMaxVelocity(60.0);

        //TODO: FOR ALTERNATE ENCODER _KO
        if(RobotContainer.getRobotSpecs().getRobotNameString().equals("CompetitionBotAlpha2024")){
          intake.setAngleSetpoint(91.0);
        }
        else{
          intake.setAngleSetpoint(100.0);
        }
        intake.setIntakeSpeed(0.6); // %
        transfer.setSpeed(35.0);
        phase = Phase.WaitingForNote;
        System.out.println("***IntakeSequence:WaitingForNote....***");
        break;
      case WaitingForNote:
        if (transfer.hasNote()) {
          intake.setMaxVelocity(120.0);
          intake.setAngleVelocity(0.0);
          phase = Phase.Finished;
          System.out.println("***IntakeSequence:Finished....***");
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
    //TODO: edge case #2 - If the driver releases button as intake is coming up, it will go down before coming back up again
    if (interrupted) {
      // Creates a command to continue going down until we get to the bottom before
      // moving back up, to minimize belt slippage
      var cmd = new SequentialCommandGroup();
      if(!intake.angleAtSetpoint()){
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
    transfer.setSpeed(0.0);
    intake.setIntakeSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase == Phase.Finished;

  }
}
