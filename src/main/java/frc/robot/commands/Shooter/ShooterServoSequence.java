// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.BlinkyLights.BlinkyLightUser;
import frc.robot.subsystems.ShooterServo;
import frc.robot.subsystems.Transfer;

/**
 * Button pressed starts command
 * Make sure we detect a note
 * Turn the shooter motors on
 * Once our shooter motors reach the proper speed, turn on the transfer motors
 * Once the lightgate no longer detects the note, wait x amount of frames
 * After x amount of frames, turn the blinky lights red, then turn everything
 * off (command ended)
 */

// TODO: probs can remove all angle stuff
public class ShooterServoSequence extends BlinkyLightUser {
  /** Creates a new ShooterSequence. */
  // use simple Shooter, even if ShooterServo is created because this command can
  // work with either.
  boolean stay;
  final ShooterServo shooter;
  final Transfer transfer;
  final int DONE_COUNT = 100; // TODO: find actual value (around 10-20)
  double speed;
  double angle;
  int count = 0;
  Phase phase;
  boolean sensed_note;

  public enum Phase {
    HasNote, ShooterMotorOn, TransferMotorOn, Finished;
  }

  public ShooterServoSequence(double angle, double speed, boolean stay) {
    this.stay = stay;
    this.angle = angle;
    this.speed = speed;
    this.shooter = RobotContainer.getSubsystem(ShooterServo.class);
    this.transfer = RobotContainer.getSubsystem(Transfer.class);
    addRequirements(shooter, transfer);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public ShooterServoSequence(double angle, double speed) {
    this(angle, speed, false);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sensed_note = false;
    System.out.println("***ShooterSequence:init....***");
    count = 0;
    phase = Phase.HasNote;
  }

  public Color8Bit colorProvider() {
    // make sure not is safely in our possession before going back
    return (!transfer.hasNote()) ? BlinkyLights.RED : BlinkyLights.GREEN;
  };

  @Override
  public boolean requestBlink() {
    return false; // we want solid lights
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (phase) {
      case HasNote:
        // System.out.println("***ShooterSequence:HasNote....***");
        shooter.setAngleSetpoint(angle);
        shooter.setRPM(speed, speed); // placeholder
        if (shooter.atSetpoint()) {
           System.out.println("***ShooterSequence:ShooterMotorOn....***");
          phase = Phase.ShooterMotorOn;
        }
        break;
      case ShooterMotorOn:
        if (shooter.isAtRPM(50)) {
          transfer.setSpeed(40.0);
          phase = Phase.TransferMotorOn;
          System.out.println("***ShooterSequence:TransferMotorOn....***");
        }
        break;
      case TransferMotorOn:
        // sensed_note = transfer.senseNote() || sensed_note ? true : false;
        count++;
        if (/*sensed_note ||*/ count >= DONE_COUNT) {
          phase = Phase.Finished;
          System.out.println("***ShooterSequence:finished....***");
        }
        break;
      case Finished:

        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transfer.setHasNote(false);
    transfer.setSpeed(0.0);
    shooter.setRPM(0.0, 0.0);
    if(!stay){shooter.setAngleSetpoint(ShooterServo.MIN_DEGREES);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase == Phase.Finished;
  }
}
