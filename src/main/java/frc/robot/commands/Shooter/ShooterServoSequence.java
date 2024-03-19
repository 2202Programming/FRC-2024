// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
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

public class ShooterServoSequence extends BlinkyLightUser {
  boolean stay; // if true, shooter stays in current position
  final ShooterServo shooter;
  final Transfer transfer;
  final double TransferSpeed = 40.0; // [cm/s]
  final double NoteTravelDist = 15.0; // [cm] max distance needed for note to move
  final int DONE_COUNT = (int) Math.ceil((NoteTravelDist / TransferSpeed) / Constants.DT);

  double speed;
  double percent_rpm = 0.005; //3%
  double angle;
  int count = 0;
  Phase phase;

  public enum Phase {
    WaitingForSetpoints, WaitingForFinish, Finished;
  }

  public ShooterServoSequence(double angle, double speed, boolean stay) {
    this.stay = stay;
    this.angle = angle;
    this.speed = speed;
    this.shooter = RobotContainer.getSubsystem(ShooterServo.class);
    this.transfer = RobotContainer.getSubsystem(Transfer.class);
    addRequirements(shooter, transfer);
  }

  public ShooterServoSequence(double angle, double speed) {
    this(angle, speed, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    phase = Phase.WaitingForSetpoints;

    // start motor right away
    shooter.setAngleSetpoint(angle);
    shooter.setRPM(speed, speed); // placeholder
    System.out.println("***ShooterSequence:init.... motors on ***");
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
      case WaitingForSetpoints:
        if (shooter.atSetpoint() && shooter.isAtRPM(speed*percent_rpm)) {
          transfer.setSpeed(40.0);
          System.out.println("***ShooterSequence:Setpoints reached, transfer moving ....***");
          phase = Phase.WaitingForFinish;
        }
        break;
      case WaitingForFinish:
        if (++count >= DONE_COUNT) // || transfer.senseNote()) // end cmd faster if we see Note go by
        {
          phase = Phase.Finished;
          System.out.println("***ShooterSequence:finished....***");
        }
        break;
      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transfer.setHasNote(false);
    transfer.setSpeed(0.0);
    shooter.setRPM(0.0, 0.0);
    if (!stay) {
      shooter.setAngleSetpoint(ShooterServo.MIN_DEGREES);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase == Phase.Finished;
  }
}
