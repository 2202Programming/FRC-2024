// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.BlinkyLights.BlinkyLightUser;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
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

public class ShooterSequence extends BlinkyLightUser {
  /** Creates a new ShooterSequence. */
  //use simple Shooter, even if ShooterServo is created because this command can work with either.
  final Shooter shooter;
  final Transfer transfer;
  final Intake intake;
  final int DONE_COUNT = 20;
  final int PNEUMATICS_DONE_COUNT = 20;
  double speed;
  int pneumatics_count = 0;
  int count = 0;
  Phase phase;
  boolean shootHigh;

  public enum Phase {
    HasNote, ShooterMotorOn, TransferMotorOn, Finished;
  }

  public ShooterSequence(boolean shootHigh, double speed) {
    this.shootHigh = shootHigh;
    this.speed = speed;
    this.shooter = RobotContainer.getSubsystem("SHOOTER");
    this.transfer = RobotContainer.getSubsystem(Transfer.class);
    this.intake = RobotContainer.getSubsystem(Intake.class);
    addRequirements(shooter, transfer, intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public ShooterSequence(double speed){
    this(false, speed);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    pneumatics_count = 0;
    phase = Phase.HasNote;
    if(shootHigh){
      intake.setMaxVelocity(60.0);
      intake.setAngleSetpoint(Intake.ShootingPos);
    }
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
      if(intake.angleAtSetpoint()){
          if(shootHigh){
            shooter.deploy();
            pneumatics_count++;
          }
          shooter.setRPM(speed, speed); // placeholder
          if(pneumatics_count >= PNEUMATICS_DONE_COUNT || !shootHigh){
          phase = Phase.ShooterMotorOn;
          }
      }
        break;
      case ShooterMotorOn:
        if (shooter.isAtRPM(100)) {
          transfer.setSpeed(35.0);
          phase = Phase.TransferMotorOn;
        }
        break;
      case TransferMotorOn:
        count++;
        if (count >= DONE_COUNT) {
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
    transfer.setHasNote(false);
    transfer.setSpeed(0.0);
    shooter.setRPM(0.0, 0.0);
    shooter.retract();
    intake.setMaxVelocity(120.0);  //[deg/s] 2.sec to retract
    intake.setAngleSetpoint(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase == Phase.Finished;
  }
}
