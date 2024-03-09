// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.BlinkyLights.BlinkyLightUser;
import frc.robot.subsystems.Intake;
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
  /** Creates a new ShooterSequence. */
  //use simple Shooter, even if ShooterServo is created because this command can work with either.
  final ShooterServo shooter;
  final Transfer transfer;
  final Intake intake;
  final int DONE_COUNT = 20;
  double speed;
  double angle;
  int count = 0;
  Phase phase;

  public enum Phase {
    HasNote, ShooterMotorOn, TransferMotorOn, Finished;
  }

  public ShooterServoSequence(double angle, double speed) {
    this.angle = angle;
    this.speed = speed;
    this.shooter = RobotContainer.getSubsystem(ShooterServo.class);
    this.transfer = RobotContainer.getSubsystem(Transfer.class);
    this.intake = RobotContainer.getSubsystem(Intake.class);
    addRequirements(shooter, transfer, intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("***ShooterSequence:init....***");
    count = 0;
    phase = Phase.HasNote;
      intake.setMaxVelocity(60.0);
      intake.setAngleSetpoint(Intake.ShootingPos);
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
      System.out.println("***ShooterSequence:HasNote....***");
      if(intake.angleAtSetpoint()){
      shooter.setShooterAngleSetpoint(angle);
          shooter.setRPM(speed, speed); // placeholder
          if(shooter.atShooterAngleSetpoint()){
          phase = Phase.ShooterMotorOn;
          }
      }
        break;
      case ShooterMotorOn:
        System.out.println("***ShooterSequence:ShooterMotorOn....***");
        if (shooter.isAtRPM(100)) {
          transfer.setSpeed(35.0);
          phase = Phase.TransferMotorOn;
          System.out.println("***ShooterSequence:TransferMotorOn....***");
        }
        break;
      case TransferMotorOn:  
        count++;
        if (count >= DONE_COUNT) {
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
    shooter.setShooterAngleSetpoint(0.0); //whatever back pos is
    intake.setMaxVelocity(120.0);  //[deg/s] 2.sec to retract
    intake.setAngleSetpoint(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase == Phase.Finished;
  }
}
