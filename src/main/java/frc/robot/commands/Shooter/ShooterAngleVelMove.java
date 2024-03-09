// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterServo;

public class ShooterAngleVelMove extends Command {
  ShooterServo shooter;
  double vel;
  int count;
  double prevPos;
  final int DONE_COUNT = 5; // 0.1 sec
  /** Creates a new ShooterAngleVelMove. */
  public ShooterAngleVelMove(double vel) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = RobotContainer.getSubsystem(ShooterServo.class);
    this.vel = vel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    prevPos = shooter.getShooterAnglePosition();
    shooter.setShooterAngleVelocity(vel);
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.getShooterAnglePosition() == prevPos){
      count++;
    }
    else{
      prevPos = shooter.getShooterAnglePosition();
      count = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setShooterAnglePosition(vel);
    shooter.setShooterAngleVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count >= DONE_COUNT;
  }
}
