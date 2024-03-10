// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterServo;

public class ShooterAngleSetPos extends Command {
  ShooterServo shooter;
  double desired_pos;
  /** Creates a new ShooterAngleCalibrate. */
  public ShooterAngleSetPos(double desired_pos) {
    shooter = RobotContainer.getSubsystem(ShooterServo.class);
    this.desired_pos = desired_pos;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setAngleSetpoint(desired_pos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(shooter.atSetpoint()){
      return true;
    }
    return false;
  }
}
