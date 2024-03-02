// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber.Climber;

public class ClimbVel extends Command {
  Climber climber;
  boolean right;
  boolean left;
  double speed;
  /** Creates a new ClimbVel. */
  public ClimbVel(boolean right, boolean left, double speed) {
    climber = RobotContainer.getSubsystem(Climber.class);
    this.right = right;
    this.left = left;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(left){
    climber.setLeftArmVelocity(speed);
    }
    if(right){
    climber.setRightArmVelocity(speed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setLeftArmVelocity(0.0);
    climber.setRightArmVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
