// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber.Climber;

public class ClimbVel extends Command {
  Climber climber;
  double speed;
  

  /**
   * Create a new ClimbVel Command
   * The arm will move the entire time this command is running, therefore
   * this command should be run are whileTrue() only
   * @param speed Arm movment vel, cm/s
   */
  public ClimbVel(double speed) {
    climber = RobotContainer.getSubsystem(Climber.class);
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setArmVelocity(speed);
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
climber.setArmVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
