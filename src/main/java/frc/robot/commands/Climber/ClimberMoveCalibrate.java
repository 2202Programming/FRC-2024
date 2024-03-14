// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber.Climber;

public class ClimberMoveCalibrate extends Command {
  Climber climber;
  final int DELAY_COUNT = 3;
  int count;
  final double tolerance = 0.8;
  /** Creates a new ClimberMoveCalibrate. */
  public ClimberMoveCalibrate() {
    climber = RobotContainer.getSubsystem(Climber.class);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    count++;
    if(count >=DELAY_COUNT){
        climber.setArmVelocity(-3.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setClimberPos(0.0);
    climber.setArmVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(climber.getClimberVelocity()) < tolerance;
  }
}
