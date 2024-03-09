// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber.Climber;

public class Climb extends Command {
  /** Creates a new Climber. */
  private Climber climber;
  private double pos;
  private boolean done;
  
  public Climb(double pos) {  
    // Use addRequirements() here to declare subsystem dependencies.
    climber = RobotContainer.getSubsystem(Climber.class);
    this.pos = pos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setArmHeight(pos);
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: Do we want to monitor current while this is occuring?
    if(climber.leftArmAtSetpoint() && climber.rightArmAtSetpoint()){
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}