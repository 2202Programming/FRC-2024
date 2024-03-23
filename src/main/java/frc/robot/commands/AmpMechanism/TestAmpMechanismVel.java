// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AmpMechanism;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AmpMechanism;

public class TestAmpMechanismVel extends Command {
  private AmpMechanism AmpMechanism;
  double vel;
  /** Creates a new TestAmpVel. */
  public TestAmpMechanismVel(double vel) {
    AmpMechanism = RobotContainer.getSubsystem(AmpMechanism.class);
    this.vel = vel;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AmpMechanism.setVelocity(vel);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    AmpMechanism.setVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
