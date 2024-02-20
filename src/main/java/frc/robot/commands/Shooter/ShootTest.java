// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShootTest extends Command {
  /** Creates a new ShootTest. */
  final Shooter shooter;
  double speed;
  public ShootTest(double speed) {
        this.shooter = RobotContainer.getSubsystem(Shooter.class);
        this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setRPM(speed, speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setRPM(0, 0);
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
