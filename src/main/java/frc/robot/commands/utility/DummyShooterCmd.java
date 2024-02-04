// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.utility;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

//TODO replace all dummy commands with real code not this pls -er

public class DummyShooterCmd extends Command {
  /** Creates a new DummyShooterCmd. */

  public Shooter shooter;

  public DummyShooterCmd() {

    this.shooter = RobotContainer.getSubsystem("SHOOTER");
    // protect incase we don't have a shooter
    // addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Shooter is on, if we had one! :3");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Shooter is off ");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
