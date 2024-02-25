// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class PneumaticsTest extends Command {
  /** Creates a new PneumaticsTest. */
  final Shooter shooter;
  boolean extend;

  public PneumaticsTest(boolean extend) {
    this.extend = extend;
    this.shooter = RobotContainer.getSubsystem("SHOOTER");
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (extend) {
      shooter.deploy();
      System.out.println("deploying");
    } else {
      shooter.retract();
      System.out.println("retracting");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
