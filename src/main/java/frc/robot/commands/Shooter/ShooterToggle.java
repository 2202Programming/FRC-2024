// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;

public class ShooterToggle extends Command {
  /** Creates a new ShooterToggle. */
  public final Shooter shooter;
  public final Intake intake;
  public final Transfer transfer;
  final boolean pneumatics = false; // YES FOR SUSSEX NO AFTER???
  final int DELAY = 20; // figure out this number
  int count = 0;
  boolean RPM_dropped;
  int aprilTarget;

  public ShooterToggle() {
    this.shooter = RobotContainer.getSubsystem(Shooter.class);
    this.intake = RobotContainer.getSubsystem(Intake.class);
    this.transfer = RobotContainer.getSubsystem(Transfer.class);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RPM_dropped = false;
    if (intake.hasNote()) {
      shooter.setRPM(0.5, 0.5);
    } else {
      end(true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.isAtRPM(100) && intake.angleAtSetpoint()) {// Check the RPM tolerance
      transfer.transferMotorOn();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setRPM(0, 0);
    transfer.transferMotorOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(shooter.getRPM() - shooter.getDesiredRPM()) > 100) {
      RPM_dropped = true;
    }
    if (RPM_dropped) {
      count++;
    }
    if (count > DELAY) {
      return true;
    } else {
      return false;
    }
  }
}
