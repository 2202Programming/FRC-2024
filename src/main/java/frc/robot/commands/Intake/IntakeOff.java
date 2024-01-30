// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake_Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeOff extends Command {
  /** Creates a new intakeForward. */
  public final Intake intake;
  public IntakeOff() {
    this.intake = RobotContainer.RC().intake; //fixed when merge
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.intakeOff();
    intake.retract();
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
    return false;
  }
}
