// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake_Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeOn extends Command {
  /** Creates a new intakeForward. */
  public final Intake intake;
  double intake_speed;
  boolean use_dashboard = false;

  public IntakeOn(double intake_speed) {
    this.intake = RobotContainer.RC().intake; // fixed when merge
    this.intake_speed = intake_speed;
  }

  public IntakeOn() {
    // use default
    this(Intake_Constants.IntakeMotorDefault);
  }

  public IntakeOn(boolean use_dashboard) {
    this();
    if (use_dashboard) {
      this.intake_speed = SmartDashboard.getNumber("Intake Speed", Intake_Constants.IntakeMotorDefault);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setMotorSpeed(intake_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (use_dashboard) {
        intake_speed = SmartDashboard.getNumber("Intake Speed", 0);
        intake.setMotorSpeed(intake_speed);
    } 
  }

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
