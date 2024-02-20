// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class CalibratePos extends Command {
  /** Creates a new AnglePos. */
  public final Intake intake;
  double desiredPos;
  public CalibratePos(double desiredPos) {
    this.intake = RobotContainer.getSubsystem(Intake.class);
    this.desiredPos = desiredPos;
    // Use addRequirements() here to declare subsystem dependencies.
  }

    
  @Override
  public void initialize() {
    intake.setAnglePosition(desiredPos);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return true;
}
}
