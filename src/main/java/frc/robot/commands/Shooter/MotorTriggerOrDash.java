// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;

public class MotorTriggerOrDash extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandXboxController m_driverController;
  private final Shooter m_mtr;
  private double requestedPercent;
  private boolean triggerMode = false;
  private double currentTriggerPercent;
  private double currentMotorSpeed;

  public MotorTriggerOrDash(CommandXboxController driverController, Shooter mtr) {
    m_driverController = driverController;
    m_mtr = mtr;
    requestedPercent = 0.0;
    addRequirements(mtr);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Current Motor RPM", m_mtr.getMotorRPM());
    SmartDashboard.putNumber("Trigger Percent", m_driverController.getLeftTriggerAxis());
    SmartDashboard.putNumber("Requested Percent", requestedPercent);
    SmartDashboard.putNumber("Motor Percent", m_mtr.getLeftMotorOutput());
    SmartDashboard.putBoolean("Trigger Mode", triggerMode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentTriggerPercent = m_driverController.getLeftTriggerAxis();
    currentMotorSpeed = m_mtr.getMotorRPM();
    SmartDashboard.putNumber("Current Motor RPM", m_mtr.getMotorRPM());
    
    //if trigger is being pulled, motor should go off trigger instead of smart dashboard request
    if (currentTriggerPercent > 0.01) {
      triggerMode = true;
    }
    else {
      triggerMode = false;
    }

    SmartDashboard.putNumber("Trigger Percent", currentTriggerPercent);
    SmartDashboard.putNumber("Motor Percent", currentMotorSpeed);
    SmartDashboard.putBoolean("Trigger Mode", triggerMode);
    requestedPercent = SmartDashboard.getNumber("Requested Percent", 0.0);

    if(triggerMode){
      //set motor to be left trigger %
      m_mtr.setMotorSpeed(m_driverController.getLeftTriggerAxis());
    }
    else{ //trigger isn't activated, use smart dashboard to guide motor speed
      if (Math.abs(currentMotorSpeed - requestedPercent) > 0.01){ //there is a difference betweent the requested motor speed and current motor speed
        m_mtr.setMotorSpeed(requestedPercent);
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
