// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class RPMShooter extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooter;
  private double requestedShooterRPM = 0.0;
  private double requestedP = 0.001;
  private double requestedI = 0.0;
  private double requestedD = 0.0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  public RPMShooter(ShooterSubsystem shooter) {
    m_shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     SmartDashboard.putNumber("Current Shooter RPM",0.0);
     SmartDashboard.putNumber("Current Motor RPM",0.0);
     SmartDashboard.putNumber("Requested Shooter RPM",0.0);
     
    SmartDashboard.putNumber("Requested P",requestedP);
    SmartDashboard.putNumber("Requested I",requestedI);
    SmartDashboard.putNumber("Requested D",requestedD);

    SmartDashboard.putNumber("Current P",0.0);
    SmartDashboard.putNumber("Current I",0.0);
    SmartDashboard.putNumber("Current D",0.0);    


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     SmartDashboard.putNumber("Current Shooter RPM",m_shooter.getShooterRPM());
     SmartDashboard.putNumber("Current Motor RPM", m_shooter.getMotorRPM());
     requestedShooterRPM = SmartDashboard.getNumber("Requested Shooter RPM",0.0);
     m_shooter.setShooterRPM(requestedShooterRPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooterRPM(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
