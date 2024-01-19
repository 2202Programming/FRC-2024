// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class RPMShooter extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooter;
  private final CommandXboxController m_driverController;
  private double requestedShooterRPM;
  private double requestedPercent;
  private boolean triggerMode = false;
  private double currentTriggerPercent;

  private double requestedP = 0.0001;
  private double requestedI = 0.0;
  private double requestedD = 0.0;

  private double currentP = 0.0;
  private double currentI = 0.0;
  private double currentD = 0.0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  public RPMShooter(CommandXboxController controller, ShooterSubsystem shooter) {
    m_shooter = shooter;
    m_driverController = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    requestedPercent = 0.0;
    requestedShooterRPM = 0.0;

    SmartDashboard.putNumber("Trigger Percent", m_driverController.getLeftTriggerAxis());
    SmartDashboard.putNumber("Requested Percent", requestedPercent);
    SmartDashboard.putNumber("Motor Percent", m_shooter.getMotorSpeed());
    SmartDashboard.putBoolean("Trigger Mode", triggerMode);
    SmartDashboard.putNumber("Requested Shooter RPM",0.0);
     
    SmartDashboard.putNumber("Requested P",requestedP);
    SmartDashboard.putNumber("Requested I",requestedI);
    SmartDashboard.putNumber("Requested D",requestedD);

    currentP = m_shooter.getP();
    currentI = m_shooter.getI();
    currentD = m_shooter.getD();
  
    SmartDashboard.putNumber("Current P", currentP);
    SmartDashboard.putNumber("Current I", currentI);
    SmartDashboard.putNumber("Current D", currentD);    


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentTriggerPercent = m_driverController.getLeftTriggerAxis();
    
    SmartDashboard.putNumber("Trigger Percent", currentTriggerPercent);
    SmartDashboard.putNumber("Requested Percent", requestedPercent);
    SmartDashboard.putNumber("Motor Percent", m_shooter.getMotorSpeed());
    SmartDashboard.putBoolean("Trigger Mode", triggerMode);

    requestedShooterRPM = SmartDashboard.getNumber("Requested Shooter RPM",0.0);
    requestedPercent = SmartDashboard.getNumber("Requested Percent", 0.0);

    switch(m_shooter.getShooterMode()){
      case Trigger: //this mode uses left trigger as motor %
        m_shooter.setMotorSpeed(currentTriggerPercent);
        break;
      case Percent: //this mode uses requested % off smart dashboard as motor %
        m_shooter.setMotorSpeed(requestedPercent);
        break;
      case RPM: //this mode uses requested RPM off smart dashboard in velocity controlled mode
        m_shooter.setShooterRPM(requestedShooterRPM);
        break;
    }
    
    requestedP = SmartDashboard.getNumber("Requested P",requestedP);
    requestedI = SmartDashboard.getNumber("Requested I",requestedI);
    requestedD = SmartDashboard.getNumber("Requested D",requestedD);
    
    currentP = m_shooter.getP();
    currentI = m_shooter.getI();
    currentD = m_shooter.getD();
  
    SmartDashboard.putNumber("Current P", currentP);
    SmartDashboard.putNumber("Current I", currentI);
    SmartDashboard.putNumber("Current D", currentD);  
    
    setPIDs();

  }

  private void setPIDs(){
    if(currentP != requestedP) m_shooter.setP(requestedP);
    if(currentI != requestedI) m_shooter.setI(requestedI);
    if(currentD != requestedD) m_shooter.setD(requestedD);
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
