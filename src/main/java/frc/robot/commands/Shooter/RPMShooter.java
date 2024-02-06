// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

/** An example command that uses an example subsystem. */
public class RPMShooter extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter m_shooter;
  private final CommandXboxController m_driverController;
  private double requestedLeftShooterRPM;
  private double requestedRightShooterRPM;
  private double requestedBothShooterRPM;
  private double requestedPercent;
  private boolean triggerMode = false;
  private double currentTriggerPercent;
  private double lastRequestedLeftShooterRPM;
  private double lastRequestedRightShooterRPM;

  private double requestedP = 0.0001;
  private double requestedI = 0.0;
  private double requestedD = 0.0;

  private double currentP = 0.0;
  private double currentI = 0.0;
  private double currentD = 0.0;

  public RPMShooter(CommandXboxController controller) {
    m_shooter = RobotContainer.getSubsystem(Shooter.class);
    m_driverController = controller;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    requestedPercent = 0.0;
    requestedLeftShooterRPM = 0.0;
    requestedRightShooterRPM = 0.0;

    SmartDashboard.putNumber("Trigger Percent", m_driverController.getLeftTriggerAxis());
    SmartDashboard.putNumber("Requested Percent", requestedPercent);

    SmartDashboard.putBoolean("Trigger Mode", triggerMode);
    SmartDashboard.putNumber("Requested Left Shooter RPM",0.0);
    SmartDashboard.putNumber("Requested Right Shooter RPM",0.0);
     
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
    SmartDashboard.putBoolean("Trigger Mode", triggerMode);

    lastRequestedLeftShooterRPM = requestedLeftShooterRPM;
    lastRequestedRightShooterRPM = requestedRightShooterRPM;

    requestedLeftShooterRPM = SmartDashboard.getNumber("Requested Left Shooter RPM",0.0);
    requestedRightShooterRPM = SmartDashboard.getNumber("Requested Right Shooter RPM",0.0);
    requestedBothShooterRPM = SmartDashboard.getNumber("Requested Shooter RPM: ",0.0);
    requestedPercent = SmartDashboard.getNumber("Requested Percent", 0.0);

    switch(m_shooter.getShooterMode()){
      case Trigger: //this mode uses left trigger as motor %
        m_shooter.setMotorSpeed(currentTriggerPercent);
        break;
      case Percent: //this mode uses requested % off smart dashboard as motor %
        m_shooter.setMotorSpeed(requestedPercent);
        break;
      case RPM: //this mode uses requested RPM off smart dashboard in velocity controlled mode
        if ((lastRequestedLeftShooterRPM != requestedLeftShooterRPM) || (lastRequestedRightShooterRPM != requestedRightShooterRPM)) {
          m_shooter.setShooterRPM(requestedLeftShooterRPM, requestedRightShooterRPM);
        }
        /*this SHOULD set both motors at the same time
        *TODO: test this*/
        if ((requestedBothShooterRPM > 0)) {
          if ((requestedBothShooterRPM != requestedLeftShooterRPM) && (requestedBothShooterRPM != requestedRightShooterRPM)) {
            m_shooter.setShooterRPM(requestedBothShooterRPM, requestedBothShooterRPM);          
          } else {
            m_shooter.setShooterRPM(requestedLeftShooterRPM, requestedRightShooterRPM);
        }
      }
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
    m_shooter.setShooterRPM(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
