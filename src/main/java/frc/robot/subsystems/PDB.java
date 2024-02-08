// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;

public class PDB extends SubsystemBase {
  private PowerDistribution pd;

  public PDB() {
    pd = new PowerDistribution(CAN.PDP, ModuleType.kRev);
  }

  @Override
  public void periodic() {
    double totalPower = pd.getTotalPower();
    SmartDashboard.putNumber("Total Power", totalPower);
    double temperatureCelsius = pd.getTemperature();
    SmartDashboard.putNumber("Temperature", temperatureCelsius);
    double voltage = pd.getVoltage();
    SmartDashboard.putNumber("Voltage", voltage);
    double currentDrive = pd.getCurrent(0);
    SmartDashboard.putNumber("FLCurrentDrive", currentDrive);
    double currentAngle = pd.getCurrent(1);
    SmartDashboard.putNumber("FLCurrentAngle", currentAngle);
  }
}
