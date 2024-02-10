// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;

public class PneumaticsControl extends SubsystemBase {

    // instance variables
    Compressor compressor;
    Solenoid m_solenoid;
    PneumaticHub m_PneumaticHub;

    private final int PRESSURE_SENSOR1 = 0; //analong pressure sensor
    private double minPressure = 70;
    private double maxPressure = 120;
    /** Creates a new Pneumatics Pressure Controller. */
    public PneumaticsControl() {
        // NOTE
        // new PneumaticsControlModule(CAN.PCM1) causes CAN token errors
        m_PneumaticHub = new PneumaticHub(CAN.PCM1);
        //m_solenoid = new Solenoid(CAN.PCM1, PneumaticsModuleType.REVPH, 6);
        //compressor = new Compressor(CAN.PCM1, PneumaticsModuleType.REVPH);
        compressor_on();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Compressor Pressure 1", get_tank_pressure(PRESSURE_SENSOR1)); //temp, move to smart logging later
    }

    public double get_tank_pressure(int channel) {
        return m_PneumaticHub.getPressure(channel);
    }

    public boolean get_compressor_status() {
        return m_PneumaticHub.getCompressor();
    }

    public void compressor_on() {
        m_PneumaticHub.enableCompressorAnalog(minPressure,maxPressure);
    }

    public void compressor_off() {
        m_PneumaticHub.disableCompressor();
    }
}
