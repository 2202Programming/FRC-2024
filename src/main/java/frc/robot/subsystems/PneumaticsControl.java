// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsControl extends SubsystemBase {

    // instance variables
    Compressor compressor;

    /** Creates a new Pneumatics Pressure Controller. */
    public PneumaticsControl() {
        // Initializes an pneumatics Analog Input on port 0
        compressor = new Compressor(PneumaticsModuleType.REVPH);
    }

    @Override
    public void periodic() {

    }

    public double get_tank_pressure() {
        return compressor.getPressure();
    }

    public boolean get_compressor_status() {
        return compressor.isEnabled();
    }

    public void compressor_on() {
        compressor.enableAnalog(70, 120);
    }

    public void compressor_off() {
        compressor.disable();
    }
}
