// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashSet;
import java.util.Set;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.RobotSpecs.RobotNames;

public class PneumaticsControl extends SubsystemBase {
    /** Creates a new Pneumatics Pressure Controller. */
    public PneumaticsControl() {

    }

    @Override
    public void periodic() {

    }

    public double get_tank_pressure() {
        return -1000.0;
    }

    public boolean get_compressor_status() {
        return false;
    }

    public void compressor_on() {

    }

    public void compressor_off() {

    }

    /*
     * Todo:
     * Create Functions - What functionality is required for this class?
     * Whats are the return type?
     * Whats are the parameters?
     */
}
