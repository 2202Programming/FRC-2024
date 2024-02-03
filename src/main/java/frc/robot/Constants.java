// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Swerve.Config.CANConfig;
import frc.robot.subsystems.Swerve.Config.CANModuleConfig;
import frc.robot.subsystems.Swerve.Config.ChassisConfig;
import frc.robot.subsystems.Swerve.Config.ChassisInversionSpecs;
import frc.robot.subsystems.Swerve.Config.ModuleInversionSpecs;
import frc.robot.subsystems.Swerve.Config.WheelOffsets;
import frc.robot.util.PIDFController;
import frc.robot.util.SubsystemConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // nominal system frame rate, DT (delta T)
  public static final double DT = 0.02; // [s] 20ms framerate 50Hz

  // Handy feet to meters
  public static final double FTperM = 3.28084;
  public static final double MperFT = 1.0 / FTperM;
  public static final int NEO_COUNTS_PER_REVOLUTION = 42;

  /*------------------------Drivetrain-------------------------*/
  public static final class DriveTrain {
    // motor constraints
    public static final double motorMaxRPM = 5600; // motor limit

    // Constraints on speeds enforeced in DriveTrain
    public static final double kMaxSpeed = 21.0 * MperFT; // [m/s]
    public static final double kMaxAngularSpeed = 2 * Math.PI; // [rad/s]
    // TODO: FROM LAST YEAR, NEED TO REVIEW
    /****
     * ### REMINDER - enable these once we have basics working
     * // Other constraints
     * public static final int smartCurrentMax = 60; //amps in SparkMax, max setting
     * public static final int smartCurrentLimit = 35; //amps in SparkMax, inital
     * setting
     */

    // SmartMax PID values [kp, ki, kd, kff] - these get sent to hardware controller
    // DEBUG - SET FF first for drive, then add KP

    // DriveTrain pid values
    public static final PIDFController drivePIDF = new PIDFController(0.09 * FTperM, 0.0, 0.0, 0.08076 * FTperM);
    public static final PIDFController anglePIDF = new PIDFController(0.01, 0.0, 0.0, 0.0); // maybe 1.0,0.0,0.1 from
                                                                                            // SDS sample code?
    /*
     * Settings for different swerve bot chassis
     */
    // FOR SWERVEBOT, aka Tim 2.0
    public static final WheelOffsets swerveBotOffsets = new WheelOffsets(-98.942, 91.33, -177.035, -28.215);
    public static final ChassisConfig swerveBotChassisConfig = new ChassisConfig(10.5 / 12, 10.5 / 12, 0.995,
        99.5 / 1000.0, 12.8, 8.14);

    // FOR 2022 Chad Bot - degrees
    public static final WheelOffsets chadBotOffsets = new WheelOffsets(-175.60, -115.40, -162.15, 158.81);
    public static final ChassisConfig chadBotChassisConfig = new ChassisConfig(MperFT * (21.516 / 12.0) / 2.0,
        MperFT * (24.87 / 12) / 2, 0.995, 99.5 / 1000.0, 12.8, 8.14);

    // For 2023 CompetitionBot - Doof
    public static final WheelOffsets doofBotOffsets = new WheelOffsets(129.03, -83.94, -57.83, 139.38);
    public static final ChassisConfig doofBotChassisConfig = new ChassisConfig(
        MperFT * (23.5 / 12.0) / 2.0, // based on CAD in reference_links
        MperFT * (19.5 / 12.0) / 2.0, // based on CAD in reference_links
        0.999, // scale [] <= 1.0
        MperFT * (4.0 / 12.0), // wheel diameter[m] Comp bot is 4" wheels
        12.8, // confirmed with vince
        8.14); // confirmed with vince

    // TODO: For 20424 CompetitionBot ***NOT YET CONFIRMED
    public static final WheelOffsets comp2024BotOffsets = new WheelOffsets(43.85746387, 24.096825 , -65.21481, -43.066333125);
    public static final ChassisConfig comp2024BotChassisConfig = new ChassisConfig(
        MperFT * (23.5 / 12.0) / 2.0,
        MperFT * (19.5 / 12.0) / 2.0,
        0.999, // scale [] <= 1.0
        MperFT * (4.0 / 12.0),
        12.8,
        8.14);

    // TODO: confirm this when start working on 2024 bot
    public static final ChassisInversionSpecs comp2024BotInversionSpecs = new ChassisInversionSpecs(
        new ModuleInversionSpecs(true, true, false), // FR
        new ModuleInversionSpecs(false, true, false), // FL
        new ModuleInversionSpecs(true, true, false), // BR
        new ModuleInversionSpecs(false, true, false)); // BL

    public static final ChassisInversionSpecs swerveBotChassisInversionSpecs = new ChassisInversionSpecs(
        new ModuleInversionSpecs(true, false, false), // FR
        new ModuleInversionSpecs(false, false, false), // FL
        new ModuleInversionSpecs(true, false, false), // BR
        new ModuleInversionSpecs(false, false, false)); // BL

    public static final ChassisInversionSpecs chadBotChassisInversionSpecs = new ChassisInversionSpecs(
        new ModuleInversionSpecs(true, false, false), // FR
        new ModuleInversionSpecs(false, false, false), // FL
        new ModuleInversionSpecs(true, false, false), // BR
        new ModuleInversionSpecs(false, false, false)); // BL

    public static final ChassisInversionSpecs doofBotChassisInversionSpecs = new ChassisInversionSpecs(
        new ModuleInversionSpecs(true, false, false), // FR
        new ModuleInversionSpecs(false, false, false), // FL
        new ModuleInversionSpecs(true, false, false), // BR
        new ModuleInversionSpecs(false, false, false)); // BL

    // Support for multiple robots on same code base

    public static final SubsystemConfig comp2024BotSubsystemConfig = new SubsystemConfig(false, false, false, true,
        false,
        false, false, true, true);
    public static final SubsystemConfig swerveBotSubsystemConfig = new SubsystemConfig(false, false, false, false,
        false,
        false, false, true, true);
    public static final SubsystemConfig chadBotSubsystemConfig = new SubsystemConfig(true, true, false, false, true,
        true,
        true, true, true);
    public static final SubsystemConfig doofBotSubsystemConfig = new SubsystemConfig(true, false, false, true, false,
        false, false, true, true);

    public static final CANModuleConfig comp2024CAN_FL = new CANModuleConfig(29,24,25);
    public static final CANModuleConfig comp2024CAN_FR = new CANModuleConfig(30,26,27);
    public static final CANModuleConfig comp2024CAN_BL = new CANModuleConfig(28,22,23);
    public static final CANModuleConfig comp2024CAN_BR = new CANModuleConfig(31,20,21);

    public static final CANModuleConfig swerveBotCAN_FL = new CANModuleConfig(7,20,21);
    public static final CANModuleConfig swerveBotCAN_FR = new CANModuleConfig(30,26,27);
    public static final CANModuleConfig swerveBotCAN_BL = new CANModuleConfig(28,22,23);
    public static final CANModuleConfig swerveBotCAN_BR = new CANModuleConfig(31,24,25);

    public static final CANConfig comp2024BotCANConfig = new CANConfig(comp2024CAN_FL, comp2024CAN_FR, comp2024CAN_BL, comp2024CAN_BR);
    public static final CANConfig swerveBotCANConfig = new CANConfig(swerveBotCAN_FL, swerveBotCAN_FR, swerveBotCAN_BL, swerveBotCAN_BR);
    public static final CANConfig chadBotCANConfig = new CANConfig(swerveBotCAN_FL, swerveBotCAN_FR, swerveBotCAN_BL, swerveBotCAN_BR);
    public static final CANConfig doofBotCANConfig = new CANConfig(swerveBotCAN_FL, swerveBotCAN_FR, swerveBotCAN_BL, swerveBotCAN_BR);

  } // end DriveTrain

  /*-------------------------Ports/CAN-------------------------------- */
  /**
   * CAN bus IDs
   * 
   * Please keep in order ID order
   * 
   */
  public static final class CAN {
    public static final int ARM_RIGHT_Motor = 1000; // placeholder
    public static final int ARM_LEFT_Motor = 100; // placeholder

    // Intake
    public static final int INTAKE_MTR = 200; // placeholder
    public static final int ANGLE_MTR = 1000; // placeholder
    // drive train CANCoders
    public static final int DT_BL_CANCODER = 28;
    public static final int DT_BR_CANCODER = 31;
    public static final int DT_FR_CANCODER = 30;
    public static final int DT_FL_CANCODER = 7;

    // drive train drive / angle motors - sparkmax neo
    public static final int DT_FL_DRIVE = 20;
    public static final int DT_FL_ANGLE = 21;
    public static final int DT_BL_DRIVE = 22;
    public static final int DT_BL_ANGLE = 23;
    public static final int DT_BR_DRIVE = 24;
    public static final int DT_BR_ANGLE = 25;
    public static final int DT_FR_DRIVE = 26;
    public static final int DT_FR_ANGLE = 27;

    // Nose Roller
    public static final int NOSE_MOTOR_ANGLE = 140;
    public static final int NOSE_MOTOR_FIRE = 150;

    // Claw
    public static final int CLAW_WHEEL_MOTOR = 16;

    // IMU
    public static final int PIGEON_IMU_CAN = 60;

    // Whether to burn flash or not
    public static final boolean BURN_FLASH = false; // swerve-mk3
  }

  public static final class AnalogIn {
    public static final int Pressure_Sensor = 0;
    // public static final int MAGAZINE_ANGLE = 0;
  }

  // pnumatics control module 1
  public final class PCM1 {
  }

  // pnumatics control module 2
  public final class PCM2 {
  }

  public final class DigitalIO {
    public static final int IntakeLightGate = 1;
  }

  public static final class Intake_Constants {
    public static double IntakeMotorDefault = 0.01; // placeholder
  }

  /*-------NT------- */
  public final static class NTStrings {
    public final static String NT_Name_Position = "Position";
  }

  // public static final class CAN{
  // /* 1/20/24
  // * CAN IDs:
  // * Corner 3 Drive Motor BL: 22
  // * Corner 3 Direction Motor BL: 23
  // * BL Encoder: 28
  // *
  // * Corner 2 Drive Motor FL: 20
  // * Corner 2 Direction Motor FL: 21
  // * FL Encoder: 29
  // *
  // * Corner 4 Drive Motor BR: 24
  // * Corner 4 Direction Motor BR: 25
  // * BR Encoder
  // *
  // * Corner 1 Direction Motor FR: 27
  // * Corner 1 Drive Motor FR: 26
  // * FR Encoder: 30
  // */

  // // CAN IDs for DT encoders
  // public static final int DT_BL_ENCODER = 28;
  // public static final int DT_FL_ENCODER = 29;
  // public static final int DT_FR_ENCODER = 30;
  // public static final int DT_BR_ENCODER = 31;

  // // CAN IDs for drivetrain motors
  // public static final int DT_FL_DRIVE = 20;
  // public static final int DT_FL_DIRECTION = 21;

  // public static final int DT_BL_DRIVE = 22;
  // public static final int DT_BL_DIRECTION = 23;

  // public static final int DT_BR_DRIVE = 24;
  // public static final int DT_BR_DIRECTION = 25;

  // public static final int DT_FR_DRIVE = 26;
  // public static final int DT_FR_DIRECTION = 27;

}