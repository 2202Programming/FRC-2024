// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.ModuleInversionSpecs;
import frc.robot.util.PIDFController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static final class DriverControls {

    public enum Id {
      Driver(0), Operator(1), SwitchBoard(2), Phantom(3);

      public final int value;

      Id(int value) {
        this.value = value;
      }
    }

    public enum DriverMode {
      Arcade(0), Tank(1), XYRot(2);

      public final int value;

      DriverMode(int value) {
        this.value = value;
      }
    }
  }
  // Handy feet to meters
  public static final double FTperM = 3.28084;
  public static final double MperFT = 1.0 / FTperM;
  public static final int NEO_COUNTS_PER_REVOLUTION = 42;
    public static final class SubsystemConfig {

    //Support for multiple robots on same code base
    public final boolean HAS_INTAKE;
    public final boolean HAS_SHOOTER;
    public final boolean HAS_ANALOG_PNEUMATICS;
    public final boolean IS_COMPETITION_BOT;
    public final boolean HAS_MAGAZINE;
    public final boolean HAS_CLIMBER;
    public final boolean HAS_POSITIONER;
    public final boolean HAS_DRIVETRAIN;
    public final boolean HAS_LIMELIGHT;
    

    public SubsystemConfig(boolean HAS_INTAKE, boolean HAS_SHOOTER, boolean HAS_ANALOG_PNEUMATICS, boolean IS_COMPETITION_BOT, boolean HAS_MAGAZINE,
        boolean HAS_CLIMBER,
        boolean HAS_POSITIONER, boolean HAS_DRIVETRAIN, boolean HAS_LIMELIGHT ) {
      this.HAS_INTAKE = HAS_INTAKE;
      this.HAS_SHOOTER = HAS_SHOOTER;
      this.HAS_ANALOG_PNEUMATICS = HAS_ANALOG_PNEUMATICS;
      this.IS_COMPETITION_BOT = IS_COMPETITION_BOT;
      this.HAS_MAGAZINE = HAS_MAGAZINE;
      this.HAS_CLIMBER = HAS_CLIMBER;
      this.HAS_POSITIONER = HAS_POSITIONER;
      this.HAS_DRIVETRAIN = HAS_DRIVETRAIN;
      this.HAS_LIMELIGHT = HAS_LIMELIGHT;
    }
  }

  /*------------------------Drivetrain-------------------------*/
  public static final class ChassisConfig {

    // Kinematics model - wheel offsets from center of robot (0, 0)
    // Left Front given below, symmetry used for others
    public final double XwheelOffset; // meters, half of X wheelbase
    public final double YwheelOffset; // meters, half of Y wheelbase

    public final double wheelCorrectionFactor; // percent
    public final double wheelDiameter; // meters
    public final double kSteeringGR; // [mo-turns to 1 angle wheel turn]
    public final double kDriveGR; // [mo-turn to 1 drive wheel turn]

    public ChassisConfig(double XwheelOffset, double YwheelOffset, double wheelCorrectionFactor, double wheelDiameter,
        double kSteeringGR,
        double kDriveGR) {
      this.XwheelOffset = XwheelOffset;
      this.YwheelOffset = YwheelOffset;
      this.wheelCorrectionFactor = wheelCorrectionFactor;
      this.wheelDiameter = wheelDiameter * wheelCorrectionFactor;
      this.kSteeringGR = kSteeringGR;
      this.kDriveGR = kDriveGR;
    }
  }

  // CANCoder offsets for absolure calibration - stored in the magnet offset of
  // the CC. [degrees]
  public static final class WheelOffsets {
    public final double CC_FL_OFFSET;
    public final double CC_BL_OFFSET;
    public final double CC_FR_OFFSET;
    public final double CC_BR_OFFSET;

    public WheelOffsets(double FL, double BL, double FR, double BR) {
      this.CC_FL_OFFSET = FL;
      this.CC_BL_OFFSET = BL;
      this.CC_FR_OFFSET = FR;
      this.CC_BR_OFFSET = BR;
    }
  }

  public static final class DriveTrain {
    // motor constraints
    public static final double motorMaxRPM = 5600; // motor limit

    // Constraints on speeds enforeced in DriveTrain
    public static final double kMaxSpeed = 21.0 * MperFT; // [m/s]
    public static final double kMaxAngularSpeed = 2 * Math.PI; // [rad/s]
    //TODO: FROM LAST YEAR, NEED TO REVIEW
    /****
     * ### REMINDER - enable these once we have basics working
     * // Other constraints
     * public static final int smartCurrentMax = 60; //amps in SparkMax, max setting
     * public static final int smartCurrentLimit = 35; //amps in SparkMax, inital
     * setting
     */

     // CAN IDs for climber
     // CAN IDs for DT encoders
     public static final int DT_BL_ENCODER = 28;
     public static final int DT_FL_ENCODER = 29;
     public static final int DT_FR_ENCODER = 30;
     public static final int DT_BR_ENCODER = 31;
    // SmartMax PID values [kp, ki, kd, kff] - these get sent to hardware controller
    // DEBUG - SET FF first for drive, then add KP

    // public static final PIDFController drivePIDF = new
    // PIDFController(0.09*MperFT, 0.0, 0.0, 0.08076*MperFT);
    public static final PIDFController drivePIDF = new PIDFController(0.09 * FTperM, 0.0, 0.0, 0.08076 * FTperM);
    public static final PIDFController anglePIDF = new PIDFController(0.01, 0.0, 0.0, 0.0); // maybe 1.0,0.0,0.1 from
                                                                                            // SDS sample code?

    // FOR SWERVEBOT, aka Tim 2.0
    public static final WheelOffsets swerveBotOffsets = new WheelOffsets(-98.942, 91.33, -177.035, -28.215);
    public static final ChassisConfig swerveBotChassisConfig = new ChassisConfig(10.5 / 12, 10.5 / 12, 0.995,
        99.5 / 1000.0, 12.8, 8.14);

    // FOR 2022 Chad Bot - degrees
    public static final WheelOffsets chadBotOffsets = new WheelOffsets(-175.60, -115.40, -162.15, 158.81);
    public static final ChassisConfig chadBotChassisConfig = new ChassisConfig(MperFT * (21.516 / 12.0) / 2.0,
        MperFT * (24.87 / 12) / 2, 0.995, 99.5 / 1000.0, 12.8, 8.14);

    // For 2023 CompetitionBot
    public static final WheelOffsets compBotOffsets = new WheelOffsets(129.03, -83.94, -57.83, 139.38);
    public static final ChassisConfig compBotChassisConfig = new ChassisConfig(
        MperFT * (23.5 / 12.0) / 2.0, //based on CAD in reference_links
        MperFT * (19.5 / 12.0) / 2.0, //based on CAD in reference_links
        0.999, // scale [] <= 1.0
        MperFT * (4.0/12.0), // wheel diameter[m] Comp bot is 4" wheels
        12.8, //confirmed with vince
        8.14); //confirmed with vince

  }
  //Support for multiple robots on same code base
  public static final ChassisInversionSpecs swerveBotChassisInversionSpecs = new ChassisInversionSpecs(
    new ModuleInversionSpecs(true,false,false), //FR
    new ModuleInversionSpecs(false,false,false), //FL
    new ModuleInversionSpecs(true,false,false), //BR
    new ModuleInversionSpecs(false,false,false)); //BL

  public static final ChassisInversionSpecs chadBotChassisInversionSpecs = new ChassisInversionSpecs(
    new ModuleInversionSpecs(true,false,false), //FR
    new ModuleInversionSpecs(false,false,false), //FL
    new ModuleInversionSpecs(true,false,false), //BR
    new ModuleInversionSpecs(false,false,false)); //BL
  
  public static final ChassisInversionSpecs compBotChassisInversionSpecs = new ChassisInversionSpecs(
    new ModuleInversionSpecs(true,false,false), //FR
    new ModuleInversionSpecs(false,false,false), //FL
    new ModuleInversionSpecs(true,false,false), //BR
    new ModuleInversionSpecs(false,false,false)); //BL
  
  public static final SubsystemConfig comp2024BotSubsystemConfig = new SubsystemConfig(true,true,true, true, false, true, false, true, true);
  public static final SubsystemConfig swerveBotSubsystemConfig = new SubsystemConfig(false,false, false, false, false, false,      false, true, true);
  public static final SubsystemConfig chadBotSubsystemConfig = new SubsystemConfig(true,true,false, false, true, true, true, true, true);
  public static final SubsystemConfig comp2023BotSubsystemConfig = new SubsystemConfig(true,false,false,true,false,false,false,true,true);
    //Support for multiple robots on same code base
  public static final class ChassisInversionSpecs{
    public ModuleInversionSpecs FR;
    public ModuleInversionSpecs FL;
    public ModuleInversionSpecs BR;
    public ModuleInversionSpecs BL;
  
    public ChassisInversionSpecs(ModuleInversionSpecs FR, ModuleInversionSpecs FL, ModuleInversionSpecs BR, ModuleInversionSpecs BL){
        this.FR = FR;
        this.FL = FL;
        this.BR = BR;
        this.BL = BL;
    }
  }




  

  /*-------------------------Ports/CAN-------------------------------- */
  /**
   * CAN bus IDs
   * 
   * Please keep in order ID order
   * 
   */
  public static final class CAN {
    public static final int ARM_RIGHT_Motor = 1000; //placeholder
     public static final int ARM_LEFT_Motor = 100; //placeholder 
    
     //Intake
     public static final int INTAKE_MTR = 200; //placeholder
     public static final int ANGLE_MTR = 1000; //placeholder
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
  
    
    // IMU
    public static final int PIGEON_IMU_CAN = 60;

    // Whether to burn flash or not
    public static final boolean BURN_FLASH = false; // swerve-mk3
    
  }
  

  public static final class AnalogIn {
    public static final int Pressure_Sensor = 0;
    // public static final int MAGAZINE_ANGLE = 0;
  }







  /*-------NT------- */
  public final static class NTStrings {
    public final static String NT_Name_Position = "Position";
  }
  public static final double DT = 0.02; // 20ms framerate 50Hz
  public static final double Tperiod = 0.02; // framerate period 20ms, 50Hz
  
  // public static final class CAN{
  //    /* 1/20/24
  //    * CAN IDs:
  //    * Corner 3 Drive Motor BL: 22
  //    * Corner 3 Direction Motor BL: 23
  //    * BL Encoder: 28
  //    * 
  //    * Corner 2 Drive Motor FL: 20
  //    * Corner 2 Direction Motor FL: 21
  //    * FL Encoder: 29
  //    * 
  //    * Corner 4 Drive Motor BR: 24
  //    * Corner 4 Direction Motor BR: 25
  //    * BR Encoder
  //    * 
  //    * Corner 1 Direction Motor FR: 27
  //    * Corner 1 Drive Motor FR: 26
  //    * FR Encoder: 30
  //    */

  //    // CAN IDs for DT encoders
  //    public static final int DT_BL_ENCODER = 28;
  //    public static final int DT_FL_ENCODER = 29;
  //    public static final int DT_FR_ENCODER = 30;
  //    public static final int DT_BR_ENCODER = 31;
     
  //    // CAN IDs for drivetrain motors
  //    public static final int DT_FL_DRIVE = 20;
  //    public static final int DT_FL_DIRECTION = 21;

  //    public static final int DT_BL_DRIVE = 22;
  //    public static final int DT_BL_DIRECTION = 23;
     
  //    public static final int DT_BR_DRIVE = 24;
  //    public static final int DT_BR_DIRECTION = 25;
     
  //    public static final int DT_FR_DRIVE = 26;
  //    public static final int DT_FR_DIRECTION = 27;

  
  public static class PowerOnPos{
    public static final double arm = 0.0;
  }
  
  
  public final class PCM1{
    public static final int RT_INTAKE_UP_SOLENOID_PCM = 400; 
    public static final int RT_INTAKE_DOWN_SOLENOID_PCM = 500; 
    public static final int LT_INTAKE_UP_SOLENOID_PCM = 600; 
    public static final int LT_INTAKE_DOWN_SOLENOID_PCM = 700; 
}
public final class DigitalIO{
  public static final int IntakeLightGate= 800;
}
public static final class Intake_Constants{
  public static double IntakeMotorDefault = 0.01; //placeholder
}
}

