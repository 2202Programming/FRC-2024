// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static final double DT = 0.02; // 20ms framerate 50Hz

  public static final class CAN{
     /* 1/20/24
     * CAN IDs:
     * Corner 3 Drive Motor BL: 22
     * Corner 3 Direction Motor BL: 23
     * BL Encoder: 28
     * 
     * Corner 2 Drive Motor FL: 20
     * Corner 2 Direction Motor FL: 21
     * FL Encoder: 29
     * 
     * Corner 4 Drive Motor BR: 24
     * Corner 4 Direction Motor BR: 25
     * BR Encoder
     * 
     * Corner 1 Direction Motor FR: 27
     * Corner 1 Drive Motor FR: 26
     * FR Encoder: 30
     */

     // CAN IDs for DT encoders
     public static final int DT_BL_ENCODER = 28;
     public static final int DT_FL_ENCODER = 29;
     public static final int DT_FR_ENCODER = 30;
     public static final int DT_BR_ENCODER = 31;
     
     // CAN IDs for drivetrain motors
     public static final int DT_FL_DRIVE = 20;
     public static final int DT_FL_DIRECTION = 21;

     public static final int DT_BL_DRIVE = 22;
     public static final int DT_BL_DIRECTION = 23;
     
     public static final int DT_BR_DRIVE = 24;
     public static final int DT_BR_DIRECTION = 25;
     
     public static final int DT_FR_DRIVE = 26;
     public static final int DT_FR_DIRECTION = 27;

  }
}
