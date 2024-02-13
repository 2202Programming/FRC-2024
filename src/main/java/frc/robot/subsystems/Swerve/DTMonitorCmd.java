// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Constants.DriveTrain;
import frc.robot.commands.utility.WatcherCmd;

public class DTMonitorCmd extends WatcherCmd {

  /** Creates a new DTMonitorCmd. */

    private Pose2d m_pose;
    
    // Use addRequirements() here to declare subsystem dependencies.
    NetworkTableEntry currentX;
    NetworkTableEntry currentY;
    NetworkTableEntry currentHeading;

    NetworkTableEntry velocityFL;
    NetworkTableEntry velocityFR;
    NetworkTableEntry velocityBL;
    NetworkTableEntry velocityBR;

    NetworkTableEntry nt_drivePIDF;
    

  @Override
  public String getTableName() {
    return SwerveDrivetrain.class.getSimpleName();
  }

  @Override
  public void ntcreate() {
    NetworkTable table = getTable();
    nt_drivePIDF = table.getEntry("drive PIDF");
    currentX = table.getEntry("current x");
    currentY = table.getEntry("current y");
    currentHeading = table.getEntry("current heading");
    velocityFL = table.getEntry("drive FL velocity");
    velocityFR = table.getEntry("drive FR velocity");
    velocityBL = table.getEntry("drive BL velocity");
    velocityBR = table.getEntry("drive BR velocity");
  }

  @Override
  public void ntupdate() {
    //nt_drivePIDF.setValue(DriveTrain.drivePIDF);
    currentX.setDouble(m_pose.getX());
    currentY.setDouble(m_pose.getY());
    currentHeading.setDouble(m_pose.getRotation().getDegrees());
    //TODO Mr L look at this
    /*
     * velocityFL.setDouble(modules[0].getVelocity());
     * velocityFR.setDouble(modules[1].getVelocity());
     * velocityBL.setDouble(modules[2].getVelocity());
     * velocityBR.setDouble(modules[3].getVelocity());
     */

  }
  
}
