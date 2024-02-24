// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AccelWatcher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.pheonix.sensors.PigeonIMU;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;


//TODO CMD to watch the accel of the robot (watch the drop in the accel)
public class AccelWatcher extends Command{
    final Sensors_Subsystem sensors;
    final SwerveDrivetrain drivetrain;
    final double d_time = constants.DT;
    final double targ_accel;
    boolean exceeded_target;
    double accel;
    boolean use_neg_case;
    double prev_velocity;//[m/s]
    double velocity;//[m/s]

/*
 * 
 * 
 * 
 * 
 */
    public AccelWatcher(double targ_accel,boolean use_absolute) {
        this.sensors = RobotContainer.getSubsystem(Sensors_Subsystem.class);
        this.drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
        this.targ_accel = targ_accel;
        this.use_absolute = use_absolute;
        if(use_absolute){
            targ_accel = Math.abs(targ_accel);
        }
    }
    public AccelWatcher(double targ_accel) {
        this(targ_accel,true);
    }
    @Override
    public void initialize() {
        velocity = prev_velocity = sensors.getVelocity(); //runs when called? atleast I think
        exceeded_target = false;
    }
    
    @Override
    public void execute() {
        //TODO Convert Velocity to accel
        velocity = sensors.getVelocity();// setting 2nd velocity later so that it can run
        accel = (velocity - prev_velocity)/d_time;
        accel = (use_absolute)? Math.abs(accel) : accel;
        if (targ_accel < 0.0) {
           exceeded_target = (accel < targ_accel);
        }else {
           exceeded_target = (accel > targ_accel);
        }
        
        prev_velocity = velocity; //setting 1st after everyting has run a bit to get a more accurate reading of accel;

    }   

    @Override
    public boolean isFinished() {
        return exceeded_target;
    }
}



