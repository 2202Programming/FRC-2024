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
    private final Sensors_Subsystem sensors;
    private final SwerveDrivetrain drivetrain;
    double time;
    boolean is_accel_neg;
    double accel;
    double velocity1;
    double velocity2;


    public AccelWatcher(Sensors_Subsystem sensors, SwerveDrivetrain drivetrain) {
        this.sensors = sensors;
        this.drivetrain = drivetrain;
        time = constants.DT;
    }

    @Override
    public void initialize() {
        velocity1 = sensors.getVelocity(); //runs when called? atleast I think
        is_accel_neg = false;
    }
    
    @Override
    public void execute() {
        //TODO Convert Velocity to accel ---> done someone check pls (AG)
        velocity2 = sensors.getVelocity();// setting 2nd velocity later so that it can run
        accel = (velocity2 - velocity1)/time;
        if(accel < -0.5){
            is_accel_neg = true;
        }   
        velocity1 = sensors.getVelocity(); //setting 1st after everyting has run a bit to get a more accurate reading of accel;

    }
    
    @Override
    public void end(boolean interrupted) {
        //TODO; wat you do?
        if (interrupted) {
            velocity1 = 0;
            velocity2 = 0;
        }
    }

    @Override
    public boolean isFinished() {
        if(is_accel_neg){ 
            return true;
        }
        else{
            return false; 
        }
    }
}



