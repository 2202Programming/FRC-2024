package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.CAN;
import frc.robot.util.NeoServo;
import frc.robot.util.PIDFController;

public class ShooterServo extends Shooter {
    //tbd for all units
    final static double ShooterAngleGearRatio = 10.0;
    final static int STALL_CURRENT = 5;
    final static int FREE_CURRENT = 15;
    final static double maxVel = 10.0; 
    final static double maxAccel = 10.0;
    final static double posTol = 1.0;
    final static double velTol = 1.0;

    final static double DeployAngle = 100.0;//tbd both
    final static double RetractAngle = 50.0;

    private final NeoServo shooterAngle;

    PIDController shooterPos = new PIDController(1.0, 0.0, 0.0);
    PIDFController hwShooterVelPID = new PIDFController(1.0, 0.0, 0.0, 0.0);
    public ShooterServo(){
        super(false);
         shooterAngle = new NeoServo(CAN.SHOOTER_ANGLE, shooterPos, hwShooterVelPID, false);
        // Servo setup for angle_servo
        hwShooterVelPID.copyTo(shooterAngle.getController().getPIDController(), 0);
    shooterAngle.setConversionFactor(360.0 / ShooterAngleGearRatio) // [deg]
        .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
        .setVelocityHW_PID(maxVel, maxAccel)
        .setTolerance(posTol, velTol)
        .setMaxVelocity(maxVel)
        .burnFlash();
        
    }

    @Override
    public void deploy(){
        setShooterAngleSetpoint(DeployAngle);
    }
    @Override
    public void retract(){
        setShooterAngleSetpoint(RetractAngle);
    }

    public void setShooterAngleSetpoint(double speed){
        shooterAngle.setSetpoint(speed);
      }
      public double getShooterAngleSetpoint(){
        return shooterAngle.getPosition();
      }
      public void setShooterAngleMaxVel(double velLimit){
        shooterAngle.setMaxVelocity(velLimit);
      }
      public double getShooterAngleSpeed(){
        return shooterAngle.getVelocity();
      }
      public boolean atShooterAngleSetpoint(){
        return shooterAngle.atSetpoint();
      }
}
