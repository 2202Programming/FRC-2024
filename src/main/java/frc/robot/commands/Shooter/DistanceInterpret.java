// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/** Add your docs here. */
public class DistanceInterpret {

    private InterpolatingTreeMap<Double, Double> table;

    public DistanceInterpret() {
    InverseInterpolator<Double> distance = InverseInterpolator.forDouble(); 
    Interpolator<Double> angle = Interpolator.forDouble();
    
    table = new InterpolatingTreeMap<>(distance, angle);        
    
    //TODO Set table values key=distance value = RPM(distance)
    //set up table with measured values
    //3500 Constant RPM, limelight distance, calculating angle
    table.put(353.0, 36.0); //3000 rpm  
    table.put(358.0, 36.0);
    table.put(425.0, 32.0);
    table.put(495.0, 29.0); //4000 rpm
    table.put(544.0, 28.6); //4500 rpm

    //test 
    assert 480.0 == table.get(150.0) ;
    //var x = table.get(225.0);

    }

    //return interpolated angle from a distance
    public double getAngleFromDistance(double distance){
        return table.get(distance);
    }
}
