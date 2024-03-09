// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/** Add your docs here. */
public class DistanceInterpretor {

    private InterpolatingTreeMap<Double, Double> table;

    public DistanceInterpretor() {
    InverseInterpolator<Double> distance = InverseInterpolator.forDouble(); 
    Interpolator<Double> angle = Interpolator.forDouble();
    
    table = new InterpolatingTreeMap<>(distance, angle);        
    
    //set up table with measured values
    table.put(100.0, 450.0);
    table.put(200.0, 510.0);
    table.put(268.0, 525.0);
    table.put(312.0, 550.0);
    table.put(326.0, 650.0);

    //test 
    assert 480.0 == table.get(150.0) ;
    //var x = table.get(225.0);

    }

    //return interpolated angle from a distance
    public double getAngleFromDistance(double distance){
        return table.get(distance);
    }
}
