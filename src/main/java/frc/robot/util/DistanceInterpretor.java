// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

/** Add your docs here. */
public class DistanceInterpretor {
    public DistanceInterpretor(){
    InterpolatingTreeMap<Double, Double> table = new InterpolatingTreeMap<>();

    table.put(125.0, 450.0);
    table.put(200.0, 510.0);
    table.put(268.0, 525.0);
    table.put(312.0, 550.0);
    table.put(326.0, 650.0);
    }
}
