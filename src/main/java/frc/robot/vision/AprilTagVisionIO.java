// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.vision;

public interface AprilTagVisionIO {
  public static class AprilTagVisionIOInputs {
    public double[] timestamps = new double[] {};
    public double[][] frames = new double[][] {};
    public double[] demoFrame = new double[] {};
    public long fps = 0;
  }

  public default void updateInputs(AprilTagVisionIOInputs inputs) {}
}
