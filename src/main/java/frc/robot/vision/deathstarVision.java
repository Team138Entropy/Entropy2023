package frc.robot.vision;

import java.util.List;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;

public class deathstarVision {

  private static final int cameraId = -1;
  private static final int cameraResolutionWidth = 1280;
  private static final int cameraResolutionHeight = 720;
  private static final int cameraAutoExposure = 1;
  private static final int cameraExposure = 10;
  private static final int cameraGain = 25;

  private final DoubleArraySubscriber observationSubscriber;
  private final IntegerSubscriber fpsSubscriber;

  //private static final double disconnectedTimeout = 0.5;
  //"Alert" class is part of 6328s util library, I don't think it will be needed here tho
  //private final Alert disconnectedAlert;
  //private final Timer disconnectedTimer = new Timer();

  
  public static final double fieldLength = Units.inchesToMeters(651.25);
  public static final double fieldWidth = Units.inchesToMeters(315.5) + (Units.inchesToMeters(3.0));
  public static final double doubleSubstationCenterY = fieldWidth - Units.inchesToMeters(49.76);
  public static final double[] nodeY = new double[] {
              Units.inchesToMeters(20.19 + 22.0 * 0),
              Units.inchesToMeters(20.19 + 22.0 * 1),
              Units.inchesToMeters(20.19 + 22.0 * 2),
              Units.inchesToMeters(20.19 + 22.0 * 3),
              Units.inchesToMeters(20.19 + 22.0 * 4),
              Units.inchesToMeters(20.19 + 22.0 * 5),
              Units.inchesToMeters(20.19 + 22.0 * 6),
              Units.inchesToMeters(20.19 + 22.0 * 7),
              Units.inchesToMeters(20.19 + 22.0 * 8 + 2.5)
    };
  

    public deathstarVision(String identifier){
        System.out.println("[Init] Creating deathstarVision (" + identifier + ")");
        var northstarTable = NetworkTableInstance.getDefault().getTable(identifier);
        System.out.println("check 1");
        var configTable = northstarTable.getSubTable("config");
        configTable.getIntegerTopic("camera_id").publish().set(cameraId);
        configTable.getIntegerTopic("camera_resolution_width").publish().set(cameraResolutionWidth);
        configTable.getIntegerTopic("camera_resolution_height").publish().set(cameraResolutionHeight);
        configTable.getIntegerTopic("camera_auto_exposure").publish().set(cameraAutoExposure);
        configTable.getIntegerTopic("camera_exposure").publish().set(cameraExposure);
        configTable.getIntegerTopic("camera_gain").publish().set(cameraGain);
        configTable.getDoubleTopic("fiducial_size_m").publish().set(Units.inchesToMeters(6.0));
        System.out.println("check 2");
        try {
            configTable
                .getStringTopic("tag_layout")
                .publish()
                .set(new ObjectMapper().writeValueAsString(aprilTags));
                System.out.println("check 3: try");
          } catch (JsonProcessingException e) {
            System.out.println("check 3: catch");
            throw new RuntimeException("Failed to serialize AprilTag layout JSON for Northstar");
          }
          var outputTable = northstarTable.getSubTable("output");
          System.out.println("check 4");
          observationSubscriber =
              outputTable
                  .getDoubleArrayTopic("observations")
                  .subscribe(
                      new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
          fpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0);
          System.out.println("check 5: fin");
      
          //String disconnectedAlert = "No data from \"" + identifier + "\"";
          //disconnectedTimer.start();
    }

    //real vision stuff: not needed for dummy robot code 
    /* 
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        var queue = observationSubscriber.readQueue();
        inputs.timestamps = new double[queue.length];
        inputs.frames = new double[queue.length][];
        for (int i = 0; i < queue.length; i++) {
          inputs.timestamps[i] = queue[i].timestamp / 1000000.0;
          inputs.frames[i] = queue[i].value;
        }
        inputs.fps = fpsSubscriber.get();
    
        // Update disconnected alert
        if (queue.length > 0) {
          disconnectedTimer.reset();
        }
        //disconnectedAlert.set(disconnectedTimer.hasElapsed(disconnectedTimeout));
      }
      */

    public static final AprilTagFieldLayout aprilTags =
       
          new AprilTagFieldLayout(
              List.of(
                  new AprilTag(
                      1,
                      new Pose3d(
                          Units.inchesToMeters(610.125),
                          Units.inchesToMeters(43.5),
                          Units.inchesToMeters(19.25),
                          new Rotation3d(0.0, 0.0, Math.PI))),
                  new AprilTag(
                      2,
                      new Pose3d(
                          Units.inchesToMeters(610.375),
                          Units.inchesToMeters(109.5),
                          Units.inchesToMeters(19.25),
                          new Rotation3d(0.0, 0.0, Math.PI))),
                  new AprilTag(
                      3,
                      new Pose3d(
                          Units.inchesToMeters(610.0),
                          Units.inchesToMeters(176.0),
                          Units.inchesToMeters(19.25),
                          new Rotation3d(0.0, 0.0, Math.PI))),
                  new AprilTag(
                      4,
                      new Pose3d(
                          Units.inchesToMeters(635.375),
                          Units.inchesToMeters(272.0),
                          Units.inchesToMeters(27.25),
                          new Rotation3d(0.0, 0.0, Math.PI))),
                  new AprilTag(
                      5,
                      new Pose3d(
                          Units.inchesToMeters(14.25),
                          doubleSubstationCenterY,
                          Units.inchesToMeters(27.38),
                          new Rotation3d())),
                  new AprilTag(
                      6,
                      new Pose3d(
                          Units.inchesToMeters(40.45),
                          nodeY[7],
                          Units.inchesToMeters(18.22),
                          new Rotation3d())),
                  new AprilTag(
                      7,
                      new Pose3d(
                          Units.inchesToMeters(40.45),
                          nodeY[4],
                          Units.inchesToMeters(18.22),
                          new Rotation3d())),
                  new AprilTag(
                      8,
                      new Pose3d(
                          Units.inchesToMeters(40.45),
                          nodeY[1],
                          Units.inchesToMeters(18.22),
                          new Rotation3d()))),
              fieldLength,
              fieldWidth);
}
