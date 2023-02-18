package frc.robot.vision;

import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Vector;
import java.util.regex.*;

import javax.swing.text.html.HTMLDocument.Iterator;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.opencv.core.Range;
import org.opencv.photo.Photo;
import org.photonvision.*;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
//import frc.robot.Logger;
import frc.robot.Enums.*;

// PhotonVision Class
//  Yaw is positive to the right. If an object is offset to your right, its a positive yaw
public class photonVision {
    public static photonVision mInstance = null;

    
    // TODO - Likely two more camera to come to be a 3 camera system
    //      - this should be in an array with a corresponding transform3d, enums to index
    
    
    public static final PhotonCamera frontCamera = new PhotonCamera("frontApriltagCam");
    public static final PhotonCamera backCamera = new PhotonCamera("backApriltagCam");
    //public static final PhotonCamera grasperCamera = new PhotonCamera("grasperCam");
    // Transform2d (and all of the geometry classes in WPILIB) use meters
    // All these inches measurements are from the center of the robot
    // The more accurate these numbers are the better the vision calculation will be
    // Z Plane does not matter a ton
    // X: 0 Represents Robots Center. +X is closer to robot's front bumper. -X is closer to robots back bumper
    // Y: 0 Represents Robot Center. +Y is closer to robot's left bumper. -Y is closer to robots right bumper
    // Z: 0 Represents Robot Center. +Z is above robot center middle point. -Z is below centered middle pointer
    // Yaw: Which side the camera faces on the robot. 0 is straight on. 180 is facing the back. Geometry requires radians
    public static final Transform3d robotToFrontCam = 
                    new Transform3d(new Translation3d(0, Units.inchesToMeters(5), 0), //Camera is slightly to the left
                    new Rotation3d(0,0,0)); 
    public static final Transform3d robotToBackCam = 
                    new Transform3d(new Translation3d(0, 0.0, 0), 
                    new Rotation3d(0,0,Units.degreesToRadians(180))); //Camera is mirrored on the backside, so yaw will be flipped 180
   /*
                    public static final Transform3d robotToGrasperCam = 
                    new Transform3d(new Translation3d(0, 0.0, 0), 
                    new Rotation3d(0.5,0,0.5)); 
*/
    // TODO - Add Additional Cameras
    public static final List<Pair<PhotonCamera, Transform3d>> CameraList = List.of(
      new Pair<PhotonCamera, Transform3d>(frontCamera, robotToFrontCam),
      new Pair<PhotonCamera, Transform3d>(backCamera, robotToBackCam)
      /*
      ,
      new Pair<PhotonCamera, Transform3d>(grasperCamera, robotToGrasperCam)
      */
    );

    
     // Simulated Test Code TEMP TEMP TEMP
        // Simulated Vision System.
    // Configure these to match your PhotonVision Camera,
    // pipeline, and LED setup.
    double camDiagFOV = 75.0; // degrees
    double camPitch = 0; // degrees
    double camHeightOffGround = 0.85; // meters
    double maxLEDRange = 20; // meters
    int camResolutionWidth = 640; // pixels
    int camResolutionHeight = 480; // pixels
    double minTargetArea = 10; // square pixels

    // EntropyVisionSim, which extends PhotonVisionSim
    public EntropyVisionSim simFrontCamera =
            new EntropyVisionSim(
                    "frontApriltagCam",
                    camDiagFOV,
                    robotToFrontCam,
                    maxLEDRange,
                    camResolutionWidth,
                    camResolutionHeight,
                    minTargetArea);

    public EntropyVisionSim simBackCamera =
                    new EntropyVisionSim(
                            "backApriltagCam",
                            camDiagFOV,
                            robotToBackCam,
                            maxLEDRange,
                            camResolutionWidth,
                            camResolutionHeight,
                            minTargetArea);

    public boolean hasTarget = false;
    
    //Transform2d pose = target.getCameraToTarget();
    //List<TargetCorner> corners = target.getCorners();
  
    public static synchronized photonVision getInstance() {
      if (mInstance == null) {
        mInstance = new photonVision();
      }
      return mInstance;
    }
  
    public synchronized PhotonPipelineResult getPipeLine(cameraType cam) 
    {
      PhotonCamera camera = CameraList.get(cam.ordinal()).getFirst();
      PhotonPipelineResult result = camera.getLatestResult();
      return result;
    }
  
    public synchronized List<String> getTargetIds() {
      List<String> targetIDs = new Vector<>();
  
  
      return targetIDs;
    }

    // ProcessSimFrame
    // Updates Sim Camera Objects with current robot pose
    public synchronized void processSimFrame(Pose2d framePose)
    {
      simFrontCamera.processFrame(framePose);
      simBackCamera.processFrame(framePose);
    }

    // AddSimVisiontargets
    // Add AprilTag Locations 
    public synchronized void addSimVisionTargets(AprilTagFieldLayout tagLayout)
    {
      simFrontCamera.addVisionTargets(tagLayout);
      simBackCamera.addVisionTargets(tagLayout);
    }
    
    public synchronized String getTargetID() {
      //System.out.println("calling getTargetID");
      String myIDString = "0";
      int myIDInt = 0;
      List<PhotonTrackedTarget> myItems = getTargetList();
      String stringTargetList = myItems.toString();
      Pattern pattern = Pattern.compile("(?<=fiducialId=)(.*)(?=, cam)", Pattern.CASE_INSENSITIVE);
      Matcher matcher = pattern.matcher(stringTargetList);
      while (matcher.find()){
        myIDString =  matcher.group(1);
        //System.out.println(myIDString);
      }
      /*
      try{
        System.out.println("before string to int");
        System.out.println(myIDString);
        myIDInt= Integer.valueOf(myIDString);
        System.out.println("after string to int");
      }finally{
        System.out.println("connot convert string to int");
      }
      */
      
      return myIDString;
    }
  
    public synchronized boolean seesTargets(cameraType cam){
      PhotonCamera camera = CameraList.get(cam.ordinal()).getFirst();
      var result = camera.getLatestResult();
      boolean haveATarget = result.hasTargets();
      
      return haveATarget;
    }
  
    public synchronized double getBestTargetYaw(cameraType cam){
      PhotonCamera camera = CameraList.get(cam.ordinal()).getFirst();
  
      double targetYaw = Double.NaN;
      PhotonTrackedTarget myTarget = null;
  
  
      var result = camera.getLatestResult();
      boolean seesTargets = seesTargets(cameraType.FRONT_CAMERA);
      //System.out.println("sees targets: " + seesTargets);
      
      if(seesTargets){
        
        myTarget = result.getBestTarget();
        targetYaw = myTarget.getYaw();
      }
      else{
        targetYaw = 0;
        
      }
        
      return targetYaw;
    }
  
    //function that will return the yaw of a target, it will only track the target with the ID given
    public synchronized Double getTargetYaw(int id){
      List<PhotonTrackedTarget> targets = getTargetList();
      java.util.Iterator<PhotonTrackedTarget> it = targets.iterator();
      while(it.hasNext()){
        PhotonTrackedTarget activeTarget = it.next();
        if(activeTarget.getFiducialId() == id){
          return activeTarget.getYaw();
        }  
      }
      return 0.0;
    }
    
    public synchronized List<PhotonTrackedTarget> getTargetList() {
      try{
        //System.out.println("calling getTargetList");
        PhotonPipelineResult pipeLine = getPipeLine(cameraType.FRONT_CAMERA);
        List<PhotonTrackedTarget> targets = pipeLine.getTargets();
  
        java.util.Iterator<PhotonTrackedTarget> it = targets.iterator();
        /*
        while(it.hasNext()){
          PhotonTrackedTarget activeTarget = it.next();
          int id = activeTarget.getFiducialId();
          //if(id == 13){
              System.out.println(id);
            try{
              double yaw = activeTarget.getYaw();
              System.out.println(yaw);
            }catch(Exception e){
              System.out.println("yaw broke for" + id);
            }
          //}
          
          
          
          //org.photonvision.targeting.PhotonTrackedTarget
          //System.out.println("item: " + it.next());
          //PhotonTrackedTarget thistarget = it.next();
      
        }*/
  
        //System.out.println(targets);
        return targets;
      }finally{}
    }
  
    
  
  
    public synchronized  PhotonTrackedTarget bestTarget(){
      //options for deciding what target is best (for later)
      /*
      Largest, Smallest, Highest (towards the top of the image) , Lowest , Rightmost (Best target on the right, worst on left), Leftmost, Centermost
      */
      //System.out.println("calling bestTarget");
      PhotonPipelineResult pipeLine = getPipeLine(cameraType.FRONT_CAMERA);
      return pipeLine.getBestTarget();
    }
  
    public synchronized double targetDist(cameraType cam){
      PhotonCamera camera = CameraList.get(cam.ordinal()).getFirst();
      
      try{
        //System.out.println("calling targetDist");
        //TODO input camera values
        double CAMERA_HEIGHT_METERS = 1;
        double TARGET_HEIGHT_METERS = 0.5;
        double CAMERA_PITCH_RADIANS = 1;
        double range = 0;
        var result = camera.getLatestResult();
        //System.out.println(result.getBestTarget().getYaw());
        /*
        System.out.println("result:" + result);
        PhotonTrackedTarget myBestTarget = result.getBestTarget();
        System.out.println("Yaw: " + myBestTarget.getYaw());
        */
  
        try{
          if (result.hasTargets()) {
  
            // First calculate range
            range =
              PhotonUtils.calculateDistanceToTargetMeters(
                      CAMERA_HEIGHT_METERS,
                      TARGET_HEIGHT_METERS,
                      CAMERA_PITCH_RADIANS,
                      Units.degreesToRadians(result.getBestTarget().getPitch()));
                      
                      //System.out.println("Range:"+range);
                      return range;}
          
          else{
            //System.out.println("returning 0.0");
            return 0.0;
          }
          
  
        }finally{}
      }finally{
  
      }
    }
  
  /*
    @Override
    public void zeroSensors() {
      // TODO Auto-generated method stub
      
    }
    @Override
    public void checkSubsystem() {
      // TODO Auto-generated method stub
      
    }
    */  

  public void updateSmartDashboard()
    {
      final String key = "Vision/";
      //SmartDashboard.putNumber(key + "Best Target Yaw", getBestTargetYaw(cameraType.FRONT_CAMERA));

      // Cameras
      final String cameraKey = key + "Cameras/";
      for(int i = 0; i < CameraList.size(); ++i)
      {
        PhotonCamera currentCamera = CameraList.get(i).getFirst();
        Transform3d currentTransform = CameraList.get(i).getSecond();
        final String currentKey = cameraKey + currentCamera.getName() + "/";
        SmartDashboard.putString(currentKey + "Transform3d", currentTransform.toString());
        SmartDashboard.putBoolean(currentKey + "Connected", currentCamera.isConnected());

        // Targets this camera can currently see
        final String targetKey = currentKey + "Targets/";
        PhotonPipelineResult pipelineResult = currentCamera.getLatestResult();
        var targets = pipelineResult.targets;
        for(int j = 0; j < targets.size(); ++j)
        {
          PhotonTrackedTarget target = targets.get(j);
          final String currentTargetKey = targetKey + "Target " + target.getFiducialId() + "/";
          SmartDashboard.putNumber(currentTargetKey + "Yaw", target.getYaw());
          SmartDashboard.putNumber(currentTargetKey + "Area", target.getArea());
          SmartDashboard.putNumber(currentTargetKey + "Pitch", target.getPitch());
          SmartDashboard.putNumber(currentTargetKey + "Skew", target.getSkew());
  
          // Ratio of Pose Projection Errors
          SmartDashboard.putNumber(currentTargetKey + "Ambiguity", target.getPoseAmbiguity());
        }

        // Target Count
        SmartDashboard.putNumber(currentKey + "Target Count", targets.size());
      }
    }
}
