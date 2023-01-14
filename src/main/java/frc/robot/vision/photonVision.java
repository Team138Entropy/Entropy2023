package frc.robot.vision;

import java.util.List;
import java.util.Vector;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.SimVisionSystem;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Enums.pipelineIDs;

public class photonVision {
    public static photonVision mInstance = null;

    

    // TODO - Likely two more camera to come to be a 3 camera system
    //      - this should be in an array with a corresponding transform3d, enums to index
    public static final PhotonCamera camera = new PhotonCamera("camera138");
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d robotToCam = 
                    new Transform3d(new Translation3d(0.5, 0.0, 0.5), 
                    new Rotation3d(0,0,0)); 

    // TODO - Add Additional Cameras
    public static final List<Pair<PhotonCamera, Transform3d>> CameraList = List.of(
      new Pair<PhotonCamera, Transform3d>(camera, robotToCam)
    );


    // Simulated Test Code TEMP TEMP TEMP
        // Simulated Vision System.
    // Configure these to match your PhotonVision Camera,
    // pipeline, and LED setup.
    double camDiagFOV = 75.0; // degrees
    double camPitch = 15.0; // degrees
    double camHeightOffGround = 0.85; // meters
    double maxLEDRange = 20; // meters
    int camResolutionWidth = 640; // pixels
    int camResolutionHeight = 480; // pixels
    double minTargetArea = 10; // square pixels

    public SimVisionSystem simVision =
            new SimVisionSystem(
                    "camera138",
                    camDiagFOV,
                    robotToCam,
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
  
    public synchronized PhotonPipelineResult getPipeLine() {
      PhotonPipelineResult result = null;
      try{
        result = camera.getLatestResult();
      }finally{
  
      }
      return result;
    }
  
    public synchronized List<String> getTargetIds() {
      List<String> targetIDs = new Vector<>();
  
  
      return targetIDs;

    }

    public synchronized void setPipeLine(int pipeLineNum) {
      camera.setPipelineIndex(pipeLineNum);
    }

    public synchronized int getPipeLineNum() {
      return camera.getPipelineIndex();
    }

    public synchronized String getPipeLineName() {
      if(camera.getPipelineIndex() == pipelineIDs.APRILTAG_PIPELINE.pipelineID){
        return "apriltagPipeline";
      }else if(camera.getPipelineIndex() == pipelineIDs.CONE_PIPELINE.pipelineID){
        return "conePipeline";
      }else if(camera.getPipelineIndex() == pipelineIDs.CUBE_PIPELINE.pipelineID){
        return "cubePipeline";
      }
      return "no pipeline";
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
      
      return myIDString;
    }
    /*
    private synchronized double getBestTarget2DPose(){
      if(seesTargets()){
        PhotonTrackedTarget target = bestTarget();
        return target.();
        
      }
      return 0;
    }
    */
  
    public synchronized boolean seesTargets(){
      var result = camera.getLatestResult();
      boolean haveATarget = result.hasTargets();
      
      return haveATarget;
    }
  
    public synchronized double getBestTargetYaw(){
  
      double targetYaw = Double.NaN;
      PhotonTrackedTarget myTarget = null;
  
  
      var result = camera.getLatestResult();
      boolean seesTargets = seesTargets();
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
        PhotonPipelineResult pipeLine = getPipeLine();
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
      PhotonPipelineResult pipeLine = getPipeLine();
      return pipeLine.getBestTarget();
    }
  
    public synchronized double targetDist(){
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
}
