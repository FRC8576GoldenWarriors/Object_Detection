package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants.cameraRotationConstants;
import frc.robot.Constants.VisionConstants.cameraTranslationConstants;
public class ObjectStats extends SubsystemBase {
    //Creating new object for the arducam
    private PhotonCamera m_arduCam;
    
    //Object representation of the field
    private AprilTagFieldLayout m_layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    
    //Positional object representation of the camera on the robot
    private final Transform3d m_robotToCam = new Transform3d(new Translation3d(cameraTranslationConstants.tX, cameraTranslationConstants.tY, cameraTranslationConstants.tZ), new Rotation3d(cameraRotationConstants.rRoll, cameraRotationConstants.rPitch, cameraRotationConstants.rYaw));
    //Shuffleboard tab named vision, and 4 different widgets for yaw, pitch, tag id, and distance to a tag
    private ShuffleboardTab m_tab;
    private GenericEntry m_yawEntry, m_pitchEntry, m_noteEntry, m_areaEntry, m_typeEntry;
    //Variables to hold all of the widget values
    private double m_yaw, m_pitch, m_distance, m_area;
    private boolean m_note;
    private Class m_noteType;

    //Generic starting position of the robot
    // private final Pose3d m_startPose3d = new Pose3d(12.5, 5.5, 0, new Rotation3d());
    private StructPublisher<Pose2d> m_publisher;
    private PhotonTrackedTarget target;
    
    public ObjectStats(String cameraName, String publishName, String tabName) {
        m_publisher = NetworkTableInstance.getDefault().getStructTopic(publishName, Pose2d.struct).publish();
        m_tab = Shuffleboard.getTab(tabName);
        m_arduCam = new PhotonCamera(cameraName);
        m_arduCam.setPipelineIndex(1);
        m_yawEntry = m_tab.add("yaw", m_yaw).getEntry();
        m_pitchEntry = m_tab.add("pitch", m_pitch).getEntry();
        m_noteEntry = m_tab.add("Note Present", m_note).getEntry();
        m_typeEntry = m_tab.add("Note Type",m_noteType).getEntry();
        m_areaEntry = m_tab.add("area", m_area).getEntry();
    }
    @Override
  public void periodic() {
    updateView();
}

    public Transform3d getObjectTransform(){
        target = getTarget();
        if(target!=null)
            return target.getBestCameraToTarget();
        return null;
    }
    private PhotonTrackedTarget getTarget() {
        return m_arduCam.getLatestResult().getBestTarget();
    }

    public Class getNoteType(){
        target = getTarget();
        if(target!=null)
            return target.getClass();
        return null;
    }

    public boolean pingCam() {
        return m_arduCam.isConnected();
    }

    public void updateData() {
        //pushes yaw, pitch, id, and distance between the robot and tag to ShuffleBoard (Meant for testing if values are being passed to variables)
        m_yawEntry.setDouble(getYaw());
        m_pitchEntry.setDouble(getPitch());
        m_noteEntry.setBoolean(hasTarget());
        m_areaEntry.setDouble(getArea());
    }
    public double getArea(){
        target = getTarget();
        if(target != null)
            return target.getArea();
        return 0;
    }

    public List<TargetCorner> getCorners(){
        target = getTarget();
        if(target!=null)
            return target.getMinAreaRectCorners();
        return null;
        
    }
    

    public double getYaw() {
        target = getTarget();
        if (target != null) {
            return target.getYaw();
        }
        return 0;
    }

    public double getPitch() {
        target = getTarget();
        if (target != null) {
            return target.getPitch();
        }
        return 0;
    }


    public boolean hasTarget() {
        if (m_arduCam.hasTargets() && pingCam()) {
            return true;
        }
        return false;
    }

    
    public double getTimeStamp(double latency){
        return Timer.getFPGATimestamp()-(latency/1000d);
    }
    public double getLatency(){
        return m_arduCam.getLatestResult().getLatencyMillis();
    }

    public void updateView() {
        // TODO Auto-generated method stub
        m_yawEntry.setDouble(getYaw());
        m_pitchEntry.setDouble(getPitch());
        m_typeEntry.setValue(getClass());
        m_areaEntry.setDouble(getArea());
        m_noteEntry.setBoolean(hasTarget());
        throw new UnsupportedOperationException("Unimplemented method 'updateView'");
    }
}