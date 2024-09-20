package org.firstinspires.ftc.teamcode;
import com.roboticslib.motion.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.RobotValues.*;


public class MainBot extends MecanumChassis{
    
    public AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    
    IMU imu;
    static double headingOffset = 0;
    
    
    int lfTicksPrev = 0;
    int rfTicksPrev = 0;
    int lbTicksPrev = 0;
    int rbTicksPrev = 0;
    double headingPrev = 0;
    
    
    public MainBot(HardwareMap hm){
            super(hm);
            initAprilTag(hm);
            initIMU(hm);
    }
    
    
    public void setPositionWithCamera(AprilTagDetection detection){
        double range = detection.ftcPose.range;
        double bearing = detection.ftcPose.bearing;
        double yaw = detection.ftcPose.yaw;
        double tagx = detection.metadata.fieldPosition.get(0);
        double tagy = detection.metadata.fieldPosition.get(1);
        double theta = Math.toRadians(getHeading() + bearing);
        double fx = tagx - Math.cos(theta) * range;
        double fy = tagy - Math.sin(theta) * range;
    }
    
    public void initIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
            new RevHubOrientationOnRobot(LOGO_DIR, USB_DIR));
        imu.initialize(params);    
    }
    public void initAprilTag(HardwareMap hardwareMap){
        
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
            hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        
    }
    
    
    
    public double getIMUHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    
    public void setHeading(double h) {
        imu.resetYaw();
        headingOffset = h - getIMUHeading();
    }
    
    public double getHeading() {
        return -AngleUnit.normalizeDegrees(headingOffset + getIMUHeading());
    }
    
    public boolean setManualExposure(LinearOpMode op,
                                     int exposureMS,
                                     int gain) {
        if (visionPortal == null) { return false; }
        while (!op.isStopRequested()
                   && (visionPortal.getCameraState()
                        != VisionPortal.CameraState.STREAMING)) {
            op.sleep(20);
        }
        if (!op.isStopRequested()) {
            ExposureControl exposureControl =
                visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                op.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS,
                                        TimeUnit.MILLISECONDS);
            op.sleep(20);
            GainControl gainControl =
                visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            op.sleep(20);
            return (true);
        }
        return (false);
    }
    public int getLfTicksPrev(){
        return lfTicksPrev;
    }
    public int getRfTicksPrev(){
        return rfTicksPrev;
    }
    public int getRbTicksPrev(){
        return rbTicksPrev;
    }
    public int getLbTicksPrev(){
        return lbTicksPrev;
    }
    public double getHeadingPrev(){
        return headingPrev;
    }
    
    public void setLfTicksPrev(int tick){
        lfTicksPrev = tick;
    }
    public void setRfTicksPrev(int tick){
        rfTicksPrev = tick;
    }
    public void setRbTicksPrev(int tick){
        rbTicksPrev = tick;
    }
    public void setLbTicksPrev(int tick){
        lbTicksPrev = tick;
    }
    
    public void setHeadingPrev(double deg){
        headingPrev = deg;
    }
}

