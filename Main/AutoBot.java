package Main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.concurrent.TimeUnit;
import static Main.BotValues.*;

public class AutoBot extends ArmBot{

    public AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private HuskyLens huskyLens;
    
    
    public AutoBot(HardwareMap hm)
    { //contructor method
        super(hm);
        initAprilTag(hm);
        huskyLens = hm.get(HuskyLens.class, "huskylens");
        
    }

    public void initAprilTag(HardwareMap hardwareMap)
    { //initializing the camera and processor
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
            hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
    }

    public boolean setManualExposure(LinearOpMode op, int exposureMS, int gain)
    { //applying settings for the camera
        if (visionPortal == null) { return false; }
        while (!op.isStopRequested()
                && (visionPortal.getCameraState()
                != VisionPortal.CameraState.STREAMING))
        {
            op.sleep(20);
        }
        if (!op.isStopRequested()) 
        {
            ExposureControl exposureControl =
                visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) 
            {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                op.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            op.sleep(20);
            GainControl gainControl =
                visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            op.sleep(20);
            return (true);
        }
        return (false);
    }
    
    public int getSampleX(int id){
        HuskyLens.Block sample = null;
        for(HuskyLens.Block block : huskyLens.blocks()){
            if(block.id == id && block.y > 120){
                if(sample == null) sample = block;
                if(Math.abs(block.x-260) < Math.abs(sample.x-180)){
                    sample = block;
                    break;
                }
            }
        }
        //process block
        try{
            return sample.x;
        }
        catch( Exception e){
            return 160;
        }
        
    }
    
    
}
