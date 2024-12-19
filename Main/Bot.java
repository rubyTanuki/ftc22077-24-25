//holds motors
//manages all drivetrain
//only sensor is IMU
package Main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
import static Main.BotValues.*;


public class Bot{

    //create motors
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;

    public IMU imu;
    double heading;

    private VoltageSensor voltageSensor;
    
    //current motor positions
    int frontLeftPos  = 0;
    int frontRightPos = 0;
    int backLeftPos   = 0;
    int backRightPos  = 0;

    //previous motor positions
    int frontLeftPosPrev  = 0;
    int frontRightPosPrev = 0;
    int backLeftPosPrev   = 0;
    int backRightPosPrev  = 0;

    double headingPrev = 0;

    public Bot(HardwareMap hm)
    { //constructor
        initMotors(hm);
        initIMU(hm);
        setDirections();
        voltageSensor = hm.voltageSensor.iterator().next();
        //resetEncoders();
    }

    private void initMotors(HardwareMap hm)
    { //initializing motors
        frontLeft = hm.get(DcMotorEx.class, "frontLeft");
        frontRight = hm.get(DcMotorEx.class, "frontRight");
        backLeft = hm.get(DcMotorEx.class, "backLeft");
        backRight = hm.get(DcMotorEx.class, "backRight");
        
    }

    public void initIMU(HardwareMap hardwareMap)
    { //initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
            new RevHubOrientationOnRobot(LOGO_DIR, USB_DIR));
        imu.initialize(params);
    }
    
    public double getVoltage(){
        return voltageSensor.getVoltage();
    }

    

    //getter methods
    public DcMotorEx getFL(){ return frontLeft;     }
    public DcMotorEx getFR(){ return frontRight;    }
    public DcMotorEx getBL(){ return backLeft;      }
    public DcMotorEx getBR(){ return backRight;     }
    public int getFLPos(){ return frontLeftPos;  }
    public int getFRPos(){ return frontRightPos; }
    public int getBLPos(){ return backLeftPos;   }
    public int getBRPos(){ return backRightPos;  }
    public int getFLPosPrev() { return frontLeftPosPrev;  }
    public int getFRPosPrev() { return frontRightPosPrev; }
    public int getBLPosPrev() { return backLeftPosPrev;   }
    public int getBRPosPrev() { return backRightPosPrev;  }

    public double getHeading(){ return -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);}
    public void resetHeading(){ imu.resetYaw(); }
    
    public void resetPrevEncoders(){
        frontLeftPosPrev = 0;
        frontRightPosPrev = 0;
        backLeftPosPrev = 0;
        backRightPosPrev = 0;
    }

    public void setDirections()
    { //setting the directions of left motors to reverse
        frontLeft.setDirection(LEFTDIR);
        frontRight.setDirection(RIGHTDIR);
        backLeft.setDirection(LEFTDIR);
        backRight.setDirection(RIGHTDIR);
    }
    
    public void setMode(DcMotor.RunMode mode)
    { //setting motors to specifiec runmode
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public void resetEncoders()
    { //Resetting encoders to 0, keeping current mode
        resetPrevEncoders();
        DcMotor.RunMode mode = frontLeft.getMode();

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public void updateEncoders()
    { //updating current position numbers
        frontLeftPosPrev = frontLeftPos;
        frontRightPosPrev = frontRightPos;
        backLeftPosPrev = backLeftPos;
        backRightPosPrev = backRightPos;
        frontLeftPos = frontLeft.getCurrentPosition();
        frontRightPos = frontRight.getCurrentPosition();
        backLeftPos = backLeft.getCurrentPosition();
        backRightPos = backRight.getCurrentPosition();
    }

    public void enableBrakeMode(boolean enabled)
    { //turning on or off brakemode
        DcMotor.ZeroPowerBehavior state = enabled ? BRAKE : FLOAT;
        frontLeft.setZeroPowerBehavior(state);
        frontRight.setZeroPowerBehavior(state);
        backLeft.setZeroPowerBehavior(state);
        backRight.setZeroPowerBehavior(state);
    }

    public void driveXYW(double rx, double ry, double rw)
    { //set proportional power to drive motors
        double denom = Math.max(Math.abs(rx)+Math.abs(ry)+Math.abs(rw),1);
        double voltageMulti = getVoltage() / 12;
        double lfPower = ((rx - ry - rw) / denom) / voltageMulti;
        double rfPower = ((rx + ry + rw) / denom) / voltageMulti;
        double lbPower = ((rx + ry - rw) / denom) / voltageMulti;
        double rbPower = ((rx - ry + rw) / denom) / voltageMulti;
        
        frontLeft.setPower(lfPower);
        frontRight.setPower(rfPower);
        backLeft.setPower(lbPower);
        backRight.setPower(rbPower);
    }
    
    public void driveFieldXYW(double fx, double fy, double fw, double rot)
    { //rotate field orientation to robot orientation
        double theta = Math.toRadians(getHeading()) + rot;
        double rx = fx * Math.cos(-theta) - fy * Math.sin(-theta);
        double ry = fx * Math.sin(-theta) + fy * Math.cos(-theta);

        driveXYW(rx, ry, fw);
    }
}
