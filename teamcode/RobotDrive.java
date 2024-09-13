package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class RobotDrive {

    DcMotorEx lf;
    DcMotorEx rf;
    DcMotorEx lb;
    DcMotorEx rb;
    
    IMU imu;
    double headingOffset = 0;
    
    public void init(HardwareMap hardwareMap){
        lf = initDcMotor(hardwareMap, "fl", DcMotor.Direction.REVERSE);
        rf = initDcMotor(hardwareMap, "fr", DcMotor.Direction.FORWARD);
        lb = initDcMotor(hardwareMap, "bl", DcMotor.Direction.REVERSE);
        rb = initDcMotor(hardwareMap, "br", DcMotor.Direction.FORWARD);
        initIMU(hardwareMap);
    }
    
    public double getIMUHeading(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    
    public DcMotorEx initDcMotor(HardwareMap hardwareMap, String name, DcMotor.Direction dir){
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
        m.setDirection(dir);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return m;
    }
    
    public void initIMU(HardwareMap hardwareMap){
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
            new RevHubOrientationOnRobot(RobotValues.LOGO_DIR, RobotValues.USB_DIR));
        imu.initialize(params);
    }
    
    public void driveXYW(double rx, double ry, double rw) {
        double lfPower = rx - ry - rw;
        double rfPower = rx + ry + rw;
        double lbPower = rx + ry - rw;
        double rbPower = rx - ry + rw;

        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);
    }
    
    public void setHeading(double h){
        headingOffset = h - getIMUHeading();
    }
    
    public double getHeading(){
        return headingOffset + getIMUHeading();
    }
    
}