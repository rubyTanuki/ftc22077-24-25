package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.RobotValues.*;

public class RobotDrivePm {

    DcMotorEx lf;
    DcMotorEx rf;
    DcMotorEx lb;
    DcMotorEx rb;
    
    IMU imu;
    static double headingOffset = 0;
    
    Pose field = new Pose(0,0,0);
    
    int lfTicksPrev = 0;
    int rfTicksPrev = 0;
    int lbTicksPrev = 0;
    int rbTicksPrev = 0;
    double headingPrev = 0;
    
    public void init(HardwareMap hardwareMap) {
        lf = initDcMotor(hardwareMap, "frontLeft", LEFTDIR);
        rf = initDcMotor(hardwareMap, "frontRight", RIGHTDIR);
        lb = initDcMotor(hardwareMap, "backLeft", LEFTDIR);
        rb = initDcMotor(hardwareMap, "backRight", RIGHTDIR);
        initIMU(hardwareMap);
        setFieldXY(0,0);
    }
    
    public void initIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
            new RevHubOrientationOnRobot(LOGO_DIR, USB_DIR));
        imu.initialize(params);    
    }
    
    public double getIMUHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    
    public void setHeading(double h) {
        imu.resetYaw();
        headingOffset = h - getIMUHeading();
    }
    
    public double getHeading() {
        return AngleUnit.normalizeDegrees(headingOffset + getIMUHeading());
    }
    
    public void setFieldXY(double fx, double fy) {
        updateTracking();
        field.x = fx;
        field.y = fy;
        field.h = getHeading();
    }
    
    public DcMotorEx initDcMotor(HardwareMap hardwareMap,
                            String name,
                            DcMotor.Direction dir) {
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
        m.setDirection(dir);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return m;
    }
    
    public void driveXYW(double rx, double ry, double rw) {
        double denom = Math.max(
                            Math.abs(rx)+Math.abs(ry)+Math.abs(rw),
                            1);
        double lfPower = (rx - ry - rw) / denom;
        double rfPower = (rx + ry + rw) / denom;
        double lbPower = (rx + ry - rw) / denom;
        double rbPower = (rx - ry + rw) / denom;
        
        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);
    }
    
    public void driveFieldXYW(double fx, double fy, double fw, double rot) {
        // rotate field orientation to robot orientation
        double theta = Math.toRadians(getHeading()) + rot;
        double rx = fx * Math.cos(-theta) - fy * Math.sin(-theta);
        double ry = fx * Math.sin(-theta) + fy * Math.cos(-theta);

        driveXYW(rx, ry, fw);
    }
    
    public void updateTracking() {
        double heading = getHeading();
        
        // get current motor ticks
        int lfTicks = lf.getCurrentPosition();
        int rfTicks = rf.getCurrentPosition();
        int lbTicks = lb.getCurrentPosition();
        int rbTicks = rb.getCurrentPosition();
        
        // determine angular delta (rotations) for each motor
        double lfD = (lfTicks - lfTicksPrev) / TICKS_PER_REVOLUTION;
        double rfD = (rfTicks - rfTicksPrev) / TICKS_PER_REVOLUTION;
        double lbD = (lbTicks - lbTicksPrev) / TICKS_PER_REVOLUTION;
        double rbD = (rbTicks - rbTicksPrev) / TICKS_PER_REVOLUTION;
        
        // remember new tick values
        lfTicksPrev=lfTicks; rfTicksPrev=rfTicks; lbTicksPrev=lbTicks; rbTicksPrev=rbTicks;

        // calculate delta distances in field units (rdx, rdy, rdw)
        double rdx = ((lfD + rfD + lbD + rbD) * DISTANCE_PER_REVOLUTION) / 4.0;
        double rdy = ((-lfD + rfD + lbD - rbD) * DISTANCE_PER_REVOLUTION) / 4.0;
        double rdw = Math.toRadians(heading - headingPrev);
        headingPrev = heading;
        
        // calculate pose exponential (pdx, pdy, pdw)
        // from https://file.tavsys.net/control/controls-engineering-in-frc.pdf
        //     Figure 10.2.1
        double s;
        double c;
        if (rdw < -0.1 || rdw > 0.1) {
            s = Math.sin(rdw) / rdw;
            c = (1-Math.cos(rdw)) / rdw;
        } else {
            // for angles near zero, we approximate with taylor series
            s = 1 - ((rdw*rdw) / 6);            // sin(w)/w     ~~> (1-(w*w)/6)
            c = rdw / 2;                        // (1-cos(w))/w ~~> (w/2)
        }
        double pdx = s * rdx - c * rdy ;
        double pdy = c * rdx + s * rdy;
        double pdw = rdw;
        
        // compute delta change in field coordinates (fdx, fdy, fdw)
        double fw0 = Math.toRadians(field.h);
        double fdx = Math.cos(fw0) * pdx - Math.sin(fw0) * pdy;
        double fdy = Math.sin(fw0) * pdx + Math.cos(fw0) * pdy;
        double fdw = rdw;
        
        // integrate into field coordinates
        field.x += fdx;
        field.y += fdy;
        field.h = heading;
    }

    public double computeHeadingW(double h) {
        return (h - getHeading()) * 0.06;
    }
    
    public double driveToXY(double tx, double ty, double vel) {
        return driveToPose(tx, ty, 0, vel);
    }
    public double driveToPose(Pose p, double vel) {
        return driveToPose(p.x, p.y, computeHeadingW(p.h), vel);
    }
    public double driveToPose(double tx, double ty, double rw, double vel) {
        double fdx = tx - field.x;
        double fdy = ty - field.y;
        double dist = Math.hypot(fdx, fdy);
        double absHeadingRad = Math.atan2(fdy, fdx);
        double relHeadingRad = absHeadingRad - Math.toRadians(getHeading());
        double rdx = Math.cos(relHeadingRad) * dist;
        double rdy = Math.sin(relHeadingRad) * dist;
        double dScale = Math.abs(rdx) + Math.abs(rdy);
        //dScale = dScale*5;
        double rx = rdx / dScale;
        double ry = rdy / dScale;
        
        vel = vel*Math.max(Math.min(dist/15, 1), .4);
        
        
        driveXYW(rx * vel, ry * vel, rw*.3);
        return dist;
    }

    public static class Pose {
        double y;
        double x;
        double h;
        double vel;
        double lr;
        
        static double lrDefault = 18;
        static double velDefault = 0.5;

        public Pose() { setValues(0,0,0,velDefault,lrDefault); }
        public Pose(double x, double y) { setValues(x,y,0,velDefault,lrDefault); }
        public Pose(double x, double y, double h) { setValues(x,y,h,velDefault,lrDefault); }
        public Pose(Pose p) { setValues(p.x,p.y,p.h,p.vel,p.lr); }

        public Pose setValues(double x, double y, double h, double vel, double lr) {
            this.x = x;
            this.y = y;
            this.h = h;
            this.vel = vel;
            this.lr = lr;
            return this;
        }
        
        public String toString() {
            return String.format("x=%.2f, y=%.2f, h=%.1f", x, y, h);
        }
        
    }
}
