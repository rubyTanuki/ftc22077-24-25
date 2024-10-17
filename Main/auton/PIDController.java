package Main.auton;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;
import Main.util.Mathf;
import Main.*;
import org.firstinspires.ftc.teamcode.*;

public class PIDController {
    private AutoBot mc = null;

    public PID xPID;
    public PID yPID;
    public PID thetaPID;

    public double targetX = 0;
    public double targetY = 0;
    public double targetAngle = 0; //radians
    
    public double maxAngSpeed = .6;
    public double maxSpeed = .7;
    
    public BotOdometry odo = null;
    ElapsedTime timer = null;
    
    public boolean nextState = false;
    public double v = 1;
    
    public PIDController(AutoBot mc, BotOdometry odo){
        this.mc = mc;
        this.odo = odo;
    }
    public void setPID(PID x, PID y){
        xPID = x;
        yPID = y;
    }
    public void setTurnPID(PID theta){
        thetaPID = theta;
    }
    
    public void start(){
        xPID.start();
        yPID.start();
        thetaPID.setTarget(0);
    }
    
    public double update(){
        double curX = odo.getX();
        double curY = odo.getY();
        double botHeading = -Math.toRadians(odo.getHeading()); //radian
        
        double xVal = xPID.update(curX);
        double yVal = yPID.update(curY);

        targetY = yPID.targetVal;

        double inputTheta = Math.atan2(yVal, xVal);
        double inputPower = Range.clip(Math.sqrt(Math.abs(xVal * xVal + yVal * yVal)), -1, 1);
        //double inputTurn = Mathf.angleWrap(targetAngle - botHeading);
        double inputTurn = -Mathf.angleWrap(targetAngle - botHeading);
        if(Math.abs(inputTurn) > Math.toRadians(2)){
            inputTurn = thetaPID.update(inputTurn);
            inputTurn = Range.clip(inputTurn, -maxAngSpeed, maxAngSpeed);
        }
        else{
            inputTurn = 0;
        }
        
        double sin = Math.sin(inputTheta + Math.PI/4 - botHeading);
        double cos = Math.cos(inputTheta + Math.PI/4 - botHeading);
        
        double maxed = Math.max(Math.abs(cos), Math.abs(sin));
        inputPower *= v;
        double frontLeft = inputPower * cos/maxed - inputTurn;
        double frontRight = inputPower * sin/maxed + inputTurn;
        double backLeft = inputPower * sin/maxed - inputTurn;
        double backRight = inputPower * cos/maxed + inputTurn;
        
        double max1 = Math.max(Math.abs(frontLeft), Math.abs(frontRight));
        double max2 = Math.max(Math.abs(backLeft), Math.abs(backRight));
        double max = Math.max(1, Math.max(max1, max2));
        
        mc.getFL().setPower(frontLeft / max);
        mc.getFR().setPower(frontRight / max);
        mc.getBL().setPower(backLeft / max);
        mc.getBR().setPower(backRight / max);
        return max;
    }
    
    public void moveTo(double x, double y, double angle){
        xPID.setTarget(x);
        yPID.setTarget(y);
        targetAngle = -Math.toRadians(angle);
    }
    
    public double getTargetX(){
        return targetX;
    }
    public double getTargetY(){
        return targetY;
    }
    public double getTargetAngle(){
        return targetAngle;
    }
}
