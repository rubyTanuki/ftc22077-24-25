package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.RobotValues.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.roboticslib.motion.*;

public class RobotDriveOdo implements Odometry{
    MainBot bot;
    
    double fieldHeading;
    double fieldX;
    double fieldY;
    
    public RobotDriveOdo(MainBot b){
        bot = b;
    }
    
    @Override
    public double getX() {
            return fieldX;
    }

    @Override
    public double getY() {
            return fieldY;
    }

    @Override
    public double getHeading() {
            return fieldHeading;
    }
    
    public void setHeading(double h){
        fieldHeading = h;
    }
    
    
    public void setFieldXY(double fx, double fy) {
        updateTracking();
        fieldX = fx;
        fieldY = fy;
        fieldHeading = bot.getHeading();
    }
    
    
    public void updateTracking() {
        
        double heading = bot.getHeading();
        
        // get current motor ticks
        int lfTicks = bot.getFLPos();
        int rfTicks = bot.getFRPos();
        int lbTicks = bot.getBLPos();
        int rbTicks = bot.getBRPos();
        
        // determine angular delta (rotations) for each motor
        double lfD = (lfTicks - bot.lfTicksPrev) / TICKS_PER_REVOLUTION;
        double rfD = (rfTicks - bot.rfTicksPrev) / TICKS_PER_REVOLUTION;
        double lbD = (lbTicks - bot.lbTicksPrev) / TICKS_PER_REVOLUTION;
        double rbD = (rbTicks - bot.rbTicksPrev) / TICKS_PER_REVOLUTION;
        
        // remember new tick values
        bot.lfTicksPrev=lfTicks; bot.rfTicksPrev=rfTicks; bot.lbTicksPrev=lbTicks; bot.rbTicksPrev=rbTicks;

        // calculate delta distances in field units (rdx, rdy, rdw)
        double rdx = ((lfD + rfD + lbD + rbD) * DISTANCE_PER_REVOLUTION) / 4.0;
        double rdy = ((-lfD + rfD + lbD - rbD) * DISTANCE_PER_REVOLUTION) / 4.0;
        double rdw = Math.toRadians(heading - bot.headingPrev);
        bot.headingPrev = heading;
        
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
        double fw0 = Math.toRadians(fieldHeading);
        double fdx = Math.cos(fw0) * pdx - Math.sin(fw0) * pdy;
        double fdy = Math.sin(fw0) * pdx + Math.cos(fw0) * pdy;
        double fdw = rdw;
        
        
        // integrate into field coordinates
        fieldX += fdx;
        fieldY += fdy;
        fieldHeading = heading;
    }
    
    
}
