package Main.util;

public class ArmController{

    public double arm = 0;
    public double slide = 0;
    public double pitch = 0;
    public double roll = 0;
    public boolean claw = false;
    
    public void setArm(double a)   { arm   = a; }
    public void setSlide(double s) { slide = s; }
    public void setPitch(double p) { pitch = p; }
    public void setRoll(double r)  { roll  = r; }
    public void setClaw(boolean c)  { claw  = c; }

    public double getArm(){ return arm; }
    public double getSlide() {return slide;}
    public double getPitch() {return pitch; }
    public double getRoll()  { return roll; }
    public boolean getClaw()  { return claw; }
}