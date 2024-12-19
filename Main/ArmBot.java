//extends Bot to hold an arm with a slide
//manages all drive train and arm
package Main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static Main.BotValues.*;

public class ArmBot extends Bot{
    private DcMotorEx arm;
    private DcMotorEx slide;
    public DcMotorEx hang;

    private Servo pitch;
    private Servo roll;
    private Servo claw;
    
    private Servo finger;

    int armPos   = 0;
    int slidePos = 0;
    int hangPos = 0;

    int armPosPrev = 0;
    int slidePosPrev = 0;

    public ArmBot(HardwareMap hm)
    { //constructor method
        super(hm);
        initMotors(hm);
        initServos(hm);
        //resetEncoders();
    }

    private void initMotors(HardwareMap hm)
    { //initialising motors
        arm = hm.get(DcMotorEx.class, "arm");
        slide = hm.get(DcMotorEx.class, "slide");
        hang = hm.get(DcMotorEx.class, "hang");
        
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        arm.setDirection(DcMotor.Direction.REVERSE);
    }

    private void initServos(HardwareMap hm)
    { //initialising servos
        pitch = hm.get(Servo.class, "pitch");
        roll = hm.get(Servo.class, "roll");
        claw = hm.get(Servo.class, "claw");
        
        finger = hm.get(Servo.class, "finger");
    }
    
    public void setArmMode(DcMotor.RunMode mode)
    {
        arm.setMode(mode);
        slide.setMode(mode);
    }
    
    public void resetEncoders()
    { //Resetting encoders to 0, keeping current mode
        super.resetEncoders(); //Bot.resetEncoders()
        DcMotorEx.RunMode armMode = arm.getMode();
        DcMotorEx.RunMode slideMode = slide.getMode();

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(armMode);
        slide.setMode(slideMode);
    }

    public void updateEncoders()
    { //updates current and last encoder positions
        super.updateEncoders(); //Bot.updateEncoders
        armPosPrev = armPos;
        slidePosPrev = slidePos;

        armPos = arm.getCurrentPosition();
        slidePos = slide.getCurrentPosition();
        hangPos = hang.getCurrentPosition();
    }
    
    //getter methods
    public DcMotorEx getArm()   { return arm;   }
    public DcMotorEx getSlide() { return slide; }
    public DcMotorEx getHang()  { return hang;  }
    public int getArmPos()      { return armPos;   }
    public int getSlidePos()    { return slidePos; }
    public int getHangPos()     { return hangPos;  }

    public Servo getPitch()     { return pitch; }
    public Servo getRoll()      { return roll;  }
    public Servo getClaw()      { return claw;  }
    public double getPitchPos() { return pitch.getPosition(); }
    public double getRollPos()  { return roll.getPosition();  }
    public double getClawPos()  { return claw.getPosition();  }

    public void setArm(int armPos, int slidePos)
    { //running arm and slide motors to requested position
        setArm(armPos);
        setSlide(slidePos);
    }

    public void setArm(int armPos)
    { //running arm motor to requested position
        arm.setTargetPosition(armPos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(6000);
    }

    public void setSlide(int slidePos)
    { //running slide motor to requested position
        slide.setTargetPosition(slidePos);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setVelocity(10000);
    }
    

    public void setPitch(double p)
    { //setting position of pitch servo
        pitch.setPosition(p);
    }

    public void setRoll(double r)
    { //setting position of roll servo
        roll.setPosition(r);
    }

    public void setClaw(double c)
    { //setting position of claw servo
        claw.setPosition(c);
    }

    public void setWrist(double p, double r, double c)
    { //setting position of all wrist servos
        setPitch(p);
        setRoll(r);
        setClaw(c);
    }
    
    public void setFinger(double f){
        finger.setPosition(f);
    }
}
