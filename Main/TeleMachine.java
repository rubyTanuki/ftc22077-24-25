package Main;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.State;
import Main.util.*;
import org.firstinspires.ftc.teamcode.*;
import java.util.ArrayList;

public class TeleMachine {
    AutoBot bot = null;
    public BotState currentState = new BotState();
    
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    
    private boolean lastUp;
    private boolean lastDown;
    private boolean lastLeft;
    private boolean lastRight;
    private boolean lastTrigger;

    private final int ARM_GAIN = 40; //10
    private final int SLIDE_GAIN = 30; //10
    private final double PITCH_GAIN = 0.01; //0.01
    private final double ROLL_GAIN = 0.01; //0.01
    private final double CLAW_GAIN = 0.01; //0.01
    
    private double ROLL_FLAT = 1;
    private final double CLAW_CLOSED = .65;
    private final double CLAW_OPEN = 1;
    public boolean clawIsOpen = false;
    
    private double clawTarget = .3;
    

    int current = 0;

    public TeleMachine(AutoBot b)
    { //contructor me
        bot = b;
    }
    
    public String toString(){
        return currentState.getName();
    }

    public void start()
    { //calling the current state's start
        currentState.start();
    }

    public void update()
    { //calling the current state's update
        currentState.update();
    }

    public void end()
    { //calling the current state's end
        currentState.end();
    }

    public void armClosed()
    { //arm fully closed, in bottom position
        if(!currentState.getName().equals("ArmClosed") || currentState == null){
            currentState = new BotState(){
                @Override
                public String getName(){ return "ArmClosed"; }
                ElapsedTime servoTimer;

                @Override
                void start(){
                    servoTimer = new ElapsedTime();
                }
                @Override
                void update(){
                    bot.setPitch(0);
                    if(bot.getSlidePos()>50) bot.setArm(bot.getArmPos(), 0);
                    else bot.setArm(5, 0);
                    
                    if(gamepad2 != null)bot.hang.setPower(-gamepad2.left_stick_y);
                }
                @Override
                void end(){
                    bot.setArm(300);
                    bot.setWrist(0, ROLL_FLAT, clawTarget);
                }
            };
            currentState.start();
        }
        
    }

    public void armClosing()
    { //arm moving towards being closed
        if(!currentState.getName().equals("ArmClosing") 
        || !currentState.getName().equals("ArmClosed")
        || currentState == null){
            currentState = new BotState(){
                @Override
                public String getName(){ return "ArmClosing";}
                ElapsedTime timer;

                @Override
                void start(){
                    timer = new ElapsedTime();
                }

                @Override
                void update(){
                    if(bot.getSlidePos()>50) bot.setArm(bot.getArmPos(), 0);
                    else bot.setArm(2000, 0);
                    if(bot.getArmPos()>500)bot.setWrist(0, ROLL_FLAT, clawTarget);

                    if((Math.abs(bot.getArmPos()-1000) <10 && bot.getSlidePos()<50) || bot.getArmPos()>1100);
                    {
                        if(timer.seconds()>1)armClosed();
                    }
                }
            };
            currentState.start();
        }
    }
    
    public void floorPickup()
    { //picking up sample/specimen from floor
        if(!currentState.getName().equals("FloorPickup") || currentState == null){
            currentState = new BotState(){
                @Override
                public String getName(){ return "FloorPickup";}
                private int armTarget = 50;
                private int slideTarget = 0;
                private double pitchTarget = .7;
                
                @Override
                void start(){
                    //bot.setWrist(.6, .5, clawTarget);
                }

                @Override
                void update(){
                    if(gamepad2 != null){
                        armTarget += Math.round(ARM_GAIN* -gamepad2.left_stick_y);
                        slideTarget += Math.round(SLIDE_GAIN* -gamepad2.right_stick_y);
                    
                        armTarget = Math.max(10, armTarget);
                        slideTarget = Math.max(10, slideTarget);
                        slideTarget = Math.min(1100, slideTarget);
                        
                        if(gamepad2.dpad_down && !lastDown) pitchTarget-=.01;
                        if(gamepad2.dpad_up && !lastUp) pitchTarget +=.01;
                    }else{
                        if(clawIsOpen)clawTarget = CLAW_OPEN;
                        else clawTarget = CLAW_CLOSED;
                    }
                    bot.setArm(armTarget, slideTarget);
                    bot.setWrist(pitchTarget, ROLL_FLAT, clawTarget);

                }
            };
        }
    }
    public void goingToPickup(){
        if(!currentState.getName().equals("GoingToPickup")){
            currentState = new BotState(){
                @Override
                public String getName(){ return "GoingToPickup"; }
                ElapsedTime timer;

                @Override
                void start(){
                    timer = new ElapsedTime();
                }

                @Override
                void update(){
                    if(bot.getSlidePos()>50) bot.setArm(bot.getArmPos(), 10);
                    else bot.setArm(2000, 10);
                    if(bot.getArmPos()>1500) bot.setWrist(.8, ROLL_FLAT, .7);
                    else bot.setWrist(0, ROLL_FLAT, 0.4);
                    

                    if((Math.abs(bot.getArmPos()-1000) <10 && bot.getSlidePos()<50) || bot.getArmPos()>1100);
                    {
                        if(timer.seconds()>1.2)floorPickup();
                    }
                }
            };
            currentState.start();
        }
    }
    
    public void pullBack()
    {
        if(currentState.getName().equals("FloorPickup")){
            currentState = new BotState(){
                @Override
                public String getName(){ return "PullBack";}
                
                double clawTarget = CLAW_CLOSED;
                
                @Override
                void start(){
                    
                }
                
                @Override
                void update(){
                    if(clawIsOpen)clawTarget = CLAW_OPEN;
                    else clawTarget = CLAW_CLOSED;
                    bot.setArm(600, 10);
                    bot.setWrist(.7, ROLL_FLAT, clawTarget);
                }
            };
            currentState.start();
        }
    }
    
    public void specimen()
    {
        if(!currentState.getName().equals("Specimen") || currentState == null){
            currentState = new BotState(){
                @Override
                public String getName(){ return "Specimen"; }
                ElapsedTime timer;
                private int armTarget = 1350;
                private int slideTarget = 300;
                double[] rollTargets = {0.3, 0.6, ROLL_FLAT};
                int index = 2;
                
                @Override
                void start(){
                    timer = new ElapsedTime();
                    //bot.setClaw(clawTarget);
                }
                
                @Override
                void update(){
                    if(clawIsOpen)clawTarget = CLAW_OPEN;
                    else clawTarget = CLAW_CLOSED;
                    if(gamepad2 != null){
                        armTarget += Math.round(ARM_GAIN* -gamepad2.left_stick_y);
                        slideTarget += Math.round(SLIDE_GAIN* -gamepad2.right_stick_y);
                        
                        armTarget = Math.max(10, armTarget);
                        slideTarget = Math.max(10, slideTarget);
                        double pitchTarget = bot.getPitchPos();
                        if(gamepad2.dpad_up && lastUp) pitchTarget +=PITCH_GAIN;
                        if(gamepad2.dpad_down && lastDown) pitchTarget -=PITCH_GAIN;
                        
                        if(pitchTarget >1) pitchTarget = 0;
                        if(pitchTarget <0) pitchTarget = 1;
                        
                        
                        if(gamepad2.dpad_left && !lastLeft && index-1>=0) index--;
                        if(gamepad2.dpad_right && !lastRight && index+1<rollTargets.length) index++;
                    }
                    
                    // if(index>rollTargets.length-1) index = 0;
                    // if(index<0) index = rollTargets.length-1;
                    
                    
                    bot.setArm(armTarget, bot.getArmPos()>1000?slideTarget:bot.getSlidePos());
                    if(bot.getArmPos()>1000){
                        if(timer.seconds()>2)bot.setWrist(.7, rollTargets[index], clawTarget);
                        else bot.setWrist(.7, ROLL_FLAT, clawTarget);
                    }
                }
            };
            currentState.start();
        }
    }
    public void specimenUp()
    {
        if(!currentState.getName().equals("SpecimenUp")){
            currentState = new BotState(){
                @Override
                public String getName(){ return "SpecimenUp";}
                private int armTarget = 1550;
                private int slideTarget = 300;
                
                @Override
                void start(){
                    
                }
                
                @Override
                void update(){
                    bot.setArm(armTarget, bot.getArmPos()>1000?slideTarget:bot.getSlidePos());
                    bot.setWrist(.7, ROLL_FLAT, clawTarget);
                }
            };
            currentState.start();
        }
    }
    
    public void goingToDrop()
    {
        if(!currentState.getName().equals("GoingToDrop") 
            || !currentState.getName().equals("DropInBucket")
            || currentState == null){
            currentState = new BotState(){
                @Override
                public String getName(){ return "GoingToDrop"; }
                ElapsedTime timer;
                
                @Override
                void start(){
                    timer = new ElapsedTime();
                }
                
                @Override
                void update(){
                    
                    if(gamepad2 == null){
                        if(clawIsOpen) clawTarget = CLAW_OPEN;
                        else clawTarget = CLAW_CLOSED;
                    }
                    bot.setArm(2800, Math.max(0, bot.getSlidePos()));
                    bot.setWrist(0, ROLL_FLAT, clawTarget);
                    
                    if(Math.abs(bot.getArmPos()-2800) <10 && timer.seconds()>.5);
                    {
                        if(timer.seconds()>0.5)dropInBucket();
                    }
                }
            };
            currentState.start();
        }
    }
    public void dropInBucket()
    {
        if(!currentState.getName().equals("DropInBucket")){
            currentState = new BotState(){
                @Override
                public String getName(){ return "DropInBucket"; }
                private int armTarget = 2900;
                private int slideTarget = 1550;
                
                @Override
                void start(){
                    //bot.setWrist();
                }
                
                @Override
                void update(){
                    if(gamepad2 != null){
                        armTarget += Math.round(ARM_GAIN* -gamepad2.left_stick_y);
                        slideTarget += Math.round(SLIDE_GAIN* -gamepad2.right_stick_y);
                        
                        armTarget = Math.max(10, armTarget);
                        slideTarget = Math.max(10, slideTarget);
                    }
                    else{
                        if(clawIsOpen) clawTarget = CLAW_OPEN;
                        else clawTarget = CLAW_CLOSED;
                    }
                    bot.setArm(armTarget, slideTarget);
                    bot.setWrist(.4, ROLL_FLAT, clawTarget);
                }
            };
        }
    }

    public void updateGamepad(Gamepad gp1, Gamepad gp2)
    { //updating the values for manually controlling the arm
        if(gamepad2 != null){
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastRight = gamepad1.dpad_right;
            lastLeft = gamepad1.dpad_left;
            clawTarget = gamepad2.right_trigger>0?CLAW_OPEN:CLAW_CLOSED;
            //lastTrigger = gamepad2.right_trigger
        }
        gamepad1 = gp1;
        gamepad2 = gp2;
        
    }

}

//state class
class BotState{
    public String getName(){ return "";};
    void start(){};
    void update(){};
    void end(){};
}