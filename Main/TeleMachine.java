package Main;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.State;
import Main.util.*;
import Main.statemachine.*;
import org.firstinspires.ftc.teamcode.*;
import java.util.ArrayList;

public class TeleMachine {
    AutoBot bot = null;
    public BotState currentState = new BotState();
    
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    
    private boolean lastUp;
    private boolean lastDown;

    private final int ARM_GAIN = 50; //10
    private final int SLIDE_GAIN = 50; //10
    private final double PITCH_GAIN = 0.01; //0.01
    private final double ROLL_GAIN = 0.01; //0.01
    private final double CLAW_GAIN = 0.01; //0.01
    
    private final double CLAW_CLOSED = .4;
    private final double CLAW_OPEN = .15;

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
                    if(bot.getSlidePos()>50) bot.setArm(bot.getArmPos(), 10);
                    else bot.setArm(5, 10);
                }
                @Override
                void end(){
                    bot.setArm(300);
                }
            };
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
                    if(bot.getSlidePos()>50) bot.setArm(bot.getArmPos(), 10);
                    else bot.setArm(1000, 10);
                    bot.setWrist(0, .7, .3);

                    if((Math.abs(bot.getArmPos()-1000) <10 && bot.getSlidePos()<50) || bot.getArmPos()>1100);
                    {
                        if(timer.seconds()>0.5)armClosed();
                    }
                }
            };
        }
    }
    
    public void floorPickup()
    { //picking up sample/specimen from floor
        if(!currentState.getName().equals("FloorPickup") || currentState == null){
            currentState = new BotState(){
                @Override
                public String getName(){ return "FloorPickup";}
                private int armTarget = 300;
                private int slideTarget = 150;
                
                @Override
                void start(){
                    bot.setWrist(.65, .6, .3);
                }

                @Override
                void update(){
                    armTarget += Math.round(ARM_GAIN* -gamepad2.left_stick_y);
                    slideTarget += Math.round(SLIDE_GAIN* -gamepad2.right_stick_y);
                    
                    armTarget = Math.max(10, armTarget);
                    slideTarget = Math.max(10, slideTarget);
                    double pitchTarget = bot.getPitchPos();
                    if(gamepad2.dpad_up && lastUp) pitchTarget +=PITCH_GAIN;
                    if(gamepad2.dpad_down && lastDown) pitchTarget -=PITCH_GAIN;
                    double rollTarget = .5;
                    double clawTarget = .3;
                    
                    bot.setArm(bot.getSlidePos()>200?bot.getArmPos():armTarget, slideTarget);
                    bot.setWrist(pitchTarget, rollTarget, clawTarget);

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
                    else bot.setArm(1000, 10);
                    bot.setWrist(0, .7, .3);

                    if((Math.abs(bot.getArmPos()-1000) <10 && bot.getSlidePos()<50) || bot.getArmPos()>1100);
                    {
                        if(timer.seconds()>0.5)floorPickup();
                    }
                }
            };
        }
    }
    
    public void specimen()
    {
        if(!currentState.getName().equals("Specimen") || currentState == null){
            currentState = new BotState(){
                @Override
                public String getName(){ return "Specimen"; }
                ElapsedTime timer;
                private int armTarget = 1800;
                private int slideTarget = 300;
                
                @Override
                void start(){
                    timer = new ElapsedTime();
                }
                
                @Override
                void update(){
                    armTarget += Math.round(ARM_GAIN* -gamepad2.left_stick_y);
                    slideTarget += Math.round(SLIDE_GAIN* -gamepad2.right_stick_y);
                    
                    armTarget = Math.max(10, armTarget);
                    slideTarget = Math.max(10, slideTarget);
                    double pitchTarget = bot.getPitchPos();
                    if(gamepad2.dpad_up && lastUp) pitchTarget +=PITCH_GAIN;
                    if(gamepad2.dpad_down && lastDown) pitchTarget -=PITCH_GAIN;
                    double rollTarget = .5;
                    double clawTarget = .3;
                    
                    bot.setArm(
                        gamepad2.right_trigger>0?armTarget-300:armTarget,
                        bot.getArmPos()>1000?slideTarget:bot.getSlidePos());
                    bot.setWrist(pitchTarget, rollTarget, clawTarget);
                }
            };
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
                    bot.setArm(2800, Math.max(100, bot.getSlidePos()));
                    
                    if(Math.abs(bot.getArmPos()-2800) <10 && timer.seconds()>.5);
                    {
                        if(timer.seconds()>0.5)dropInBucket();
                    }
                }
            };
            
            
        }
    }
    public void dropInBucket()
    {
        if(!currentState.getName().equals("DropInBucket")){
            currentState = new BotState(){
                @Override
                public String getName(){ return "DropInBucket"; }
                private int armTarget = 2800;
                private int slideTarget = 1600;
                
                @Override
                void start(){
                    //bot.setWrist();
                }
                
                @Override
                void update(){
                    armTarget += Math.round(ARM_GAIN* -gamepad2.left_stick_y);
                    slideTarget += Math.round(SLIDE_GAIN* -gamepad2.right_stick_y);
                    
                    armTarget = Math.max(10, armTarget);
                    slideTarget = Math.max(10, slideTarget);
                    double pitchTarget = bot.getPitchPos();
                    if(gamepad2.dpad_up && lastUp) pitchTarget +=PITCH_GAIN;
                    if(gamepad2.dpad_down && lastDown) pitchTarget -=PITCH_GAIN;
                    double rollTarget = .5;
                    double clawTarget = .3;
                    
                    bot.setArm(armTarget, slideTarget);
                    bot.setWrist(pitchTarget, rollTarget, clawTarget);
                }
            };
        }
    }

    public void updateGamepad(Gamepad gp1, Gamepad gp2)
    { //updating the values for manually controlling the arm
        if(gamepad2 != null){
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
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