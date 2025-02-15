package Main.shorty;

import Main.*;
import Main.auton.*;
import Main.util.*;
import org.firstinspires.ftc.teamcode.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.HashMap;

public class Arm_FSM
{
    private ArmBot bot;
    private BotState currentState;
    
    public BPad gpad1;
    public BPad gpad2;
    
    public Arm_FSM(ArmBot bot, BPad g1, BPad g2){
        this.bot = bot;
        gpad1 = g1;
        gpad2 = g2;
        init();
        currentState = BotState.get("IDLE");
    }
    
    public void setState(String s){
        if(!currentState.toString().equals(BotState.get(s).toString())){
            currentState = BotState.get(s);
            start();
        }
    }
    
    public void start(){
        if(currentState!=null)currentState.start(); 
    }
    public void update(){
        if(currentState!=null)currentState.update(); 
    }
    public void end(){
        if(currentState!=null)currentState.end(); 
    }

    private void init(){
        new BotState("NEW_STATE", false){
            @Override
            void start(){
                
            }
            @Override
            void update(){

            }
        };
        new BotState("IDLE"){
            @Override
            void start(){
                bot.setSlides(0);
            }
            @Override
            void update(){
                if(gpad2.Up()){
                    bot.setForearm(0);
                }
                if(gpad2.Left()){
                    bot.setForearm(.25);
                }
                if(gpad2.Down()){
                    bot.setForearm(.5);
                }
                if(gpad2.Right()){
                    bot.setForearm(1);
                }
            }
        };
        new BotState("INTAKE_OUT"){
            @Override
            void start(){
                //move pivot up
                //send slides out
            }
            @Override
            void update(){
                //go to INTAKE when its out
            }
        };
        new BotState("INTAKE"){
            @Override
            void start(){
                //put the intake down (pivot)
            }
            @Override
            void update(){
                //give turret control of intake
            }
        };
        new BotState("INTAKE_IN"){
            @Override
            void start(){
                //intake pivot up
                //pull in
            }
        };
        new BotState("TRANSFER"){
            @Override
            void start(){
                //close servo on sample
                //check that i have the sample
                //if not return to idle
                //else send to outtake
            }
        };
        new BotState("OUTTAKE_UP"){
            @Override
            void start(){
                //move slides up until forearm is clear of obstacles
                //start turning forearm
                //send to outtake
            }
            @Override
            void update(){
                bot.setSlides(2000);
                bot.setForearm(0);
                //once slides hit target, send to OUTTAKE
            }
        };
        new BotState("OUTTAKE"){
            @Override
            void start(){
                //set vertical slides to top position
                //set forearm position to back
            }
            
            @Override
            void update(){
                //allow either gamepad to open claw
                //allow gpad2 to adjust forearm manually within min/max
            }
        };
        new BotState("OUTTAKE_DOWN"){
            @Override
            void start(){
                //bring forearm around
                //move claw to idle position
            }
            @Override
            void update(){
                //move slides down toward idle
                bot.setSlides(-50);
                bot.setForearm(1);
                //once down, transition to idle
            }
        };
        new BotState("HANG"){
            @Override
            void start(){
                //set vert slides to put hook above bar 
            }
            @Override
            void update(){
                //vertical slides manually pull down with stick on gpad2
            }
        };
    }
    
    public String toString(){
        return currentState.toString();
    }
    
}

