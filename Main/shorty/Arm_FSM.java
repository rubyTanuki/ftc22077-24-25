package Main.shorty;

import Main.*;
import Main.auton.*;
import org.firstinspires.ftc.teamcode.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.HashMap;

public class Arm_FSM
{
    private Bot bot;
    private BotState currentState;

    public Arm_FSM(Bot bot){
        this.bot = bot;
        init();
        currentState = BotState.get("DEFAULT");
    }

    public void start(){
        currentState.start(); 
    }
    public void update(){
        currentState.update(); 
    }
    public void end(){
        currentState.end(); 
    }

    private void init(){
        new BotState("NEW_STATE"){
            @Override
            void start(){

            }
            @Override
            void update(){

            }
        };
    }
    
}

