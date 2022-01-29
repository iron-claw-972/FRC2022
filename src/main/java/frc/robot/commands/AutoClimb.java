package frc.robot.commands;

/*
    NOTE: NONE OF THIS IS FINAL UNTIL CLIMBER DESIGN IS FINALIZED

    In summary, autonomize:
    Rotate moving hooks very slightly away from bar
    Extend moving hooks
    Rotate moving hooks onto bar
    Rotate stationary hooks away from bar
    Rotate stationary hooks onto bar
    Rotate to bar to put stationary hooks on
    Rotate moving hooks 47 degrees
    Extend moving hooks
    Rotate moving hooks back
    Compress moving hooks (Might not be needed)
*/

public class AutoClimb {
    /*
        the plan for this climber autonomous:
        the driver presses d-pad right (preferably)
        the "step" of the autonomous processes increases
        if d-pad left is pressed, then it goes back a step
        we need to prevent double-press during the autonomous process
    */

    private int curStep = 0;
    private boolean stepPreventer = false;

    //when called, the step goes up
    public void stepIncreaser() {
        if(stepPreventer == false) {
            curStep++;
        }
    }

    //when called, the step decreases
    public void stepDecreaser() {
        if(stepPreventer == false) {
            curStep--;
        }
    }

    public AutoClimb() {
        /*step-by-step process of going through a routine.
        be sure to set stepPreventer to true at the start of these processes, but false at the end of them*/
        switch(curStep) {
            case 1:
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                break;
            case 5:
                break;
        }
    }
}