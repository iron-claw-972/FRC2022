package frc.robot.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

import org.junit.Test;

public class BeltTest {
    
    private Belt belt = new Belt();
    
    @Test
    public void BeltInherentlyEnabled() {
        assertFalse(belt.isEnabled());
    }
    
    @Test 
    public void BeltPowerCorrect(){
        belt.setPower(0.5); 
        assertEquals(0.5, belt.getPower(),0);
    }
}