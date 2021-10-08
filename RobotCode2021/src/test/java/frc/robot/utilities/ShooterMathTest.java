package frc.robot.utilities;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static frc.robot.utilities.ShooterMath.ShotOutcome;

import org.junit.Test;

/**
 * <h4> ShooterMathTest </h4>
 * Tests the ShooterMath utility.
 */
public class ShooterMathTest {

    @Test
    public void test20_10() {
        ShooterMath shooterMathClass = ShooterMath.getInstance();
        // Values
        shooterMathClass.setPosition(20, 10);
        assertEquals(21.27769, shooterMathClass.getVelocity(), 0.0001);
        assertSame(shooterMathClass.getPossibleShot(), ShotOutcome.INNER);
    }

    @Test
    public void test40_8() {
        ShooterMath shooterMathClass = ShooterMath.getInstance();
        // Values
        shooterMathClass.setPosition(40, 8);
        assertEquals(11.48286, shooterMathClass.getVelocity(), 0.0001);
        assertSame(shooterMathClass.getPossibleShot(), ShotOutcome.INNER);
    }

    @Test
    public void test40_3() {
        ShooterMath shooterMathClass = ShooterMath.getInstance();
        // Values
        shooterMathClass.setPosition(40, 3);
        assertEquals(13.46558, shooterMathClass.getVelocity(), 0.0001);
        assertSame(shooterMathClass.getPossibleShot(), ShotOutcome.NONE);
    }

    //Tests for number extremes

    @Test
    public void test90_8() {
        ShooterMath shooterMathClass = ShooterMath.getInstance();
        // Values
        shooterMathClass.setPosition(90, 8);
        assertEquals(-1.0, shooterMathClass.getVelocity(), 0.0001);
        assertSame(shooterMathClass.getPossibleShot(), ShotOutcome.NONE);
    }

    @Test
    public void test91_8() {
        ShooterMath shooterMathClass = ShooterMath.getInstance();
        // Values
        shooterMathClass.setPosition(91, 8);
        assertEquals(-1.0, shooterMathClass.getVelocity(), 0.0001);
        assertSame(shooterMathClass.getPossibleShot(), ShotOutcome.NONE);
    }
    
    @Test
    public void test0_8() {
        ShooterMath shooterMathClass = ShooterMath.getInstance();
        // Values
        shooterMathClass.setPosition(0, 8);
        assertEquals(-1.0, shooterMathClass.getVelocity(), 0.0001);
        assertSame(shooterMathClass.getPossibleShot(), ShotOutcome.NONE);
    }

    @Test
    public void testneg1_8() {
        ShooterMath shooterMathClass = ShooterMath.getInstance();
        // Values
        shooterMathClass.setPosition(-1, 8);
        assertEquals(-1.0, shooterMathClass.getVelocity(), 0.0001);
        assertSame(shooterMathClass.getPossibleShot(), ShotOutcome.NONE);
    }
    
    @Test
    public void test25_100() {
        ShooterMath shooterMathClass = ShooterMath.getInstance();
        // Values
        shooterMathClass.setPosition(25, 100);
        assertEquals(-1.0, shooterMathClass.getVelocity(), 0.0001);
        assertSame(shooterMathClass.getPossibleShot(), ShotOutcome.NONE);
    }

    @Test
    public void test25_0() {
        ShooterMath shooterMathClass = ShooterMath.getInstance();
        // Values
        shooterMathClass.setPosition(25, 0);
        assertEquals(-1.0, shooterMathClass.getVelocity(), 0.0001);
        assertSame(shooterMathClass.getPossibleShot(), ShotOutcome.NONE);
    }
    
    @Test
    public void test25_neg1() {
        ShooterMath shooterMathClass = ShooterMath.getInstance();
        // Values
        shooterMathClass.setPosition(25, -1);
        assertEquals(-1.0, shooterMathClass.getVelocity(), 0.0001);
        assertSame(shooterMathClass.getPossibleShot(), ShotOutcome.NONE);
    }
}