package frc.robot.utilities;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.commands.colorwheelcommands.positionalcontrolcommands.PositionalControlTrackerCommand;
import frc.robot.subsystems.ColorSensorSubsystem;


import org.junit.Ignore;
import org.junit.Test;


public class PositionalControlTrackerCommandTest {

    ColorSensorSubsystem subsystem = mock(ColorSensorSubsystem.class);
    PositionalControlTrackerCommand positionalControlTrackerCommand = new PositionalControlTrackerCommand(subsystem);

    @Test 
    @Ignore
    public void getNearestColorTest() {
        //all color tests need to run initalized
        positionalControlTrackerCommand.initialize();

        //Color wheel order and makes some colors to test
        Color blueColor = new Color(0.143, 0.427, 0.429); //Color Blue
        Color greenColor = new Color(0.197, 0.561, 0.240); //Color Green
        Color redColor = new Color(0.561, 0.232, 0.114); //Color Red
        Color yellowColor = new Color(0.361, 0.524, 0.113); //Color Yellow
        Color blackColor = new Color(0.0, 0.0, 0.0); //Color Black
        Color whiteColor = new Color(0.333, 0.333, 0.333); //Color White

        //when class getSensorColor() gets called in ColorSensorSubsystem.class, makes it return Blue for the test
        when(subsystem.getSensorColor()).thenReturn(blueColor);
        String outputString = positionalControlTrackerCommand.getNearestColor(blueColor); //tests color Blue with getNearestColor
        assertEquals("Blue", outputString); //checks that the color Blue is returned when blueColor is sent through

        //when class getSensorColor() gets called in ColorSensorSubsystem.class, makes it return Red for the test
        when(subsystem.getSensorColor()).thenReturn(redColor);
        outputString = positionalControlTrackerCommand.getNearestColor(redColor); //tests color Red with getNearestColor
        assertEquals("Red", outputString); //checks that the color Red is returned when redColor is sent through

        //when class getSensorColor() gets called in ColorSensorSubsystem.class, makes it return Green for the test
        when(subsystem.getSensorColor()).thenReturn(greenColor);
        outputString = positionalControlTrackerCommand.getNearestColor(greenColor); //tests color Green with getNearestColor
        assertEquals("Green", outputString); //checks that the color Green is returned when greenColor is sent through

        //when class getSensorColor() gets called in ColorSensorSubsystem.class, makes it return Yellow for the test
        when(subsystem.getSensorColor()).thenReturn(yellowColor);
        outputString = positionalControlTrackerCommand.getNearestColor(yellowColor); //tests color Yellow with getNearestColor
        assertEquals("Yellow", outputString); //checks that the color Yellow is returned when yellowColor is sent through

        //when class getSensorColor() gets called in ColorSensorSubsystem.class, makes it return Unknown for the test
        // when(subsystem.getSensorColor()).thenReturn(blackColor);
        // outputString = positionalControlTrackerCommand.getNearestColor(blackColor); //tests color Black with getNearestColor
        // assertEquals("Unknown", outputString); //checks that the color Black is returned when blackColor is sent through

        // //when class getSensorColor() gets called in ColorSensorSubsystem.class, makes it return Unknown for the test
        // when(subsystem.getSensorColor()).thenReturn(whiteColor);
        // outputString = positionalControlTrackerCommand.getNearestColor(whiteColor); //tests color White with getNearestColor
        // assertEquals("Unknown", outputString); //checks that the color White is returned when whiteColor is sent through
    }
}