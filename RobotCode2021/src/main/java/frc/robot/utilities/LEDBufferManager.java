package frc.robot.utilities;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * <h4>LEDBufferManager</h4>
 * <p>
 * Utility class to manage and manipulate buffers to change the LEDs.
 * </p>
 * 
 * <p>
 * Manipulates buffer to change the LEDs. Buffer can access each LED on the strip and 
 * receive color values to make them light up. Class contains methods that will show a 
 * certain pattern on the strip when another LED class runs. Other LED classes run when 
 * certain actions/conditions are applied to the robot
 * </p>
 * 
 * <p>
 * TODO: need to test setting color outside buffer size
 * </p>
 */
public class LEDBufferManager {

    //Arrays storing the HSV values for each color
    //  HSV: hue, saturation, value
    private final int[] red;
    private final int[] red_orange;
    private final int[] orange;
    private final int[] yellow_orange;
    private final int[] gold;
    private final int[] yellow;
    private final int[] yellow_green;
    private final int[] green;
    private final int[] teal;
    private final int[] blue;
    private final int[] purple;
    private final int[] violet;
    private final int[] white;
    private final int[] off;

    //Patterns can change colors based on alliance
    private final int redTeam;
    private final int blueTeam;

    //HSV variables represent the different indexes in an array
    private final int HUE;
    private final int SAT;
    private final int VAL;

    //buffer will be changed in each method to change the LEDs
    private AddressableLEDBuffer buffer;

    /**
     * <h4>LEDBufferManager</h4>
     * Creates a utility class to manage and manipulate buffers to change the LEDs.
     * 
     * Manipulates buffer to change the LEDs. Buffer can access each LED on the strip and 
     * receive color values to make them light up. Class contains methods that will show a 
     * certain pattern on the strip when another LED class runs. Other LED classes run when 
     * certain actions/conditions are applied to the robot  
     * 
     * @param bufferLength
     */
    public LEDBufferManager(int bufferLength) {
        buffer = new AddressableLEDBuffer(bufferLength);
        HUE = 0;
        SAT = 1;
        VAL = 2;
        redTeam = 0;
        blueTeam = 1;
        //need to update HSV values
        red = new int[] {0, 255, 200};
        red_orange = new int[] {2500, 255, 255};
        orange = new int[] {4000, 255, 255};
        yellow_orange = new int[] {7000, 255, 255};
        gold = new int[] {26760, 255, 255};
        yellow = new int[] {10800, 255, 255};
        yellow_green = new int[] {28, 50, 0};
        green = new int[] {0, 50, 0};
        teal = new int[] {0, 50, 25};
        blue = new int[] {43690, 255, 255};
        purple = new int[] {25, 0, 50};
        violet = new int[] {30, 15, 40};
        white = new int[] {0, 0, 255};
        off = new int[] {0, 0, 0};
    }

    /**
     * <h4>getColor</h4>
     * The color handed in will set the HSV value to an empty array. This array will be
     * returned with the corresponding HSV values
     * 
     * @param colorInput Color to get HSV values for
     */
    private int[] getColor(String colorInput) {
        int[] color;
        if (colorInput.equalsIgnoreCase("red")) {
            color = red;
        }
        if (colorInput.equalsIgnoreCase("red_orange")) {
            color = red_orange;
        }
        if (colorInput.equalsIgnoreCase("orange")) {
            color = orange;
        }
        if (colorInput.equalsIgnoreCase("yellow_orange")) {
            color = yellow_orange;
        }
        if (colorInput.equalsIgnoreCase("gold")) {
            color = gold;
        }
        if (colorInput.equalsIgnoreCase("yellow")) {
            color = yellow;
        }
        if (colorInput.equalsIgnoreCase("yellow_green")) {
            color = yellow_green;
        }
        if (colorInput.equalsIgnoreCase("green")) {
            color = green;
        }
        if (colorInput.equalsIgnoreCase("teal")) {
            color = teal;
        }
        if (colorInput.equalsIgnoreCase("blue")) {
            color = blue;
        }
        if (colorInput.equalsIgnoreCase("purple")) {
            color = purple;
        }
        if (colorInput.equalsIgnoreCase("violet")) {
            color = violet;
        }
        if (colorInput.equalsIgnoreCase("white")) {
            color = white;
        }
        else {
            color = off;
        }
        return color;
    }

    /**
     * <h4>colorSet</h4>
     * Sets the color of an LED with a given position and String for color
     * 
     * @param position
     * @param _color
     */
    private void colorSet(int position, String _color) {
        int[] color = this.getColor(_color);
        buffer.setHSV(position, color[HUE], color[SAT], color[VAL]);
    }

    /**
     * <h4>clear</h4>
     * Turns off all LEDs.
     */
    private void clear() {
        for (int i = 0; i < buffer.getLength(); i++) {
            this.colorSet(i, "off");
        }
    }

    /**
     * <h4>allianceSet</h4>
     * Shows which alliance our team is on by turning all the LEDs to the alliance's color
     * 
     * @param _team
     */
    public AddressableLEDBuffer allianceSet(int _team) {
        String team;
        team = (_team == redTeam) ? "red" : "blue";
        for (int i = 0; i < buffer.getLength(); i++) {
            this.colorWipe(i, team);
        }
        return buffer;
    }

    /**
     * <h4>colorWipe</h4>
     * Sets all LEDs to a single color one by one
     * 
     * @param counter
     * @param color
     * @return
     */
    public AddressableLEDBuffer colorWipe(int counter, String color) {
        if (counter < buffer.getLength()) {
            this.colorSet(counter, color);
            counter++;
        }
        return buffer;
    }

    /**
     * <h4>movingSegmentLoop</h4>
     * Places a segment of lit LEDs along strip, then starts over from the beginning
     * 
     * @param counter
     * @param color
     * @param segLength
     * @return
     */
    public AddressableLEDBuffer movingSegmentLoop(int counter, String color, int segLength) {
        if (counter < buffer.getLength()) {
            this.colorSet(counter, color);
        } else {
            this.colorSet((counter) - buffer.getLength(), color);
        }
        return buffer;
    }

    /**
     * <h4>movingSegmentPong</h4>
     * Same as movingSegmentLoop, but instead of starting over from beginning, it rebounds
     * back and goes the other way
     * 
     * @param counter
     * @param color Color of the bouncing segment
     * @param segLength Length of the bouncing segment
     */
    public AddressableLEDBuffer movingSegmentPong(int counter, String color, int segLength) {
        if (counter < buffer.getLength() - segLength) {
            for (int j = 0; j < segLength; j++) {
                this.colorSet(j + counter, color);
                if(j + counter > 0) {
                    this.colorSet(j + counter - 1, "off");
                }
            }
        } 
        else if(counter < 2*(buffer.getLength() - segLength)) {
            for (int j = 0; j < segLength; j++) {
                this.colorSet(counter - j, color);
                if(counter - j < buffer.getLength()) {
                    this.colorSet(j + counter + 1, "off");
                }
            }
        }
        else {
            for(int i = 0; i < buffer.getLength(); i++){
                colorSet(i, "off");
            }
        }
        return buffer;
    }

    /**
     * <h4>twoColorAlt</h4>
     * Alternates between two colors down the strip
     * 
     * @param color1
     * @param color2
     * @return
     */
    public AddressableLEDBuffer twoColorAlt(String color1, String color2) {
        String[] twoColors = new String[] {color1, color2};
        //Changes between both colors
        for(int i = 0; i < buffer.getLength(); i++) {
            colorSet(i, twoColors[i%2]);
        }
        return buffer;
    }

    /**
     * <h4>twoColorAnim</h4>
     * Uses twoColorAlt to swap the positions of the alternating colors
     * 
     * @param color1
     * @param color2
     * @return
     */
    public AddressableLEDBuffer twoColorAnim(String color1, String color2) {
        //Shifts pattern back and forth
        this.twoColorAlt(color1, color2);
        this.twoColorAlt(color2, color1);
        return buffer;
    }

    /**
     * <h4>theaterChase</h4>
     * Lights every third LED along the strip at a given position
     * 
     * @param counter
     * @param color
     * @return
     */
    public AddressableLEDBuffer theaterChase(int counter, String color) {
        //Shifts pattern over
        for (int i = counter; i < buffer.getLength(); i += 3) {
                this.colorSet(i, color);
        }
        return buffer;
    }

    /**
     * <h4>rainbowWheel</h4>
     * Color wheel that rotates about the LED strip
     * 
     * @param counter
     * @return
     */
    public AddressableLEDBuffer rainbowWheel(int counter) {
        int dif = 65536/buffer.getLength();
        if(counter < buffer.getLength()) {
            //Adjusts color slowly
            for (int j = 0; j < dif; j += 1000) {
                //Fills all LEDs in the strip
                for (int i = 0; i < buffer.getLength(); i++) {
                    //Wraps pattern to create a "wheel"
                    if (i - counter >= 0) {
                        buffer.setHSV(i - counter, (i * dif) + j, 255, 255);
                    }
                    else {
                        buffer.setHSV((i - counter) + buffer.getLength(), (i * dif) + j, 255, 255);
                    }
                }
            }
        }
        return buffer;
    }

    //TODO: Will be worked on

        //Input a value 0 to 255 to get a color value.
        //The colours are a transition r - g - b - back to r.
    /*public int Wheel(byte WheelPos) {
        WheelPos = 255 - WheelPos;
        if(WheelPos < 85) {
            return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
        }
        if(WheelPos < 170) {
            WheelPos -= 85;
            return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
        }
        WheelPos -= 170;
        return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
        }*/

    /*public AddressableLEDBuffer theaterChaseRainbow(int wait) {
        for (int j = 0; j < 256; j++) {      //cycle all 256 colors in the wheel
            for (int q = 0; q < 3; q++) {
                for (int i = 0; i < buffer.getLength(); i += 3) {
                    Wheel((i + j) % 255);
                }
                delay(wait);
                for (int i = 0; i < buffer.getLength(); i += 3) {
                    colorSet(i + q, "off");        //turn every third pixel off
                }
            }
        }
        return buffer;
    }*/
};