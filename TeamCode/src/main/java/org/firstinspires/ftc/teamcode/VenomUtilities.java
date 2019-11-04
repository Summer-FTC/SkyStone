package org.firstinspires.ftc.teamcode;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.HashMap;
import java.util.Properties;

public class VenomUtilities {


    static HashMap lastValues = new HashMap();
    /** This function helps to find out if the gamepad button value changed or not.
     * This helps to avoid seeing the same button press each loop, so that you only see
     * the transition from on to off, or off to on.
     * Example:
     *  void loop () {
     *      if (isValueChanged("1.x",gamepad1.x)) {
     *          if (gamepad1.x) ... // button just depressed, won't trigger again until released and pressed again
     *
     *      }
     *  }
     * @param key
     * @param newValue
     * @return
     */
    public static boolean isValueChanged(String key, Object newValue) {
        boolean valueChanged=false;
        Object lastValue=lastValues.get(key);
        if (lastValue!=null && lastValue!=newValue) valueChanged=true;
        lastValues.put(key,newValue);  // store new value so that we know if it changed next time.
        return valueChanged;
    }

    /** wrapper function to check if value was changed AND also equal to a given target
     *
     * @param key
     * @param newValue
     * @param targetValue
     * @return
     */
    public static boolean isValueChangedAndEqualTo(String key, Object newValue, Object targetValue) {
        return (isValueChanged(key,newValue) && newValue==targetValue);
    }
    static public Properties props = new Properties();

    public static void readProperties(){
        InputStream propertiesFile = null;
        try {
            File file = new File("Moto E (4)\\Internal Storage\\Download\\VenomRobot.properties");
            propertiesFile = new FileInputStream(file);
            props.load(propertiesFile);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            try{
                if(propertiesFile !=null){
                    propertiesFile.close();
                }
            } catch(IOException e){
                e.printStackTrace();
            }
        }
    }

    public static void writeProperties(){
        OutputStream propertiesFile = null;
        try {
            File file = new File("Moto E (4)\\Internal Storage\\Download\\VenomRobot.properties");
            propertiesFile = new FileOutputStream(file);
            props.store(propertiesFile,"Venom 6209");
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            try{
                if(propertiesFile !=null){
                    propertiesFile.close();
                }
            } catch(IOException e){
                e.printStackTrace();
            }
        }
    }

    public static double getPropDouble(String propertyName, Double defaultValue){
        try{
            return Double.parseDouble(props.getProperty(propertyName, defaultValue.toString()));
        } catch (Exception e){

            return defaultValue;
        }

    }
    public static int getPropInteger(String propertyName, int defaultValue){
        try{
            return Integer.parseInt(props.getProperty(propertyName, Integer.toString(defaultValue)));
        } catch (Exception e){

            return defaultValue;
        }

    }
    public static boolean getPropBoolean(String propertyName, boolean defaultValue){
        try{
            return Boolean.parseBoolean(props.getProperty(propertyName, Boolean.toString(defaultValue)));
        } catch (Exception e){

            return defaultValue;
        }

    }

    public static void setPropDouble(String propertyName, Double value){
        props.setProperty(propertyName, value.toString());
    }
    public static void setPropInteger(String propertyName, int value) {
        props.setProperty(propertyName, Integer.toString(value));
    }
    public static void setPropBoolean(String propertyName, boolean value){
        props.setProperty(propertyName, Boolean.toString(value));
    }

    /**
     * Clamps a value to a given range.
     * @param value The value to clamp.
     * @param min The min clamp.
     * @param max The max clamp.
     * @return The clamped value.
     */
    public static double clampValue(double value, double min, double max) {
        return Math.min(max, Math.max(min, value));
    }
}
