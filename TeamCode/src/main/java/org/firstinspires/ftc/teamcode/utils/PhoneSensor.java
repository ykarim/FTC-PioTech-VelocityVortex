package org.firstinspires.ftc.teamcode.utils;

/**
 * Makes it easier to use built-in phone sensors
 * @author Samuel "Red" Donovan
 * @version 11.26.16
 * @since 11/26/2016
 * @see <a href="https://github.com/saminjapan/FTC_phoneSensor"></a>
 * @instructions Copy-paste into your TeamCode/src/main/java/org.firsinspires.ftc.teamcode
 */

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;


public class PhoneSensor implements SensorEventListener{ //ONLY INT SENSORS

    /**
     * Stores values of sensor
     * @see <a href="https://developer.android.com/reference or android/hardware/SensorEvent.html#values">SensorEvent.values</a>
     */
    public float[] values;

    private float RTmp[] = new float[9];
    private float Rot[] = new float[9];
    private float I[] = new float[9];
    private float grav[] = new float[3];
    private float mag[] = new float[3];
    private float results[] = new float[3];
    protected float[] gravSensorVals;
    protected float[] magSensorVals;
    static final float ALPHA = 0.25f; // if ALPHA = 1 OR 0, no filter applies.

    /**Holds the sensor manager*/ //Maybe should Static, to prevent excess battery use???//
    private SensorManager sensorMngr;
    /**Holds the sensor's accuracy of the last reading*/
    private int sensorAccuraccy = 0;
    /**Holds the sensor's type*/
    private Sensor sensor;
    /**Holds context*/
    private Context c;


    /**
     * Constructor
     * @param sensorType The type of sensor as an int (See <a href="https://developer.android.com/reference or android/hardware/Sensor.html#Summary">Android Sensors</a> or android.hardware.Sensor)
     * @param delay The speed at which the sensor refreshes (faster = more battery usage) (See <a href="https://developer.android.com/reference or android/hardware/SensorManager.html#SENSOR_DELAY_FASTEST">SensorManager.SENSOR_DELAY_</a> or android.hardware.SensorManager)
     * @param hwMap The hardware map for the OpMode
     */
    public PhoneSensor(int sensorType, int delay, HardwareMap hwMap) {
        c = hwMap.appContext;
        sensorMngr = (SensorManager) c.getSystemService(Context.SENSOR_SERVICE);
        this.sensor = sensorMngr.getDefaultSensor(sensorType);
        sensorMngr.registerListener(this, this.sensor, delay);
    }

    /** Changes how often values refresh
     * @param delay The speed at which the sensor refreshes (See <a href="https://developer.android.com/reference or android/hardware/SensorManager.html#SENSOR_DELAY_FASTEST">SensorManager.SENSOR_DELAY_</a> or android.hardware.SensorManager)
     */
    public void setDelay(int delay){
        sensorMngr.unregisterListener(this);
        sensorMngr.registerListener(this, this.sensor, delay);
    }

    /**Stops listening (Not sure if it does this automatically so do it at end of OpMode just in case)*/
    public void unregister(){
        sensorMngr.unregisterListener(this);
    }

    /**Restarts the listening
     * @param sensorType The type of sensor (See <a href="https://developer.android.com/reference or android/hardware/Sensor.html#Summary">Android Sensors</a> or android.hardware.Sensor)
     * @param delay The speed at which the sensor refreshes (See <a href="https://developer.android.com/reference or android/hardware/SensorManager.html#SENSOR_DELAY_FASTEST">SensorManager.SENSOR_DELAY_</a> or android.hardware.SensorManager)
     */
    public void register(int sensorType, int delay) {
        this.sensor = sensorMngr.getDefaultSensor(sensorType);
        sensorMngr.registerListener(this, this.sensor, delay);
    }

    /**Holds weather or not to collect data*/
    private boolean collect = true;

    /**Holds requested accuracy*/
    private int collectAccuracy=sensorMngr.SENSOR_STATUS_ACCURACY_HIGH;

    /**Changes accuracy required to collect values
     * @param newAccuracy The accuracy required of a sensor reading for values to be updated (See <a href="https://developer.android.com/reference or android/hardware/SensorManager.html#SENSOR_STATUS_ACCURACY_HIGH">SensorManager.SENSOR_STATUS_ACCURACY_</a> or android.hardware.SensorManager}
     */
    public void setAccuraccy(int newAccuracy){
        this.collectAccuracy=newAccuracy;
    }

    /**Called when a sensor that is registered with a listener changes value(s)*/
    public void onSensorChanged(SensorEvent event) {
        if(collect)
            this.values=event.values;

        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            gravSensorVals = lowPass(event.values.clone(), gravSensorVals);
            grav[0] = event.values[0];
            grav[1] = event.values[1];
            grav[2] = event.values[2];
        } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            magSensorVals = lowPass(event.values.clone(), magSensorVals);
            mag[0] = event.values[0];
            mag[1] = event.values[1];
            mag[2] = event.values[2];
        }
        if (gravSensorVals != null && magSensorVals != null) {
            SensorManager.getRotationMatrix(RTmp, I, gravSensorVals, magSensorVals);
            int rotation = Compatibility.getRotation((Activity) c);
            if (rotation == 1) {
                SensorManager.remapCoordinateSystem(RTmp, SensorManager.AXIS_X, SensorManager.AXIS_MINUS_Z, Rot);
            } else {
                SensorManager.remapCoordinateSystem(RTmp, SensorManager.AXIS_Y, SensorManager.AXIS_MINUS_Z, Rot);
            }
            SensorManager.getOrientation(Rot, results);
            GyroUtils.azimuth = (float)(((results[0]*180)/Math.PI)+180);
            GyroUtils.pitch = (float)(((results[1]*180/Math.PI))+90);
            GyroUtils.roll = (float)(((results[2]*180/Math.PI)));
        }
    }

    /**Called when a sensor that is registered with a listener's accuracy is changed*/
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        if (accuracy==this.collectAccuracy)
            collect=true;
    }

    /**@return A list of available sensors on the phone*/
    public static String[] getAvaiableSensors(HardwareMap hwMap){
        SensorManager sensorManager = (SensorManager) hwMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        List<Sensor> sensorsList = sensorManager.getSensorList(Sensor.TYPE_ALL);
        String[] sArray = new String[sensorsList.size()+1];
        if(android.os.Build.VERSION.SDK_INT >= 20) { //If API is >=20, return fancy
            for (int p = 0; p < sArray.length - 1; p++)
                sArray[p] = "TYPE_"+sensorsList.get(p).getStringType().substring(15).toUpperCase();
        } else { //If not, return normal
            for (int p = 0; p < sArray.length - 1; p++)
                sArray[p] = sensorsList.get(p).getName();
        }
        return sArray;
    }

    /**The type of sensor as a comprehensible String
     * @return The type of sensor as a comprehensible String
     * @see <a href="https://developer.android.com/reference/android/hardware/Sensor.html#getStringType()">Sensor.getStringType()</a>
     * @see Sensor*/
    public String getStringType(){
        if(android.os.Build.VERSION.SDK_INT >= 20)
            return this.sensor.getStringType().substring(15).toUpperCase();
        return this.sensor.getName();
    }

    /**The type of sensor as the int equivalent of Sensor.TYPE_SENSOR_TYPE
     * @return The type of sensor as the int equivalent of Sensor.TYPE_SENSOR_TYPE
     * @see <a href="https://developer.android.com/reference/android/hardware/Sensor.html#getType()">Sensor.getType()</a>
     * @see Sensor*/
    public int getType(){
        return this.sensor.getType();
    }

    /**Vendor string of the sensor
     * @return Vendor string of the sensor
     * @see <a href="https://developer.android.com/reference/android/hardware/Sensor.html#getVendor()">Sensor.getVendor()</a>
     * @see Sensor*/
    public String getVendor(){
        return this.sensor.getVendor();
    }

    /**Power consumption of the sensor in mA
     * @return Power consumption of the sensor in mA
     * @see <a href="https://developer.android.com/reference/android/hardware/Sensor.html#getPower()">Sensor.getPower()</a>
     * @see Sensor*/
    public float getPower(){
        return this.sensor.getPower();
    }

    /**Resolution of the sensor in the sensor's units
     * @return Resolution of the sensor in the sensor's units
     * @see <a href="https://developer.android.com/reference/android/hardware/Sensor.html#getResolution()">Sensor.getResolution()</a>
     * @see Sensor*/
    public float getResolution(){
        return this.sensor.getResolution();
    }

    /**The minimum delay allowed between two events in microsecond or zero if this sensor only returns a value when the data it's measuring changes.
     * @return The minimum delay allowed between two events in microsecond or zero if this sensor only returns a value when the data it's measuring changes.
     * @see <a href="https://developer.android.com/reference/android/hardware/Sensor.html#getMinDelay()">Sensor.getMinDelay()</a>
     * @see Sensor
     */
    public int getMinDelay(){
        return this.sensor.getMinDelay();
    }
    /**Maximum range of the sensor in the sensor's unit.
     * @return Maximum range of the sensor in the sensor's unit.
     * @see <a href="https://developer.android.com/reference/android/hardware/Sensor.html#getMaximumRange()">Sensor.getMaximumRange()</a>
     * @see Sensor*/

    public float getMaximumRange(){
        return this.sensor.getMaximumRange();
    }
    /**A string representation of the object.
     * @return A string representation of the object.
     * @see <a href="https://developer.android.com/reference/android/hardware/Sensor.html#toString()">Sensor.toString()</a>
     * @see Sensor*/

    public String toString(){
        return sensor.toString();
    }

    private float[] lowPass(float[] input, float[] output) {
        if ( output == null )
            return input;
        for ( int i=0; i<input.length; i++ ) {
            output[i] = output[i] + ALPHA * (input[i] - output[i]);
        } return output;
    }

}