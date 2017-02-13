package org.firstinspires.ftc.robotcontroller.internal;

import android.app.Activity;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.text.InputType;
import android.text.method.NumberKeyListener;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.RadioGroup;
import android.widget.Spinner;

import com.qualcomm.ftcrobotcontroller.R;

public class AutonomousSettings extends Activity {

    private Spinner teamColorSpinner;
    private RadioGroup ballsToShoot;
    private RadioGroup beaconsToHit;
    private Spinner pushCapBall;
    private Spinner parkLocation;
    public EditText autoDelay;
    private Button doneButton;
    private Button cancelButton;

    private static final String PREFS_NAME = "AutoSettings";
    private SharedPreferences preferences;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_autonomous_settings);

        teamColorSpinner = (Spinner) findViewById(R.id.teamColorSpinner);
        ballsToShoot = (RadioGroup) findViewById(R.id.ballsToShoot);
        beaconsToHit = (RadioGroup) findViewById(R.id.beaconsToHit);
        pushCapBall = (Spinner) findViewById(R.id.push_cap_spinner);
        parkLocation = (Spinner) findViewById(R.id.park_location_spinner);
        autoDelay = (EditText) findViewById(R.id.autonomous_delay);
        doneButton = (Button) findViewById(R.id.doneButton);
        cancelButton = (Button) findViewById(R.id.cancelButton);

        preferences = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);
        setListeners();
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        new MenuInflater(this).inflate(R.menu.autonomous_settings, menu);
        return super.onCreateOptionsMenu(menu);
    }

    @Override
    protected void onSaveInstanceState(Bundle outState) {
        super.onSaveInstanceState(outState);

    }

    @Override
    protected void onRestoreInstanceState(Bundle savedInstanceState) {
        super.onRestoreInstanceState(savedInstanceState);
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        return super.onOptionsItemSelected(item);
    }

    @Override
    protected void onResume() {
        super.onResume();
    }

    @Override
    protected void onPause() {
        super.onPause();
    }

    @Override
    protected void onStop() {
        super.onStop();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
    }

    public void setListeners() {
        autoDelay.setKeyListener(new NumberKeyListener() {
            @Override
            protected char[] getAcceptedChars() {
                return new char[] {
                        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9'
                };
            }

            @Override
            public int getInputType() {
                return InputType.TYPE_CLASS_NUMBER;
            }
        });

        doneButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                SharedPreferences.Editor prefEditor = preferences.edit();

                prefEditor.putString("Team Color", teamColorSpinner.getSelectedItem().toString());

                int ballsToShootId = ballsToShoot.getCheckedRadioButtonId();
                if (ballsToShootId == R.id.zeroBalls) {
                    prefEditor.putInt("Balls", 0);
                } else if (ballsToShootId == R.id.oneBall) {
                    prefEditor.putInt("Balls", 1);
                } else if (ballsToShootId == R.id.twoBalls) {
                    prefEditor.putInt("Balls", 2);
                }

                int beaconsToHitId = beaconsToHit.getCheckedRadioButtonId();
                if (beaconsToHitId == R.id.zeroBeacons) {
                    prefEditor.putInt("Beacons", 0);
                } else if (beaconsToHitId == R.id.oneBeacon) {
                    prefEditor.putInt("Beacons", 1);
                } else if (beaconsToHitId == R.id.twoBeacons) {
                    prefEditor.putInt("Beacons", 2);
                }

                if (("Yes").equals(pushCapBall.getSelectedItem().toString())) {
                    prefEditor.putBoolean("Push Cap Ball", true);
                } else if (("No").equals(pushCapBall.getSelectedItem().toString())) {
                    prefEditor.putBoolean("Push Cap Ball", false);
                }

                if (("Corner Vortex").equals(parkLocation.getSelectedItem().toString())) {
                    prefEditor.putString("Park Location", "Corner");
                } else if (("Center Vortex").equals(parkLocation.getSelectedItem().toString())) {
                    prefEditor.putString("Park Location", "Center");
                } else if (("NA").equals(parkLocation.getSelectedItem().toString())) {
                    prefEditor.putString("Park Location", "None");
                }

                prefEditor.putInt("Delay", Integer.valueOf(autoDelay.getText().toString()));
                prefEditor.apply();
                finish();
            }
        });

        cancelButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                finish();
            }
        });
    }
}
