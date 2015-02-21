char report[80];
static bool bFirstRun = true;

void loop() {
    if(bFirstRun) {
        Serial.println("AHRS Test");
        
        OnConnection(); // calibrate with running motors to test with interference
        for(int i=0; i<countof(g_amotors); ++i) {
            analogWrite(g_amotors[i].POWER, 128);
        }
        bFirstRun = false;
    } else if(updateAHRS()) {
        
        snprintf(report, sizeof(report), "roll: %+6d pitch: %+6d yaw: %+6d", g_nRoll, g_nPitch, g_nYaw);
        Serial.println(report);
    }
}