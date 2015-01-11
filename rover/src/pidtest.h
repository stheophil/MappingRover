
// PID Test
bool bFirstRun = true;

static const int g_anSpeeds[] = { 200, 500, 100, 300, 0, 400 };
static const unsigned long g_nCycle = 5000;

unsigned long g_nLastChange = 0;
int g_iSpeed = 0;

float g_afTicksPerSecond[] = { 0.0, 0.0, 0.0, 0.0 };

void loop() {
    if(bFirstRun) {
        OnConnection();
        
        Serial.println("SetSpeed, M0 Ticks/s, M1 Ticks/s, M2 Ticks/s, M3 Ticks/s, M0 PWR, M1 PWR, M2 PWR, M3 PWR");
        
        for(int i=0; i<countof(g_amotors); ++i) {
            g_amotors[i].m_fSpeed = 200.0;
            g_amotors[i].m_fPower = 0.0;
            g_amotors[i].m_nLastCompute = millis();
            
            digitalWrite(g_amotors[i].DIR, HIGH);
            analogWrite(g_amotors[i].POWER, (int)g_amotors[i].m_fPower);
            g_apid[i].SetMode(AUTOMATIC);
        }
        delay(200);
        
        g_nLastChange = millis();
        bFirstRun = false;
    } else {
        if(millis() - g_nLastChange >= g_nCycle) {
            ++g_iSpeed;
            g_nLastChange = millis();
            if( g_iSpeed < countof(g_anSpeeds) ) {
                for(int i=0; i<countof(g_amotors); ++i) {
                    g_amotors[i].m_fSpeed = g_anSpeeds[g_iSpeed];
                }
            } else if(g_iSpeed == countof(g_anSpeeds)) { // turn left
                for(int i=0; i<countof(g_amotors); ++i) {
                    analogWrite(g_amotors[i].POWER, (int)0);
                }
                delay(200);
                
                g_amotors[0].SetSpeed(-200);
                g_amotors[2].SetSpeed(-200);
                
                g_amotors[1].SetSpeed(200);
                g_amotors[3].SetSpeed(200);
            } else if(g_iSpeed == countof(g_anSpeeds) + 1) { // turn right
                for(int i=0; i<countof(g_amotors); ++i) {
                    analogWrite(g_amotors[i].POWER, (int)0);
                }
                delay(200);
                
                g_amotors[0].SetSpeed(300);
                g_amotors[2].SetSpeed(300);
                
                g_amotors[1].SetSpeed(-300);
                g_amotors[3].SetSpeed(-300);
            }
        }

        bool bPrint = false;
        for(int i=0; i<countof(g_amotors); ++i) {
            if( g_amotors[i].ComputePID(g_apid[i]) ) {
                g_afTicksPerSecond[i] = g_amotors[i].m_fTicksPerSecond;
                bPrint = true;
            }
        }
        
        if(bPrint) {
            Serial.print(g_amotors[0].m_fSpeed);
            Serial.print(", ");
            for(int i=0; i<countof(g_amotors); ++i) {
                Serial.print(g_afTicksPerSecond[i]);
                Serial.print(", ");
            }
            
            for(int i=0; i<countof(g_amotors); ++i) {
                Serial.print(g_amotors[i].m_fPower);
                Serial.print(", ");
            }
            Serial.println("");
        }
    }
}