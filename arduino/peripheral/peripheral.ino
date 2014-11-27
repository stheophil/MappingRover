
#include <SPI.h>
#include <boards.h>
#include <RBL_nRF8001.h>

void setup()
{
  Serial.begin(57600);
  delay(3000);  //3 seconds delay for enabling to see the start up comments on the serial board
 
  // BLE setup
  ble_set_name("rcontrol2");
  ble_begin();
  
  Serial.println("BLE Arduino RobotController");
}

static byte buf_len = 0;

void ble_write_string(byte *bytes, uint8_t len)
{
  if (buf_len + len > 20)
  {
    for (int j = 0; j < 15000; j++)
      ble_do_events();
    
    buf_len = 0;
  }
  
  for (int j = 0; j < len; j++)
  {
    ble_write(bytes[j]);
    buf_len++;
  }
    
  if (buf_len == 20)
  {
    for (int j = 0; j < 15000; j++)
      ble_do_events();
    
    buf_len = 0;
  }  
}

bool bConnected = false;
void loop()
{
  if(ble_available())
  {
    byte cmd;
    cmd = ble_read();
    Serial.println(cmd);
    
    ble_do_events();
    return;
  } else if(ble_connected()!=bConnected) {
    bConnected=ble_connected();
    Serial.print("Connected = ");
    Serial.println(bConnected);
  }
  ble_do_events();
}

