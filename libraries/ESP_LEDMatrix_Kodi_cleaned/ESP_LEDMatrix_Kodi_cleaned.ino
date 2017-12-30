/*
  ESP-01 pinout from top:

  GND    GP2 GP0 RX/GP3
  TX/GP1 CH  RST VCC

  MAX7219
  ESP-1 from rear
  Re Br Or Ye
  Gr -- -- --

  USB to Serial programming
  ESP-1 from rear, FF to GND, RR to GND before upload
  Gr FF -- Bl
  Wh -- RR Vi

  GPIO 2 - DataIn
  GPIO 1 - LOAD/CS
  GPIO 0 - CLK

  ------------------------
  NodeMCU 1.0 pinout:

  D8 - DataIn
  D7 - LOAD/CS
  D6 - CLK

*/


#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>

WiFiClient client;

int dx = 0, dy = 0;
int h, m, s;
float elapsed;
int elapsedH, elapsedM, elapsedS;
int totalH, totalM, totalS;
int state;

#define NUM_MAX 6

// for ESP-01 module
//#define DIN_PIN 2 // D4
//#define CS_PIN  3 // D9/RX
//#define CLK_PIN 0 // D3

// for NodeMCU 1.0
#define DIN_PIN 15  // D8
#define CS_PIN  13  // D7
#define CLK_PIN 12  // D6

#include "max7219.h"
#include "fonts.h"

const char* ssid     = "yourssid";     // SSID of local network
const char* password = "yourpasswd";   // Password on network
// change Kodi IP in the code below

// =======================================================================

void setup() {
  Serial.begin(115200);
  initMAX7219();
  sendCmdAll(CMD_SHUTDOWN, 1);
  sendCmdAll(CMD_INTENSITY, 0);
  Serial.print("Connecting to WiFi ");
  WiFi.begin(ssid, password);
  printStringWithShift("Connecting", 15);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected. MyIP: "); Serial.println(WiFi.localIP());
  clr();
  printStringWithShift((String("MyIP: ")+WiFi.localIP().toString()).c_str(), 20);
  getTime();
  delay(600);
}

// =======================================================================

void loop()
{
  //Serial.println("Getting data ...");

  int ret1 = 0, ret2 = 0;
  //ret1 = getKodi("192.168.1.129");
  ret2=getKodi("192.168.1.99");
  //ret1=1;
  if (!ret1 && !ret2) {
    clr();
    printStringWithShift("No Kodi !!! ", 15);
    delay(3000);
  } else {
    //elapsedH=0;  elapsedM=25; totalH=3; totalM=55; state=1; elapsed = 67;
    updateTime();
    showElapsed();
    //showElapsedOld();
    delay(1000);
  }
}

// =======================================================================

int printVal(int val, int i)
{
  int r0 = val / 100;
  int r1 = val / 10 - (val / 100) * 10;
  int r2 = val % 10;
  if (r0) {
    showDigit(r0,  i, dig3x5);
    i += (r0 == 1) ? 3 : 4;
  }
  if (r1 || r0) {
    showDigit(r1,  i, dig3x5);
    i += (r1 == 1) ? 3 : 4;
  }
  showDigit(r2, i, dig3x5);
  i += (r2 == 1) ? 3 : 4;
  return i;
}

// =======================================================================

void showElapsed()
{
  unsigned int total = totalH * 3600 + totalM * 60 + totalS;
  unsigned int elaps = elapsedH * 3600 + elapsedM * 60 + elapsedS;
  unsigned int remain = (total - elaps + 30) / 60;
  total = (total + 30) / 60;
  elaps = (elaps + 30) / 60;

  dx = dy = 0;
  clr();
  if (total == 0) {    // stop
    scr[0] = B01110000;
    scr[1] = B01110000;
    scr[2] = B01110000;
  } else if (state == 1) {    // play
    scr[0] = B11111000;
    scr[1] = B01110000;
    scr[2] = B00100000;
  } else if (state > 1) {    // ff
    scr[0] = B10001000;
    scr[1] = B01010000;
    scr[2] = B00100000;
  } else if (state < 0) {    // rew
    scr[0] = B00100000;
    scr[1] = B01010000;
    scr[2] = B10001000;
  } else if (state == 0) {    // pause
    scr[0] = B01110000;
    scr[1] = B00000000;
    scr[2] = B01110000;
  }
  if (total > 0)  {
    int x = remain >= 200 ? 4 : 5;
    //x=printVal(remain,x);
    x=printVal(elaps,x);
    scr[x++] = B00100000;
    scr[x++] = B00100000;
    x++;
    //x=printVal(total,x);
    x=printVal(remain,x);
  }

  if (h / 10)
    showDigit(h / 10, h/10==1 ? 32 : 31, dig3x5);
  showDigit(h % 10, 35, dig3x5);
  showDigit(m / 10, 41, dig3x5);
  showDigit(m % 10, 45, dig3x5);
  setCol(39, B01010000);

  for (int i = 0; i < NUM_MAX * 8; i++) {
    byte val = i & 1 ? 0x80 : 0;
    if (i < elapsed * NUM_MAX * 8 / 100) val = 0xc0;
    scr[i] = (scr[i] >> 3) | val;
  }

  refreshAll();
}

// =======================================================================

void showElapsedOld()
{
  dx = dy = 0;
  clr();
  showDigit(elapsedH / 10,  0, dig3x5);
  showDigit(elapsedH % 10,  4, dig3x5);
  showDigit(elapsedM / 10, 10, dig3x5);
  showDigit(elapsedM % 10, 14, dig3x5);

  showDigit(totalH / 10, 20, dig3x5);
  showDigit(totalH % 10, 24, dig3x5);
  showDigit(totalM / 10, 30, dig3x5);
  showDigit(totalM % 10, 34, dig3x5);
  int el = (int)elapsed;
  showDigit(el / 10, 41, dig3x5);
  showDigit(el % 10, 45, dig3x5);
  setCol(8, B01010000);
  setCol(28, B01010000);

  for (int i = 0; i < NUM_MAX * 8; i++) {
    byte val = i & 1 ? 0x80 : 0;
    if (i < elapsed * NUM_MAX * 8 / 100) val = 0xc0;
    scr[i] = (scr[i] >> 3) | val;
  }

  refreshAll();
}

// =======================================================================

void showDigit(char ch, int col, const uint8_t *data)
{
  if (dy < -8 | dy > 8) return;
  int len = pgm_read_byte(data);
  int w = pgm_read_byte(data + 1 + ch * len);
  col += dx;
  for (int i = 0; i < w; i++)
    if (col + i >= 0 && col + i < 8 * NUM_MAX) {
      byte v = pgm_read_byte(data + 1 + ch * len + 1 + i);
      if (!dy) scr[col + i] = v; else scr[col + i] |= dy > 0 ? v >> dy : v << -dy;
    }
}

// =======================================================================

void setCol(int col, byte v)
{
  if (dy < -8 | dy > 8) return;
  col += dx;
  if (col >= 0 && col < 8 * NUM_MAX)
      if (!dy) scr[col] = v; else scr[col] |= dy > 0 ? v >> dy : v << -dy;
}

// =======================================================================

int showChar(char ch, const uint8_t *data)
{
  int len = pgm_read_byte(data);
  int i, w = pgm_read_byte(data + 1 + ch * len);
  for (i = 0; i < w; i++)
    scr[NUM_MAX * 8 + i] = pgm_read_byte(data + 1 + ch * len + 1 + i);
  scr[NUM_MAX * 8 + i] = 0;
  return w;
}

// =======================================================================
int dualChar = 0;

unsigned char convertPolish(unsigned char _c)
{
  unsigned char c = _c;
  if (c == 196 || c == 197 || c == 195) {
    dualChar = c;
    return 0;
  }
  if (dualChar) {
    switch (_c) {
      case 133: c = 1 + '~'; break; // 'ą'
      case 135: c = 2 + '~'; break; // 'ć'
      case 153: c = 3 + '~'; break; // 'ę'
      case 130: c = 4 + '~'; break; // 'ł'
      case 132: c = dualChar == 197 ? 5 + '~' : 10 + '~'; break; // 'ń' and 'Ą'
      case 179: c = 6 + '~'; break; // 'ó'
      case 155: c = 7 + '~'; break; // 'ś'
      case 186: c = 8 + '~'; break; // 'ź'
      case 188: c = 9 + '~'; break; // 'ż'
      //case 132: c = 10+'~'; break; // 'Ą'
      case 134: c = 11 + '~'; break; // 'Ć'
      case 152: c = 12 + '~'; break; // 'Ę'
      case 129: c = 13 + '~'; break; // 'Ł'
      case 131: c = 14 + '~'; break; // 'Ń'
      case 147: c = 15 + '~'; break; // 'Ó'
      case 154: c = 16 + '~'; break; // 'Ś'
      case 185: c = 17 + '~'; break; // 'Ź'
      case 187: c = 18 + '~'; break; // 'Ż'
      default:  break;
    }
    dualChar = 0;
    return c;
  }
  switch (_c) {
    case 185: c = 1 + '~'; break;
    case 230: c = 2 + '~'; break;
    case 234: c = 3 + '~'; break;
    case 179: c = 4 + '~'; break;
    case 241: c = 5 + '~'; break;
    case 243: c = 6 + '~'; break;
    case 156: c = 7 + '~'; break;
    case 159: c = 8 + '~'; break;
    case 191: c = 9 + '~'; break;
    case 165: c = 10 + '~'; break;
    case 198: c = 11 + '~'; break;
    case 202: c = 12 + '~'; break;
    case 163: c = 13 + '~'; break;
    case 209: c = 14 + '~'; break;
    case 211: c = 15 + '~'; break;
    case 140: c = 16 + '~'; break;
    case 143: c = 17 + '~'; break;
    case 175: c = 18 + '~'; break;
    default:  break;
  }
  return c;
}

// =======================================================================

void printCharWithShift(unsigned char c, int shiftDelay) 
{
  c = convertPolish(c);
  if (c < ' ' || c > MAX_CHAR) return;
  c -= 32;
  int w = showChar(c, font);
  for (int i = 0; i < w + 1; i++) {
    delay(shiftDelay);
    scrollLeft();
    refreshAll();
  }
}

// =======================================================================

void printStringWithShift(const char* s, int shiftDelay) 
{
  while (*s) {
    printCharWithShift(*s, shiftDelay);
    s++;
  }
}

// =======================================================================

/*
JSON output:
{"id":"1","jsonrpc":"2.0","result":{"percentage":17.42333984375,"time":{"hours":0,"milliseconds":18,"minutes":24,"seconds":22},"totaltime":{"hours":2,"milliseconds":146,"minutes":19,"seconds":51}}}
*/
int getKodi(char *kodiIP)
{
  Serial.print("getting data from "); Serial.println(kodiIP);
  if (client.connect(kodiIP, 8080)) {
    client.println(String("GET /jsonrpc?request={%22jsonrpc%22:%222.0%22,%22method%22:%22Player.GetProperties%22,%22params%22:{%22playerid%22:1,%22properties%22:[%22time%22,%22totaltime%22,%22percentage%22,%22speed%22]},%22id%22:%221%22}%20}\r\n") +
                   "Host: " + kodiIP + "\r\nUser-Agent: ArduinoWiFi/1.1\r\n" +
                   "Connection: close\r\n\r\n");
  } else {
    Serial.println("connection to kodi failed");
    return 0;
  }
  String line;
  int repeatCounter = 0;
  while (!client.available() && repeatCounter < 20) {
    delay(500);
    Serial.print("k.");
    repeatCounter++;
  }
  while (client.connected() && client.available()) {
    char c = client.read();
    if (c == '[' || c == ']') c = ' ';
    line += c;
  }

  client.stop();
  //Serial.println(line);

  DynamicJsonBuffer jsonBuf;
  JsonObject &root = jsonBuf.parseObject(line);
  if (!root.success())
  {
    Serial.println("parseObject() failed");
    printStringWithShift("json error!", 50);
    delay(500);
    return 0;
  }
  elapsed = root["result"]["percentage"];
  elapsedH = root["result"]["time"]["hours"];
  elapsedM = root["result"]["time"]["minutes"];
  elapsedS = root["result"]["time"]["seconds"];
  totalH = root["result"]["totaltime"]["hours"];
  totalM = root["result"]["totaltime"]["minutes"];
  totalS = root["result"]["totaltime"]["seconds"];
  state = root["result"]["speed"];

  //String t = "Elapsed: "+String(elapsed)+" Time: "+String(elapsedH)+":"+String(elapsedM)+" TotalTime: "+String(totalH)+":"+String(totalM)+" speed:"+String(state);
  //Serial.println(t);
  return 1;
}

// =======================================================================

float utcOffset = 1;
long localEpoc = 0;
long localMillisAtUpdate = 0;

void getTime()
{
  WiFiClient client;
  if (!client.connect("www.google.com", 80)) {
    Serial.println("connection to google failed");
    return;
  }

  client.print(String("GET / HTTP/1.1\r\n") +
               String("Host: www.google.com\r\n") +
               String("Connection: close\r\n\r\n"));
  int repeatCounter = 0;
  while (!client.available() && repeatCounter < 10) {
    delay(500);
    Serial.print("t.");
    repeatCounter++;
  }

  String line;
  client.setNoDelay(false);
  while (client.connected() && client.available()) {
    line = client.readStringUntil('\n');
    line.toUpperCase();
    // date: Thu, 19 Nov 2015 20:25:40 GMT
    if (line.startsWith("DATE: ")) {
      h = line.substring(23, 25).toInt();
      m = line.substring(26, 28).toInt();
      s = line.substring(29, 31).toInt();
      //Serial.println(String(h) + ":" + String(m) + ":" + String(s));

      localMillisAtUpdate = millis();
      localEpoc = (h * 60 * 60 + m * 60 + s);
      //Serial.println(localEpoc);
    }
  }
  client.stop();
}

// =======================================================================

void updateTime()
{
  long curEpoch = localEpoc + ((millis() - localMillisAtUpdate) / 1000);
  long epoch = round(curEpoch + 3600 * utcOffset + 86400L) % 86400L;
  h = ((epoch  % 86400L) / 3600) % 24;
  m = (epoch % 3600) / 60;
  s = epoch % 60;
}

// =======================================================================

