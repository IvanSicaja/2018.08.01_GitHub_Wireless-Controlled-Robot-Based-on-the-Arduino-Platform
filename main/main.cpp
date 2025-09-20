#include <Arduino.h>

// Uključivanje potrebnih biblioteka
#include <WiFi.h>
#include <Servo.h>

// Definiranje pinova L298N dvostrukog H-Mosta
#define DIRECTION1_PIN_L 2  // Smjer motora
#define DIRECTION2_PIN_L 4  // Smjer motora
#define SPEED_PIN_L 26      // Pin za kontroliranje brzine motora
#define DIRECTION1_PIN_R 27 // Smjer motora
#define DIRECTION2_PIN_R 33 // Smjer motora
#define SPEED_PIN_R 25      // Pin za kontroliranje brzine motora

// Definiranje pinova HC-SR04 ultrazvučnog senzora
#define ECHO_PIN 13 // Ultrazvučni Echo pin povezan na pin 13
#define TRIG_PIN 12 // Ultrazvučni Trig pin povezan na pin 12

// Definiranje varijabli HC-SR04 ultrazvučnog senzora
long Duration;  // Vrijeme potrebno da senzor osluhne odaslani ultrazvučni val
float Distance; // Trenutna udaljenost

// Varijable autonomnog zaobilaženja prepreka
float CenterDistance, LeftDistance, RightDistance, LeftDiagonalDistance, RightDiagonalDistance;
float DistanceLimit = 30;
float CentrDistanceLimit = 45;

// Definiranje brzine koju ćemo koristiti
const uint16_t SPEED = 150;
const uint16_t RIGHT_SPEED = (int)SPEED * 1.2;
const uint16_t OBSTACLE_AVOIDANCE_SPEED = 75;
const uint16_t OBSTACLE_AVOIDANCE_RIGHT_SPEED = 95;
const uint16_t LeftAndRightSPEED = 200; // Brzina rotiranja robota
int RotateTime = 2000;                  // Vrijeme rotiranja

// Definiranje kanala i PWM karaktreristika signala brzine vrtnje motora
const uint16_t FREQUENCY = 5000;
const uint8_t RESOLUTION = 8;
const uint8_t CHANNEL_L = 0;
const uint8_t CHANNEL_R = 1;

// Definiranje kanala i PWM karaktreristika signala položaja vrtnje servo motora
int PWM_FREQUENCY = 50;
int PWM_CHANNEL = 0;
int PWM_RESOUTION = 8;
int GPIOPIN = 14;

// Postavljanje naziva i lozinke kreirane lokalne Wi-Fi mreže
const char *ssid = "ESP32-Access-Point";
const char *password = "123456789";

// Postavljnje Web servera na port 80
WiFiServer server(80);

// Varijabla koja pohrnjuje HTTP zahtjev
String header;

// Prototipi funkcija
void InitRobotCar();
void SetMotorSpeed(const uint16_t speed_L, const uint16_t speed_R);
void GoForward();
void GoBack();
void GoLeft();
void GoRight();
void StopCar();
int READ_DISTANCE();
void OBSTACLE_AVOIDANCE();
void SERVO_ROTATE();

// Funkcija za provjeru položaja servo motora
void SERVO_ROTATE()
{
  // Kreiranje PWM signala za određeni položaj
  int dutyCycle2 = 1;
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOUTION);
  ledcWrite(PWM_CHANNEL, dutyCycle2);
  delay(300);
  // Kreiranje PWM signala za određeni položaj
  dutyCycle2 = 11;
  ledcWrite(PWM_CHANNEL, dutyCycle2);
  delay(300);
  Serial.print("Dijagonala");
  // Kreiranje PWM signala za određeni položaj
  dutyCycle2 = 20;
  ledcWrite(PWM_CHANNEL, dutyCycle2);
  delay(300);
  // Kreiranje PWM signala za određeni položaj
  dutyCycle2 = 24;
  ledcWrite(PWM_CHANNEL, dutyCycle2);
  delay(300);
  Serial.print("Dijagonala");
  // Kreiranje PWM signala za određeni položaj
  dutyCycle2 = 40;
  ledcWrite(PWM_CHANNEL, dutyCycle2);
  delay(300);
}

// Postavljanje početnog uvjeta robota
void InitRobotCar()
{
  // Ispisivanje podataka u serijski monitor
  Serial.println();
  Serial.println(F("Initializing Robot Car..."));
  // Postavljanje svojstava kanala
  ledcSetup(CHANNEL_L, FREQUENCY, RESOLUTION);
  ledcSetup(CHANNEL_R, FREQUENCY, RESOLUTION);
  // Postavljanje kanala na pin pomoću kojeg će bit kontroliran
  ledcAttachPin(SPEED_PIN_L, CHANNEL_L);
  ledcAttachPin(SPEED_PIN_R, CHANNEL_R);
  // Postavljanje početnih uvjeta HC-SR04 ultrazvučnog senzora
  pinMode(DIRECTION1_PIN_L, OUTPUT);
  pinMode(DIRECTION2_PIN_L, OUTPUT);
  pinMode(SPEED_PIN_L, OUTPUT);
  pinMode(DIRECTION1_PIN_R, OUTPUT);
  pinMode(DIRECTION2_PIN_R, OUTPUT);
  pinMode(SPEED_PIN_R, OUTPUT);
  StopCar();

  // Pauza određeni broj mikro-sekundi
  delay(4000);
  // Ispisivanje podataka u serijski monitor
  Serial.println(F("Robot Car initialized successfully."));
}

// Funkcija za postavljanje brzine vrtnje motora ESP-32 mikrokontrolera
void SetMotorSpeed(const uint16_t speed_L, const uint16_t speed_R)
{
  ledcWrite(CHANNEL_L, speed_L);
  ledcWrite(CHANNEL_R, speed_R);
  /* analogWrite() nije dostupna na ESP-32
     analogWrite(speedPinL,speed_L);
     analogWrite(speedPinR,speed_R);   */
}

/*  Kontrola motora */

// Idi Naprijed
void GoForward()
{
  SetMotorSpeed(SPEED, RIGHT_SPEED);
  digitalWrite(DIRECTION1_PIN_L, HIGH);
  digitalWrite(DIRECTION2_PIN_L, LOW);
  digitalWrite(DIRECTION1_PIN_R, HIGH);
  digitalWrite(DIRECTION2_PIN_R, LOW);
}

// Idi Naprijed u načinu rada autonomnog zaobilaženja prepreka
void ObstacleGoForward()
{
  SetMotorSpeed(SPEED, RIGHT_SPEED);
  digitalWrite(DIRECTION1_PIN_L, HIGH);
  digitalWrite(DIRECTION2_PIN_L, LOW);
  digitalWrite(DIRECTION1_PIN_R, HIGH);
  digitalWrite(DIRECTION2_PIN_R, LOW);
}

// Idi Lijevo
void GoLeft()
{
  SetMotorSpeed(LeftAndRightSPEED, LeftAndRightSPEED);
  digitalWrite(DIRECTION1_PIN_L, HIGH);
  digitalWrite(DIRECTION2_PIN_L, LOW);
  digitalWrite(DIRECTION1_PIN_R, LOW);
  digitalWrite(DIRECTION2_PIN_R, HIGH);
}

// Idi Desno
void GoRight()
{
  SetMotorSpeed(LeftAndRightSPEED, LeftAndRightSPEED);
  digitalWrite(DIRECTION1_PIN_L, LOW);
  digitalWrite(DIRECTION2_PIN_L, HIGH);
  digitalWrite(DIRECTION1_PIN_R, HIGH);
  digitalWrite(DIRECTION2_PIN_R, LOW);
}

// Idi nazad
void GoBack()
{
  SetMotorSpeed(SPEED, SPEED);
  digitalWrite(DIRECTION1_PIN_L, LOW);
  digitalWrite(DIRECTION2_PIN_L, HIGH);
  digitalWrite(DIRECTION1_PIN_R, LOW);
  digitalWrite(DIRECTION2_PIN_R, HIGH);
}

// Stop
void StopCar()
{
  digitalWrite(DIRECTION1_PIN_L, LOW);
  digitalWrite(DIRECTION2_PIN_L, LOW);
  digitalWrite(DIRECTION1_PIN_R, LOW);
  digitalWrite(DIRECTION2_PIN_R, LOW);
}

int READ_DISTANCE()
{
  // Retartiranje Trig pina
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Postavljane Trig pina u aktivan rad 10 mikro-sekundi
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Očitavanje Echo piva koji vraća vrijednost trajanja putovanj ultrzvučnog vala u mikrosekundama
  Duration = pulseIn(ECHO_PIN, HIGH);

  // Računanje udaljenosti
  Distance = Duration * 0.01657;
  return (Distance);

  // Ispisivanje udaljenosti u serijski monitor
  Serial.println("");
  Serial.println("");
  Serial.print("Distance: ");
  Serial.println(Distance);
  Serial.println("");
  Serial.println("");
}

// Funkcija autonomnog zaobilaženja prepreka
void OBSTACLE_AVOIDANCE()
{
  WiFiClient client = server.available();

  // Brojač
  int i = 0;

  void SetMotorSpeed();
  // Ispisivanje podataka u serijski monitor
  Serial.println("Speed iznosi");
  Serial.println(SPEED);
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOUTION);

  while (i < 15)
  {
    i++;

    // Ispisivanje podataka u serijski monitor
    Serial.print(" I iznosi ");
    Serial.println(i);

    ObstacleGoForward();

    int ObstacleAvoidanceTime = 300;

    // Provjera stana udaljenosti za različite položaje servo motora
    int dutyCycle2 = 20;
    ledcWrite(PWM_CHANNEL, dutyCycle2);
    delay(ObstacleAvoidanceTime);
    READ_DISTANCE();
    CenterDistance = Distance;
    Serial.print("Centralna udaljenost iznosi ");
    Serial.println(CenterDistance);

    if (CenterDistance < CentrDistanceLimit)
    {
      StopCar();
      Serial.println("Zaustavljam  auto, centralna kriticna.");
      Serial.println(" ");
    }

    dutyCycle2 = 33;
    ledcWrite(PWM_CHANNEL, dutyCycle2);
    delay(ObstacleAvoidanceTime);
    READ_DISTANCE();
    LeftDistance = Distance;
    Serial.print("Lijeva udaljenost iznosi ");
    Serial.println(LeftDistance);

    if (LeftDistance < DistanceLimit)
    {
      GoRight();
      delay(RotateTime / 4);
      StopCar();
      Serial.println("Zaustavljam  auto, lijeva kriticna.");
      Serial.println(" ");
    }

    dutyCycle2 = 24;
    ledcWrite(PWM_CHANNEL, dutyCycle2);
    delay(ObstacleAvoidanceTime);
    READ_DISTANCE();
    LeftDiagonalDistance = Distance;
    Serial.print("Lijeva dijagonalna udaljenost iznosi ");
    Serial.println(LeftDiagonalDistance);

    if (LeftDiagonalDistance < DistanceLimit)
    {
      GoRight();
      delay(RotateTime / 4);
      StopCar();
      Serial.println("Zaustavljam  auto, lijeva dijagonalna kriticna.");
      Serial.println(" ");
    }

    dutyCycle2 = 20;

    ledcWrite(PWM_CHANNEL, dutyCycle2);
    delay(ObstacleAvoidanceTime);
    READ_DISTANCE();
    CenterDistance = Distance;
    Serial.print("Centralna udaljenost iznosi ");
    Serial.println(CenterDistance);

    if (CenterDistance < CentrDistanceLimit)
    {
      StopCar();
      Serial.println("Zaustavljam  auto, centralna kriticna.");
      Serial.println(" ");
    }

    dutyCycle2 = 11;

    ledcWrite(PWM_CHANNEL, dutyCycle2);
    delay(ObstacleAvoidanceTime);
    READ_DISTANCE();
    LeftDiagonalDistance = Distance;
    Serial.print("Desna dijagonalna udaljenost iznosi ");
    Serial.println(LeftDiagonalDistance);

    if (LeftDiagonalDistance < DistanceLimit)
    {
      GoLeft();
      delay(RotateTime / 4);
      StopCar();
      Serial.println("Zaustavljam  auto, desna dijagonalna kriticna.");
      Serial.println(" ");
    }

    dutyCycle2 = 1;

    ledcWrite(PWM_CHANNEL, dutyCycle2);
    delay(ObstacleAvoidanceTime);
    READ_DISTANCE();
    RightDistance = Distance;
    Serial.print("Desna udaljenost iznosi ");
    Serial.println(RightDistance);

    if (RightDistance < DistanceLimit)
    {
      GoLeft();
      delay(RotateTime / 4);
      StopCar();
      Serial.println("Zaustavljam  auto, desna kriticna.");
      Serial.println(" ");
    }

    dutyCycle2 = 20;

    ledcWrite(PWM_CHANNEL, dutyCycle2);
    delay(ObstacleAvoidanceTime);
    READ_DISTANCE();
    CenterDistance = Distance;
    Serial.print("Centralna udaljenost iznosi ");
    Serial.println(CenterDistance);

    if (CenterDistance < CentrDistanceLimit)
    {
      StopCar();
      Serial.println("Zaustavljam  auto, centralna kriticna.");
      Serial.println(" ");
    }

    // Dodatni uvjeti kontrole i definiranja ponašanja robota u datim uvjetima
    if ((CenterDistance < CentrDistanceLimit) && (LeftDiagonalDistance < DistanceLimit) &&
        (RightDiagonalDistance < DistanceLimit))
    {
      ObstacleGoForward();
      Serial.print("Uvjet iznosi ");
      Serial.println((CenterDistance < CentrDistanceLimit) && (LeftDiagonalDistance < DistanceLimit) && (RightDiagonalDistance < DistanceLimit));
      Serial.println("Idem naprijed");
      Serial.println(" ");
    }

    if ((CenterDistance < CentrDistanceLimit) && (LeftDistance < DistanceLimit) && (RightDistance < DistanceLimit))
    {
      GoRight();
      delay(RotateTime);
      StopCar();
      Serial.print("Uvjet iznosi ");
      Serial.println((CenterDistance < DistanceLimit) && (LeftDistance < DistanceLimit) && (RightDistance < DistanceLimit));
      Serial.println("Idem nazad");
      Serial.println(" ");
    }
  }
    StopCar();
}

// Petlja koja se izvršava samo jednom
void setup(){
  // Početak serijske komunikacije
  Serial.begin(9600);

  // Definiranje svojstava kanala i PWM signala
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOUTION);
  ledcAttachPin(GPIOPIN, PWM_CHANNEL);

  InitRobotCar();

  pinMode(TRIG_PIN, OUTPUT); // Postavnjen Trigpin kao izlaz
  pinMode(ECHO_PIN, INPUT);  // Postavnjen Echo pin kao ulaz

  // Spajanje na kreiranu Wi-Fi mrežu sa željenim nazivom i lozinkom
  Serial.print("Setting AP (Access Point)…");

  // Uklonite lozinku ako želite  da vaša Wi-Fi mreža bude slobodna
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  server.begin();
}

// Petlja koja se vrti beskonačno mnogo puta
void loop()
{
  WiFiClient client = server.available(); // Čekanje na klijente

  if (client)
  {                                // Ako imamo novog klijenta
    Serial.println("New Client."); // ispiši poruku u Serijski monitor
    String currentLine = "";       // napravi string koji drži podatke od klijenta
    while (client.connected())     // Loop petlja ako je klijent spojen na mrežu
    {
      if (client.available())
      {                         // Ako se primaju podatci od klijenta
        char c = client.read(); // tad ih ispiši u Serijski monitor
        Serial.write(c);
        header += c;
        if (c == '\n') // Ako je podatak za novu praznu liniju
        {
          if (currentLine.length() == 0)
          {
            // HTTP naslov običn počinju s npr. HTTP/1.1 200 OK
            // kreiranje prazne linije na serveru
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // Ako IP adresa završava s "NAPRIJED", izvrši if naredbu grananja
            if (header.indexOf("GET /NAPRIJED") >= 0)
            {
              GoForward();
            }
            // Ako IP adresa završava s "NAZAD", izvrši if naredbu grananja
            else if (header.indexOf("GET /NAZAD") >= 0)
            {
              GoBack();
            }
            // Isti princip kao i za prethodne dvije naredbe grananja
            else if (header.indexOf("GET /LIJEVO") >= 0)
            {
              GoLeft();
            }
            // Isti princip kao i za prethodne dvije naredbe grananja
            else if (header.indexOf("GET /DESNO") >= 0)
            {
              GoRight();
            }
            // Isti princip kao i za prethodne dvije naredbe grananja
            else if (header.indexOf("GET /STOP") >= 0)
            {
              StopCar();
            }
            // Isti princip kao i za prethodne dvije naredbe grananja
            else if (header.indexOf("GET /OBSTACLE_AVOIDANCE") >= 0)
            {
              OBSTACLE_AVOIDANCE();
            }
            // Isti princip kao i za prethodne dvije naredbe grananja
            else if (header.indexOf("GET /SERVO_ROTATE") >= 0)
            {
              SERVO_ROTATE();
            }
            // Isti princip kao i za prethodne dvije naredbe grananja
            else if (header.indexOf("GET /DEGREES90_ROTATE") >= 0)
            {
              GoLeft();
              Serial.println("EVO ME U 90 DEGREES");
              delay(RotateTime / 2);
              StopCar();
            }
            // Isti princip kao i za prethodne dvije naredbe grananja
            else if (header.indexOf("GET /DEGREES180_ROTATE") >= 0)
            {
              GoLeft();
              Serial.println("EVO ME U 180 DEGREES");
              delay(RotateTime);
              StopCar();
            }

            /// Definiranje nčina klientskog prikaz HTML Web strance
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px; text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");
            client.println("<body><h1>ESP32 Web Server</h1>");
            client.println("<p><a href=\"/NAPRIJED\">NAPRIJED</a></p>");
            client.println("<p><a href=\"/NAZAD\">NAZAD</a></p>");
            client.println("<p><a href=\"/LIJEVO\">LIJEVO</a></p>");
            client.println("<p><a href=\"/DESNO\">DESNO</a></p>");
            client.println("<p><a href=\"/STOP\">STOP</a></p>");
            client.println("<p><a href=\"/OBSTACLE_AVOIDANCE\">OBSTACLE_AVOIDANCE</a></p>");
            client.println("<p><a href=\"/SERVO_ROTATE\">SERVO_ROTATE</a></p>");
            client.println("<p><a href=\"/DEGREES90_ROTATE\">DEGREES90_ROTATE</a></p>");
            client.println("<p><a href=\"/DEGREES180_ROTATE\">DEGREES180_ROTATE</a></p>");
            client.println("</body></html>");

            // Kraj HTTP zatjeva percipira se sa sljedećom praznom linijom
            client.println();
            // Izlazak iz "While" petlje
            break;
          }
          else
          { // Ako imate novu liniju obrišite trenutnu
            currentLine = "";
          }
        }
        else if (c != '\r')
        { // Ako imate bilo što, vrati karakter te ga dodaj na trenutnu liniju
          currentLine += c;
        }
      }
    }
    // Obriši "header" varijablu
    header = "";
    // Prekini vezu
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}
