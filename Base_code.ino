#include <SoftwareSerial.h>
#include <TinyGPS.h>

/*************************************************
 *          Clase para manejar motor
 *************************************************/
class MotorManager {
private:
    int motorPin;

public:
    MotorManager(int pin) : motorPin(pin) {}

    void begin() {
        pinMode(motorPin, OUTPUT);
        digitalWrite(motorPin, LOW); // Apagado al inicio
    }

    void encender() {
        digitalWrite(motorPin, HIGH);
    }

    void apagar() {
        digitalWrite(motorPin, LOW);
    }

    // Función que activa motor según distancia
    void activarPorDistancia(long distancia, long umbral = 100) {
        if (distancia > 0 && distancia < umbral) {
            encender();
        } else {
            apagar();
        }
    }
};

/*************************************************
 *      Clase para manejar el GPS (TinyGPS)
 *************************************************/
class GPSManager {
private:
    TinyGPS gps;
    SoftwareSerial ss;
    unsigned long lastReadTime = 0;

public:
    GPSManager(int rxPin, int txPin) : ss(rxPin, txPin) {}

    void begin(long baudRate) {
        ss.begin(baudRate);
        Serial.print("TinyGPS v");
        Serial.println(TinyGPS::library_version());
    }

    void update() {
        while (ss.available()) {
            gps.encode(ss.read());
        }
    }

    bool getCoordenadas(float &lat, float &lon, unsigned long &age) {
        gps.f_get_position(&lat, &lon, &age);
        if (lat == TinyGPS::GPS_INVALID_F_ANGLE || 
            lon == TinyGPS::GPS_INVALID_F_ANGLE) {
            return false;
        }
        return true;
    }

    int getSatelites() {
        int sat = gps.satellites();
        return (sat == TinyGPS::GPS_INVALID_SATELLITES ? -1 : sat);
    }

    int getHDOP() {
        int h = gps.hdop();
        return (h == TinyGPS::GPS_INVALID_HDOP ? -1 : h);
    }

    bool leerDurante1s() {
        bool newData = false;
        unsigned long start = millis();
        while (millis() - start < 1000) {
            while (ss.available()) {
                char c = ss.read();
                if (gps.encode(c)) newData = true;
            }
        }
        return newData;
    }

    void mostrarInfo() {
        bool newData = leerDurante1s();
        float lat, lon;
        unsigned long age;

        if (newData) {
            gps.f_get_position(&lat, &lon, &age);

            Serial.print("LAT=");
            Serial.print(lat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : lat, 6);
            Serial.print(" LON=");
            Serial.print(lon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : lon, 6);

            Serial.print(" SAT=");
            Serial.print(getSatelites());

            Serial.print(" HDOP=");
            Serial.print(getHDOP());

            Serial.println();
        }
        else {
            Serial.println("Sin FIX aún... (revisa cableado)");
        }

        unsigned long chars;
        unsigned short sentences, failed;
        gps.stats(&chars, &sentences, &failed);

        Serial.print("CHARS=");
        Serial.print(chars);
        Serial.print(" SENTENCES=");
        Serial.print(sentences);
        Serial.print(" CSUM ERR=");
        Serial.println(failed);

        if (chars == 0)
            Serial.println("** No characters received from GPS: check wiring **");
    }
};

/*************************************************
 *      Clase HC-SR04
 *************************************************/
class UltrasonicManager {
private:
    int trigPin;
    int echoPin;

public:
    UltrasonicManager(int trig, int echo) : trigPin(trig), echoPin(echo) {}

    void begin() {
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);
    }

    long getDistancia() {
        digitalWrite(trigPin, LOW); delayMicroseconds(5);
        digitalWrite(trigPin, HIGH); delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        long duracion = pulseIn(echoPin, HIGH, 30000UL);
        if (duracion == 0) return -1;

        long distancia = duracion * 0.034 / 2;
        if (distancia < 2 || distancia > 400) return -1;
        return distancia;
    }

    long getDistanciaPromedio(int lecturas = 5) {
        long suma = 0;
        int validas = 0;
        for(int i=0;i<lecturas;i++){
            long d = getDistancia();
            if(d>0){ suma+=d; validas++; }
            delay(50);
        }
        return (validas>0) ? suma/validas : -1;
    }

    void mostrarInfo() {
        long d = getDistanciaPromedio();
        if(d<0){
            Serial.println("HC-SR04: sin lectura válida");
        } else {
            Serial.print("HC-SR04 distancia promedio: "); Serial.print(d); Serial.println(" cm");
        }
    }
};

/*************************************************
 *            Instancias
 *************************************************/
GPSManager gpsManager(4,3);           // RX, TX GPS
UltrasonicManager ultrasonic(5,6);    // TRIG, ECHO HC-SR04
MotorManager motor1(7);               // Motor 1 pin Arduino
MotorManager motor2(8);               // Motor 2 pin Arduino

/*************************************************
 *                    SETUP
 *************************************************/
void setup() {
    Serial.begin(115200);
    //gpsManager.begin(9600);
    ultrasonic.begin();
    motor1.begin();
    //motor2.begin();
    Serial.println("Sistema iniciado");
}

/*************************************************
 *                     LOOP
 *************************************************/
void loop() {
    //gpsManager.mostrarInfo();
    ultrasonic.mostrarInfo();

    long distancia = ultrasonic.getDistancia();
    motor1.activarPorDistancia(distancia);

    delay(1000);
    Serial.println("----------------------------");
}
