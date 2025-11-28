#include <SoftwareSerial.h>
#include <TinyGPS.h>

/*************************************************
 *      Clase para manejar el GPS (TinyGPS)
 *************************************************/
class GPSManager {
private:
    TinyGPS gps;
    SoftwareSerial ss;
    unsigned long lastReadTime = 0;

public:
    // Constructor: indicamos pines RX y TX
    GPSManager(int rxPin, int txPin) : ss(rxPin, txPin) {}

    void begin(long baudRate) {
        ss.begin(baudRate);
        Serial.print("TinyGPS v");
        Serial.println(TinyGPS::library_version());
    }

    // Llamar constantemente desde loop()
    void update() {
        while (ss.available()) {
            gps.encode(ss.read());
        }
    }

    // Devuelve coordenadas en lat/lon
    bool getCoordenadas(float &lat, float &lon, unsigned long &age) {
        gps.f_get_position(&lat, &lon, &age);

        if (lat == TinyGPS::GPS_INVALID_F_ANGLE || 
            lon == TinyGPS::GPS_INVALID_F_ANGLE) 
        {
            return false;  // No hay FIX
        }
        return true;
    }

    // Satélites en uso
    int getSatelites() {
        int sat = gps.satellites();
        return (sat == TinyGPS::GPS_INVALID_SATELLITES ? -1 : sat);
    }

    // HDOP
    int getHDOP() {
        int h = gps.hdop();
        return (h == TinyGPS::GPS_INVALID_HDOP ? -1 : h);
    }

    // Estadísticas (opcional para debug)
    void printStats() {
        unsigned long chars;
        unsigned short sentences, failed;

        gps.stats(&chars, &sentences, &failed);

        Serial.print("CHARS=");
        Serial.print(chars);
        Serial.print(" SENTENCES=");
        Serial.print(sentences);
        Serial.print(" CSUM ERR=");
        Serial.println(failed);
    }

    bool leerDurante1s() {
        bool newData = false;

        unsigned long start = millis();
        while (millis() - start < 1000) {
            while (ss.available()) {
                char c = ss.read();
                if (gps.encode(c)) {
                    newData = true;
                }
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
            Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());

            Serial.print(" HDOP=");
            Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());

            Serial.println();
        }
        else {
            Serial.println("Sin FIX aún... (revisa cableado)");
        }

        // Estadísticas
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
 *          Instancias de módulos
 *************************************************/

// GPS en pines RX=4, TX=3 (SoftwareSerial)
GPSManager gpsManager(4, 3);
    

/*************************************************
 *                    SETUP
 *************************************************/
void setup() {
    Serial.begin(115200);
    gpsManager.begin(9600);   // Aquí ajustas el baud del GPS
    Serial.println("Sistema iniciado.");
}


/*************************************************
 *                     LOOP
 *************************************************/
void loop() {
    gpsManager.mostrarInfo();
    delay(1000);
}

