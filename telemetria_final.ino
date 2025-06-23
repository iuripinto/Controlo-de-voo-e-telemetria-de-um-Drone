#include "DHTesp.h"
#include <Wire.h>
#include <Adafruit_MLX90640.h>

// ------------------------- LIMITES -------------------------
#define TEMP_AR_LIMITE    50.0    // Temperatura do ar (°C)
#define HUMIDADE_LIMITE   70.0    // Humidade (%)
#define CO2_LIMITE        800     // CO2 ppm
#define TEMP_SOLO_LIMITE  70.0    // Temperatura térmica (°C)

// ------------------------- DEFINIÇÕES DE PINOS -------------------------
#define DHTPIN 4
#define CO2PIN 15
#define RX_PIN 16
#define TX_PIN 17

// ------------------------- DADOS SIM -------------------------
const char* NUMERO_TELEFONE = "+351961936066";
const char* PIN_SIM = "1404";

// ------------------------- OBJETOS GLOBAIS -------------------------
DHTesp dht;
Adafruit_MLX90640 mlx;
bool mlxInicializado = false;
float frame[32 * 24];

unsigned long ultimaLeitura = 0;
unsigned long ultimaMensagemStatus = 0;

const unsigned long INTERVALO_LEITURA = 10000;      // 10 segundos
const unsigned long INTERVALO_STATUS = 600000;      // 10 minutos

bool houveAlerta = false;

// ------------------------- GPS -------------------------
String latitude = "Indisponível", longitude = "Indisponível";
String bufferGPS = "";

// ------------------------- DECLARAÇÕES -------------------------
void lerSensores();
void enviarSMS(const String& msg);
void lerGPS();
void analisarGNGGA(String sentence);
void analisarGNRMC(String sentence);
void dividirString(String str, char delimiter, String* parts, int maxParts);
float converterParaDecimal(String coord, String direcao);
String formatarCoordenada(String coord, String direcao);
uint16_t lerCO2(int pin);

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(3000);

  Serial.println("Inicialização...");

  // GSM
  Serial1.println("AT");
  delay(1000);
  Serial1.println("AT+CMGF=1");
  delay(1000);
  Serial1.print("AT+CPIN=\"");
  Serial1.print(PIN_SIM);
  Serial1.println("\"");
  delay(1000);

  // GPS
  Serial1.println("AT+GPS=1");
  delay(1000);
  Serial1.println("AT+GPSRD=1");
  delay(1000);

  dht.setup(DHTPIN, DHTesp::DHT11);

  if (!mlx.begin()) {
    Serial.println("ERRO: MLX90640 não encontrado!");
    enviarSMS("ERRO: MLX90640 não encontrado!");
  } else {
    mlxInicializado = true;
    mlx.setMode(MLX90640_CHESS);
    mlx.setResolution(MLX90640_ADC_18BIT);
    mlx.setRefreshRate(MLX90640_2_HZ);
    Serial.println("MLX90640 inicializado com sucesso.");
  }

  Serial.println("Setup completo.");
}

void loop() {
  lerGPS();

  if (millis() - ultimaLeitura >= INTERVALO_LEITURA) {
    ultimaLeitura = millis();
    lerSensores();
  }
}

// ------------------------- GPS -------------------------
void lerGPS() {
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    if (c == '\n') {
      if (bufferGPS.startsWith("$GNGGA")) {
        analisarGNGGA(bufferGPS);
      } else if (bufferGPS.startsWith("$GNRMC")) {
        analisarGNRMC(bufferGPS);
      }
      bufferGPS = "";
    } else {
      bufferGPS += c;
    }
  }
}

void analisarGNGGA(String sentence) {
  String parts[15];
  dividirString(sentence, ',', parts, 15);

  String lat = parts[2];
  String dirLat = parts[3];
  String lon = parts[4];
  String dirLon = parts[5];

  latitude = formatarCoordenada(lat, dirLat);
  longitude = formatarCoordenada(lon, dirLon);

  Serial.println("Latitude: " + latitude);
  Serial.println("Longitude: " + longitude);
}

void analisarGNRMC(String sentence) {
  String parts[15];
  dividirString(sentence, ',', parts, 15);

  String lat = parts[3];
  String dirLat = parts[4];
  String lon = parts[5];
  String dirLon = parts[6];

  latitude = formatarCoordenada(lat, dirLat);
  longitude = formatarCoordenada(lon, dirLon);
}

void dividirString(String str, char delimiter, String* parts, int maxParts) {
  int partIndex = 0;
  int lastIndex = 0;

  for (int i = 0; i < str.length(); i++) {
    if (str[i] == delimiter || i == str.length() - 1) {
      if (partIndex < maxParts) {
        parts[partIndex++] = str.substring(lastIndex, i);
        lastIndex = i + 1;
      }
    }
  }
}

float converterParaDecimal(String coord, String direcao) {
  String graus, minutos;

  if (coord.length() >= 5 && coord.indexOf('.') > 2) {
    if (coord.indexOf('.') == 4) {
      graus = coord.substring(0, 2);
      minutos = coord.substring(2);
    } else if (coord.indexOf('.') == 5) {
      graus = coord.substring(0, 3);
      minutos = coord.substring(3);
    } else {
      return 0.0;
    }
  } else {
    return 0.0;
  }

  float g = graus.toFloat();
  float m = minutos.toFloat();
  float decimal = g + (m / 60.0);

  if (direcao == "S" || direcao == "W") {
    decimal *= -1;
  }

  return decimal;
}

String formatarCoordenada(String coord, String direcao) {
  if (coord.length() < 4) return "Indisponível";
  float decimal = converterParaDecimal(coord, direcao);
  return String(decimal, 6);
}

// ------------------------- LEITURA DE SENSORES -------------------------
void lerSensores() {
  Serial.println("A ler sensores...");
  houveAlerta = false;

  float temp = dht.getTemperature();
  float hum = dht.getHumidity();
  uint16_t co2 = lerCO2(CO2PIN);

  if (!isnan(temp) && temp > TEMP_AR_LIMITE) {
    enviarSMS("ALERTA: Temperatura do ar elevada!");
    houveAlerta = true;
  }

  if (!isnan(hum) && hum > HUMIDADE_LIMITE) {
    enviarSMS("ALERTA: Humidade elevada!");
    houveAlerta = true;
  }

  if (co2 > CO2_LIMITE) {
    enviarSMS("ALERTA: Nível de CO2 elevado");
    houveAlerta = true;
  }

  // --- Leitura térmica ---
  bool alertaSolo = false;
  float soma = 0, max = -1000, min = 1000;
  int pixelsQuentes = 0;

  if (mlxInicializado && mlx.getFrame(frame) == 0) {
    for (int i = 0; i < 768; i++) {
      float t = frame[i];
      soma += t;
      if (t > TEMP_SOLO_LIMITE) {
        alertaSolo = true;
        pixelsQuentes++;
      }
      if (t > max) max = t;
      if (t < min) min = t;
    }

    if (alertaSolo) {
      enviarSMS("ALERTA: Temperatura elevada no solo");
      houveAlerta = true;
    }
  }

  // --- Enviar relatório completo em caso de alerta ou a cada 10 minutos ---
  if (houveAlerta || (millis() - ultimaMensagemStatus >= INTERVALO_STATUS)) {
    if (!isnan(temp)) {
      enviarSMS("Temperatura do ar: " + String(temp, 1) + "°C");
    }
    if (!isnan(hum)) {
      enviarSMS("Humidade: " + String(hum, 1) + "%");
    }
    enviarSMS("Nível de CO2: " + String(co2) + " ppm");

    if (mlxInicializado) {
      float media = soma / 768.0;
      enviarSMS("Temperatura média do solo: " + String(media, 1) + "°C");
      enviarSMS("Máxima: " + String(max, 1) + "°C | Pixeis quentes: " + String(pixelsQuentes));
    }

    if (latitude != "Indisponível" && longitude != "Indisponível") {
      enviarSMS("Localização GPS: Latitude " + latitude + " | Longitude  " + longitude);
    }

    ultimaMensagemStatus = millis();
  }
}

uint16_t lerCO2(int pin) {
  static bool calibrado = false;
  static float vBase = 0;
  float v = analogRead(pin) * (3.3 / 4095.0);

  if (!calibrado) {
    vBase = v;
    calibrado = true;
    return 400;
  }

  float diff = v - vBase;
  return constrain((diff * 1000.0 / 1.6) + 400, 400, 1000);
}

// ------------------------- SMS -------------------------
void enviarSMS(const String& msg) {
  Serial.println("A enviar SMS: " + msg);
  Serial1.println("AT+CMGS=\"" + String(NUMERO_TELEFONE) + "\"");
  delay(300);
  Serial1.print(msg);
  Serial1.write(26); // Ctrl+Z
  delay(1000);
}
