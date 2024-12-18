#include <WiFi.h>
#include <PubSubClient.h>
#include <DHTesp.h>

// --- Configurações Wi-Fi ---
const char* ssid = "SSID";      // Substitua pelo SSID da sua rede Wi-Fi
const char* password = "WIFI_PASSWORD"; // Substitua pela senha da sua rede Wi-Fi

// --- Configurações MQTT ---
const char* mqtt_server = "BROCKER_MQTT"; // Substitua pelo endereço do broker MQTT
const char* device_id = "esp32_01";            // Identificador único do dispositivo
const char* topic_sensor = "boards/esp32_01/sensor";
const char* topic_button = "boards/esp32_01/button";
const char* topic_led = "boards/esp32_01/led";

// --- Configuração do DHT11 ---
#define DHTPIN 4         // Pino conectado ao DHT11
DHTesp dht;

// --- Configurações LED e Botão ---
#define LED_PIN 2        // Pino do LED (D2 no ESP32)
#define BUTTON_PIN 5     // Pino do botão
bool button_last_state = LOW;

// --- Instâncias ---
WiFiClient espClient;
PubSubClient client(espClient);

// --- Prototipação ---
void connectWiFi();
void connectMQTT();
void sendSensorData();
void sendButtonState();
void callback(char* topic, byte* message, unsigned int length);

// --- Funções ---
void connectWiFi() {
  Serial.print("Conectando ao Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi conectado.");
}

void connectMQTT() {
  while (!client.connected()) {
    Serial.print("Conectando ao broker MQTT...");
    if (client.connect(device_id)) {
      Serial.println("Conectado.");
      // Inscreve-se no tópico do LED
      client.subscribe(topic_led);
    } else {
      Serial.print("Falha, rc=");
      Serial.print(client.state());
      Serial.println(" Tentando novamente em 5 segundos...");
      delay(5000);
    }
  }
}

void sendSensorData() {
  static unsigned long last_time = 0;
  const unsigned long interval = 5000; // Envia dados a cada 5 segundos

  if (millis() - last_time > interval) {
    last_time = millis();

    // Lê temperatura e umidade do sensor
    TempAndHumidity data = dht.getTempAndHumidity();

    // Verifica se a leitura foi bem-sucedida
    if (dht.getStatus() != 0) {
      Serial.println("Erro ao ler o sensor DHT11: " + String(dht.getStatusString()));
      return;
    }

    float temperature = data.temperature;
    float humidity = data.humidity;

    // Exibe os dados no Monitor Serial
    Serial.print("Temperatura: ");
    Serial.print(temperature, 1);
    Serial.println(" °C");

    Serial.print("Umidade: ");
    Serial.print(humidity, 1);
    Serial.println(" %");

    // Constrói o payload JSON e publica no MQTT
    String payload = String("{\"temperature\":") + temperature + ",\"humidity\":" + humidity + "}";
    client.publish(topic_sensor, payload.c_str());
    Serial.println("Dados do sensor enviados: " + payload);
  }
}

void sendButtonState() {
  bool current_state = digitalRead(BUTTON_PIN) == LOW; // Botão pressionado é LOW
  if (current_state != button_last_state) {
    button_last_state = current_state;

    // Constrói e envia o estado do botão
    String payload = String("{\"buttonState\":") + (current_state ? "true" : "false") + "}";
    client.publish(topic_button, payload.c_str());
    Serial.println("Estado do botão enviado: " + payload);
  }
}

void callback(char* topic, byte* message, unsigned int length) {
  String msg = "";
  for (int i = 0; i < length; i++) {
    msg += (char)message[i];
  }
  Serial.println("Mensagem recebida no tópico " + String(topic) + ": " + msg);

  // Verifica se é o tópico do LED
  if (String(topic) == topic_led) {
    if (msg == "ON") {
      digitalWrite(LED_PIN, HIGH);
      Serial.println("LED ligado.");
    } else if (msg == "OFF") {
      digitalWrite(LED_PIN, LOW);
      Serial.println("LED desligado.");
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Inicializa o sensor DHT11
  Serial.println("Inicializando o sensor DHT11...");
  dht.setup(DHTPIN, DHTesp::DHT11);

  // Inicializa LED e botão
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Conecta ao Wi-Fi e configura MQTT
  connectWiFi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  // Reconecta ao MQTT, se necessário
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();

  // Enviar dados do sensor periodicamente
  sendSensorData();

  // Verificar e enviar estado do botão
  sendButtonState();
}

