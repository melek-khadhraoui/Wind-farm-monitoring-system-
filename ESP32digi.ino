#include <WiFi.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>

// Configuration du Wi-Fi
const char* ssid = "VOTRE_SSID";
const char* password = "VOTRE_MOT_DE_PASSE";

// Configuration du serveur MQTT
const char* mqttServer = "Adresse_IP_du_serveur_MQTT_local";
const int mqttPort = 1883;  // Port du serveur MQTT

// Créer une instance de WiFiClient et PubSubClient
WiFiClient espClient;
PubSubClient client(espClient);

// Créer une instance de HardwareSerial pour la communication UART
HardwareSerial mySerial(1);  // Utiliser le port série 1 (TX2, RX2)

// Définir les topics MQTT
const char* topicTemperature = "capteurs/temperature";
const char* topicHumidite = "capteurs/humidite";
const char* topicAngle = "capteurs/angle";
const char* topicCourant = "capteurs/courant";
const char* topicVibration = "capteurs/vibration";
const char* topicVitesseVent = "capteurs/vitesse_vent";  // Ajout du topic pour la vitesse du vent

// Variables pour stocker les valeurs
float temperature = 0.0;
float humidite = 0.0;
float angle = 0.0;
float courant = 0.0;
float vitesseVent = 0.0;  // Ajout de la variable pour la vitesse du vent
bool vibration = false;

void setup() {
  // Initialiser la communication série
  Serial.begin(115200);

  // Initialiser le port série 1 pour la communication UART
  mySerial.begin(9600, SERIAL_8N1, 16, 17);  // RX sur GPIO16, TX sur GPIO17

  // Configurer la connexion Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connecté au Wi-Fi");

  // Configurer le serveur MQTT
  client.setServer(mqttServer, mqttPort);
}

void loop() {
  // Reconnecter au serveur MQTT si nécessaire
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Lire les données depuis l'UART
  if (mySerial.available()) {
    String message = mySerial.readStringUntil('\n');
    Serial.println("Message reçu: " + message);

    // Identifier le type de message et le traiter
    if (message.startsWith("VAL")) {
      // Message contenant les valeurs des capteurs
      String payload = message.substring(4);  // Enlever le préfixe "VAL "
      int firstComma = payload.indexOf(',');
      int secondComma = payload.indexOf(',', firstComma + 1);
      int thirdComma = payload.indexOf(',', secondComma + 1);
      int fourthComma = payload.indexOf(',', thirdComma + 1);

      if (firstComma != -1 && secondComma != -1 && thirdComma != -1 && fourthComma != -1) {
        vitesseVent = payload.substring(0, firstComma).toFloat();  // Lecture de la vitesse du vent
        temperature = payload.substring(firstComma + 1, secondComma).toFloat();
        humidite = payload.substring(secondComma + 1, thirdComma).toFloat();
        angle = payload.substring(thirdComma + 1, fourthComma).toFloat();
        courant = payload.substring(fourthComma + 1).toFloat();
        
        // Publier les valeurs sur les topics MQTT
        client.publish(topicVitesseVent, String(vitesseVent).c_str());  // Publier la vitesse du vent
        client.publish(topicTemperature, String(temperature).c_str());
        client.publish(topicHumidite, String(humidite).c_str());
        client.publish(topicAngle, String(angle).c_str());
        client.publish(topicCourant, String(courant).c_str());
      }
    } else if (message.startsWith("VIB")) {
      // Message contenant l'état de la vibration
      String payload = message.substring(4);  // Enlever le préfixe "VIB "
      vibration = payload.toInt() > 0;
      
      // Publier l'état de la vibration sur le topic MQTT
      client.publish(topicVibration, vibration ? "1" : "0");
    }
  }
}

void reconnect() {
  // Boucle de reconnexion au serveur MQTT
  while (!client.connected()) {
    Serial.print("Tentative de connexion au MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println("Connecté");
    } else {
      Serial.print("Échec, rc=");
      Serial.print(client.state());
      Serial.println(" Essai dans 5 secondes");
      delay(5000);
    }
  }
}
