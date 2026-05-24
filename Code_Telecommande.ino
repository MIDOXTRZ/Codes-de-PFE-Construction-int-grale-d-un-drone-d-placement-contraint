/*
 * ============================================================
 *  TÉLÉCOMMANDE DRONE – ESP32 (VERSION AMÉLIORÉE)
 *  Architecture :
 *   1. Vérification connexion ESP-NOW
 *   2. Réception télémétrie drone
 *   3. Mise à jour LEDs système
 *   4. Gestion armement
 *   5. Lecture joystick
 *   6. Envoi commandes drone
 *   7. Mise à jour OLED
 * ============================================================
 */

#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===================== OLED =====================
#define LARGEUR_OLED   128
#define HAUTEUR_OLED    64
#define ADRESSE_OLED  0x3C

Adafruit_SSD1306 ecran(LARGEUR_OLED, HAUTEUR_OLED, &Wire, -1);

// ===================== LEDS =====================
#define PIN_LED_ROUGE    14
#define PIN_LED_VERT     23
#define PIN_LED_ORANGE    4
#define PIN_LED_BLEU     19

// ===================== ENTRÉES =====================
#define PIN_BOUTON       18
#define PIN_JOY_GAU_Y    33

// ===================== ESP-NOW =====================
// Remplacer par MAC du drone
uint8_t adresse_drone[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ===================== STRUCTURES =====================

// Commandes envoyées AU DRONE
typedef struct {
  int  gaz;
  bool arme;
} DonneesCommande;

DonneesCommande commandes;

// Données reçues DU DRONE
typedef struct {
  float batterie;
  float altitude;
} DonneesDrone;

DonneesDrone telemetrie;

// ===================== VARIABLES =====================
bool drone_arme        = false;
bool esp_connecte      = false;

// Bouton
bool bouton_precedent  = HIGH;
bool en_cours_appui    = false;
unsigned long temps_appui = 0;

// Clignotement LED bleue
bool etat_cligno = false;
unsigned long temps_cligno = 0;

// Temps dernière réception drone
unsigned long dernier_paquet = 0;

// ===================== CALLBACK ENVOI =====================
void sur_envoi(const uint8_t *mac, esp_now_send_status_t statut) {

  if (statut == ESP_NOW_SEND_SUCCESS) {
    esp_connecte = true;
  } else {
    esp_connecte = false;
  }
}

// ===================== CALLBACK RÉCEPTION =====================
void sur_reception(const esp_now_recv_info *info,
                   const uint8_t *data,
                   int len) {

  memcpy(&telemetrie, data, sizeof(telemetrie));

  dernier_paquet = millis();
}

// ===================== LEDS BATTERIE =====================
void gerer_leds_batterie(float pct) {

  if (pct < 20.0) {
    digitalWrite(PIN_LED_ROUGE, HIGH);
    digitalWrite(PIN_LED_VERT, LOW);
  }
  else {
    digitalWrite(PIN_LED_ROUGE, LOW);
    digitalWrite(PIN_LED_VERT, HIGH);
  }
}

// ===================== LED BLEUE =====================
void gerer_led_bleue() {

  // Si données reçues récemment → connecté
  if (millis() - dernier_paquet < 1000) {

    esp_connecte = true;
    digitalWrite(PIN_LED_BLEU, HIGH);
  }
  else {

    esp_connecte = false;

    // Clignotement
    if (millis() - temps_cligno > 500) {

      temps_cligno = millis();
      etat_cligno = !etat_cligno;

      digitalWrite(PIN_LED_BLEU, etat_cligno);
    }
  }
}

// ===================== BOUTON ARMEMENT =====================
void gerer_bouton() {

  bool etat_bouton = digitalRead(PIN_BOUTON);

  // Détection début appui
  if (etat_bouton == LOW && bouton_precedent == HIGH) {

    temps_appui = millis();
    en_cours_appui = true;
  }

  // Relâchement bouton
  if (etat_bouton == HIGH &&
      bouton_precedent == LOW &&
      en_cours_appui) {

    unsigned long duree = millis() - temps_appui;

    // ARMEMENT
    if (!drone_arme &&
        esp_connecte &&
        duree >= 3000) {

      drone_arme = true;

      digitalWrite(PIN_LED_ORANGE, HIGH);

      Serial.println("DRONE ARMÉ");
    }

    // DÉSARMEMENT
    else if (drone_arme &&
             duree < 3000) {

      drone_arme = false;

      digitalWrite(PIN_LED_ORANGE, LOW);

      commandes.gaz = 1000;

      Serial.println("DRONE DÉSARMÉ");
    }

    en_cours_appui = false;
  }

  bouton_precedent = etat_bouton;
}

// ===================== LECTURE GAZ =====================
int lire_gaz() {

  int brut = analogRead(PIN_JOY_GAU_Y);

  int gaz = map(brut, 0, 4095, 1000, 2000);

  return constrain(gaz, 1000, 2000);
}

// ===================== OLED =====================
void afficher_oled() {

  ecran.clearDisplay();

  ecran.setTextSize(1);
  ecran.setTextColor(SSD1306_WHITE);

  // Batterie drone
  ecran.setCursor(0, 0);
  ecran.print("BAT   : ");
  ecran.print((int)telemetrie.batterie);
  ecran.println("%");

  // ESP-NOW
  ecran.setCursor(0, 16);
  ecran.print("ESP   : ");
  ecran.println(esp_connecte ? "OK" : "...");

  // Armement
  ecran.setCursor(0, 32);
  ecran.print("DRONE : ");
  ecran.println(drone_arme ? "ARME" : "DESARME");

  // Gaz
  ecran.setCursor(0, 48);
  ecran.print("THR   : ");
  ecran.println(commandes.gaz);

  ecran.display();
}

// ===================== SETUP =====================
void setup() {

  Serial.begin(115200);

  // LEDs
  pinMode(PIN_LED_ROUGE, OUTPUT);
  pinMode(PIN_LED_VERT, OUTPUT);
  pinMode(PIN_LED_ORANGE, OUTPUT);
  pinMode(PIN_LED_BLEU, OUTPUT);

  // Bouton
  pinMode(PIN_BOUTON, INPUT_PULLUP);

  // ADC
  analogSetAttenuation(ADC_11db);

  // OLED
  if (!ecran.begin(SSD1306_SWITCHCAPVCC, ADRESSE_OLED)) {

    Serial.println("OLED NON DETECTE");
  }

  // WiFi obligatoire
  WiFi.mode(WIFI_STA);

  // ESP-NOW
  if (esp_now_init() != ESP_OK) {

    Serial.println("ERREUR ESP-NOW");
    return;
  }

  // Callbacks
  esp_now_register_send_cb(sur_envoi);
  esp_now_register_recv_cb(sur_reception);

  // Pair drone
  esp_now_peer_info_t peerInfo = {};

  memcpy(peerInfo.peer_addr, adresse_drone, 6);

  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_now_add_peer(&peerInfo);

  Serial.println("TELECOMMANDE PRETE");
}

// ===================== LOOP =====================
void loop() {

  // =====================================================
  // 1. Vérification état ESP-NOW
  // =====================================================
  gerer_led_bleue();

  // =====================================================
  // 2. Réception données drone
  //    (callback automatique)
  // =====================================================

  // =====================================================
  // 3. LEDs batterie selon télémétrie drone
  // =====================================================
  gerer_leds_batterie(telemetrie.batterie);

  // =====================================================
  // 4. Sécurité connexion
  // =====================================================
  if (!esp_connecte) {

    drone_arme = false;

    digitalWrite(PIN_LED_ORANGE, LOW);

    commandes.gaz = 1000;
  }

  // =====================================================
  // 5. Gestion bouton armement
  // =====================================================
  gerer_bouton();

  // =====================================================
  // 6. Lecture joystick
  // =====================================================
  if (drone_arme) {

    commandes.gaz = lire_gaz();
  }
  else {

    commandes.gaz = 1000;
  }

  commandes.arme = drone_arme;

  // =====================================================
  // 7. Envoi ESP-NOW
  // =====================================================
  esp_now_send(
    adresse_drone,
    (uint8_t *)&commandes,
    sizeof(commandes)
  );

  // =====================================================
  // 8. OLED
  // =====================================================
  afficher_oled();

  // =====================================================
  // DEBUG
  // =====================================================
  Serial.print("BAT: ");
  Serial.print(telemetrie.batterie);

  Serial.print("% | ALT: ");
  Serial.print(telemetrie.altitude);

  Serial.print("m | GAZ: ");
  Serial.print(commandes.gaz);

  Serial.print(" | ARME: ");
  Serial.print(drone_arme ? "OUI" : "NON");

  Serial.print(" | ESP: ");
  Serial.println(esp_connecte ? "OK" : "...");

  // =====================================================
  // 50 Hz
  // =====================================================
  delay(20);
}
