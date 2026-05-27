/*
 * ============================================================
 *  TELECOMMANDE DRONE ESP32
 * ============================================================
 */

#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===================== ecran oled =====================
// petit ecran pour afficher batterie + etat
#define LARGEUR_OLED   128
#define HAUTEUR_OLED    64
#define ADRESSE_OLED   0x3C

Adafruit_SSD1306 ecran(LARGEUR_OLED, HAUTEUR_OLED, &Wire, -1);

// ===================== leds =====================
// juste pour voir l'etat du systeme rapidement
#define PIN_LED_ROUGE   14
#define PIN_LED_VERT    23
#define PIN_LED_ORANGE   4
#define PIN_LED_BLEU    19

// ===================== boutons et joystick =====================
// bouton pour armer le drone
#define PIN_BOUTON      18
// axe y du joystick pour le gaz
#define PIN_JOY_Y       32

// ===================== adresse du drone =====================
// (a remplacer par la vraie mac plus tard)
uint8_t adresse_drone[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ===================== structure communication =====================
// ce qu'on envoie vers le drone
typedef struct {
  int gaz;
  bool arme;
} DonneesCommande;

DonneesCommande commandes;

// ce qu'on recoit du drone
typedef struct {
  float batterie;
} DonneesDrone;

DonneesDrone telemetrie;

// ===================== variables =====================
// etat du drone
bool drone_arme = false;
bool esp_connecte = false;

// dernier paquet recu (pour savoir si ca marche encore)
unsigned long dernier_paquet = 0;

// bouton
bool old_btn = HIGH;
unsigned long temps_appui = 0;

// =====================================================
// callback quand on envoie
// =====================================================
void onSend(const uint8_t *mac_addr, esp_now_send_status_t status) {

  // si ca passe ou pas on met juste un etat simple
  esp_connecte = (status == ESP_NOW_SEND_SUCCESS);
}

// =====================================================
// callback quand on recoit
// =====================================================
void onRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {

  // on copie juste les donnees batterie
  memcpy(&telemetrie, data, sizeof(telemetrie));

  // on reset le timer de connexion
  dernier_paquet = millis();

  esp_connecte = true;
}

// =====================================================
// lecture du joystick
// =====================================================
int lire_gaz() {

  int brut = analogRead(PIN_JOY_Y);

  // conversion simple vers 1000 - 2000
  int gaz = map(brut, 0, 4095, 1000, 2000);

  return constrain(gaz, 1000, 2000);
}

// =====================================================
// setup
// =====================================================
void setup() {

  Serial.begin(115200);

  // leds en sortie
  pinMode(PIN_LED_ROUGE, OUTPUT);
  pinMode(PIN_LED_VERT, OUTPUT);
  pinMode(PIN_LED_ORANGE, OUTPUT);
  pinMode(PIN_LED_BLEU, OUTPUT);

  // bouton avec pullup (donc actif a LOW)
  pinMode(PIN_BOUTON, INPUT_PULLUP);

  // pour avoir une meilleure precision analogique
  analogSetAttenuation(ADC_11db);

  // ecran oled
  ecran.begin(SSD1306_SWITCHCAPVCC, ADRESSE_OLED);

  // wifi en mode station
  WiFi.mode(WIFI_STA);

  // initialisation esp now
  if (esp_now_init() != ESP_OK) {
    Serial.println("erreur esp now");
    return;
  }

  // callbacks
  esp_now_register_send_cb(onSend);
  esp_now_register_recv_cb(onRecv);

  // ajout du drone comme partenaire
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, adresse_drone, 6);
  peer.channel = 0;
  peer.encrypt = false;

  esp_now_add_peer(&peer);

  Serial.println("telecommande prete");
}

// =====================================================
// loop principal
// =====================================================
void loop() {

  // si pas de message depuis longtemps => plus de connexion
  if (millis() - dernier_paquet > 1000) {
    esp_connecte = false;
  }

  // ================= bouton =================
  bool btn = digitalRead(PIN_BOUTON);

  // detection debut appui
  if (btn == LOW && old_btn == HIGH) {
    temps_appui = millis();
  }

  // detection relachement
  if (btn == HIGH && old_btn == LOW) {

    unsigned long duree = millis() - temps_appui;

    // si appui long => armement
    if (!drone_arme && duree > 3000 && esp_connecte) {
      drone_arme = true;
      digitalWrite(PIN_LED_ORANGE, HIGH);
    }

    // sinon desarmement
    else if (drone_arme) {
      drone_arme = false;
      digitalWrite(PIN_LED_ORANGE, LOW);
      commandes.gaz = 1000;
    }
  }

  old_btn = btn;

  // ================= envoi commandes =================
  if (drone_arme) {
    commandes.gaz = lire_gaz();
  } else {
    commandes.gaz = 1000;
  }

  commandes.arme = drone_arme;

  esp_now_send(adresse_drone, (uint8_t*)&commandes, sizeof(commandes));

  // ================= led simple =================
  digitalWrite(PIN_LED_BLEU, esp_connecte ? HIGH : LOW);

  // ================= affichage oled =================
  ecran.clearDisplay();
  ecran.setTextSize(1);
  ecran.setTextColor(SSD1306_WHITE);

  ecran.setCursor(0,0);
  ecran.print("BAT: ");
  ecran.print((int)telemetrie.batterie);
  ecran.println("%");

  ecran.setCursor(0,20);
  ecran.print("ESP: ");
  ecran.println(esp_connecte ? "OK" : "...");

  ecran.setCursor(0,40);
  ecran.print("ARM: ");
  ecran.println(drone_arme ? "OUI" : "NON");

  ecran.display();

  // petit delay pour stabilite
  delay(20);
}
