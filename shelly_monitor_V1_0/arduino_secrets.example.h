// ---------------------------------------------------------------------------
// Diese Datei nach  "arduino_secrets.h"  kopieren und mit eigenen Werten fuellen.
// arduino_secrets.h ist per .gitignore ausgeschlossen und wird NICHT committet.
//
// Copy this file to  "arduino_secrets.h"  and fill in your own values.
// arduino_secrets.h is excluded via .gitignore and will NOT be committed.
// ---------------------------------------------------------------------------
#ifndef ARDUINO_SECRETS_H
#define ARDUINO_SECRETS_H

#define SECRET_SSID        "DEIN_WLAN_NAME"        // WiFi SSID
#define SECRET_PASS        "DEIN_WLAN_PASSWORT"    // WiFi password
#define SECRET_SHELLY_HOST "192.168.1.50"          // lokale IP des Shelly Pro 3EM
#define SECRET_ZEN_HOST    "192.168.1.51"          // lokale IP des Zendure SolarFlow

// Optional -- nur noetig, wenn CONTROL_WATCH_ENABLE im Sketch auf 1 gesetzt ist.
// IPs des lokalen Nulleinspeisungs-Duos (Regler + MQTT-Broker).
#define SECRET_REGLER_HOST "192.168.1.52"          // Regler (controller)
#define SECRET_BROKER_HOST "192.168.1.53"          // MQTT-Broker

#endif
