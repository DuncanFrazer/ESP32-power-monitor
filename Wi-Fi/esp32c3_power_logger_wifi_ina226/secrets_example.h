/*
  Wi-Fi credentials template
  ---------------------------
  1. Copy this file to "secrets.h" in the same folder as the .ino
     (Arduino sketches include local headers by placing them alongside
     the .ino file — the sketch's #include "secrets.h" will pick it up).
  2. Fill in your own SSID/password below in secrets.h.
  3. Don't commit secrets.h to version control — add it to .gitignore.
     secrets_example.h (this file) is safe to commit since it holds no
     real credentials.
*/

#ifndef SECRETS_H
#define SECRETS_H

#define SECRET_WIFI_SSID     "YOUR_SSID"
#define SECRET_WIFI_PASSWORD "YOUR_PASSWORD"

#endif
