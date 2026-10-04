# Firmware

| Cartella | Stato | Cosa fa |
|---|---|---|
| `banco_sysid_v2/` | **in uso** | ponte PC ↔ ESC + HX711: comandi con cifra di controllo, HX711 letto senza bloccare gli interrupt, watchdog. Serve sia per la sysid sia per le prove della policy |
| `esc_manual_control/` | funzionante | controllo manuale dell'ESC da seriale (0–180, 90 = neutro); è lo sketch con cui è stato verificato che l'ESC è bidirezionale |
| `archivio/banco_sysid_v1/` | **non usare** | prima versione del ponte: la libreria HX711 bloccava gli interrupt e i comandi si corrompevano (colpi a stop e a piena potenza). Tenuto come documentazione, vedi [docs/06](../docs/06_lezioni_imparate.md) |
