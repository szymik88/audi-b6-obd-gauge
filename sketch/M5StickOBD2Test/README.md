# M5StickC Plus2 – wskaźnik dla Audi A4 B6 1.8T

Wskaźnik na M5StickC Plus2. Łączy się przez Bluetooth z adapterem ELM327 v1.5, tanim klonem bez nazwy BT.
Odczytuje dane ze sterownika silnika Bosch ME7.5 (8E0909518AL) przez **bloki pomiarowe VAG**
(KWP2000, inicjalizacja 5-baud, adres 01, sesja `10 89`, odczyt `21 xx`).
Ten sterownik nie udostępnia doładowania przez standardowe OBD2.

## Ekrany
Przycisk A: następny ekran. Przycisk B: poprzedni. Przytrzymanie A: zerowanie max.

1. **Główny**: doładowanie [bar], RPM, akumulator [V] ze stanem ładowania, płyn [°C]
2. **Zapłon/stuki**: IAT, kąt zapłonu, cofanie zapłonu cyl. 1–4 (+max)
3. **Turbo**: doładowanie zadane/rzeczywiste, ostrzeżenie o niedoborze, N75 %, load, MAF
4. **Mieszanka**: prędkość, przepustnica, lambda, korekty paliwa, płyn, akumulator

| Dane | Blok.pole |
|---|---|
| Ciśnienie atmosferyczne | 113.4 (odczyt raz po połączeniu) |
| RPM, load, doładowanie zadane/rzeczywiste | 115.1–115.4 |
| Napięcie (ECU), płyn, IAT | 4.2, 4.3, 4.4 |
| MAF, przepustnica, kąt zapłonu | 3.2, 3.3, 3.4 |
| Cofanie zapłonu cyl. 1–4 | 20.1–20.4 |
| N75 % | 118.3 |
| Prędkość | 5.3 |
| Lambda / korekty | 1.3, 32.1, 32.2 |
| Akumulator | `ATRV` (pomiar w adapterze) |

Temperatury oleju nie da się odczytać przez ELM327. Czujnik jest podłączony do zegarów (adres 17),
a te odrzucają usługi `21`, `22` i `1A`.

## Budowanie
- ESP32 Arduino core **2.0.17** (nie działa z 3.x)
- Biblioteka **M5Unified**
- Płytka: `ESP32 Dev Module`, Partition Scheme: `Huge APP`, Upload Speed: **115200**
  (przy 921600 wgrywanie przez CH9102 na Macu się wywala)

arduino-cli:
```
arduino-cli core install esp32:esp32@2.0.17
arduino-cli lib install M5Unified
arduino-cli compile -b esp32:esp32:esp32:PartitionScheme=huge_app .
arduino-cli upload  -b esp32:esp32:esp32:PartitionScheme=huge_app,UploadSpeed=115200 -p <port> .
```
