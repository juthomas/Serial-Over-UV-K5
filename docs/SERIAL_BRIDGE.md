# UV-K5 Serial Bridge

Tunnel série transparent entre deux PC via deux Quansheng UV-K5, sur la
fréquence VFO courante. Trois familles air (même écran) :

- **FSK** ~1200 baud (modem interne BK4819) — rapide, sonne comme un hash digital
- **Data** lent — trame nibble identique ; **`F` (`#`)** cycle les gammes
  (TEL / CHR / MAJ / MIN / DOR / PEN / BLU)
- **Morse (CW)** — texte UART en Morse ITU (dit 800 Hz, dah 600 Hz)
- **`*`** cycle **FSK → data → Morse → FSK**. **SIDE1** : tempo data /
  1200↔2400 FSK, ou WPM Morse (20…100)

`TEL` est le DTMF téléphone (deux tons). Les autres gammes émettent un
**ton pur** par symbole, dans la bande Goertzel du BK4819 (~784–1865 Hz).
Les deux radios doivent afficher la **même** gamme (et le même tempo SIDE1).

```
PC1  --USB/UART 38400-->  UV-K5 A  --FSK / DTMF / CW RF-->  UV-K5 B  --USB/UART-->  PC2
```

Base : firmware communautaire [egzumer](https://github.com/egzumer/uv-k5-firmware-custom)
avec le flag `ENABLE_SERIAL_BRIDGE`.

## Rappel réglementaire (France / UE)

- **Émettre** sur les bandes radioamateur nécessite une **licence radioamateur**
  valide et le respect des fréquences, puissances et modes autorisés.
- Ce firmware n’ajoute **pas** de chiffrement (interdit ou fortement restreint
  sur bande amateur dans la plupart des pays).
- L’utilisateur est seul responsable du choix de fréquence / puissance.
  Ne pas utiliser hors cadre légal (PMR446, etc. selon règles locales).

## Matériel

- 2 × Quansheng UV-K5 (ou compatible famille K5)
- 2 × câbles de programmation Kenwood (USB–UART CH340 / CP210x / FTDI)
- 2 PC, ou 1 PC avec 2 ports série USB
- Antennes adaptées ; idéalement tests à faible puissance et courte distance

Le câble de programmation parle **UART au MCU** (pas l’audio). Aucun AIOC
n’est requis : FSK, DTMF et Morse sont générés dans le BK4819.

## Radios V1 vs V3 (flash)

Deux MCU différents : **ne jamais** envoyer un binaire V1 à uvtools2, ni un
binaire V3 à uvtools. C’est ce qui laisse l’écran noir sur un UV-K5(8).

| Radio | MCU | Make | Fichier | Outil |
|-------|-----|------|---------|-------|
| UV-K5 **(88)** / V1 | DP32G030 (Cortex-M0) | `make` ou `make v1` | `compiled-firmware/firmware-v1.packed.bin` (alias `firmware.packed.bin`) | [uvtools](https://egzumer.github.io/uvtools/) — **packed** egzumer |
| UV-K5 **(8)** / V3 | PY32F071 (Cortex-M0+) | `make v3` | `compiled-firmware/firmware-v3.bin` | [uvtools2](https://armel.github.io/uvtools2/) — **`.bin` brut**, pas packed |

`make` sans argument reste **V1**. `make all-hw` construit les deux.

Les trames air FSK, data et Morse sont **identiques** entre V1 et V3 : un V1
et un V3 peuvent se parler **s’ils sont sur le même mode** (les deux FSK, les
deux data **et la même gamme**, ou les deux `SER CW`, plus le **même tempo**
SIDE1 / WPM).

Le V3 (bootloader `7.00.07`) se récupère avec un firmware F4HWN Fusion V3
(ex. `f4hwn.fusion.v5.7.0.bin`) via uvtools2 si l’écran reste noir.

## Build

Prérequis :

- Toolchain ARM GNU (`arm-none-eabi-gcc`) — ce dépôt accepte un tarball
  officiel dans `.toolchain/` (ignoré par git). Le V3 utilise le même
  compilateur avec `-mcpu=cortex-m0plus` (CMake/Ninja).
- Python 3 + venv pour packer le firmware V1 et l’outil PC
- `cmake` ≥ 3.22 et `ninja` pour le build V3
- Sous-module / clone `vendor/uv-k5-v3` (F4HWN tag `v5.7.0`) — `make v3`
  le récupère si besoin

```bash
# Optionnel : toolchain locale (si Homebrew est incomplet)
# placer arm-gnu-toolchain-*-arm-none-eabi dans .toolchain/

python3 -m venv .venv
.venv/bin/pip install crcmod pyserial

make -j$(sysctl -n hw.ncpu)          # V1
make v3                              # V3
make -j$(sysctl -n hw.ncpu) all-hw   # les deux
# artefacts dans compiled-firmware/
```

Flags utiles (déjà réglés par défaut dans ce fork) :

| Flag | Défaut | Rôle |
|------|--------|------|
| `ENABLE_SERIAL_BRIDGE` | 1 | Mode tunnel FSK / data / Morse |
| `SERIAL_BRIDGE_DEFAULT_MODE` | 0 (FSK) | Mode au boot (`1` = data / TEL, `2` = Morse) |
| `ENABLE_ANALOG_PTT` | 0 | PTT vocal aussi hors du bridge (off = VFO principal USB-safe) |
| `ENABLE_UART` | 1 | Câble prog + bridge |
| `ENABLE_SPECTRUM` / `FMRADIO` / `VOX` / `DTMF_CALLING` | 0 | Désactivés pour libérer de la flash |

Taille typique : ~48 Ko de code (budget flash ~60 Ko utilisables).

## Flash

1. Éteindre la radio.
2. Maintenir **PTT**, allumer → LED blanche fixe (mode flash).
3. Brancher le câble de programmation.
4. Flasher le **bon** fichier :
   - **V1 (88)** : `compiled-firmware/firmware-v1.packed.bin` avec
     [uvtools](https://egzumer.github.io/uvtools/).
   - **V3 (8)** : `compiled-firmware/firmware-v3.bin` avec
     [uvtools2](https://armel.github.io/uvtools2/).
5. Relâcher / redémarrer normalement.

Répéter sur **les deux** radios (même fréquence, **même mode** FSK / data /
CW, et en data la **même** gamme ; **même** tempo SIDE1 / WPM).

## Activation du mode Serial Bridge

Le firmware **démarre directement** sur l’écran `SER FSK` (après l’écran
d’accueil), tuné sur **433.000 MHz**. **EXIT** quitte le mode et revient à la
radio normale ; le prochain allumage relance le bridge (à nouveau sur 433.000,
mode FSK par défaut).

Pour y revenir sans redémarrer : Menu → `F1Shrt` / `F1Long` / `F2Shrt` /
`F2Long` / `M Long` → assigner **`SER BRIDGE`** à une touche latérale, puis
appuyer sur cette touche.

Sur l’écran `SER FSK` / `SER TEL` / `SER CW` / … :

1. Taper la fréquence (6 chiffres, ex. `433000` → 433.000 MHz) ou **UP/DOWN**
   pour stepper. **EXIT** efface un chiffre, ou quitte le mode si rien n’est
   en saisie.
2. **`*`** cycle FSK → data → Morse (`SER CW`) → FSK (les deux radios
   doivent matcher). Le footer `*=…` indique le **prochain** mode.
3. **`F` (`#`)** : en data, cycle les gammes (no-op en FSK, et pendant une
   saisie de fréquence). En Morse, le titre peut afficher **1T** / **2T**
   (footer `F=…`) ; l’air est **toujours** dit 800 Hz / dah 600 Hz (un IRQ
   SelCall ne porte pas la durée : l’ancien 1T à 600 Hz sortait `EEEE`).
   Les deux radios doivent matcher le WPM.
4. **SIDE1** (ou SIDE2) : en FSK / data, alterne le tempo **actuel** et
   **2K**. Titre `SER FSK` / `SER MAJ` ↔ `SER FSK 2K` / `SER MAJ 2K`.
   En data : **2K** = 50+30 ms (DTMF et SelCall n’entendent pas 20 ms).
   En FSK : **2400 baud** au lieu de 1200. En Morse,
   SIDE1 **cycle le WPM** : `SER CW 20` → `30` → `40` → `60` → `80` →
   `100` → `20` (unité PARIS 60…12 ms). Les deux radios doivent matcher,
   sinon `E:` / silence.
5. **PTT** = TX vocal analogique FM (comme un talkie). Relâcher pour revenir
   en écoute. Les coeffs Goertzel stock / DTMF téléphone sont restaurés
   pendant la QSO et à la sortie du bridge.
6. **XOR data / analogique** : au repos le modem écoute (HP fermé). Un burst
   FSK, une gamme ou du Morse **n’est plus** traité comme une QSO voix (ça
   coupait `REG_58` et jetait le paquet). Le HP reste fermé ; seul le
   **PTT local** coupe le modem (`LeaveFsk`). En data / Morse, une rafale
   n’ouvre pas le HP.

Les deux radios doivent être sur **la même fréquence**, **le même mode**, et
en data **la même gamme** (et le **même tempo** SIDE1). En Morse, le même
WPM. CTCSS/DCS off, bande passante identique, puissance adaptée (LOW pour
les essais).

### Gammes (bande Goertzel, touche F)

Même alphabet air (`0-9A-D*#` → nibble 0–15). `*` de sync = degré 14 de la
table. En `TEL`, RX = décodeur DTMF téléphone (`DTMF_GetCharacter`). En
gamme, RX = **index SelCall 0–15** brut (pas le mapping clavier).

Le décodeur BK4819 n’entend à peu près que **~700–1900 Hz** (comme le DTMF).
Les anciennes tables en do4 (~262 Hz) étaient trop graves : le SelCall ne
tirait pas, d’où « seul TEL marche ». Les 16 notes sont maintenant dans
**784–1865 Hz**, avec un coefficient Goertzel distinct par nibble (sinon
égalité → pas d’IRQ, comme le Morse 1T). `CHR` = 12-TET depuis sol5 ; les
autres gardent le profil d’intervalles (majeur / mineur / …) comprimé dans
cette fenêtre — plus une gamme de do grave, plutôt un registre aigu.

| UI | Mode | 16 hauteurs (symboles 0–F) |
|----|------|------------------------------|
| `TEL` | DTMF téléphone | paires clavier (pas de `REG_09` custom) |
| `CHR` | Chromatique | sol5 → si♭6, demi-tons 12-TET |
| `MAJ` | Majeur | profil do-ré-mi-fa-sol-la-si, comprimé 784–1865 Hz |
| `MIN` | Mineur naturel | profil do-ré-mib-fa-sol-lab-sib, idem |
| `DOR` | Dorien | profil do-ré-mib-fa-sol-la-sib |
| `PEN` | Pentatonique majeur | profil do-ré-mi-sol-la |
| `BLU` | Blues | profil do-mib-fa-fa#-sol-sib |

Un talkie tiers en `MAJ` entend une montée d’intervalles majeurs, pas un
clavier. Si une gamme discrimine mal, revenir sur **`TEL`**.

Pendant le bridge :

- Le protocole Chirp/programmation UART est **suspendu**.
- Voix analogique et tunnel data partagent le même canal (half-duplex) :
  pendant une rafale le PTT attend ; pendant le PTT l’UART est bufferisé.
- Les rafales **FSK** s’entendent comme un modem / bruit blanc. Les rafales
  **data** s’entendent comme un clavier (`TEL`) ou une gamme (`MAJ`, …).
  Le **Morse** s’entend comme du CW (dit 800 Hz, dah 600 Hz). Le squelch ouvert par le FSK
  ne coupe plus le modem ni n’ignore le FIFO.
- Hors de cet écran, le PTT analogique reste coupé si `ENABLE_ANALOG_PTT=0`
  (protection USB/Mac). Recompiler avec `make ENABLE_ANALOG_PTT=1` pour le
  réactiver aussi sur le VFO principal.

## PTT + câble USB + Mac (écran qui bug)

C’est un problème connu du connecteur Kenwood, **pas un bug d’affichage macOS** :

1. Sur beaucoup de câbles CH340, la bague PTT du jack 2,5 mm est câblée sur **DTR/RTS**.
   Ouvrir le port série (ou appuyer sur PTT) **fait émettre la radio**.
2. L’émission à côté du Mac, avec le câble USB comme antenne, produit des
   parasites USB → clignotement / freeze de l’écran, déconnexion du port.
3. Les octets binaires reçus (bruit UART) sont interprétés par Terminal.app /
   `screen` comme des séquences ANSI → l’écran du terminal « part en sucette ».

À faire :

- En mode `SER BRIDGE`, le PTT vocal est **actif**. Avec le câble USB branché
  sur un ordinateur, **débrancher le câble avant d’émettre à la voix**, ou
  s’attendre à des glitches USB / écran.
- Utiliser l’outil `tools/serial_bridge_pc.py` (il force DTR/RTS à 0 et filtre
  les caractères de contrôle). Éviter `screen` / Serial Monitor pendant les tests.
- Utiliser `/dev/cu.usbserial-*` (pas `/dev/tty.usbserial-*`).
- Puissance **LOW**, radio un peu éloignée du Mac, ferrite sur le USB si besoin.

## Usage PC

```bash
source .venv/bin/activate   # ou .venv/bin/python …

python tools/serial_bridge_pc.py list
python tools/serial_bridge_pc.py terminal /dev/cu.usbserial-XXXX
python tools/serial_bridge_pc.py test /dev/cu.usbserial-A /dev/cu.usbserial-B
python tools/serial_bridge_pc.py send /dev/cu.usbserial-A --bytes 400
```

Baud : **38400 8N1** (identique au firmware egzumer).

## Raspberry Pi 3B+

Sur Linux le câble CH340/CP210x apparaît en `/dev/ttyUSB0` (pas `/dev/cu.*`).

**Une fois sur le Pi** (SSH ou clavier) :

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip
sudo usermod -aG dialout "$USER"
# se déconnecter / reconnecter (ou reboot) pour le groupe dialout
```

Copier l’outil depuis le Mac (adapter l’IP du Pi) :

```bash
# sur le Mac
scp /Users/juthomas/Documents/Electronics/UV-K5/tools/serial_bridge_pc.py pi@IP_DU_PI:~/
scp /Users/juthomas/Documents/Electronics/UV-K5/tools/requirements.txt pi@IP_DU_PI:~/
```

Puis sur le Pi :

```bash
python3 -m venv ~/uvk5-venv
source ~/uvk5-venv/bin/activate
pip install pyserial

# brancher le câble, radio en SER BRIDGE
ls -l /dev/ttyUSB* /dev/ttyACM*
python ~/serial_bridge_pc.py list
```

Usage (remplacer `ttyUSB0` si `list` montre autre chose) :

```bash
source ~/uvk5-venv/bin/activate

# écouter
python ~/serial_bridge_pc.py terminal /dev/ttyUSB0

# envoyer un long texte
python ~/serial_bridge_pc.py send /dev/ttyUSB0 --bytes 400
python ~/serial_bridge_pc.py send /dev/ttyUSB0 --text "Bonjour depuis le Pi"
```

Deux radios = deux Pi (ou un Pi avec 2 câbles : `ttyUSB0` et `ttyUSB1`) :

```bash
# Pi / port A = TX
python ~/serial_bridge_pc.py send /dev/ttyUSB0 --bytes 400

# Pi / port B = RX (autre terminal SSH)
python ~/serial_bridge_pc.py terminal /dev/ttyUSB1
```

Alim Pi 3B+ : alimenter au **chargeur officiel 2.5 A**, pas en bus USB depuis le Mac.
Radio en **LOW**, un peu éloignée du Pi (le RF peut quand même faire décrocher l’USB).

Si `terminal` affiche `device reports readiness to read but returned no data`
(ou `UART … disconnected`) : le CH340 s’est **débranché tout seul** pendant
un TX (EMI). Ce n’est pas un crash du firmware. Rebrancher le câble,
`ls /dev/ttyUSB*` (souvent `ttyUSB1` au lieu de `ttyUSB0`), relancer
`terminal`. Recopier `tools/serial_bridge_pc.py` depuis ce dépôt : l’outil
sort alors proprement au lieu d’un traceback dans le thread lecteur.

### Tester un long texte

Il faut **deux radios** en `SER BRIDGE` (même fréquence). Un seul terminal
n’affichera jamais le texte émis : le lien n’est pas une boucle locale.

**PC récepteur** (laisser tourner) :

```bash
python tools/serial_bridge_pc.py terminal /dev/cu.usbserial-B
```

**PC émetteur** (autre terminal) :

```bash
# FSK : ~400 octets, cadencés par XON/XOFF (~50–70 o/s)
# data (TEL / gamme) : même commande, beaucoup plus lent (~2–3 o/s) — XOFF tout seul
python tools/serial_bridge_pc.py send /dev/cu.usbserial-A --bytes 400

# ou un fichier / un paragraphe
python tools/serial_bridge_pc.py send /dev/cu.usbserial-A --file message.txt
python tools/serial_bridge_pc.py send /dev/cu.usbserial-A --text "Votre long texte ici…"
```

`terminal` et `send` cadencent via XON/XOFF (le firmware envoie XOFF avant
chaque rafale). En FSK, 56 o / ~1 s. En data, 8 o / ~2–3 s : coller un roman
prendra plusieurs minutes. `--pause` plus long si un vieux firmware n’envoie
pas de XON.

Sur l’écran radio A : compteur `TX` qui monte. Sur B : `RX`, et le texte sort
sur le PC B. Durée typique FSK : 400 octets ≈ 7–8 s. DTMF : 400 octets ≈
2–3 min.

### Test recommandé

1. Deux radios en bridge, **même mode** (`*` → les deux `SER FSK`, les deux
   `SER TEL`, ou les deux `SER CW`), antennes proches, LOW power, fréquence
   légale, CTCSS off.
2. `serial_bridge_pc.py test PORT_A PORT_B` (timeout 20 s, OK FSK et TEL).
3. Les deux `F` jusqu’à `MAJ` : `test --timeout 20` ; un talkie tiers entend
   une montée majeure (registre aigu), pas un clavier.
4. Une radio `MAJ` et l’autre `MIN` : `E:` / pas de texte (normal).
5. `CHR` : 16 demi-tons sol5→si♭6 audibles.
6. `*` jusqu’à `SER CW` des deux côtés : taper `HELLO` — audible en Morse
   (dit 800 / dah 600) ; le PC B reçoit `HELLO`. Octet hors table (`=` inclus) → `=HH`
   (ex. `=` → `=3D`).
7. `*` FSK toujours OK ; PTT voix OK ; hors bridge le décodeur DTMF stock
   est restauré.
8. Compteurs `TX`/`RX` à l’écran et `E:` proche de 0 quand les modes
   matchent.

## Protocole air (V1 et V3)

### FSK

Trame FSK fixe 72 octets (comme Air Copy), 1200 baud :

| Champ | Contenu |
|-------|---------|
| sync | `0xABCD` |
| seq + len | `(seq << 8) \| len` avec `len` ∈ 1..56 |
| payload | jusqu’à 56 octets (+ padding) |
| CRC16-CCITT | sur 66 octets (mot `[1]` + 64 octets zone payload) |
| sync fin | `0xDCBA` |

- Half-duplex, **sans ACK air** (best-effort). Les doublons de `seq` sont ignorés.
- CSMA simple : pas de TX si le squelch est ouvert (canal occupé).
- Flush UART → RF après ~30 ms d’inactivité ou buffer plein (56 B).
- Câble : XON/XOFF (firmware) pour que le PC n’inonde pas le buffer UART.

Débit utile typique FSK : **~50–70 octets/s** (56 o / trame + mute TX).

### Data (TEL / gammes)

Trame de nibbles (symbole = 4 bits : `0-9 A-D * #`), **inchangée** d’une
gamme à l’autre :

| Champ | Symboles |
|-------|----------|
| sync | `*` `*` |
| LEN | 1 nibble, payload 1..8 octets |
| SEQ | 1 nibble (0..15), anti-doublon |
| DATA | 2×LEN nibbles (high puis low) |
| CRC8 | 2 nibbles, poly 0x07 sur LEN + SEQ + DATA |

- Ton 80 ms (`TEL`) ou 120 ms (gammes) + silence 80 ms → **~2–3 octets/s**.
  SIDE1 **2K** = 50+30 ms pour TEL **et** les gammes (le SelCall n’entend
  pas 20 ms). Le décodeur SelCall reste armé pendant la rafale (le squelch
  analogique ne le remplace plus par le DTMF téléphone).
- Flush UART après 8 octets ou ~80 ms d’inactivité.
- Half-duplex, sans ACK, CSMA squelch, comme le FSK.
- Un talkie stock n’affiche rien : il entend seulement les tons (`TEL` =
  clavier, les autres = notes de la gamme).
- `s_scale` et `s_tempo_fast` sont en RAM (session), comme le mode FSK / data.

### Morse (`SER CW`)

Pas de trame `** LEN SEQ CRC` : le texte UART est envoyé **tel quel** en
Morse ITU (A–Z, 0–9, espace, ponctuation courante). Minuscules → majuscules.
`\r` `\n` `\t` → espace. Un octet hors table (et `=` lui-même) devient
`=` + 2 hex Morse (`=` → `=3D`).

| Élément | Durée |
|---------|--------|
| unité | SIDE1 : 60 / 40 / 30 / 20 / 15 / 12 ms (20…100 WPM, PARIS) |
| dit | 1 u ON + 1 u OFF |
| dah | 3 u ON + 1 u OFF |
| lettre | 3 u OFF |
| mot | 7 u OFF (après le flush idle, comme le data) |

- Air : dit **800 Hz**, dah **600 Hz** (bins SelCall 0 / 1). Un IRQ 5-tone
  est une impulsion : allonger chaque hit à 1 unité transformait tout en
  dits (`E` `I` `S` `H`). PA monté le temps du flush.
- RX : dit/dah par le **bin**, pas par la durée. Les deux radios en
  `SER CW`, même WPM.
- Flush UART après 16 octets ou ~80 ms d’inactivité.
- Un talkie stock n’affiche rien : il entend le Morse (un ou deux tons).

## Limites

- Half-duplex uniquement (pas de full-duplex simultané) : PTT analogique et
  TX data ne peuvent pas coexister.
- Un mix FSK↔data↔CW, deux gammes différentes, deux tempos SIDE1 / WPM,
  ou deux WPM Morse = silence ou `E:` (pas de crash).
- La voix peut déclencher de faux symboles DTMF : sync `**` + CRC8 les
  rejettent (`E:` peut bouger un peu pendant une QSO).
- En n’émettant que d’un côté, le terminal peut voir du bruit USB/UART pendant
  le TX FSK (CH340 : `\xff`, PL2303 : `\x00`). Ce n’est **pas** un message
  reçu. L’outil `terminal` les ignore. Inoffensif.
- Pertes possibles sans retransmission.
- Pas de TCP/IP, VPN, ni chiffrement.
- Chirp inutilisable pendant le bridge.
- Le FSK BK4819 n’est pas un modem haute vitesse. Le mode data est
  volontairement lent pour rester musical. `TEL` reste la planche de salut
  si une gamme SelCall discrimine mal.

## Fichiers principaux

- [`app/serial_bridge.c`](../app/serial_bridge.c) — state machine UART ↔ FSK/data/Morse (V1)
- [`ui/serial_bridge.c`](../ui/serial_bridge.c) — écran (V1)
- [`v3-overlay/App/app/serial_bridge.c`](../v3-overlay/App/app/serial_bridge.c) — port PY32 (V3)
- [`patches/v3-serial-bridge.patch`](../patches/v3-serial-bridge.patch) — hooks F4HWN V3
- [`tools/serial_bridge_pc.py`](../tools/serial_bridge_pc.py) — outil PC
- [`Makefile`](../Makefile) — `make` / `make v1` / `make v3` / `make all-hw`

## Licence

Le firmware de base reste sous Apache-2.0 (Dual Tachyon / egzumer). Les ajouts
Serial Bridge dans ce dépôt suivent la même licence.
