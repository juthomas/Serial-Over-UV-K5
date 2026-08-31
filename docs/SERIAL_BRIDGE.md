# UV-K5 Serial Bridge

Tunnel série transparent entre deux PC via deux Quansheng UV-K5, sur la
fréquence VFO courante, en FSK ~1200 baud (modem interne BK4819).

```
PC1  --USB/UART 38400-->  UV-K5 A  --FSK RF-->  UV-K5 B  --USB/UART-->  PC2
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
n’est requis : le FSK est généré dans le BK4819.

## Radios V1 vs V3 (flash)

Deux MCU différents : **ne jamais** envoyer un binaire V1 à uvtools2, ni un
binaire V3 à uvtools. C’est ce qui laisse l’écran noir sur un UV-K5(8).

| Radio | MCU | Make | Fichier | Outil |
|-------|-----|------|---------|-------|
| UV-K5 **(88)** / V1 | DP32G030 (Cortex-M0) | `make` ou `make v1` | `compiled-firmware/firmware-v1.packed.bin` (alias `firmware.packed.bin`) | [uvtools](https://egzumer.github.io/uvtools/) — **packed** egzumer |
| UV-K5 **(8)** / V3 | PY32F071 (Cortex-M0+) | `make v3` | `compiled-firmware/firmware-v3.bin` | [uvtools2](https://armel.github.io/uvtools2/) — **`.bin` brut**, pas packed |

`make` sans argument reste **V1**. `make all-hw` construit les deux.

La trame FSK air est **identique** (`0xABCD` / seq+len / CRC / `0xDCBA`) : un
V1 et un V3 peuvent se parler.

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
| `ENABLE_SERIAL_BRIDGE` | 1 | Mode tunnel FSK |
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

Répéter sur **les deux** radios (même fréquence, même trame air).

## Activation du mode Serial Bridge

Le firmware **démarre directement** sur l’écran `SER BRIDGE` (après l’écran
d’accueil), tuné sur **433.000 MHz**. **EXIT** quitte le mode et revient à la
radio normale ; le prochain allumage relance le bridge (à nouveau sur 433.000).

Pour y revenir sans redémarrer : Menu → `F1Shrt` / `F1Long` / `F2Shrt` /
`F2Long` / `M Long` → assigner **`SER BRIDGE`** à une touche latérale, puis
appuyer sur cette touche.

Sur l’écran `SER BRIDGE` :

1. Taper la fréquence (6 chiffres, ex. `433000` → 433.000 MHz) ou **UP/DOWN**
   pour stepper. **EXIT** efface un chiffre, ou quitte le mode si rien n’est
   en saisie.
2. **PTT** = TX vocal analogique FM (comme un talkie). Relâcher pour revenir
   en écoute.
3. **XOR FSK / analogique** : au repos le modem FSK écoute (HP fermé). Pendant
   une QSO analogique (`StartListening` / PTT) le FSK est coupé ; le HP n’est
   ouvert/fermé que par le firmware stock. Fin de QSO → réarmement FSK.

Les deux radios doivent être sur **la même fréquence**. CTCSS/DCS off, bande
passante identique, puissance adaptée (LOW pour les essais).

Pendant le bridge :

- Le protocole Chirp/programmation UART est **suspendu**.
- Voix analogique et tunnel FSK partagent le même canal (half-duplex) :
  pendant un paquet FSK le PTT attend ; pendant le PTT l’UART est bufferisé.
- Les rafales FSK s’entendent comme un modem. Pendant une QSO analogique le
  FSK est coupé : plus de faux `E:` CRC dus au bruit FM.
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

### Tester un long texte

Il faut **deux radios** en `SER BRIDGE` (même fréquence). Un seul terminal
n’affichera jamais le texte émis : le lien n’est pas une boucle locale.

**PC récepteur** (laisser tourner) :

```bash
python tools/serial_bridge_pc.py terminal /dev/cu.usbserial-B
```

**PC émetteur** (autre terminal) :

```bash
# ~400 octets générés, envoyés 56 o toutes les 1 s (~56 o/s)
python tools/serial_bridge_pc.py send /dev/cu.usbserial-A --bytes 400

# ou un fichier / un paragraphe
python tools/serial_bridge_pc.py send /dev/cu.usbserial-A --file message.txt
python tools/serial_bridge_pc.py send /dev/cu.usbserial-A --text "Votre long texte ici…"
```

Ne pas coller un roman dans `terminal` d’un coup : le buffer UART de la radio
fait 128 octets, le FSK ~56 octets/trame. `send` cadence tout seul.

Sur l’écran radio A : compteur `TX` qui monte. Sur B : `RX`, et le texte sort
sur le PC B. Durée typique : 400 octets ≈ 7–8 s.

### Test recommandé

1. Deux radios en bridge, antennes proches, LOW power, fréquence légale.
2. `serial_bridge_pc.py test PORT_A PORT_B`
3. Vérifier les compteurs `TX`/`RX` à l’écran et `E:` (erreurs CRC) proche de 0.

## Protocole air (V1 et V3)

Trame FSK fixe 72 octets (comme Air Copy), 1200 baud :

| Champ | Contenu |
|-------|---------|
| sync | `0xABCD` |
| seq + len | `(seq << 8) \| len` avec `len` ∈ 1..56 |
| payload | jusqu’à 56 octets (+ padding) |
| CRC16-CCITT | sur 66 octets (mot `[1]` + 64 octets zone payload) |
| sync fin | `0xDCBA` |

- Half-duplex, **sans ACK** (best-effort). Les doublons de `seq` sont ignorés.
- CSMA simple : pas de TX si le squelch est ouvert (canal occupé).
- Flush UART → RF après ~30 ms d’inactivité ou buffer plein (56 B).

Débit utile typique : **~500–900 octets/s** selon conditions (pas un câble USB).

## Limites

- Half-duplex uniquement (pas de full-duplex simultané) : PTT analogique et
  TX FSK ne peuvent pas coexister.
- En n’émettant que d’un côté, le terminal peut voir du bruit USB/UART pendant
  le TX FSK (CH340 : `\xff`, PL2303 : `\x00`). Ce n’est **pas** un message
  reçu. L’outil `terminal` les ignore. Inoffensif.
- Pertes possibles sans retransmission.
- Pas de TCP/IP, VPN, ni chiffrement.
- Chirp inutilisable pendant le bridge.
- Le FSK BK4819 n’est pas un modem haute vitesse.

## Fichiers principaux

- [`app/serial_bridge.c`](../app/serial_bridge.c) — state machine UART ↔ FSK (V1)
- [`ui/serial_bridge.c`](../ui/serial_bridge.c) — écran (V1)
- [`v3-overlay/App/app/serial_bridge.c`](../v3-overlay/App/app/serial_bridge.c) — port PY32 (V3)
- [`patches/v3-serial-bridge.patch`](../patches/v3-serial-bridge.patch) — hooks F4HWN V3
- [`tools/serial_bridge_pc.py`](../tools/serial_bridge_pc.py) — outil PC
- [`Makefile`](../Makefile) — `make` / `make v1` / `make v3` / `make all-hw`

## Licence

Le firmware de base reste sous Apache-2.0 (Dual Tachyon / egzumer). Les ajouts
Serial Bridge dans ce dépôt suivent la même licence.
