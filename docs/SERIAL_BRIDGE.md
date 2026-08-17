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

## Build

Prérequis :

- Toolchain ARM GNU (`arm-none-eabi-gcc`) — ce dépôt accepte un tarball
  officiel dans `.toolchain/` (ignoré par git)
- Python 3 + venv pour packer le firmware et l’outil PC

```bash
# Optionnel : toolchain locale (si Homebrew est incomplet)
# placer arm-gnu-toolchain-*-arm-none-eabi dans .toolchain/

python3 -m venv .venv
.venv/bin/pip install crcmod pyserial

make -j$(sysctl -n hw.ncpu)
# artefacts : firmware.bin  firmware.packed.bin
# copie aussi possible dans compiled-firmware/
```

Flags utiles (déjà réglés par défaut dans ce fork) :

| Flag | Défaut | Rôle |
|------|--------|------|
| `ENABLE_SERIAL_BRIDGE` | 1 | Mode tunnel FSK |
| `ENABLE_ANALOG_PTT` | 0 | PTT vocal analogique (off = évite le RF dans le USB/Mac) |
| `ENABLE_UART` | 1 | Câble prog + bridge |
| `ENABLE_SPECTRUM` / `FMRADIO` / `VOX` / `DTMF_CALLING` | 0 | Désactivés pour libérer de la flash |

Taille typique : ~48 Ko de code (budget flash ~60 Ko utilisables).

## Flash

1. Éteindre la radio.
2. Maintenir **PTT**, allumer → LED blanche fixe (mode flash).
3. Brancher le câble de programmation.
4. Flasher `firmware.packed.bin` avec :
   - [uvtools](https://egzumer.github.io/uvtools/) (navigateur), ou
   - un flasher communautaire compatible K5 (`k5prog`, etc.).
5. Relâcher / redémarrer normalement.

Répéter sur **les deux** radios.

## Activation du mode Serial Bridge

Le firmware **démarre directement** sur l’écran `SER BRIDGE` (après l’écran
d’accueil). **EXIT** quitte le mode et revient à la radio normale ; le prochain
allumage relance le bridge.

Pour y revenir sans redémarrer : Menu → `F1Shrt` / `F1Long` / `F2Shrt` /
`F2Long` / `M Long` → assigner **`SER BRIDGE`** à une touche latérale, puis
appuyer sur cette touche.

Avant d’utiliser le tunnel :

1. Sur chaque radio, régler **la même fréquence**, même pas de fréquence,
   **CTCSS/DCS off**, bande passante large ou étroite identique, puissance
   adaptée (LOW pour les essais).
2. L’écran affiche `SER BRIDGE` (fréquence + compteurs TX/RX/E).

Pendant le bridge :

- Le protocole Chirp/programmation UART est **suspendu**.
- Le PTT vocal analogique est **coupé par défaut** dans ce firmware
  (`ENABLE_ANALOG_PTT=0`) : appuyer sur PTT n’émet plus en FM, donc plus de
  parasites USB/écran Mac. Le tunnel FSK continue de transmettre via le port
  série, pas via le PTT.
- Pour réactiver la voix analogique (câble **débranché**) : recompiler avec
  `make ENABLE_ANALOG_PTT=1`.

## PTT + câble USB + Mac (écran qui bug)

C’est un problème connu du connecteur Kenwood, **pas un bug d’affichage macOS** :

1. Sur beaucoup de câbles CH340, la bague PTT du jack 2,5 mm est câblée sur **DTR/RTS**.
   Ouvrir le port série (ou appuyer sur PTT) **fait émettre la radio**.
2. L’émission à côté du Mac, avec le câble USB comme antenne, produit des
   parasites USB → clignotement / freeze de l’écran, déconnexion du port.
3. Les octets binaires reçus (bruit UART) sont interprétés par Terminal.app /
   `screen` comme des séquences ANSI → l’écran du terminal « part en sucette ».

À faire :

- **Ne pas appuyer sur PTT** tant que le câble de programmation est branché
  sur un ordinateur. En mode `SER BRIDGE`, le firmware ignore déjà le PTT vocal.
- Utiliser l’outil `tools/serial_bridge_pc.py` (il force DTR/RTS à 0 et filtre
  les caractères de contrôle). Éviter `screen` / Serial Monitor pendant les tests.
- Utiliser `/dev/cu.usbserial-*` (pas `/dev/tty.usbserial-*`).
- Puissance **LOW**, radio un peu éloignée du Mac, ferrite sur le USB si besoin.
- Pour de la voix : **débrancher le câble** avant d’émettre.

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

## Protocole air (V1)

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

- Half-duplex uniquement (pas de full-duplex simultané).
- En n’émettant que d’un côté, le terminal peut afficher `\xff` : ce n’est **pas**
  un message reçu, c’est du bruit USB/UART pendant le TX FSK. Inoffensif.
- Pertes possibles sans retransmission.
- Pas de TCP/IP, VPN, ni chiffrement.
- Chirp inutilisable pendant le bridge.
- Le FSK BK4819 n’est pas un modem haute vitesse.

## Fichiers principaux

- [`app/serial_bridge.c`](../app/serial_bridge.c) — state machine UART ↔ FSK
- [`ui/serial_bridge.c`](../ui/serial_bridge.c) — écran
- [`tools/serial_bridge_pc.py`](../tools/serial_bridge_pc.py) — outil PC
- [`Makefile`](../Makefile) — `ENABLE_SERIAL_BRIDGE`

## Licence

Le firmware de base reste sous Apache-2.0 (Dual Tachyon / egzumer). Les ajouts
Serial Bridge dans ce dépôt suivent la même licence.
