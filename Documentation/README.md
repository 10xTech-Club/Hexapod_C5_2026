# How Our Robots Talk to Each Other
### A Guide for Young Makers & Students

---

## Meet the Team

We have **three robots** that work together like a team:

```
        🛰️  SATCOM (Mission Control)
            ESP32 chip
            The BOSS — receives info from both robots
            and talks to the web dashboard

       /            \
      /  ESP-NOW     \  ESP-NOW
     /   (wireless)   \  (wireless)
    /                  \

🦾 HEXAPOD            🚗 ROVER
   ESP8266 chip          ESP32 chip
   6-legged walker       Wheeled robot
   Sends: sensor data    Sends: sensor data
   Gets: move commands   Gets: move commands
```

None of them need the internet. They talk directly to each other — like walkie-talkies!

---

## Part 1: What is ESP-NOW?

### Think of it like this:

- **Normal WiFi** = Sending a letter through the post office. You write it, seal it, drop it off, wait for a postman to deliver it. Many steps, takes time.
- **ESP-NOW** = Shouting across the room to your friend. Direct, instant, no middleman!

ESP-NOW is a special radio trick that lets ESP chips talk directly chip-to-chip — super fast, no router, no internet needed.

### Why is it awesome?

| | Regular WiFi | ESP-NOW |
|--|--|--|
| Needs a router? | YES | **NO** |
| How fast? | Slow (~50ms) | **Super fast (~1ms)** |
| How far? | ~50 meters | **~200 meters** |
| Easy to set up? | Many steps | **Very easy** |

---

## Part 2: Every Robot Has a Name Tag (MAC Address)

Every WiFi chip in the world has a unique ID called a **MAC address**.
Think of it like a **robot's name tag** that is burned into the chip forever.

```
Example:   3C:8A:1F:7E:33:55
           └─┘ └─┘ └─┘ └─┘ └─┘ └─┘
            6 pairs of letters/numbers
            = 6 bytes
            = the robot's permanent address
```

### Our three robots and their name tags:

| Robot | AP MAC Address |
|-------|---------------|
| 🛰️ Satellite | `3C:8A:1F:7E:33:55` |
| 🦾 Hexapod | `42:91:51:5B:7F:A2` |
| 🚗 Rover | `88:57:21:79:B1:19` |

Since these robots never change, we write the addresses directly into the code — like writing your friend's phone number in your contacts. This way the robots know exactly who to call from the moment they turn on!

```cpp
// Inside the Satellite's code — written in advance, never changes:
static uint8_t HEXAPOD_MAC[6] = {0x42, 0x91, 0x51, 0x5B, 0x7F, 0xA2};
static uint8_t ROVER_MAC[6]   = {0x88, 0x57, 0x21, 0x79, 0xB1, 0x19};
```

### Fun fact — The Tricky +1 Offset!

Every ESP32 chip actually has **two** MAC addresses — one for connecting to WiFi (STA) and one for being a mini router (AP). The AP address is always the STA address plus 1 on the last number:

```
Satellite STA MAC:  3C:8A:1F:7E:33:54  (what the upload tool shows)
Satellite AP MAC:   3C:8A:1F:7E:33:55  (last byte +1 — this is what we use!)
```

The ESP8266 (Hexapod) does it differently — it flips a bit in the FIRST byte:
```
Hexapod STA MAC:    40:91:51:5B:7F:A2  (esptool shows this)
Hexapod AP MAC:     42:91:51:5B:7F:A2  (first byte 0x40 → 0x42 — bit flip!)
```

We always use the **AP MAC** for ESP-NOW. The firmware prints it on startup so you can check.

---

## Part 3: The WiFi Channel — Tuning In

Imagine a walkie-talkie. For two walkie-talkies to hear each other, they must be on the same **channel**.

All three of our robots are set to **Channel 6**:

```cpp
WiFi.softAP("SATCOM-ALPHA", "satellite2025", 6);   // Satellite on ch 6
WiFi.softAP("Hexapod-AP",   "hexapod123",    6);   // Hexapod on ch 6
WiFi.softAP("Rover-AP",     "rover2026",     6);   // Rover on ch 6
```

If they were on different channels — they could NOT hear each other!

---

## Part 4: The Message Envelope — Packet Formats

When a robot sends data, it packs everything neatly into a **struct** — like putting things into a labeled envelope.

Every message starts with the same two labels:

```cpp
// Byte 0: What kind of message is this?
#define ESPNOW_MSG_TELEMETRY  0x01   // "Here is my sensor data"
#define ESPNOW_MSG_COMMAND    0x02   // "Please do this action"
#define ESPNOW_MSG_CHAT       0x03   // "Hello, want to chat?"

// Byte 1: Who is sending it?
#define ESPNOW_DEVICE_HEXAPOD  0x01  // "I am the Hexapod"
#define ESPNOW_DEVICE_ROVER    0x02  // "I am the Rover"
```

The Satellite reads just these first two bytes to know immediately: **who sent this, and what is it?**

---

## Part 5: The Three Types of Messages

### Type 1 — Telemetry (Sensor Data)

Robots send their sensor readings to the Satellite regularly. Think of it like a health report!

**Hexapod sends every 2 seconds (26 bytes):**
```
📦 Hexapod Telemetry Packet:
┌─────────────────────────────────────────────┐
│ Byte  0 │ Message type (0x01 = telemetry)   │
│ Byte  1 │ Device ID (0x01 = I am hexapod)   │
│ Bytes 2-5 │ Temperature in °C               │
│ Bytes 6-9 │ Humidity %                      │
│ Bytes 10-11 │ Air quality (gas)             │
│ Bytes 12-13 │ Air quality index             │
│ Bytes 14-15 │ CO2 level                     │
│ Byte 16 │ WiFi signal strength              │
│ Byte 17 │ Is it walking? (1=yes, 0=no)      │
│ Byte 18 │ Which step in the walk cycle      │
│ Byte 19 │ How many browsers are connected   │
│ Bytes 20-25 │ Each leg's angle (6 legs!)    │
└─────────────────────────────────────────────┘
Total: 26 bytes — fits easily in ESP-NOW's 250-byte limit
```

**Rover sends every 0.5 seconds (27 bytes):**
```
📦 Rover Telemetry Packet:
┌──────────────────────────────────────────────┐
│ Byte  0 │ Message type (0x01 = telemetry)    │
│ Byte  1 │ Device ID (0x02 = I am rover)      │
│ Bytes 2-3 │ Distance to obstacle (cm)        │
│ Bytes 4-7 │ Temperature °C                   │
│ Bytes 8-11 │ Humidity %                      │
│ Bytes 12-13 │ Light sensor                   │
│ Bytes 14-15 │ Gas sensor                     │
│ Bytes 16-17 │ Magnetic field sensor           │
│ Byte 18 │ Mode (0=Manual, 1=Auto)            │
│ Byte 19 │ Which direction it is moving       │
│ Byte 20 │ Is there an obstacle? (1=yes)      │
│ Byte 21 │ Is forward blocked? (1=yes)        │
│ Bytes 22-23 │ Robot arm base angle           │
│ Bytes 24-25 │ Robot arm joint angle          │
│ Byte 26 │ Arm pose (HOME / DEPLOY / CUSTOM)  │
└──────────────────────────────────────────────┘
Total: 27 bytes
```

### Type 2 — Command (Instructions from Mission Control)

The Satellite sends orders to the robots:

```
📦 Command Packet (17 bytes):
┌──────────────────────────────────────────────┐
│ Byte  0 │ Message type (0x02 = command)      │
│ Bytes 1-16 │ The command text:               │
│           │ "forward", "backward",           │
│           │ "left", "right", "stop" ...      │
└──────────────────────────────────────────────┘
```

When the Hexapod gets a command, it acts on it:
```cpp
if (strcmp(pendingEspNowCmd, "forward") == 0) {
    currentDirection = FORWARD;
    isWalking = true;      // starts walking!
} else if (strcmp(pendingEspNowCmd, "stop") == 0) {
    stopMovement();        // stops!
}
```

### Type 3 — Chat (Text Messages Between Robots!)

Yes, our robots can send text messages to each other like phone messages!

```
📦 Chat Packet (81 bytes):
┌──────────────────────────────────────────────┐
│ Byte  0 │ Message type (0x03 = chat)         │
│ Bytes 1-16 │ Who sent it ("satellite",       │
│            │ "hexapod", "rover")             │
│ Bytes 17-80 │ The actual text message        │
│             │ e.g. "Hello Hexapod!"          │
└──────────────────────────────────────────────┘
```

### Why `__attribute__((packed))`?

Look at how each packet is defined:
```cpp
struct __attribute__((packed)) EspNowCommand {
  uint8_t  msgType;   // 1 byte
  char     cmd[16];   // 16 bytes
};  // Total: exactly 17 bytes
```

The word `packed` tells the computer: **"Don't add any secret invisible space between the fields."** This is super important because the sender and receiver must agree on the exact size. If one sends 17 bytes and the other expects 18, the message gets scrambled!

---

## Part 6: Setting Up ESP-NOW — 4 Simple Steps

Every robot follows the same recipe at startup:

### Step 1: Create a WiFi hotspot
```cpp
WiFi.mode(WIFI_AP);
WiFi.softAP("MY-ROBOT-NAME", "password", 6);  // Channel 6!
```

### Step 2: Start ESP-NOW
```cpp
esp_now_init();
```

### Step 3: Set up your "mailbox" — register callbacks
```cpp
esp_now_register_send_cb(onEspNowSent);   // runs when YOU send
esp_now_register_recv_cb(onEspNowRecv);   // runs when you RECEIVE
```
A **callback** is like telling your mailbox: "When a letter arrives, ring this bell."
You don't ring it yourself — the system rings it automatically!

### Step 4: Add the people you want to talk to (peers)
```cpp
// Satellite registers BOTH robots at startup:
esp_now_peer_info_t hexPeer = {};
memcpy(hexPeer.peer_addr, HEXAPOD_MAC, 6);  // use Hexapod's name tag
hexPeer.channel = 0;           // 0 = use whatever channel we're on (ch 6)
hexPeer.ifidx   = WIFI_IF_AP;  // use the AP radio interface
hexPeer.encrypt = false;       // no secret code needed
esp_now_add_peer(&hexPeer);
```

Once registered, the Satellite can send directly to the Hexapod anytime — no waiting!

---

## Part 7: How the Satellite Knows WHO Sent the Packet

When a radio packet arrives, **how does the Satellite know if it's from the Hexapod or the Rover?**

### The answer is in byte [1]!

```
📨 Packet arrives...

  data[0] = 0x01  →  "This is a TELEMETRY message"
  data[1] = 0x02  →  "I am the ROVER"
```

```cpp
void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
    uint8_t msgType    = data[0];  // what type?
    uint8_t deviceType = data[1];  // who sent it?

    if (deviceType == ESPNOW_DEVICE_HEXAPOD) {
        // Hexapod data! Handle hexapod stuff...
    }
    else if (deviceType == ESPNOW_DEVICE_ROVER) {
        // Rover data! Handle rover stuff...
    }
}
```

Plus, the WiFi radio automatically gives us the sender's MAC address in the `mac` parameter — we use this to track whether each robot is still online!

```
📦 Packet arrives at Satellite

  WiFi radio says:   mac = 42:91:51:5B:7F:A2   ← Hexapod's name tag
  data[0] = 0x01  →  Telemetry message
  data[1] = 0x01  →  From Hexapod

  Satellite knows:
    ✅ Who sent it   → Hexapod (from data[1])
    ✅ What it is    → Telemetry (from data[0])
    ✅ They are alive → update hexLastRxMs timer
```

---

## Part 8: Knowing If a Robot Is Still Alive — Timeout System

The Satellite keeps track of the **last time it heard from each robot**.
If it goes quiet for more than **10 seconds** — something is wrong!

```cpp
// Every time a Hexapod packet arrives:
hexLastRxMs = millis();   // save the current time

// In loop(), check if too much time has passed:
bool hexOnline = hexLastRxMs > 0 &&
                (millis() - hexLastRxMs < 10000UL);  // 10 seconds

if (hexOnline != hexWasConnected) {
    hexWasConnected = hexOnline;
    // tell the dashboard! ("hexapod just went offline!" or "came back online!")
}
```

This is like asking: "Did my friend text me in the last 10 seconds?"
If not — maybe they lost signal, or their battery died!

The dashboard shows this in real time:
```
🟢 Hexapod — ONLINE   (sent data 1 second ago)
🔴 Rover   — OFFLINE  (last heard 15 seconds ago)
```

---

## Part 9: The Safety Rule — Never Do Work in the Callback!

This is one of the most important rules in embedded programming.

### The Problem

The ESP-NOW receive callback runs in a **separate WiFi task** — it can interrupt your main code at any moment, even in the middle of something important.

If you tried to move the robot motors directly inside the callback:
- The robot could get confused
- Memory could get corrupted
- The whole chip could crash and restart!

### The Solution: Set a Flag, Act Later

Instead of acting immediately, we just leave a **sticky note**:

```cpp
// In the callback — ONLY leave a note, nothing else!
void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
    if (msgType == ESPNOW_MSG_COMMAND) {
        strncpy(pendingEspNowCmd, pkt->cmd, 15);  // copy the command
        hasPendingEspNowCmd = true;                // sticky note: "there's a command!"
    }
    // done! don't do anything else here.
}

// In loop() — read the sticky note and act safely!
void loop() {
    if (hasPendingEspNowCmd) {
        hasPendingEspNowCmd = false;  // tear off the sticky note
        // NOW we can safely move the robot:
        if (strcmp(pendingEspNowCmd, "forward") == 0) {
            currentDirection = FORWARD;
            isWalking = true;
        }
    }
}
```

The `volatile` keyword on the flag tells the computer: *"This variable can change at any moment from another task — always check the real value, never assume!"*

```cpp
static volatile bool hasPendingEspNowCmd = false;
//       ^^^^^^^^
//  "Check this variable fresh every single time"
```

---

## Part 10: The Full Picture — One Button Click to Walking Robot

Let's trace what happens when you click **"Forward"** in the browser:

```
1. 🖱️  You click "Forward" in the browser

2. 💻  Browser sends this over WebSocket:
       {"type":"command", "target":"hexapod", "command":"forward"}

3. 🛰️  Satellite receives it and packs an ESP-NOW command:
       pkt.msgType = 0x02       (command)
       pkt.cmd     = "forward"

4. 📡  Satellite sends via ESP-NOW directly to Hexapod's MAC:
       esp_now_send(HEXAPOD_MAC, &pkt, 17)

   ~~~~ travels through the air at 2.4 GHz radio waves ~~~~

5. 🦾  Hexapod's callback fires:
       copies "forward" into pendingEspNowCmd
       sets hasPendingEspNowCmd = true  (sticky note!)

6. 🔄  Hexapod's loop() reads the sticky note:
       if (hasPendingEspNowCmd) {
           currentDirection = FORWARD;
           isWalking = true;
       }

7. 🚶  Hexapod walks forward!
```

**Total time from click to walking: under 5 milliseconds!**
That's 200 times faster than you can blink.

---

## Part 11: The Complete Communication Map

```
┌──────────────────────────────────────────────────────────────────────┐
│                     FULL COMMUNICATION MAP                           │
└──────────────────────────────────────────────────────────────────────┘

  🌐 BROWSER / DASHBOARD
         │  ▲
         │  │  WebSocket over WiFi (internet-style, fast)
         │  │
         ▼  │
  ┌──────────────────┐
  │  🛰️  SATCOM       │  ESP32
  │     ALPHA         │  IP: 192.168.4.1
  │     Channel 6     │  MAC: 3C:8A:1F:7E:33:55
  └────────┬──────────┘
           │
           │   ESP-NOW (direct radio, 2.4 GHz, Channel 6)
     ┌─────┴──────────────────────────┐
     │                                │
     ▼                                ▼
┌────────────────┐             ┌────────────────┐
│  🦾 HEXAPOD    │             │  🚗 ROVER      │
│  ESP8266       │             │  ESP32         │
│  Channel 6     │             │  Channel 6     │
│  MAC: 42:91... │             │  MAC: 88:57... │
└────────────────┘             └────────────────┘

HEXAPOD → SATCOM:  Telemetry   (26 bytes, every 2 seconds)
SATCOM → HEXAPOD:  Command     (17 bytes, when you click a button)
SATCOM ↔ HEXAPOD:  Chat        (81 bytes, when you type a message)

ROVER  → SATCOM:   Telemetry   (27 bytes, every 0.5 seconds)
SATCOM → ROVER:    Command     (17 bytes, when you click a button)
SATCOM ↔ ROVER:    Chat        (81 bytes, when you type a message)

SATCOM → BROWSER:  JSON via WebSocket (sensor data + chat relay)
BROWSER → SATCOM:  JSON via WebSocket (commands + chat)
```

---

## Part 12: Key Words Explained Simply

| Word | Simple Meaning |
|------|---------------|
| **MAC Address** | Every chip's unique name tag, burned in forever |
| **AP MAC** | The name tag used when the chip acts like a mini router |
| **ESP-NOW** | A super fast direct radio talk between ESP chips |
| **Channel** | Like a walkie-talkie channel — both sides must match |
| **Peer** | A friend you registered to talk with |
| **Packet** | A bundle of data sent all at once, like an envelope |
| **Telemetry** | Sensor data sent automatically every few seconds |
| **Callback** | A function the system calls automatically when something happens |
| **volatile** | "This can change at any moment — always re-read it!" |
| **packed struct** | A data bundle with zero wasted space |
| **Deferred processing** | Leave a sticky note, act on it later in loop() |
| **Timeout** | If we don't hear from you in 10 seconds, we assume you're offline |
| **WebSocket** | A two-way live connection between the browser and Satellite |

---

## Part 13: Try It Yourself!

### Challenge 1 — Count the bytes
Add up the bytes in the Rover telemetry packet manually:
- `uint8_t` = 1 byte
- `int16_t` / `uint16_t` = 2 bytes
- `float` = 4 bytes

Do you get 27?

### Challenge 2 — Find the AP MAC
Flash any ESP32, open Serial Monitor and look for:
```
[ESP-NOW] Satellite AP MAC: 3C:8A:1F:7E:33:55
```
Is the last byte always STA + 1?

### Challenge 3 — Change the timeout
The Satellite marks a robot offline after 10 seconds.
Can you find the number in the code and change it to 5 seconds?

### Challenge 4 — Add a new message type
Can you define `ESPNOW_MSG_ALERT = 0x04` for emergency messages?
What would be inside the packet?

---

*Three robots. One satellite. Zero internet required.*
*Built with ESP-NOW, curiosity, and a lot of debugging.*
