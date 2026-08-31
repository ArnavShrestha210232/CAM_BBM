# STM32F427-NuttX-Multi-App-IPC-System-using-Ring-Buffers-and-uORB-with-OpenMV-Data-Pipeline
# STM32F427 NuttX Multi-App IPC System using Ring Buffers and uORB with OpenMV Data Pipeline

A real-time embedded system running on **STM32F427** under **Apache NuttX RTOS** that captures JPEG images from an **OpenMV camera** over UART + SPI, streams them through a **hardware-mirrored ring buffer** between concurrent NuttX tasks, and publishes camera status via **uORB** (the publish/subscribe middleware also used in PX4 Autopilot).

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Hardware Setup](#hardware-setup)
3. [Architecture](#architecture)
4. [Ring Buffer — Design & Implementation](#ring-buffer--design--implementation)
5. [Applications](#applications)
   - [launcher — Entry Point](#launcher--entry-point)
   - [camera (camera_MSN2) — Producer Task](#camera-camera_msn2--producer-task)
   - [hello — Consumer Task](#hello--consumer-task)
6. [uORB Integration](#uorb-integration)
7. [OpenMV Protocol](#openmv-protocol)
8. [File Structure](#file-structure)
9. [Build & Flash](#build--flash)
10. [Expected Output](#expected-output)

---

## System Overview

```
  OpenMV Cam
  ┌──────────┐   UART 115200    ┌──────────────────────────────────────┐
  │ openmv_  │ ───handshake──▶  │           STM32F427 / NuttX          │
  │ camera   │ ◀── CMD ──────── │                                      │
  │ .py      │ ─── ACK+size+──▶ │  launcher_main()                     │
  │          │     meta ──────▶ │    ├─ task_create → camera_MSN2_main │
  │          │   SPI 4 MHz      │    │       │ UART handshake           │
  │          │ ─── JPEG ──────▶ │    │       │ SPI JPEG read            │
  └──────────┘                  │    │       ▼                          │
                                │    │   ring_buffer (64 slots × 256 B) │
                                │    │       ▼                          │
                                │    └─ task_create → hello_main        │
                                │           │ rb_read()                 │
                                │           │ print hex JPEG bytes      │
                                │           ▼                           │
                                │       uORB: camera_status topic       │
                                └──────────────────────────────────────┘
```

The system is built around three components working in concert:

- A **ring buffer** that acts as a lock-safe, bounded FIFO between the camera producer and the display consumer, mirroring the behaviour of a hardware Verilog `CircularBuffer` module.
- **Three NuttX applications** — one launcher that spawns two tasks — enabling true concurrent execution under the NuttX scheduler.
- **uORB** publish/subscribe middleware for broadcasting camera status metadata across the system.

---

## Hardware Setup

| Signal      | STM32F427 Pin | OpenMV Pin | Notes                        |
|-------------|---------------|------------|------------------------------|
| UART TX     | USART2 TX     | P4 (RX)    | 115200 8N1                   |
| UART RX     | USART2 RX     | P4 (TX)    | 115200 8N1                   |
| SPI MISO    | SPI3 MISO     | P7         | CPOL=0 CPHA=0 (Mode 0)       |
| SPI MOSI    | SPI3 MOSI     | P8         | 4 MHz clock                  |
| SPI SCK     | SPI3 SCK      | P6         |                              |
| SPI CS      | SPI3 CS (SW)  | P5         | Asserted by STM32 in SW      |

NuttX device nodes used:
- UART: `/dev/ttyS1`
- SPI: bus 3 via `stm32_spibus_initialize(3)`

---

## Architecture

### Task Model

NuttX runs a preemptive, priority-based RTOS scheduler. This project exploits that with two independent tasks created from a single launcher:

```
NSH shell
  └─ launcher_main()          (runs once, returns immediately)
        ├─ task_create("camera", priority=100, stack=8192)
        │       └─ camera_MSN2_main()   [Producer — runs to completion]
        └─ task_create("hello",  priority=100, stack=4096)
                └─ hello_main()          [Consumer — polls ring buffer]
```

Both tasks run **concurrently**. The ring buffer with a mutex is the only shared state between them — there are no signals, no pipes, no semaphores needed beyond the mutex inside the ring buffer itself.

### Data Flow

```
camera_MSN2_main
  1. UART handshake with OpenMV
  2. Send CMD_CAPTURE (0x02)
  3. Receive: ACK (1B) + JPEG size (4B LE) + meta_len (2B LE) + meta string
  4. Wait 25 ms (SPI_CS_DELAY_US) for OpenMV to enter spi.send_recv()
  5. Assert SPI CS → discard 1024 sync bytes → read JPEG bytes
  6. Push metadata chunk (is_meta=1) into ring buffer
  7. Push JPEG data in 256-byte chunks (is_meta=0, is_last on final)

                    ──▶ ring_buffer (64 slots) ──▶

hello_main
  1. rb_read() in a tight poll loop (usleep 1 ms on empty)
  2. Skip chunks where is_meta==1
  3. Print chunk.data[] as raw hex bytes, no separator
  4. On is_last==1: print newline and exit
```

---

## Ring Buffer — Design & Implementation

### Design Philosophy

The ring buffer is written to be a **direct software mirror of a synthesisable Verilog `CircularBuffer` module**. Every register and signal in the RTL has a named equivalent in C:

| Verilog register / signal | C equivalent in `ring_buffer_s`            |
|---------------------------|--------------------------------------------|
| `wr_ptr` register         | `g_rb.wr_ptr` (ever-increasing `uint32_t`) |
| `rd_ptr` register         | `g_rb.rd_ptr` (ever-increasing `uint32_t`) |
| `count` register          | `g_rb.count`                               |
| `assign full = count==DEPTH` | `rb_full()` → `count >= RB_DEPTH`      |
| `assign empty = count==0` | `rb_empty()` → `count == 0`               |
| `mem[wr_ptr % DEPTH]`     | `g_rb.slots[wr_ptr % RB_DEPTH]`           |
| `mem[rd_ptr % DEPTH]`     | `g_rb.slots[rd_ptr % RB_DEPTH]`           |
| `wr_ptr <= wr_ptr + 1`    | `g_rb.wr_ptr++` after copy                |
| `rd_ptr <= rd_ptr + 1`    | `g_rb.rd_ptr++` after copy                |

This makes the software model independently verifiable against the RTL simulation.

### Parameters

```c
#define RB_DEPTH    64      // number of slots (must be power-of-2)
#define CHUNK_SIZE  256     // bytes per slot payload
```

Total static RAM consumed by the ring buffer: `64 × (256 + 1 + 2 + 4 + 1 + 1 + padding)` ≈ **17 KB**.

### Slot Structure — `image_chunk_s`

```c
struct image_chunk_s {
    uint8_t  data[CHUNK_SIZE];  // 256 bytes of JPEG payload or metadata string
    uint16_t len;               // number of valid bytes in data[]
    uint32_t frame_id;          // identifies which capture frame this belongs to
    uint8_t  is_last;           // 1 = this is the final JPEG chunk of the frame
    uint8_t  is_meta;           // 1 = metadata string chunk, NOT JPEG data
};
```

`is_meta` is a **dedicated field**, not a sentinel value stuffed into the payload — this prevents any corruption of JPEG byte sequences.

### API

```c
void rb_init  (void);                          // zero all state, init mutex
int  rb_write (const struct image_chunk_s *c); // returns 0=OK, -1=full
int  rb_read  (struct image_chunk_s *c);       // returns 0=OK, -1=empty
int  rb_count (void);                          // current occupancy 0..64
int  rb_full  (void);                          // 1 if count == RB_DEPTH
int  rb_empty (void);                          // 1 if count == 0
```

### Thread Safety

All read/write/count/full/empty calls acquire `g_rb_lock` (a NuttX `nxmutex_t`) before touching any shared state. The mutex is the only synchronisation primitive — there is intentionally no condition variable. Both producer and consumer use **busy-retry with `usleep()`** on failure:

```c
// Producer (camera.c)
static void rb_push(const struct image_chunk_s *c) {
    while (rb_write(c) != 0)
        usleep(500);    // retry every 0.5 ms if buffer is full
}

// Consumer (hello.c)
if (rb_read(&chunk) != 0) {
    usleep(1000);       // retry every 1 ms if buffer is empty
    continue;
}
```

---

## Applications

### launcher — Entry Point

**File:** `launcher.c`  
**NuttX entry:** `launcher_main`  
**Role:** Registered as an NSH command. Creates both tasks and returns immediately.

```c
int launcher_main(int argc, char *argv[])
{
    // Spawn camera producer at priority 100, 8 KB stack
    task_create("camera", 100, 8192, camera_MSN2_main, camera_argv);

    // Spawn hello consumer at priority 100, 4 KB stack
    task_create("hello",  100, 4096, hello_main,       hello_argv);

    return 0;  // launcher exits; both tasks continue independently
}
```

**Why a launcher?** NuttX's NSH shell is single-threaded by default. Running camera and hello directly from NSH would be sequential. `task_create()` starts each as a fully independent schedulable entity, allowing the consumer to begin draining the ring buffer while the camera is still performing SPI transfers.

---

### camera (camera_MSN2) — Producer Task

**File:** `camera.c`  
**NuttX entry:** `camera_MSN2_main`  
**Stack:** 8192 bytes  
**Role:** Owns all hardware I/O. Drives the complete capture sequence and fills the ring buffer.

#### Sequence in Detail

**Step 1 — Open UART (`/dev/ttyS1`)**

Opened with `O_RDWR | O_NOCTTY | O_NONBLOCK`. Configured to 115200 8N1 with all echo and flow control disabled via `termios`. `VMIN=0, VTIME=0` ensures non-blocking reads.

**Step 2 — Initialise SPI Bus 3**

```c
struct spi_dev_s *spi = stm32_spibus_initialize(SPI_BUS); // bus 3
```

The NuttX SPI driver handle is used with the kernel-level `SPI_LOCK / SPI_SELECT / SPI_EXCHANGE` macros — this is direct kernel SPI, not a userspace `/dev/spi` file, for minimum latency.

**Step 3 — UART Handshake**

```
STM32 sends: 0x01 (CMD_HANDSHAKE)
OpenMV replies: 0xAA (ACK_BYTE)
```

Retried up to `HANDSHAKE_TRIES=5` times, polling every 100 ms for up to 5 s per attempt. The UART RX buffer is flushed before each attempt to discard stale bytes.

**Step 4 — Send CMD_CAPTURE and receive header**

```
STM32 sends:  0x02 (CMD_CAPTURE)
OpenMV sends: 0xAA              ← ACK (1 byte)
              <size_lo>         ┐
              <size_hi_1>       │ JPEG size, 4 bytes little-endian
              <size_hi_2>       │
              <size_hi_3>       ┘
              <meta_len_lo>     ┐ metadata string length, 2 bytes LE
              <meta_len_hi>     ┘
              <meta string>       e.g. "w:320,h:240,fmt:JPEG"
```

Each multi-byte field is read with `uart_read_exact()` which polls byte-by-byte with a configurable retry/delay (up to 500 ms total).

**Step 5 — SPI CS delay (25 ms)**

After all UART bytes are received, the STM32 waits `SPI_CS_DELAY_US = 25000` µs before asserting SPI CS. This is required because OpenMV's Python script takes ~20–30 ms to finish writing UART and enter `spi.send_recv()`. Asserting CS too early results in a missed SPI window.

**Step 6 — SPI JPEG transfer**

```c
// Discard 1024 zero sync bytes sent by OpenMV before JPEG data
// Then read img_size bytes in 256-byte SPI_EXCHANGE blocks
spi_read_image(spi, image_buf, img_size);
```

SPI parameters: Mode 0, 8-bit words, 4 MHz. The STM32 drives dummy zero TX bytes while reading. The full-duplex `SPI_EXCHANGE` macro reads and discards simultaneously.

**Step 7 — JPEG validation**

```c
int hdr_ok = (image_buf[0]==0xFF && image_buf[1]==0xD8 && image_buf[2]==0xFF);
int trl_ok = (image_buf[img_size-2]==0xFF && image_buf[img_size-1]==0xD9);
```

Reports header/trailer status on the console but does not abort — bad markers are a diagnostic aid, not a hard stop.

**Step 8 — Push metadata chunk**

```c
mc.is_meta  = 1;
mc.is_last  = 0;
mc.frame_id = 1;
mc.len      = meta_len;
memcpy(mc.data, meta_buf, meta_len);
rb_push(&mc);
```

**Step 9 — Push JPEG data chunks**

The `img_size`-byte JPEG is sliced into 256-byte chunks. The final chunk has `is_last=1`. All have `is_meta=0` and `frame_id=1`.

```
Number of chunks = ceil(img_size / 256)
For a 320×240 JPEG (~20–35 KB): typically 80–140 chunks
```

---

### hello — Consumer Task

**File:** `hello.c`  
**NuttX entry:** `hello_main`  
**Stack:** 4096 bytes  
**Role:** Drains the ring buffer and outputs the raw JPEG byte stream to stdout.

#### Behaviour

```c
while (!done) {
    if (rb_read(&chunk) != 0) { usleep(1000); continue; }
    if (chunk.is_meta) continue;                    // silently skip metadata
    for (i = 0; i < chunk.len; i++)
        printf("%02X", chunk.data[i]);              // raw hex, no separators
    if (chunk.is_last) { printf("\n"); done = 1; }  // one newline at end
}
```

The output is a single line of hex characters — the complete JPEG file encoded as ASCII hex. This can be piped through a host-side tool to reconstruct the image:

```bash
# On host, after capturing NSH output to jpeg_hex.txt:
python3 -c "
import sys, binascii
hex_str = open('jpeg_hex.txt').read().strip()
open('image.jpg','wb').write(binascii.unhexlify(hex_str))
"
```

---

## uORB Integration

**File:** `camera_status_orb.c`  
**Header:** `camera_status.h`

uORB (Object Request Broker) is the publish/subscribe middleware from PX4 Autopilot, ported into NuttX's `apps/system/uorb/`. It creates character device nodes under `/dev/uorb/` and uses `poll()` for zero-CPU-overhead subscriber notification.

### Topic Definition

```c
// camera_status_orb.c
#include <uORB/uORB.h>
#include "camera_status.h"

ORB_DEFINE(camera_status,       // topic name → /dev/uorb/camera_status
           struct camera_status_s,  // payload struct (defined in camera_status.h)
           NULL);               // no debug print callback
```

The `ORB_DEFINE` macro expands to:

```c
const struct orb_metadata g_orb_camera_status = {
    "camera_status",
    sizeof(struct camera_status_s),
};
```

### Publishing

```c
int fd = orb_advertise(ORB_ID(camera_status), &initial_status);
// ... later:
orb_publish(ORB_ID(camera_status), fd, &status);
```

### Subscribing (from any other task)

```c
int sub = orb_subscribe(ORB_ID(camera_status));
struct camera_status_s status;
bool updated;
orb_check(sub, &updated);
if (updated)
    orb_copy(ORB_ID(camera_status), sub, &status);
```

### uORB vs Ring Buffer — Why Both?

| | Ring Buffer | uORB |
|---|---|---|
| **Data type** | Raw binary JPEG chunks (up to 256 B each) | Small status structs |
| **Consumers** | Single consumer (hello task) | Many potential subscribers |
| **Blocking** | Busy-poll with `usleep` | `poll()` system call |
| **Purpose** | High-throughput image streaming | Status / telemetry broadcast |
| **Location** | Shared RAM, mutex protected | `/dev/uorb/` character device |

The ring buffer handles **bulk data** between two specific tasks. uORB handles **lightweight status information** available to any future task in the system.

---

## OpenMV Protocol

The OpenMV-side Python script (`openmv_camera.py`, not in this repo) implements the following state machine:

```
Wait for 0x01 (CMD_HANDSHAKE) → reply 0xAA
Wait for 0x02 (CMD_CAPTURE)   → capture JPEG
                               → reply 0xAA
                               → send 4B JPEG size (LE)
                               → send 2B meta_len (LE)
                               → send meta string (e.g. "w:320,h:240,fmt:JPEG")
                               → pyb.delay(30)          ← wait 30 ms
                               → spi.send_recv(1024 zero bytes + JPEG padded)
```

The 30 ms delay on the OpenMV side gives the STM32 time to finish reading UART bytes and assert CS. The STM32 uses a 25 ms delay (`SPI_CS_DELAY_US`) which is intentionally shorter, ensuring CS is asserted while OpenMV is already blocked in `spi.send_recv()`.

---

## File Structure

```
nuttxspace/
├── nuttx/                          # NuttX kernel (submodule / sibling dir)
└── apps/
    └── examples/
        └── MSN2/                   # This project
            ├── camera.c            # Producer task — UART+SPI capture
            ├── hello.c             # Consumer task — ring buffer drain + hex output
            ├── launcher.c          # NSH entry point — spawns both tasks
            ├── ring_buffer.c       # Bounded FIFO, Verilog-mirrored implementation
            ├── ring_buffer.h       # Struct definitions, API declarations
            ├── camera_status.h     # uORB topic payload struct
            ├── camera_status_orb.c # ORB_DEFINE — instantiates uORB metadata
            ├── Kconfig             # NuttX menuconfig entries
            ├── Make.defs           # Build system include
            └── Makefile            # App build rules
```

---

## Build & Flash

### Prerequisites

- ARM GCC toolchain: `arm-none-eabi-gcc`
- NuttX build environment configured for `stm32f427v-disco` or equivalent board

### Configure

```bash
cd nuttxspace/nuttx
./tools/configure.sh stm32f427v-disco:nsh
make menuconfig
# Enable: Application Configuration → Examples → MSN2 Camera App
# Enable: System Libraries → uORB
```

### Build

```bash
make -j$(nproc)
# Output: nuttx.bin, nuttx.hex
```

### Flash

```bash
# Using OpenOCD:
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
        -c "program nuttx.hex verify reset exit"

# Or using ST-Flash:
st-flash write nuttx.bin 0x08000000
```

### Run

```
nsh> launcher
Starting camera + subscriber tasks...
nsh>
================ CAMERA START ================
[CAM][UART] 115200 8N1 non-blocking
[CAM][UART] handshake start
[CAM] TX handshake attempt 1
[CAM][UART] HANDSHAKE OK
[CAM] CMD_CAPTURE sent — waiting ACK...
[CAM] ACK OK
[CAM] JPEG size = 24312 B
[CAM] meta len  = 20 B
[CAM] meta      = "w:320,h:240,fmt:JPEG"
[CAM] waiting 25 ms before SPI CS...
[CAM][SPI] reading 24312 JPEG bytes (+ 1024 sync discarded)...
[CAM][SPI] 1024 sync bytes discarded
[CAM] JPEG header OK  : FF D8 FF
[CAM] JPEG trailer OK : FF D9
[CAM] meta chunk pushed
[CAM] pushing 95 JPEG chunks...
[CAM] all 95 chunks pushed — DONE
FFD8FFE0...FFD9
```

---

## Expected Output

The final line printed by `hello_main` is the complete JPEG image as uppercase hex — beginning with `FFD8FF` (JPEG SOI + APP0 marker) and ending with `FFD9` (JPEG EOI marker). This can be saved and decoded on any host system to verify a successful image transfer.

---

## License

Source files under `apps/` follow the **Apache License 2.0** as used by the Apache NuttX project. The `nuttx/` kernel tree retains its original BSD-style per-file licenses. See individual file headers for details.
# CAM_BBM
