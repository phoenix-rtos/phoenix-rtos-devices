# librmap — RMAP Packet Builder/Parser

Implementation of RMAP (Remote Memory Access Protocol) packet construction and parsing per **ECSS‐E‐ST‐50‐52C**.

CRC is not computed in software — it is assumed to be generated (write path) and verified (read path) by the SpaceWire
hardware interface.

---

## API

### Build a Write command

```c
ssize_t rmap_buildWriteCmd(const rmap_params_t *params,
                           uint8_t *header, size_t headerLen,
                           size_t dataLen);
```

Builds a Write command header into `header`. Returns the number of bytes written, or a negative `errno` on error.

### Build a Read command

```c
ssize_t rmap_buildReadCmd(const rmap_params_t *params,
                          uint8_t *header, size_t headerLen,
                          size_t readLen);
```

Builds a Read command header. Returns the number of bytes written, or a negative `errno` on error.

**Mandatory for Read:** `requireAck = true`, `verifyData = false`.

### Build a Read-Modify-Write command

```c
ssize_t rmap_buildReadModifyWriteCmd(const rmap_params_t *params,
                                     uint8_t *header, size_t headerLen,
                                     size_t dataLen);
```

Builds an RMW command header. Returns the number of bytes written, or a negative `errno` on error.

**Mandatory for RMW:** `requireAck = true`, `incrementAddr = true`.  
Valid `dataLen` values (data + mask combined): `0`, `2`, `4`, `6`, or `8` bytes.

### Parse a Reply

```c
int rmap_parseReply(const uint8_t *packet, size_t packetLen,
                    rmap_reply_t *reply);
```

Parses an incoming RMAP reply packet (stripped of the leading SpaceWire routing address bytes) into `reply`. Returns
`0` on success, or a negative `errno` on error (`-EPROTO` for malformed packets).

---

## Parameters (`rmap_params_t`)

| Field                  | Type       | Description                                                                |
| ---------------------- | ---------- | -------------------------------------------------------------------------- |
| `targetLogicalAddr`    | `uint8_t`  | Logical address of the RMAP target node.                                   |
| `initiatorLogicalAddr` | `uint8_t`  | Logical address of the initiator (reply destination).                      |
| `key`                  | `uint8_t`  | Target authorisation key.                                                  |
| `transactionId`        | `uint16_t` | Transaction identifier; used to match commands with replies.               |
| `extendedAddr`         | `uint8_t`  | Extended (high) byte of the memory address (set to `0` if unused).         |
| `memoryAddr`           | `uint32_t` | 32-bit target memory address.                                              |
| `replyPath`            | `uint8_t*` | SpaceWire path address bytes for the reply (may be `NULL` if length is 0). |
| `replyPathLen`         | `uint8_t`  | Number of bytes in `replyPath`. Maximum **12**.                            |
| `verifyData`           | `bool`     | Request data CRC verification before committing the write.                 |
| `requireAck`           | `bool`     | Request an acknowledgement reply from the target.                          |
| `incrementAddr`        | `bool`     | Increment the memory address between successive data words (block access). |

### Flag constraints

| Operation | `requireAck` | `verifyData`  | `incrementAddr` |
| --------- | :----------: | :-----------: | :-------------: |
| Write     |   optional   |   optional    |    optional     |
| Read      |   **true**   |   **false**   |    optional     |
| RMW       |   **true**   | (forced true) |    **true**     |

> The `verifyData` flag is forced on for RMW by the protocol regardless of the value supplied in `params`.

---

## Reply structure (`rmap_reply_t`)

| Field                  | Type             | Description                                                         |
| ---------------------- | ---------------- | ------------------------------------------------------------------- |
| `initiatorLogicalAddr` | `uint8_t`        | Logical address echoed from the original command.                   |
| `targetLogicalAddr`    | `uint8_t`        | Logical address of the target that generated the reply.             |
| `status`               | `uint8_t`        | Reply status code (see status codes below).                         |
| `transactionId`        | `uint16_t`       | Transaction identifier echoed from the command.                     |
| `isReadReply`          | `bool`           | `true` when the reply carries read data.                            |
| `dataLen`              | `uint32_t`       | Number of data bytes present (valid only when `isReadReply`).       |
| `data`                 | `const uint8_t*` | Zero-copy pointer into the receive buffer (valid if `isReadReply`). |

---

## Status codes

| Value  | Constant                           | Meaning                                         |
| ------ | ---------------------------------- | ----------------------------------------------- |
| `0x00` | `RMAP_STATUS_SUCCESS`              | Command executed successfully.                  |
| `0x01` | `RMAP_STATUS_GENERAL_ERROR`        | General error.                                  |
| `0x02` | `RMAP_STATUS_UNUSED_PACKET_TYPE`   | Unused/reserved packet type or command code.    |
| `0x03` | `RMAP_STATUS_INVALID_KEY`          | Invalid key supplied.                           |
| `0x04` | `RMAP_STATUS_INVALID_DATA_CRC`     | Data CRC error.                                 |
| `0x05` | `RMAP_STATUS_EARLY_EOP`            | Early End-of-Packet detected.                   |
| `0x06` | `RMAP_STATUS_TOO_MUCH_DATA`        | More data received than declared in the header. |
| `0x07` | `RMAP_STATUS_EEP`                  | Error End-of-Packet (link error).               |
| `0x09` | `RMAP_STATUS_VERIFY_BUF_OVERRUN`   | Verify buffer overrun.                          |
| `0x0a` | `RMAP_STATUS_NOTIMPL_NOTAUTH`      | Command not implemented or not authorised.      |
| `0x0b` | `RMAP_STATUS_RMW_DATA_LEN_ERR`     | Invalid data length for RMW command.            |
| `0x0c` | `RMAP_STATUS_INV_TGT_LOGICAL_ADDR` | Invalid target logical address.                 |

---

## Usage example

```c
#include "rmap.h"

/* Build a verified write with acknowledgement */
rmap_params_t params = {
    .targetLogicalAddr    = 0xfe,
    .initiatorLogicalAddr = 0x67,
    .key                  = 0x00,
    .transactionId        = 1,
    .extendedAddr         = 0,
    .memoryAddr           = 0x40000000,
    .replyPath            = NULL,
    .replyPathLen         = 0,
    .verifyData           = true,
    .requireAck           = true,
    .incrementAddr        = true,
};

uint8_t header[32];
ssize_t hdrLen = rmap_buildWriteCmd(&params, header, sizeof(header), dataLen);
if (hdrLen < 0) {
    /* handle error */
}
/* Transmit header followed by data bytes, hardware appends CRC */

/* Parse the reply received over SpaceWire */
rmap_reply_t reply;
int ret = rmap_parseReply(rxBuf, rxLen, &reply);
if (ret == 0 && reply.status == RMAP_STATUS_SUCCESS) {
    /* write acknowledged */
}
```
