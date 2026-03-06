/*
 * L.I.M.A. — Local Integrity Multi-modal Architecture
 * crypto.h — Signing API
 *
 * PSA ECDSA-P256 / SHA-256 over lima_payload_t.
 * Key stored in Nordic KMU via persistent PSA key slot.
 *
 * Call order:
 *   1. lima_crypto_init()            — once, after psa_crypto_init()
 *   2. lima_crypto_build_payload()   — populate from fsm.last_event
 *   3. lima_crypto_sign_async()      — sign, callback posts SIGNING_COMPLETE
 */

#ifndef LIMA_CRYPTO_H
#define LIMA_CRYPTO_H

#include <stdint.h>
#include <stddef.h>
#include "events.h"

/* ── Payload ─────────────────────────────────────────────────────────────── */

typedef struct {
    uint8_t  node_id[6];      /* BLE MAC or provisioned ID            */
    uint8_t  event_type;      /* lima_event_type_t                     */
    uint8_t  reserved[1];     /* alignment padding — zero-filled       */
    uint32_t sequence;        /* monotonic counter — anti-replay       */
    uint32_t timestamp_ms;    /* k_uptime_get_32() at event time       */
    float    accel_g;         /* IMU vector magnitude at trigger (g)   */
    float    delta_pa;        /* baro delta at trigger (Pa)            */
} lima_payload_t;             /* 24 bytes                              */

// typedef struct {
//     uint8_t  node_id[6];      /* 6 B */
//     uint8_t  event_type;      /* 1 B */
//     uint8_t  reserved[1];     /* 1 B — pad to 8 B boundary */
//     uint32_t sequence;        /* 4 B */
//     uint32_t timestamp_ms;    /* 4 B */
//     float    accel_g;         /* 4 B */
//     float    delta_pa;        /* 4 B */
// } lima_payload_t;             /* 24 bytes exactly */

BUILD_ASSERT(sizeof(lima_payload_t) == 24, "lima_payload_t size mismatch");

/* ── Signing Result ──────────────────────────────────────────────────────── */

typedef struct {
    uint8_t  sig[64];         /* ECDSA-P256 signature (r || s)         */
    size_t   sig_len;         /* always 64 for P-256                   */
    int      err;             /* 0 on success, PSA error code          */
} lima_sig_result_t;

/* ── Callback ────────────────────────────────────────────────────────────── */

typedef void (*lima_sign_cb_t)(const lima_sig_result_t *result);

/* ── API ─────────────────────────────────────────────────────────────────── */

/**
 * @brief Initialize PSA crypto and provision signing key if absent.
 *
 * Must be called once from main.c after board init, before fsm_init().
 * Generates a P-256 keypair into KMU slot CONFIG_LIMA_CRYPTO_KEY_ID
 * if no persistent key exists there yet.
 *
 * @return 0 on success, negative PSA error code on failure.
 */
int lima_crypto_init(void);

/**
 * @brief Populate a lima_payload_t from the current FSM event context.
 *
 * Fills node_id from BLE MAC (or static override), stamps sequence
 * counter, copies event fields. Caller owns payload memory.
 *
 * @param payload  Output struct to populate.
 * @param evt      Source event from fsm.last_event.
 */
void lima_crypto_build_payload(lima_payload_t *payload,
                               const lima_event_t *evt);

/**
 * @brief Sign a payload with ECDSA-P256 / SHA-256.
 *
 * Synchronous in v1 (CryptoCell hardware-accelerated but blocking).
 * Calls cb() with result before returning. cb() must post
 * LIMA_EVT_SIGNING_COMPLETE or LIMA_EVT_SENSOR_FAULT to the FSM queue.
 *
 * @param payload  Populated payload struct to sign.
 * @param cb       Completion callback — must be non-NULL.
 * @return 0 if signing was attempted, negative on parameter error.
 */
int lima_crypto_sign_async(const lima_payload_t *payload,
                           lima_sign_cb_t cb);

#endif /* LIMA_CRYPTO_H */