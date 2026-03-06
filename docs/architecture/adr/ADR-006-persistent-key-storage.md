# ADR-006: Persistent PSA Key Storage

**Status:** Proposed  
**Date:** 2026-03-06  
**Author:** jknoxdev  

---

## Context

LIMA nodes currently generate a fresh ECDSA-P256 keypair on every boot using
`PSA_KEY_LIFETIME_VOLATILE`. This works for development and signing validation
but breaks the gateway verification model in any real deployment scenario:

- Every power cycle produces a new keypair
- The gateway cannot verify signatures without re-registering the public key
  after every reboot
- A tamper event occurring after an unplanned power cycle produces a signature
  the gateway will reject
- The NVS sequence counter resets to zero on every boot, undermining anti-replay

This is a known correctness gap, not an oversight. The decision to defer was
made explicitly in favor of completing the BLE transmission layer first.

---

## Decision

**Deferred — implement after BLE advertising milestone.**

Persistent key storage requires:

- `CONFIG_MBEDTLS_PSA_CRYPTO_STORAGE_C=y`
- `CONFIG_PSA_NATIVE_ITS=y`
- A `storage` flash partition defined in the DTS overlay
- NVS backend wired and tested across power cycles

Estimated implementation complexity: 2–4 hours depending on partition manager
cooperation. Code changes to `crypto.c` are minimal (~20 lines). Testing across
power cycles is the non-trivial surface.

The volatile key limitation is acceptable for the current milestone because:

1. BLE advertising is not yet implemented — there is no gateway to verify
   signatures regardless of key stability
2. A working signed BLE advertisement with a volatile key is a more complete
   system demo than a persisted key with no transmission path
3. `bt_id_get()` (needed to replace the hardcoded `DEADBEEF` node_id) requires
   BLE stack initialization, which is the same session as BLE advertising

---

## Implementation Plan (when scheduled)

1. Add storage partition to DTS overlay
2. Enable `CONFIG_MBEDTLS_PSA_CRYPTO_STORAGE_C` and `CONFIG_PSA_NATIVE_ITS`
3. In `provision_key()`: swap `PSA_KEY_LIFETIME_VOLATILE` →
   `PSA_KEY_LIFETIME_PERSISTENT`, uncomment `psa_set_key_id()`
4. Restore existence check in `lima_crypto_init()` — now correct since key
   genuinely persists across reboots
5. Persist sequence counter to NVS alongside key (anti-replay across reboots)
6. Validate: power cycle, verify same public key, verify signing works,
   verify sequence counter increments correctly from last stored value

---

## Consequences

**Until resolved:**
- Gateway must re-register public key after every node power cycle
- Anti-replay sequence counter resets to zero on reboot
- Not suitable for unattended deployment

**Once resolved:**
- Single provisioning event per node — public key registered once at factory
  or first boot, stable for device lifetime
- Sequence counter survives power cycles — full anti-replay guarantee
- Production deployment viable
