# Vehicle Control System Improvement Roadmap

## Core Architecture
- [x] Implement RAII wrappers for all critical resources (OpenSSL contexts, socket handles, thread lifetimes)
   - Example: Wrap `EVP_CIPHER_CTX` initialization and cleanup in a `CipherContext` class so that destruction always frees resources.
- [x] Replace raw pointers in subsystem containers with `std::unique_ptr` for automatic cleanup
   - Example: Store subsystems in `std::vector<std::unique_ptr<Subsystem>>` instead of raw pointers.
- [ ] Design and integrate a lightweight dependency injection framework
   - Example: Define factory functions to instantiate subsystems with explicit dependencies injected.
- [ ] Formalize a subsystem lifecycle state machine (INIT → RUNNING → SUSPENDED → HALT)
   - Example: Define `enum class LifecycleState { Uninitialized, Initialized, Running, Suspended, Halted }` and transitions in a central manager.

## Network Layer
- [ ] Migrate blocking sockets to an async I/O model using `epoll` (Linux) or `kqueue` (BSD/macOS)
   - Example: Implement an `AsyncSocket` wrapper that uses edge-triggered `epoll` for read/write events.
- [ ] Add a protocol-level heartbeat and timeout mechanism
   - Example: Send a zero-length packet every 2 s and trigger reconnection after 3 missed heartbeats.
- [ ] Develop exponential backoff reconnection strategy for flaky links
   - Example: On disconnect, retry after 1 s, 2 s, 4 s, up to a maximum of 30 s between attempts.
- [ ] Implement COBS (Consistent Overhead Byte Stuffing) framing for unambiguous message boundaries
   - Example: Wrap each marshaled protobuf message with COBS encode/decode layers.
- [ ] Create a network simulation mode to inject packet loss, latency, and distortion
   - Example: Use a `SimulatedLink` class to throttle/reorder packets for HIL testing.

## Security
- [ ] Add a key-rotation mechanism for encryption keys at runtime
   - Example: Derive new keys via HKDF every 24 h and reinitialize the encryption context without downtime.
- [ ] Integrate HKDF-based key derivation for session keys
   - Example: Use `HMAC-SHA256` HKDF to derive IVs and data-encryption keys from a master secret.
- [ ] Build a secure firmware-update system with signed payloads
   - Example: Verify update packages with RSA-ECDSA signatures before applying.
- [ ] Include per-message sequence numbers and replay-protection window
   - Example: Embed a 32-bit counter and drop any out-of-order or stale packets.
- [ ] Offer an optional TLS fallback channel using mbedTLS or OpenSSL SSL API
   - Example: If the custom AES-GCM channel fails, renegotiate over `TLS1.3`.

## Configuration
- [ ] Consolidate all settings into a unified JSON/YAML configuration file
   - Example: Merge socket, encryption, and subsystem parameters into `config/system.yaml`.
- [ ] Support runtime reconfiguration via a management socket or REST API
   - Example: Apply logging level changes without restart by reloading JSON and updating loggers.
- [ ] Implement configuration versioning and upgrade path
   - Example: Embed a `version` field, auto-migrate deprecated keys, and validate schema.
- [ ] Validate configurations on startup using JSON Schema
   - Example: Reject invalid port numbers or missing fields at launch with clear errors.

## Testing
- [ ] Set up a GoogleTest (or Catch2) framework for C++ unit tests
   - Example: Write unit tests for `serialize()`/`deserialize()` in `SystemData`.
- [ ] Build a hardware-in-the-loop (HIL) test rig integrating sensor stubs
   - Example: Mock sensor inputs in software and verify control outputs before deploying.
- [ ] Develop a fault-injection harness to simulate exceptions and I/O errors
   - Example: Inject socket write failures and assert that the system recovers gracefully.
- [ ] Implement a network fuzzer to send malformed packets
   - Example: Randomize COBS frames and check that the receiver rejects invalid CRCs.
- [ ] Automate builds and tests in a CI/CD pipeline (GitHub Actions/Jenkins)
   - Example: On each PR, run `cmake --build` + `ctest` + `clang-tidy` checks.

## Performance
- [ ] Replace blocking queues with lock-free, bounded MPMC queues for inter-thread messages
   - Example: Integrate `folly::MPMCQueue` or `boost::lockfree::queue`.
- [ ] Add custom memory pools (object pools) to reduce heap fragmentation
   - Example: Use `mempool` for fixed-size `Packet` allocations.
- [ ] Explore SIMD optimizations for signal processing and data serialization
   - Example: Use `x86_64` AVX2 intrinsics in compression routines.
- [ ] Leverage shared memory (POSIX `shm_open`) channels for local IPC
   - Example: Expose telemetry via a shared-memory ring buffer.
- [ ] Implement zero-copy message passing whenever possible
   - Example: Use `mmap`-backed buffers to avoid payload copies between threads.

## Observability & Logging
- [ ] Integrate a structured logging library (spdlog or fmt) with log levels and sinks
   - Example: Output JSON logs to stdout and rotate file logs daily.
- [ ] Add Prometheus-compatible metrics (counters, gauges, histograms)
   - Example: Export subsystem heartbeat latencies and packet counts via an HTTP endpoint.
- [ ] Implement live health dashboards (Grafana or ROS 2 rqt plugins)
   - Example: Display per-module CPU/latency trends in real time.
- [ ] Develop alerting on failure thresholds (e.g., missed heartbeats)
   - Example: Trigger an e-mail or RabbitMQ alert if any subsystem fails twice.

## Documentation & Quality Assurance
- [ ] Generate Doxygen API docs and host via GitHub Pages
   - Example: Document `sensor_system.h` and link usage examples.
- [ ] Maintain a detailed CHANGELOG (Keep a Changelog format) per release
   - Example: Record new features, bug fixes, and configuration changes.
- [ ] Define and enforce coding standards with `clang-format` and `clang-tidy`
   - Example: Add a pre-commit hook to auto-format code and block style violations.
- [ ] Implement static analysis (Coverity, `cppcheck`, or `clang-analyzer`)
   - Example: Fail CI if any new high-severity issues are detected.
- [ ] Create a code-review checklist to ensure consistent reviews
   - Example: Verify thread-safety, error handling, and resource management per PR.

## Deployment & Operations
- [ ] Containerize the system with Docker and provide multi-stage builds
   - Example: Build a minimal `distroless` image separating build/runtime.
- [ ] Supply systemd service files (or launch scripts) for auto-start and watchdog
   - Example: Use `Restart=on-failure` and `StartLimitIntervalSec` in the unit file.
- [ ] Develop a rollback mechanism for safe updates
   - Example: Keep two versions on disk and switch symlinks atomically.
- [ ] Integrate remote diagnostics and over-the-air logs retrieval
   - Example: Fetch recent JSON logs via an SSH/HTTPS interface.
- [ ] Scan containers and binaries for vulnerabilities (Trivy or Clair)
   - Example: Fail deployment if a CVE > 7.0 is detected.


# Vehicle Control System Improvement Rollout Plan

This roadmap breaks the improvements into four incremental phases, ensuring core stability before adding advanced features or optimizations.

## Phase 1: Foundation & Safety
1. **Implement RAII wrappers for all critical resources** (OpenSSL contexts, socket handles, thread lifetimes)
   - Example: Create `CipherContext` to manage `EVP_CIPHER_CTX` lifecycle.
2. **Replace raw pointers with `std::unique_ptr`** in subsystem containers
   - Example: Store subsystems in `std::vector<std::unique_ptr<Subsystem>>`.
3. **Formalize a subsystem lifecycle state machine** (INIT → RUNNING → SUSPENDED → HALT)
   - Example: Define `enum class LifecycleState` and transitions in `MainSystem`.
4. **Design and integrate dependency injection framework**
   - Example: Implement factories that inject `SystemData`, `EnvironmentState`, etc.

## Phase 2: Robust Networking & Configuration
5. **Migrate to async I/O** using `epoll` (Linux) or `kqueue` (macOS)
6. **Add protocol-level heartbeat and timeout**
7. **Develop exponential backoff reconnection strategy**
8. **Consolidate settings into unified JSON/YAML config**
9. **Validate configuration on startup** via JSON Schema
10. **Implement runtime reconfiguration** through management socket or REST API

## Phase 3: Security & Observability
11. **Add key-rotation mechanism & HKDF derivation**
12. **Include per-message sequence numbers & replay protection**
13. **Implement COBS framing and optional TLS fallback**
14. **Integrate structured logging library** (spdlog/fmt)
15. **Add Prometheus metrics & live health dashboards**
16. **Develop alerting on failure thresholds**

## Phase 4: Testing, Performance & Deployment
17. **Setup GoogleTest framework & CI/CD pipeline**
18. **Develop fault-injection harness & network fuzzer**
19. **Replace blocking queues with lock-free MPMC queues**
20. **Add memory pooling & zero-copy channels**
21. **Explore SIMD optimizations**
22. **Containerize system and supply systemd service**
23. **Create rollback mechanism & remote diagnostics**
24. **Implement static analysis & coding standards enforcement**
25. **Generate Doxygen docs & maintain CHANGELOG**

