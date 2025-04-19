<details>
<summary><b><u><font size="5">Plans 0</font></u></b></summary>

- [ ] Explicit MPC
- [ ] ZeroMQ 
- [ ] ROS-like communication -> https://medium.com/@connecttosaurabh.verma/building-a-robust-publish-subscribe-mechanism-in-c-6735a7055eef
- [ ] Better communication with ground unit

</details>

<details>
<summary><b><u><font size="5">Plans 1</font></u></b></summary>

Here’s a high‑level roadmap and concrete techniques you can apply at each layer of your stack—algorithm, code, middleware and hardware—to squeeze out maximum speed and minimal footprint on a Jetson Nano.

## Summary of Recommendations

- [ ] Profile first – pinpoint your true bottlenecks (solver vs. comms vs. threading overhead).

- [ ] Algorithmic simplification – trade small amounts of optimality for huge speed gains via shorter horizons, move blocking, explicit/linear MPC, warm‐starting and code‐generated solvers.

- [ ] Lean solver stack – generate standalone C code (no CasADi overhead at runtime), switch to ultra‑light QP solvers (HPIPM, qpOASES, acados), tune tolerances and leverage sparse linear algebra.

- [ ] C++ optimizations – compile with aggressive flags, enable LTO, pre‑allocate and reuse buffers, avoid dynamic allocations in the real‑time loop, favor stack‐allocated/fixed‐size containers, and minimize branches.

- [ ] Middleware & threading – replace heavy ROS messaging with lock‑free, zero‑copy ring buffers or shared memory; pin real‑time threads to isolated cores; use Linux SCHED_FIFO.

- [ ] Leverage Jetson’s GPU – offload dense linear algebra and parallelizable portions (e.g. Cholesky, matrix operations) to CUDA kernels or cuBLAS, or convert part of your controller to a tiny neural‐net inference via TensorRT.

### 1. Profile and Identify Hotspots
Before optimizing, you need data:

Instrument your code with a lightweight tracer (e.g. LTTng, Linux perf, or even std::chrono) to measure: solver solve times, communication latencies, serialization, thread wakeups, memory allocs.

Plot histograms of loop‐cycle times to see occasional spikes vs. average.

Profile memory usage (Valgrind’s Massif or jemalloc stats) to catch heap fragmentation or accidental allocations in the real‐time loop.

By focusing on the 1–2 “hot” functions that dominate CPU time you get the best ROI on your optimization effort.

### 2. Algorithmic Level: Simplify and Precompute
Shorten the horizon – even one fewer step can cut solve time by 20–30%.

Move‑blocking – hold control inputs constant over multiple steps; reduces decision variables.

Explicit MPC – precompute the piecewise‐affine law offline if state dimension ≤ 6–8; then the online evaluation is just a table lookup plus a few multiplies.

Linearize around operating point – switch to a time‑varying linear MPC (fast QP) if nonlinearities are mild.

Warm start – take the previous solution as initial guess for the new solve; most solvers support passing the last primal/dual iterates.

Constraint softening & relaxation – loosening tolerances (e.g. absolute constraint violations of 1e‑3 instead of 1e‑6) can halve iterations.

### 3. Solver Stack: Code Generation & Lightweight QP
CasADi C code generation – export your NLP/QP directly to plain C (casadi::CodeGenerator) so you avoid any dynamic memory or Python/C++ “glue” overhead at runtime.

Switch to acados or HPIPM – these frameworks target embedded MPC, generate highly optimized C, exploit problem structure and sparse factorization.

Tune solver settings – lower max iterations, use warm starts, pick the most efficient linear solver (e.g. Cholesky vs. LDLᵀ), and exploit Hessian sparsity.

### 4. C++ & Build Optimizations
Compiler flags: -O3 -march=armv8-a+crypto -ffast-math -flto -fno-exceptions -fno-rtti

Link-time Optimization (LTO): merges translation units, inlines across files.

Pre‑allocate buffers: no new/delete inside your liveLoop().

Use fixed‐size Eigen matrices (Eigen::Matrix<double, N, N>) to enable stack allocation and loop unrolling.

Avoid virtual calls in your hot path—if liveLoop() is performance‐sensitive, flatten polymorphism or de‑virtualize.

Branch‐avoidance: replace if statements with ternary or bit‐wise tricks where safe.

### 5. Middleware & Real‑Time Scheduling
Lightweight pub/sub: swap out ROS topics for a single memory‐mapped ring buffer or ZeroMQ PUSH/PULL sockets in inproc mode; achieve zero‐copy.

Thread pinning: bind your solver thread to a big CPU core with sched_setaffinity(…, SCHED_FIFO, priority=90) so it never migrates.

Isolate cores: on the Nano, isolate one CPU core (using kernel boot parameter isolcpus=) for your real‑time tasks only.

Avoid mutex contention: use lock‑free queues (e.g. boost::lockfree::spsc_queue) or atomics for heartbeat flags and data exchange.

### 6. Hardware Acceleration on Jetson Nano
CUDA/cuBLAS – offload dense linear algebra (matrix‐vector multiplies, Cholesky factorization) to the GPU. CasADi can emit CUDA kernels, or you can hand‐write small kernels for your KKT factorization.

TensorRT micro‑net controller – approximate your MPC policy by a small feed‑forward network (e.g. ≤ 5 layers, 32 neurons each). At runtime you do one TensorRT inference (< 1 ms) instead of a full solve.

Tensor cores (if available) – can accelerate half‑precision GEMMs if your task tolerates FP16.

### 7. Continuous Testing & Validation
Automated regression – keep a suite of scenarios (straight, cornering, disturbance) and log solve times/constraint violations after every code change.

Code coverage & static analysis – tools like clang-tidy and AddressSanitizer with -fsanitize=address to ensure no hidden slow paths or memory issues slip in.

Hardware‑in‑the‑loop (HIL) – test on the actual Jetson Nano early and often; CPU throttling and OS jitter only appear on hardware, not your desktop.

Next Steps

Profile: spend a day instrumenting all critical paths.

Solver prototype: generate C code via CasADi and benchmark vs. acados/HPIPM.

Middleware proof‑of‑concept: swap ROS topics for a lock‑free queue and measure latency.

GPU offload test: implement one small dense solve on CUDA and compare timings.

By iterating through these layers—profile, simplify, compile, streamline middleware, and leverage hardware—you’ll iteratively harvest 2×–5× speedups while keeping your binary slim and deterministic. Good luck!

## Road

Below is a set of concrete changes you can make—grouped by layer—to your existing code. Each suggestion maps directly onto parts of your Subsystem/MainSystem framework and the Jetson Nano environment.

### 1. Replace per‑init thread spawn/join with a persistent worker thread
Why: creating and tearing down std::thread on every init()/halt() costs hundreds of microseconds each time and can fragment the heap.

What to change:

Move thread creation to your constructor (or first init) and never destroy it.

Use a std::condition_variable to wake the thread when you want it to run, and another to signal halt.

```
// In Subsystem.h, replace loopThread logic with:

class Subsystem {
  // …
  std::thread worker;
  std::mutex  mtx;
  std::condition_variable cv_run, cv_halt;
  bool        run_requested = false, shutdown_requested = false;

public:
  Subsystem(/*…*/) {
    // launch once
    worker = std::thread([this]{ this->workerLoop(); });
  }
  ~Subsystem() {
    {
      std::lock_guard lk(mtx);
      shutdown_requested = true;
      cv_run.notify_one();
    }
    worker.join();
  }

  void init() override {
    {
      std::lock_guard lk(mtx);
      run_requested = true;
    }
    cv_run.notify_one();
  }
  void halt() override {
    {
      std::lock_guard lk(mtx);
      run_requested = false;
    }
    cv_halt.notify_one();
  }

private:
  void workerLoop() {
    std::unique_lock lk(mtx);
    while (!shutdown_requested) {
      cv_run.wait(lk, [&]{ return run_requested || shutdown_requested; });
      if (shutdown_requested) break;
      // now do liveLoop until halted
      while (run_requested) {
        lk.unlock();
        liveLoop();            // your real-time loop body
        lk.lock();
      }
      cv_halt.wait(lk, [&]{ return !run_requested || shutdown_requested; });
    }
  }
};
```
Benefit: no more thread‐create/join in your real‐time path, eliminating that overhead every time you INIT/HALT a subsystem.

### 2. Swap out system.addPacket(...) for a lock‑free ring buffer
Why: the current addPacket is almost certainly using mutexes or I/O under the hood, adding latency and jitter.

What to change:

Introduce a single-producer, single-consumer ring buffer (e.g. boost::lockfree::spsc_queue or a custom template).

Each subsystem just does a non‑blocking push(). Your main health‐checker thread does a pop() in its own loop.

```
// Shared between MainSystem and Subsystems:
#include <boost/lockfree/spsc_queue.hpp>
struct Packet { uint8_t system, option, control, mode, order; };
static boost::lockfree::spsc_queue<Packet, boost::lockfree::capacity<1024>> packetQueue;

// In Subsystem:
packetQueue.push({0,0,1,0,order});  // instead of system.addPacket(...)

// In MainSystem’s healthCheckThread:
Packet p;
while (packetQueue.pop(p)) {
  // dispatch p to log or handleCommand
}
```
Benefit: zero‑copy, wait‑free enqueue/dequeue, no heap or mutex overhead.

### 3. Pin your real‑time threads and isolate a core
Why: on a multicore Linux system (like Jetson Nano), threads migrate and contend.

Isolate one CPU core for your control loop via kernel isolcpus= or cset shield.

In code, set your solver‑thread affinity to that core and give it SCHED_FIFO priority:

```
#include <pthread.h>
#include <sched.h>

void pinToCore(int core_id, int priority) {
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(core_id, &cpuset);
  pthread_t thr = pthread_self();
  pthread_setaffinity_np(thr, sizeof(cpuset), &cpuset);
  struct sched_param param{ .sched_priority = priority };
  pthread_setschedparam(thr, SCHED_FIFO, &param);
}

// At the start of liveLoop():
pinToCore(3 /*your isolated core*/, 80);
```
Benefit: your control thread never gives up CPU time unexpectedly, yielding more deterministic cycle times.

### 4. Pre‑allocate all buffers and eliminate dynamic memory in liveLoop()
Why: even a single hidden new/vector.push_back can trigger kmalloc and fragmentation.

What to change:

Replace any std::vector<double> in your solver call with std::array<double, N>.

If you must use Eigen, prefer fixed‐size (Eigen::Matrix<double,N,M>).

Allocate solver workspaces once at startup (CasADi codegen can give you malloc() calls—override those or supply your own allocator).

```
// Example:
static constexpr int HORIZON = 10;
std::array<Eigen::VectorXd, HORIZON> x_buffer;  // stack or global
// fill x_buffer[i] each cycle, never reallocate
```
Benefit: no heap churn, caches stay hot, dramatically smoother timings.

### 5. De‑virtualize your hot‐path
Why: virtual calls introduce an indirect jump each iteration.

What to change:

Instead of Subsystem* with a virtual liveLoop(), use a template-based CRTP or a flat switch(order) inside a single mainLoop(), inlining each subsystem’s code.

```
// CRTP sketch:
template<typename Impl>
class SubsystemCRTP {
public:
  void live() {
    static_cast<Impl*>(this)->liveLoopImpl();
  }
};

// then each MotionSystem : public SubsystemCRTP<MotionSystem> { … };
```
Benefit: the compiler can inline liveLoopImpl() and remove the vtable lookup.

### 6. GPU‑accelerate your solver’s linear algebra
Why: the Nano’s ARM CPU is slow for big matrix factorizations; its GPU is under‑utilized.

What to change:

In your CasADi codegen, enable CUDA code generation for the KKT solve.

Or extract your Hessian/isocontour solve into a small CUDA kernel or use cuBLAS/cuSOLVER:

```
// Pseudocode:
cublasDgemm(handle, CUBLAS_OP_N, CUBLAS_OP_N, N, N, N,
            &alpha, d_H, N, d_x, N, &beta, d_out, N);
// then call cuSOLVER for Cholesky on d_H
Benefit: factorization times drop from tens of milliseconds to a few.

7. Build & Compiler flags
Add to your CMakeLists.txt:

cmake
Copy
Edit
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=armv8-a+crypto -ffast-math -flto -fno-exceptions -fno-rtti")
```
Benefit: maximum inlining, vectorization, link‑time optimizations, and removed RTTI/exceptions code bloat.

Putting it all together
Refactor your Subsystem to use one persistent thread + condition vars.

Swap system.addPacket() for a shared lock‑free SPSC queue.

Pin & isolate your solver thread on core 3 (or any big core) under SCHED_FIFO.

Pre‑allocate all data buffers and use fixed‐size containers.

De‑virtualize the hot path via CRTP or flattening.

Offload linear algebra to CUDA/cuBLAS.

Rebuild with -O3 -march=… -flto.

By rolling these changes in stages—and measuring after each—you should see cycle‐time drop by 2×–5× and far fewer jitter spikes on your Jetson Nano.
</details>

<details>
<summary><b><u><font size="5">Plans 2</font></u></b></summary>

To optimize your autonomous system for speed, efficiency, and lightweight operation on Jetson Nano, here's a structured approach:

## 1. Algorithm-Level Optimization
MPC Improvements
- [ ] Code Generation: Use CasADi's codegen to create lightweight C functions for your MPC solver

- [ ] Solver Selection: Try embedded-focused solvers like HPIPM or ACADO

- [ ] Horizon Reduction: Experiment with shorter prediction/control horizons

- [ ] Model Simplification: Consider quasi-Newton approximations or residual-based methods

### Example MPC Codegen
```
// Generate C code from CasADi
Function mpc_solver = ...;
mpc_solver.generate("mpc_solver");
```
## 2. System Architecture Optimization
### Threading Improvements
```
// Set thread priorities in liveLoop
void ControlSystem::liveLoop() {
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    // Control logic
}
```
### Data Sharing Optimization
Use lock-free structures for inter-subsystem communication:

```
template<typename T>
class LockFreeQueue {
    std::atomic<size_t> head{0}, tail{0};
    T buffer[BUFFER_SIZE];
public:
    bool push(const T& item) { /*...*/ }
    bool pop(T& item) { /*...*/ }
};
```
## 3. Code-Level Optimization
### Memory Management
```
// Pre-allocate memory pools
class ControlMemoryPool {
    static constexpr size_t POOL_SIZE = 1024;
    std::array<ControlState, POOL_SIZE> buffer;
    std::atomic<size_t> index{0};
public:
    ControlState* allocate() noexcept {
        const size_t i = index.fetch_add(1) % POOL_SIZE;
        return &buffer[i];
    }
};
```
### Vectorization
```
// Ensure Eigen uses vectorization
Eigen::VectorXd calculateForces(const Eigen::VectorXd& state) {
    Eigen::VectorXd result(6);
    // SIMD-optimized operations
    result = state.cwiseProduct(Eigen::VectorXd::Constant(6, 2.0));
    return result;
}
```
## 4. System Configuration
### Compiler Flags
```
# Use ARM-specific optimizations
CXXFLAGS += -O3 -mcpu=cortex-a57 -mfpu=neon-fp-armv8 -mfloat-abi=hard -ffast-math
```
### GPU Acceleration
```
// Sample CUDA integration for matrix operations
__global__ void cudaMatrixMultiply(double* C, const double* A, const double* B, int N) {
    // CUDA matrix multiplication kernel
}
```
## 5. Real-Time Optimization
### Timing Critical Loops
```
void MotionSystem::liveLoop() {
    using clock = std::chrono::high_resolution_clock;
    auto next = clock::now();
    
    while(live) {
        next += std::chrono::milliseconds(5);  // 200Hz
        
        // Critical motion code
        calculateTrajectory();
        
        std::this_thread::sleep_until(next);
    }
}
```
## 6. Deployment Optimizations
Remove Debug Symbols: strip executable

Static Linking: Link dependencies statically

Process Isolation: Use cgroups to dedicate CPU cores to critical subsystems

## 7. Monitoring & Validation
### Implement lightweight telemetry:

```
class Telemetry {
    std::array<uint64_t, 256> counters;
public:
    void increment(size_t idx) noexcept {
        counters[idx % 256]++;
    }
    
    void log() const {
        // Fast binary logging
    }
};
```
#### Key Recommendations:
- [ ] Profile First: Use perf/gprof to identify bottlenecks

- [ ] Prioritize Hot Paths: Focus optimization on control/motion loops

- [ ] Conservative Locking: Use atomics instead of mutexes where possible

- [ ] Memory Alignment: Ensure critical data structures are cache-aligned

- [ ] Quantized Math: Consider fixed-point arithmetic for non-critical calculations

#### Expected Improvements:
- [ ] MPC execution time reduction: 30-60%

- [ ] Thread switching latency: < 10μs

- [ ] Memory footprint reduction: 20-40%

- [ ] System jitter: < 100μs

These optimizations should be applied incrementally with thorough testing at each stage to verify real-world performance improvements on your specific hardware and workload.

</details>

<details>
<summary><b><u><font size="5">Vehicle Control System Improvement Roadmap</font></u></b></summary>

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
</details>

<details>
<summary><b><u><font size="5">Vehicle Control System Improvement Rollout Plan</font></u></b></summary>

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
</details>


