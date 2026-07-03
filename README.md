# MIPS-Lite Functional & Pipeline Simulator

A C++ simulator for a reduced MIPS instruction set ("MIPS-Lite"), built for ECE 486/586 (Computer Architecture). The simulator supports three modes: a functional (instruction-level) simulator, a 5-stage pipeline simulator without forwarding, and a 5-stage pipeline simulator with forwarding - all driven from the same instruction trace.

Developed as a 4-person team project by Nikhil Swarna, Venkat Sai Sumanth Koyada, Venkata Sriram Kamarajugadda, and Hanisha Produtur.

## Overview
The simulator loads a memory image (a hex-encoded instruction trace), decodes each 32-bit instruction across 18 supported opcodes (arithmetic, logical, memory, and control categories), and executes it against a simulated register file and memory. Three modes are available:

- **Mode 0 (Part A):** Functional simulator only — executes instructions and reports final register/memory state.
- **Mode 1 (Part B):** Functional simulator + 5-stage pipeline timing **without** forwarding — models RAW hazard stalls purely by instruction issue-cycle distance.
- **Mode 2 (Part C):** Functional simulator + 5-stage pipeline timing **with** forwarding — models EX→EX and MEM→EX forwarding, leaving only the load-use hazard as a stall source.

All three modes share the same functional execution semantics; only the pipeline timing model changes.

## My Contribution
I implemented the **no-forwarding pipeline model** (Part B), corresponding to the `forwarding_ == false` branch of `PipelineSim::run()` in `src/mips_lite_sim.cpp`. This includes the RAW hazard stall-distance logic that enforces a producer's writeback (WB) completing before a consumer's decode (ID) can proceed — requiring a 3-cycle issue separation (`need = issue[j] + 3`) between a producer and any dependent consumer within a 2-instruction lookback window, modeling a 2-cycle stall for back-to-back dependent instructions and a 1-cycle stall for one-instruction-separated dependencies. I also contributed to the shared stall/branch-penalty accounting (`data_stalls_`, `branch_penalty_cycles_`) and the `printResults()` reporting logic used by both pipeline modes.

## Team Contributions

| Member | Area | Responsibility |
|---|---|---|
| Nikhil Swarna | Trace reading, types, decoding, statistics | Memory image parsing, instruction decode (`decode()`, `isRType()`, `categoryOf()`), shared types/constants, instruction category statistics |
| **Venkat Sai Sumanth Koyada** | No-forwarding pipeline | RAW hazard stall logic for the no-forwarding model (`PipelineSim::run()`, `forwarding_ == false` path) |
| Venkata Sriram Kamarajugadda | Functional simulator | `FunctionalSim` class — instruction execution, register/memory state, PC and branch update logic, top-level driver |
| Hanisha Produtur | Forwarding pipeline | RAW hazard stall logic for the forwarding model (`PipelineSim::run()`, `forwarding_ == true` path) — EX→EX/MEM→EX forwarding and load-use hazard handling |


## Project Structure
```
src/    → Simulator source (mips_lite_sim.cpp)
tests/  → Test memory images (test1.txt-test14.txt, stress_test.txt)
docs/   → Roles and responsibilities document
```

## Build & Run
```
g++ -std=c++17 -O2 -o mips_lite_sim src/mips_lite_sim.cpp
./mips_lite_sim <memory_image.txt>          # runs all three modes
./mips_lite_sim -m 0 <memory_image.txt>     # Part A only
./mips_lite_sim -m 1 <memory_image.txt>     # Part B only (no forwarding)
./mips_lite_sim -m 2 <memory_image.txt>     # Part C only (with forwarding)
```

Example:
```
./mips_lite_sim tests/test1.txt
```
