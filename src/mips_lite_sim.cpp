/******************************************************************************
 * mips_lite_sim.cpp
 *
 * ECE 486/586 Term Project — UNIFIED SIMULATOR (Parts A, B, and C)
 *
 *   Mode 0 (Part A): Functional simulator only.
 *   Mode 1 (Part B): Functional + 5-stage pipeline, NO FORWARDING.
 *   Mode 2 (Part C): Functional + 5-stage pipeline, WITH FORWARDING.
 *
 * Usage:
 *   ./mips_lite_sim <image>          Run all three modes in sequence.
 *   ./mips_lite_sim -m 0 <image>     Run only Part A.
 *   ./mips_lite_sim -m 1 <image>     Run only Part B.
 *   ./mips_lite_sim -m 2 <image>     Run only Part C.
 *   ./mips_lite_sim                  Run all three modes against "input.txt".
 *
 * Functional behavior is identical across all modes; the only thing that
 * changes is whether (and how) we compute pipeline timing.
 *
 * Pipeline conventions (from lec6_data_hazard, lec7_cntl_hazard_forwarding):
 *
 *   No-forwarding RAW stall penalty (Mode 1):
 *     producer-consumer cycle distance 1 (back-to-back) : 2-cycle stall
 *     producer-consumer cycle distance 2 (one between)  : 1-cycle stall
 *     producer-consumer cycle distance 3+                : 0-cycle stall
 *       (half-cycle convention: WB in 1st half, ID in 2nd half of the cycle)
 *
 *   With-forwarding RAW stall penalty (Mode 2):
 *     EX->EX and MEM->EX forwarding eliminate ALL ALU-to-ALU stalls.
 *     The only remaining stall is the LOAD-USE hazard:
 *       LDW followed immediately (distance 1) by an instruction
 *       reading the loaded register costs exactly 1 stall cycle.
 *     At distance 2+, MEM->EX forwarding from the load handles it
 *     with no stall.
 *
 *   Branch resolution at end of EX (both pipeline modes):
 *     taken     : 2-cycle penalty (next two IF slots flushed)
 *     not-taken : 0-cycle penalty
 *     JR        : always treated as taken (unconditional)
 *     HALT      : pipeline drains, no flush
 *
 *   Total cycles = N + 4 + data_stalls + branch_penalty_cycles
 *
 * Build:   g++ -std=c++17 -O2 -o mips_lite_sim mips_lite_sim.cpp
 * Run:     ./mips_lite_sim <memory_image.txt>
 ******************************************************************************/

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdint>
#include <iomanip>
#include <algorithm>

using namespace std;

/******************************************************************************
 * Constants and types
 ******************************************************************************/

// Memory size in bytes. Spec says images are limited to 4 KB (1024 words).
static const uint32_t MEM_SIZE_BYTES = 4096;
static const uint32_t MEM_SIZE_WORDS = MEM_SIZE_BYTES / 4;   // 1024

// Opcodes, per the project spec (see ece586_project_specs.pdf)
enum Opcode : uint8_t {
    OP_ADD  = 0x00,
    OP_ADDI = 0x01,
    OP_SUB  = 0x02,
    OP_SUBI = 0x03,
    OP_MUL  = 0x04,
    OP_MULI = 0x05,
    OP_OR   = 0x06,
    OP_ORI  = 0x07,
    OP_AND  = 0x08,
    OP_ANDI = 0x09,
    OP_XOR  = 0x0A,
    OP_XORI = 0x0B,
    OP_LDW  = 0x0C,
    OP_STW  = 0x0D,
    OP_BZ   = 0x0E,
    OP_BEQ  = 0x0F,
    OP_JR   = 0x10,
    OP_HALT = 0x11
};

// Instruction categories used in the report
enum Category {
    CAT_ARITH = 0,
    CAT_LOGIC = 1,
    CAT_MEM   = 2,
    CAT_CTRL  = 3
};

// Decoded instruction
struct Instr {
    uint32_t raw;       // original 32-bit word
    uint8_t  opcode;    // 6-bit opcode
    uint8_t  rs;        // 5-bit source
    uint8_t  rt;        // 5-bit target/source
    uint8_t  rd;        // 5-bit destination (R-type only)
    int32_t  imm;       // 16-bit signed immediate (sign-extended)
    bool     is_rtype;  // true for R-type, false for I-type
    Category category;
};

// One entry in the dynamic instruction trace.
// Part A only needs to execute; Part B needs to know, for each executed
// instruction, what its decoded form was AND whether (if it was a branch)
// the branch was taken.
struct TraceEntry {
    Instr instr;
    bool  branch_taken;   // BZ/BEQ that branched, or JR (always taken)
};

/******************************************************************************
 * Helpers
 ******************************************************************************/

// Returns true if this opcode is R-type (uses rd, no immediate).
static bool isRType(uint8_t op) {
    return op == OP_ADD || op == OP_SUB || op == OP_MUL ||
           op == OP_OR  || op == OP_AND || op == OP_XOR;
}

// Returns the report category for an opcode.
static Category categoryOf(uint8_t op) {
    switch (op) {
        case OP_ADD: case OP_ADDI:
        case OP_SUB: case OP_SUBI:
        case OP_MUL: case OP_MULI:
            return CAT_ARITH;
        case OP_OR:  case OP_ORI:
        case OP_AND: case OP_ANDI:
        case OP_XOR: case OP_XORI:
            return CAT_LOGIC;
        case OP_LDW: case OP_STW:
            return CAT_MEM;
        case OP_BZ:  case OP_BEQ:
        case OP_JR:  case OP_HALT:
            return CAT_CTRL;
        default:
            return CAT_CTRL; // unreachable for valid programs
    }
}

// Decode a 32-bit word into an Instr.
//
// R-type: [opcode:6][rs:5][rt:5][rd:5][unused:11]
// I-type: [opcode:6][rs:5][rt:5][imm:16]
static Instr decode(uint32_t word) {
    Instr i;
    i.raw      = word;
    i.opcode   = (word >> 26) & 0x3F;
    i.rs       = (word >> 21) & 0x1F;
    i.rt       = (word >> 16) & 0x1F;
    i.is_rtype = isRType(i.opcode);
    i.category = categoryOf(i.opcode);

    if (i.is_rtype) {
        i.rd  = (word >> 11) & 0x1F;
        i.imm = 0;
    } else {
        i.rd  = 0;
        // Sign-extend the 16-bit immediate to 32 bits.
        uint16_t imm16 = word & 0xFFFF;
        i.imm = static_cast<int32_t>(static_cast<int16_t>(imm16));
    }
    return i;
}

// -- Register-use helpers for pipeline analysis ------------------------------
//
// destReg(I)  : the register written by I, or -1 if none.
// srcRegs(I)  : up to two register IDs read by I.
//
// We *do not* treat R0 as a hardwired zero (consistent with Part A's spec
// reading), so a write to R0 counts as a real producer and a read of R0
// counts as a real consumer.

static int destReg(const Instr& I) {
    switch (I.opcode) {
        case OP_ADD: case OP_SUB: case OP_MUL:
        case OP_OR:  case OP_AND: case OP_XOR:
            return I.rd;                       // R-type writes rd
        case OP_ADDI: case OP_SUBI: case OP_MULI:
        case OP_ORI:  case OP_ANDI: case OP_XORI:
        case OP_LDW:
            return I.rt;                       // I-type ALU + LDW write rt
        case OP_STW:                           // store writes no register
        case OP_BZ: case OP_BEQ: case OP_JR:   // branches write no register
        case OP_HALT:
        default:
            return -1;
    }
}

// Returns true iff I reads register `r` from the register file in ID.
static bool readsReg(const Instr& I, int r) {
    if (r < 0) return false;
    switch (I.opcode) {
        // R-type ALU: reads rs and rt
        case OP_ADD: case OP_SUB: case OP_MUL:
        case OP_OR:  case OP_AND: case OP_XOR:
            return (I.rs == r) || (I.rt == r);

        // I-type ALU: reads rs only
        case OP_ADDI: case OP_SUBI: case OP_MULI:
        case OP_ORI:  case OP_ANDI: case OP_XORI:
            return (I.rs == r);

        // LDW: reads rs (base register for the effective address)
        case OP_LDW:
            return (I.rs == r);

        // STW: reads BOTH rs (base) and rt (value being stored)
        case OP_STW:
            return (I.rs == r) || (I.rt == r);

        // BZ: reads rs (compared against zero)
        case OP_BZ:
            return (I.rs == r);

        // BEQ: reads rs and rt (compared against each other)
        case OP_BEQ:
            return (I.rs == r) || (I.rt == r);

        // JR: reads rs (the jump target)
        case OP_JR:
            return (I.rs == r);

        // HALT: reads nothing
        case OP_HALT:
        default:
            return false;
    }
}

/******************************************************************************
 * Memory: 32-bit word-addressable view of a 4 KB byte-addressable space.
 * Addresses in the program are byte addresses, always multiples of 4.
 ******************************************************************************/

class Memory {
public:
    Memory() : words_(MEM_SIZE_WORDS, 0), written_(MEM_SIZE_WORDS, false) {}

    // Load memory image from a text file: one 8-hex-digit word per line.
    // Returns the number of words read.
    size_t loadFromFile(const string& path) {
        ifstream in(path);
        if (!in) {
            cerr << "Error: cannot open memory image file: " << path << "\n";
            exit(1);
        }
        size_t idx = 0;
        string line;
        while (getline(in, line)) {
            // trim whitespace
            size_t a = line.find_first_not_of(" \t\r\n");
            size_t b = line.find_last_not_of(" \t\r\n");
            if (a == string::npos) continue;          // blank line
            string tok = line.substr(a, b - a + 1);
            if (idx >= MEM_SIZE_WORDS) {
                cerr << "Error: memory image exceeds " << MEM_SIZE_WORDS
                     << " words.\n";
                exit(1);
            }
            // parse as hex
            words_[idx] = static_cast<uint32_t>(stoul(tok, nullptr, 16));
            idx++;
        }
        return idx;
    }

    // Read a 32-bit word from a byte address. Address must be 4-byte aligned
    // and within range.
    uint32_t readWord(uint32_t byte_addr) const {
        checkAccess(byte_addr);
        return words_[byte_addr / 4];
    }

    // Write a 32-bit word to a byte address (used by STW).
    // Marks the location as "modified" so we can report only changed memory.
    void writeWord(uint32_t byte_addr, uint32_t value) {
        checkAccess(byte_addr);
        uint32_t idx = byte_addr / 4;
        words_[idx]   = value;
        written_[idx] = true;
    }

    // Returns the list of (byte_addr, value) pairs that STW wrote.
    vector<pair<uint32_t, uint32_t>> changedLocations() const {
        vector<pair<uint32_t, uint32_t>> out;
        for (uint32_t i = 0; i < MEM_SIZE_WORDS; ++i) {
            if (written_[i]) {
                out.emplace_back(i * 4, words_[i]);
            }
        }
        return out;
    }

private:
    void checkAccess(uint32_t byte_addr) const {
        if (byte_addr % 4 != 0) {
            cerr << "Error: unaligned memory access at byte address "
                 << byte_addr << "\n";
            exit(1);
        }
        if (byte_addr >= MEM_SIZE_BYTES) {
            cerr << "Error: memory access out of range at byte address "
                 << byte_addr << "\n";
            exit(1);
        }
    }

    vector<uint32_t> words_;
    vector<bool>     written_;
};

/******************************************************************************
 * Functional Simulator  (Part A behavior, plus trace recording for Part B)
 ******************************************************************************/

class FunctionalSim {
public:
    FunctionalSim(Memory& mem) : mem_(mem) {
        for (int i = 0; i < 32; ++i) regs_[i] = 0;
        for (int i = 0; i < 32; ++i) regs_written_[i] = false;
        pc_ = 0;
        total_instr_ = 0;
        for (int i = 0; i < 4; ++i) cat_count_[i] = 0;
    }

    // Run until HALT.
    void run() {
        // Safety limit: a real MIPS-lite test program will execute far
        // fewer than 10 million instructions. If we exceed this, the
        // program likely contains an infinite loop and we should bail
        // out gracefully rather than hang.
        const uint64_t MAX_INSTRUCTIONS = 10000000;
        while (true) {
            if (total_instr_ >= MAX_INSTRUCTIONS) {
                cerr << "\nError: instruction limit (" << MAX_INSTRUCTIONS
                     << ") exceeded. Program likely contains an infinite loop.\n"
                     << "Current PC = " << pc_ << "\n";
                break;
            }
            // Fetch
            uint32_t word  = mem_.readWord(pc_);
            Instr    instr = decode(word);

            // Count
            total_instr_++;
            cat_count_[instr.category]++;

            // Remember pre-execute PC so we can detect taken branches.
            uint32_t old_pc = pc_;

            // Execute (also updates PC); HALT terminates the loop.
            if (instr.opcode == OP_HALT) {
                // Record HALT in the trace, then advance PC and stop.
                trace_.push_back({instr, /*branch_taken=*/false});
                pc_ += 4;
                break;
            }
            execute(instr);

            // Decide whether this instruction was a taken branch, for the
            // pipeline timing pass. JR is always taken; BZ/BEQ are taken
            // iff the new PC differs from the sequential fall-through.
            bool taken = false;
            if (instr.opcode == OP_JR) {
                taken = true;
            } else if (instr.opcode == OP_BZ || instr.opcode == OP_BEQ) {
                taken = (pc_ != old_pc + 4);
            }
            trace_.push_back({instr, taken});
        }
    }

    void printResults() const {
        cout << "Instruction counts:\n\n";
        cout << "Total number of instructions: " << total_instr_ << "\n";
        cout << "Arithmetic instructions: "      << cat_count_[CAT_ARITH] << "\n";
        cout << "Logical instructions: "         << cat_count_[CAT_LOGIC] << "\n";
        cout << "Memory access instructions: "   << cat_count_[CAT_MEM]   << "\n";
        cout << "Control transfer instructions: "<< cat_count_[CAT_CTRL]  << "\n\n";

        cout << "Final register state:\n\n";
        cout << "Program counter: " << pc_ << "\n";
        for (int i = 0; i < 32; ++i) {
            if (regs_written_[i]) {
                cout << "R" << i << ": " << static_cast<int32_t>(regs_[i]) << "\n";
            }
        }

        cout << "\nFinal memory state:\n";
        auto changed = mem_.changedLocations();
        for (auto& kv : changed) {
            cout << "Address: " << kv.first
                 << ", Contents: " << static_cast<int32_t>(kv.second) << "\n";
        }
    }

    // Accessor for Part B's pipeline analysis.
    const vector<TraceEntry>& trace() const { return trace_; }

private:
    // Treat register contents as signed 32-bit for arithmetic.
    int32_t  sreg(uint8_t r) const { return static_cast<int32_t>(regs_[r]); }

    // Write a register, treating R0 like any other (the spec does not reserve
    // R0 as hardwired-zero; PC and all GPRs initialized to 0).
    void writeReg(uint8_t r, uint32_t value) {
        regs_[r] = value;
        regs_written_[r] = true;
    }

    void execute(const Instr& I) {
        // By default the next PC is the next sequential instruction.
        uint32_t next_pc = pc_ + 4;

        switch (I.opcode) {
            // ---- Arithmetic (signed) ----
            case OP_ADD:  writeReg(I.rd, sreg(I.rs) + sreg(I.rt)); break;
            case OP_ADDI: writeReg(I.rt, sreg(I.rs) + I.imm);      break;
            case OP_SUB:  writeReg(I.rd, sreg(I.rs) - sreg(I.rt)); break;
            case OP_SUBI: writeReg(I.rt, sreg(I.rs) - I.imm);      break;
            case OP_MUL:  writeReg(I.rd, sreg(I.rs) * sreg(I.rt)); break;
            case OP_MULI: writeReg(I.rt, sreg(I.rs) * I.imm);      break;

            // ---- Logical (bitwise; sign doesn't matter) ----
            case OP_OR:   writeReg(I.rd, regs_[I.rs] |  regs_[I.rt]); break;
            case OP_ORI:  writeReg(I.rt, regs_[I.rs] |  static_cast<uint32_t>(I.imm)); break;
            case OP_AND:  writeReg(I.rd, regs_[I.rs] &  regs_[I.rt]); break;
            case OP_ANDI: writeReg(I.rt, regs_[I.rs] &  static_cast<uint32_t>(I.imm)); break;
            case OP_XOR:  writeReg(I.rd, regs_[I.rs] ^  regs_[I.rt]); break;
            case OP_XORI: writeReg(I.rt, regs_[I.rs] ^  static_cast<uint32_t>(I.imm)); break;

            // ---- Memory (signed effective address) ----
            case OP_LDW: {
                int32_t addr = sreg(I.rs) + I.imm;
                writeReg(I.rt, mem_.readWord(static_cast<uint32_t>(addr)));
                break;
            }
            case OP_STW: {
                int32_t addr = sreg(I.rs) + I.imm;
                mem_.writeWord(static_cast<uint32_t>(addr), regs_[I.rt]);
                break;
            }

            // ---- Control flow ----
            // Spec: branch target is the "x-th instruction from the current
            // instruction." Combined with the signed-arithmetic doc's PC = PC + 16
            // example, we interpret "x" as a signed byte offset added to PC+4.
            case OP_BZ:
                if (sreg(I.rs) == 0) next_pc = pc_ + (I.imm * 4);
                break;
            case OP_BEQ:
                if (sreg(I.rs) == sreg(I.rt)) next_pc = pc_ + (I.imm * 4);
                break;
            case OP_JR:
                next_pc = regs_[I.rs];
                break;

            default:
                cerr << "Error: unknown opcode 0x"
                     << hex << static_cast<int>(I.opcode) << dec
                     << " at PC=" << pc_ << "\n";
                exit(1);
        }

        pc_ = next_pc;
    }

    Memory&            mem_;
    uint32_t           regs_[32];
    bool               regs_written_[32];
    uint32_t           pc_;
    uint64_t           total_instr_;
    uint64_t           cat_count_[4];
    vector<TraceEntry> trace_;
};

/******************************************************************************
 * Pipeline Timing Simulator  (Part C — WITH FORWARDING)
 *
 * Takes the dynamic trace produced by FunctionalSim and computes:
 *
 *   - issue[i]: the cycle in which instruction i enters IF.
 *   - data_stalls : sum of load-use stall cycles inserted in ID.
 *   - branch_penalty_cycles : 2 cycles per taken branch (IF slots flushed).
 *   - total_clock_cycles : the cycle in which the last instruction's WB ends.
 *
 * Cycle layout per instruction:
 *
 *       IF       ID       EX       MEM      WB
 *   issue[i]  +1      +2       +3       +4
 *
 *   Producer's ALU result is available at END of EX  = cycle issue[j] + 2.
 *   Producer's load value is available at END of MEM = cycle issue[j] + 3.
 *   Consumer needs its source values at START of EX  = cycle issue[i] + 2.
 *
 *   With EX->EX and MEM->EX forwarding, a forwarded value from "end of
 *   cycle X" is available to the next stage at "start of cycle X+1".
 *
 * Forwarding model:
 *
 *   For each producer j read by consumer i:
 *
 *     If j is an LDW (load):
 *       Need issue[i] + 2 >= issue[j] + 3 + 1
 *       i.e.  issue[i] >= issue[j] + 2.
 *
 *       At distance 1 (issue[i] = issue[j] + 1 in a stall-free pipeline):
 *         constraint demands one extra cycle -> 1-cycle load-use stall.
 *       At distance 2+: constraint already satisfied -> no stall.
 *
 *     If j is an ALU op:
 *       Need issue[i] + 2 >= issue[j] + 2 + 1
 *       i.e.  issue[i] >= issue[j] + 1.
 *
 *       At distance 1: issue[i] = issue[j] + 1 satisfies exactly -> no stall.
 *       At distance 2+: trivially satisfied -> no stall.
 *
 *   Conclusion: under this forwarding model, the ONLY data stall that can
 *   occur is a load-use at distance 1.
 *
 * Branch penalty model (unchanged from Part B):
 *
 *   A taken branch costs 2 cycles (next two IF slots are flushed). HALT
 *   has no successor, so no penalty. JR is always treated as taken.
 *
 * Total cycle recurrence:
 *
 *   issue[0] = 1
 *   start    = issue[i-1] + 1 + (2 if i-1 is a taken branch, else 0)
 *   For producer j read by i, j = i-1 or i-2:
 *     If j is LDW : need = issue[j] + 2
 *     Else        : need = issue[j] + 1
 *     issue[i]    = max(issue[i], need)
 *   issue[i] = max(start, issue[i])
 *   data_stalls += issue[i] - start
 *
 *   total_clock_cycles = issue[N-1] + 4    (WB of last instruction)
 ******************************************************************************/

class PipelineSim {
public:
    // forwarding = false : Part B model (no forwarding, stall by cycle distance)
    // forwarding = true  : Part C model (EX->EX and MEM->EX, load-use only)
    PipelineSim(const vector<TraceEntry>& trace, bool forwarding)
        : trace_(trace), forwarding_(forwarding) {}

    void run() {
        const size_t N = trace_.size();
        total_clock_cycles_     = 0;
        data_stalls_            = 0;
        branch_penalty_cycles_  = 0;
        if (N == 0) return;

        // Cycle at which each instruction enters IF (1-indexed for readability).
        vector<uint64_t> issue(N, 0);
        issue[0] = 1;

        for (size_t i = 1; i < N; ++i) {
            // Base advance: the previous instruction's IF was 1 cycle earlier.
            uint64_t start = issue[i - 1] + 1;

            // If the previous instruction was a taken branch, the next two
            // IF slots are flushed; the target effectively starts 2 cycles later.
            if (trace_[i - 1].branch_taken) {
                start += 2;
                branch_penalty_cycles_ += 2;
            }

            // Compute the earliest cycle at which instruction i can enter IF
            // without violating any RAW dependency. The required separation
            // from a producer j depends on the forwarding model.
            //
            //   No forwarding (Part B):
            //     Consumer's ID at issue[i]+1 must be >= producer's WB at
            //     issue[j]+4, half-cycle overlap gives issue[i] >= issue[j]+3.
            //
            //   With forwarding (Part C):
            //     If j is LDW (load-use): need issue[i] >= issue[j] + 2.
            //     Else (ALU producer)  : need issue[i] >= issue[j] + 1
            //                            (always satisfied — no stall).
            uint64_t earliest_ok = start;
            for (size_t back = 1; back <= 2 && back <= i; ++back) {
                size_t j = i - back;
                int d_reg = destReg(trace_[j].instr);
                if (d_reg >= 0 && readsReg(trace_[i].instr, d_reg)) {
                    uint64_t need;
                    if (forwarding_) {
                        if (trace_[j].instr.opcode == OP_LDW) {
                            need = issue[j] + 2;  // load-use hazard
                        } else {
                            need = issue[j] + 1;  // ALU forwarding (no stall)
                        }
                    } else {
                        need = issue[j] + 3;       // no-forwarding model
                    }
                    if (need > earliest_ok) earliest_ok = need;
                }
            }
            uint64_t stall = earliest_ok - start;
            data_stalls_ += stall;
            issue[i] = earliest_ok;
        }

        // The last instruction's writeback completes 4 cycles after it
        // entered IF (IF, ID, EX, MEM, WB occupy 5 cycles total).
        total_clock_cycles_ = issue[N - 1] + 4;
    }

    void printResults() const {
        const char* label = forwarding_ ? "with forwarding" : "no forwarding";
        cout << "\nPipeline performance (5-stage, " << label << "):\n\n";
        cout << "Total clock cycles: "      << total_clock_cycles_      << "\n";
        cout << "Data hazard stalls: "      << data_stalls_             << "\n";
        cout << "Branch penalty cycles: "   << branch_penalty_cycles_   << "\n";

        // CPI as a sanity-check metric (not required, but useful).
        if (!trace_.empty()) {
            double cpi = static_cast<double>(total_clock_cycles_) /
                         static_cast<double>(trace_.size());
            cout << fixed << setprecision(3);
            cout << "CPI: " << cpi << "\n";
        }
    }

    // Public accessors so other code (e.g. tests) can read final stats.
    uint64_t totalCycles()        const { return total_clock_cycles_; }
    uint64_t dataStalls()         const { return data_stalls_; }
    uint64_t branchPenaltyCycles()const { return branch_penalty_cycles_; }

private:
    const vector<TraceEntry>& trace_;
    bool     forwarding_;
    uint64_t total_clock_cycles_;
    uint64_t data_stalls_;
    uint64_t branch_penalty_cycles_;
};

/******************************************************************************
 * Mode runners
 ******************************************************************************/

// Runs the functional simulator on a fresh memory and prints Part A output.
// Returns the trace so callers can hand it to PipelineSim if needed.
//
// Each mode reloads memory from disk because STW instructions mutate memory;
// running all three modes against the same Memory object would let Mode 0's
// stores leak into Mode 1's load addresses. A clean reload per mode keeps
// the three reports independent and reproducible.
static void runMode(int mode, const string& path) {
    Memory mem;
    size_t words = mem.loadFromFile(path);
    cout << "Loaded " << words << " words from " << path << "\n\n";

    FunctionalSim sim(mem);
    sim.run();
    sim.printResults();

    if (mode == 1) {
        PipelineSim psim(sim.trace(), /*forwarding=*/false);
        psim.run();
        psim.printResults();
    } else if (mode == 2) {
        PipelineSim psim(sim.trace(), /*forwarding=*/true);
        psim.run();
        psim.printResults();
    }
    // Mode 0: functional only, nothing else to print.

    cout << "\nProgram Halted\n";
}

static void printUsage(const char* prog) {
    cerr << "Usage:\n";
    cerr << "  " << prog << " <image>          Run all three modes.\n";
    cerr << "  " << prog << " -m 0 <image>     Part A only (functional).\n";
    cerr << "  " << prog << " -m 1 <image>     Part B only (no forwarding).\n";
    cerr << "  " << prog << " -m 2 <image>     Part C only (with forwarding).\n";
    cerr << "  " << prog << "                  Run all three against input.txt.\n";
}

/******************************************************************************
 * main
 ******************************************************************************/

int main(int argc, char** argv) {
    // Parse command line:
    //   no args         -> all modes, default image "input.txt"
    //   <image>         -> all modes, given image
    //   -m N <image>    -> single mode N (0/1/2), given image
    //   -m N            -> single mode N, default image "input.txt"
    int    selected_mode = -1;       // -1 means "run all three"
    string path          = "input.txt";

    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "-m" || arg == "--mode") {
            if (i + 1 >= argc) { printUsage(argv[0]); return 1; }
            string mstr = argv[++i];
            if (mstr != "0" && mstr != "1" && mstr != "2") {
                cerr << "Error: mode must be 0, 1, or 2 (got '"
                     << mstr << "')\n";
                printUsage(argv[0]);
                return 1;
            }
            selected_mode = stoi(mstr);
        } else if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        } else {
            path = arg;     // anything else is the image path
        }
    }

    if (selected_mode == -1) {
        // Run all three modes in sequence with clear section headers.
        const char* labels[3] = {
            "PART A: Functional Simulator",
            "PART B: 5-stage Pipeline (no forwarding)",
            "PART C: 5-stage Pipeline (with forwarding)"
        };
        for (int m = 0; m < 3; ++m) {
            cout << "============================================================\n";
            cout << labels[m] << "\n";
            cout << "============================================================\n";
            runMode(m, path);
            cout << "\n";
        }
    } else {
        runMode(selected_mode, path);
    }

    return 0;
}