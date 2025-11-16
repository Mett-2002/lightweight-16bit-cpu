# Simple 16-bit CPU in Verilog

This project implements a simple **16-bit CPU** in Verilog with a **microinstruction-based control unit**, supporting arithmetic operations, memory references, register references, and subroutine calls. The design includes a clear separation between the **Control Unit** and the **Datapath**, and is suitable for learning CPU architecture and hardware description in Verilog.

---

## Table of Contents

* [Overview](#overview)
* [Modules](#modules)
* [Instruction Set](#instruction-set)
* [Memory Map](#memory-map)
* [Datapath & Control Flow](#datapath--control-flow)
* [Simulation & Testbench](#simulation--testbench)
* [Author & Notes](#author--notes)

---

## Overview

This CPU is built around:

* **Registers:** PC, AR, DR, AC, IR, TR
* **ALU:** Supports AND, ADD, LOAD, Complement AC (COM), Shift Right (SHR), Shift Left (SHL)
* **Extend bit (E):** Used in shifts and carry operations
* **DataBus:** Multiplexes data between registers and memory
* **Memory Unit:** 4096 x 16-bit memory with preloaded subroutines
* **ControlUnit:** Finite state machine (FSM) with four states: `FETCH`, `DECODE`, `INDIRECT`, `EXECUTE`

All registers (except IR) can **load, increment, or clear**. IR can only load instructions from the bus.

The **Control Unit** uses internal **timing signals (T[0..6])** to simulate microinstruction sequencing, enabling step-by-step execution of instructions.

The CPU is designed to execute programs including subroutine calls for arithmetic operations and supports halting via the HLT instruction.

---

## Modules

### 1. DataBus

The **DataBus** selects data from registers or memory for the CPU bus.

| Selector (`s`) | Source |
| -------------- | ------ |
| 001            | AR     |
| 010            | PC     |
| 011            | DR     |
| 100            | AC     |
| 101            | IR     |
| 110            | TR     |
| 111            | Memory |

---

### 2. Registers

| Register | Inputs | Control | Behavior                               |
| -------- | ------ | ------- | -------------------------------------- |
| PC       | Bus    | 3-bit   | Load from bus, increment, clear        |
| AR       | Bus    | 3-bit   | Load from bus, increment, clear        |
| DR       | Bus    | 3-bit   | Load from bus, increment, clear        |
| AC       | ALU    | 3-bit   | Load from ALU output, increment, clear |
| TR       | Bus    | 3-bit   | Load from bus, increment, clear        |
| IR       | Bus    | 1-bit   | Load from bus only                     |

---

### 3. ALU

The ALU performs arithmetic and logical operations. Control signals:

| alu_s | Operation  | Description              |
| ----- | ---------- | ------------------------ |
| 000   | NOP        | No operation             |
| 001   | AND AC, DR | AC bitwise AND with DR   |
| 010   | ADD AC, DR | AC + DR, E holds carry   |
| 011   | LOAD_DR    | Load DR into AC          |
| 100   | COM        | Complement AC            |
| 101   | SHR        | Shift AC right through E |
| 110   | SHL        | Shift AC left through E  |
| 111   | NOP        | No operation             |

**E control (`E_ctrl`)**

| E_ctrl | Effect       |
| ------ | ------------ |
| 01     | Clear E      |
| 10     | Complement E |
| 00     | No change    |

The ALU handles the extend bit (`E`) during shifts and arithmetic carry operations.

---

### 4. Control Unit (FSM)

The **ControlUnit** uses a microinstruction-based FSM with four states:

1. **FETCH:**

   * AR ← PC
   * IR ← Memory[PC]
   * PC ← PC + 1

2. **DECODE:**

   * Decode opcode (`IR[14:12]`)
   * Set AR to operand address
   * Check indirect addressing (`IR[15]`)

3. **INDIRECT:**

   * Handle indirect memory references
   * Set `is_register_ref` for register instructions

4. **EXECUTE:**

   * Execute memory-reference instructions (AND, ADD, LDA, STA, BUN, BSA, ISZ)
   * Execute register-reference instructions (CLA, CLE, CMA, CME, CIR, CIL, INC, SPA, SNA, SZA, SZE, HLT)

**Timing signals (T[0..6])** control the microoperations step by step.

The FSM uses registers `T`, `D`, and `sc` to manage microinstruction timing, opcode decoding, and instruction sequencing.

---

### 5. Memory Unit

* 4096 x 16-bit memory
* **Control signals:**

  * `01` → Read
  * `10` → Write
* Special addresses for arithmetic operations:

| Address    | Purpose    |
| ---------- | ---------- |
| F00 (3840) | input1     |
| F01 (3841) | input2     |
| F02 (3842) | add result |
| F03 (3843) | sub result |
| F04 (3844) | mul result |

* Preloaded subroutines (Addition, Subtraction, Multiplication) and main program (BSA calls, HALT) are included.
* Temporary memory locations F05, F06, etc., are used internally for calculations.

---

### 6. Datapath

Connects all registers, ALU, bus, and memory. Handles signal propagation between CPU components and ALU input/output. It ensures proper sequencing of data flow according to control signals from the Control Unit.

---

### 7. CPU Top Module

* Integrates `ControlUnit` and `Datapath`
* Inputs: `clk`, `input1`, `input2`
* Outputs: `add`, `sub`, `mlt` results
* Monitors ALU results and memory outputs as they update in real time.

---

## Instruction Set

### Memory Reference Instructions (D[0..6])

* D[0] → AND AC, M
* D[1] → ADD M to AC
* D[2] → LDA M
* D[3] → STA M
* D[4] → BUN (Branch Unconditionally)
* D[5] → BSA (Branch and Save return address)
* D[6] → ISZ (Increment memory and skip if zero)

### Register Reference Instructions (IR[11:0])

* CLA → Clear AC
* CLE → Clear E
* CMA → Complement AC
* CME → Complement E
* CIR → Circulate right AC through E
* CIL → Circulate left AC through E
* INC → Increment AC
* SPA → Skip on positive AC
* SNA → Skip on negative AC
* SZA → Skip if AC zero
* SZE → Skip if E zero
* HLT → Halt execution

---

## Simulation & Testbench

* Synchronous design (positive-edge clock)
* Testbench inputs `input1` and `input2`
* Monitors `add`, `sub`, `mlt` results:

```verilog
input1 = 50;
input2 = 30;
$monitor("input1: %d, input2: %d | add: %d, sub: %d, mlt: %d", input1, input2, add, sub, mlt);
```

* Clock generated using: `always #20 clk = ~clk;`
* Subroutine execution and HALT are correctly simulated.
* Memory locations are updated in real time with inputs, and results are stored at designated addresses.

---

## Datapath & Control Flow

1. **Instruction Fetch:** PC → AR → Memory → IR → PC incremented
2. **Instruction Decode:** Opcode decoding → Identify instruction type → Set AR for memory reference or register reference
3. **Indirect Access:** Fetch effective address if indirect bit is set
4. **Execution:** Perform operation (ALU operation, memory read/write, branch, subroutine call)
5. **Result Storage:** Update AC, memory, or PC as needed
6. **Repeat:** FSM returns to FETCH for next instruction until HLT is encountered

---

## Author & Notes

* Designed for **learning CPU architecture** and **Verilog hardware description**
* ControlUnit FSM simulates real microinstruction timing
* Datapath and ALU provide arithmetic/logical operations with extend bit handling
* Memory preloaded with example program and arithmetic subroutines
* Registers, bus, and ALU interactions follow step-by-step microoperations
* Testbench demonstrates CPU operation with observable outputs

**Author:** Your Name
**Date:** 2025

---

This README provides a complete explanation of the CPU design, modules, memory mapping, ALU functionality, FSM behavior, and simulation details, matching exactly the provided Verilog implementation.
