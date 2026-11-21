# AXI / Valid-Ready Handshake Timing: Forward Register Slice vs. Skid Buffer

## #1. Forward vs. Backpressure Control Signals

* **Forward Scenario (`m_valid` / `out_valid`) — Sequential in BOTH:**
  * **Forward Register Slice:** **Sequential** $\rightarrow$ `assign m_valid = valid_reg;` (FF-to-FF)
  * **Skid Buffer:** **Sequential** $\rightarrow$ `assign m_valid = main_valid;` (FF-to-FF)

* **Backpressure Scenario (`s_ready` / `in_ready`) — Only Skid Buffer is Sequential:**
  * **Forward Register Slice (Pipeline):** **Combinational** $\rightarrow$ `assign s_ready = m_ready | (~valid_reg);`
  * **Skid Buffer:** **Sequential** $\rightarrow$ `assign s_ready = ~skid_valid;`

    
## 1. Overview & Problem Statement

In high-frequency `valid`/`ready`/`data` interfaces (such as AXI-Stream or AXI channels), inserting a **Forward Register Slice** is a common technique to pipeline the forward path (`valid` and `data`). However, a forward register slice leaves the **backward path (`ready`) purely combinational**—meaning there is no register-to-register boundary in the backward direction.

```
[Upstream Master] ---> (s_valid / s_data) ---> [Flop] ---> (m_valid / m_data) ---> [Downstream Slave]
                  <--- (s_ready) <----------- [ OR/AND ] <--- (m_ready) <----------
                                              ^ No flip-flop boundary!
```

* **Combinational `ready` (`m_ready | ~valid_reg`)** $\rightarrow$ No register-to-register boundary in the backward direction $\rightarrow$ Increased path delay as downstream stall logic leaks upstream $\rightarrow$ Critical timing errors (**setup violations**) at high clock frequencies.

---

## 2. Why `assign` with Any Logic Gate (`|`, `&`, `~`) Is Still Combinational

A continuous `assign` statement simply drives a `wire`—it does not make a signal sequential. What determines whether `s_ready` is **combinational** or **sequential (registered)** is what sits on the right-hand side (`RHS`) of that `assign`:

### Case A: Combinational `s_ready` (Forward Register Slice)
```verilog
assign s_ready = m_ready | (~valid_reg);
```
Even though `valid_reg` is a flip-flop output, `s_ready` is combinationally gated with the `m_ready` input port. Any transition on `m_ready` propagates immediately through the `|` gate onto `s_ready` within the **same clock cycle** without waiting for `posedge clk`.

* **What if we change `|` to `&` (`assign s_ready = m_ready & (~valid_reg);`)?**
  * **Still Combinational:** Because `m_ready` is still on the RHS of the `assign`, changes on `m_ready` still ripple through the `&` gate to `s_ready` in the same cycle—so it **does not** fix the combinational backpressure path.
  * **Lower Throughput (Bubble Cycles):** Using `&` forces `s_ready = 0` whenever `valid_reg == 1`, preventing the slice from accepting a new beat on the same clock edge that `data_reg` drains (`m_ready == 1`), cutting sustained throughput in half.

### Case B: Sequential / Registered `s_ready` (Skid Buffer)
```verilog
assign s_ready = ~skid_valid;
```
Here `skid_valid` is the direct **Q output of a flip-flop** updated inside `always @(posedge clk)`. The `assign` merely routes that flip-flop's output to `s_ready`—there is **no `m_ready` logic** in the path, and `s_ready` only changes **after `posedge clk`**.

---

## 3. Forward Register Slice RTL (The Bottleneck)

Below is the standard forward register slice (`axi_handshake_reg`), where the forward path is registered (`data_reg`, `valid_reg`) while `s_ready` remains combinational:

```verilog
module axi_handshake_reg (
    input  wire        clk,        
    input  wire        rst_n,               
                
    // Left Side: Connects to External Master (Module acts as Slave)      
    input  wire [31:0] s_data,        
    input  wire        s_valid,               
    output wire        s_ready,                                         
     
    // Right Side: Connects to External Slave (Module acts as Master)  
    output wire [31:0] m_data,   
    output wire        m_valid,             
    input  wire        m_ready          
);
    // Internal Registers      
    reg [31:0] data_reg;
    reg        valid_reg;        

    // Combinational Ready Logic: m_ready propagates directly to s_ready
    assign s_ready = m_ready | (~valid_reg);

    // Registered Forward Outputs
    assign m_data  = data_reg;
    assign m_valid = valid_reg;

    // Sequential Logic Block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_reg <= 1'b0;    
            data_reg  <= 32'h0;
        end else begin
            if (s_ready) begin
                valid_reg <= s_valid;
                if (s_valid) begin
                    data_reg <= s_data;
                end
            end
        end
    end
endmodule
```

---

## 4. How the Skid Buffer Solves the Issue

A **Skid Buffer** breaks the combinational backpressure path by turning `s_ready` into a **register-to-register** signal (`~skid_valid`) while adding a secondary **skid register (`skid_reg`)** to preserve full **1-beat-per-cycle** throughput:

1. **Breaks Backward Timing Paths:** Because `s_ready` is driven solely by `~skid_valid` (a flip-flop), downstream stall logic on `m_ready` terminates at the skid buffer's registers and never leaks upstream.
2. **Handles the 1-Cycle Stall Latency:** When downstream stalls (`m_ready == 0`), `s_ready` remains `1` for one clock cycle until `skid_valid` updates on `posedge clk`. During that cycle, if the upstream master sends an in-flight beat, it is safely parked in `skid_reg` (`skid_valid <= 1'b1`) without overwriting `main_reg`.
3. **Zero Bubble Cycles:** When downstream resumes (`m_ready == 1`), `skid_reg` drains into `main_reg` first, then normal 1-beat/cycle streaming continues seamlessly.

### Skid Buffer RTL (Registered `s_ready` + `skid_reg`)

```verilog
module axi_skid_buffer (
    input  wire        clk,
    input  wire        rst_n,

    // Left Side: Connects to External Master (Module acts as Slave)
    input  wire [31:0] s_data,
    input  wire        s_valid,
    output wire        s_ready,

    // Right Side: Connects to External Slave (Module acts as Master)
    output wire [31:0] m_data,
    output wire        m_valid,
    input  wire        m_ready
);

    reg [31:0] main_reg, skid_reg;
    reg        main_valid, skid_valid;

    // Registered Ready: breaks combinational backpressure path
    assign s_ready = ~skid_valid;
    assign m_valid = main_valid;
    assign m_data  = main_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            main_reg   <= 32'h0;
            skid_reg   <= 32'h0;
            main_valid <= 1'b0;
            skid_valid <= 1'b0;
        end else begin
            // 1. Downstream consumes current output beat
            if (m_valid && m_ready) begin
                if (skid_valid) begin
                    // Drain skidded beat into main_reg first
                    main_reg   <= skid_reg;
                    main_valid <= 1'b1;
                    skid_valid <= 1'b0;
                end else if (s_valid && s_ready) begin
                    // Skid empty & current beat consumed: load new beat directly
                    main_reg   <= s_data;
                    main_valid <= 1'b1;
                end else begin
                    main_valid <= 1'b0;
                end
            end
            // 2. Downstream stalls (!m_ready or !m_valid)
            else if (s_valid && s_ready) begin
                if (!main_valid) begin
                    // main_reg is empty: load directly
                    main_reg   <= s_data;
                    main_valid <= 1'b1;
                end else if (!skid_valid) begin
                    // main_reg is occupied: park in-flight beat in skid_reg
                    skid_reg   <= s_data;
                    skid_valid <= 1'b1;
                end
            end
        end
    end

endmodule
```

---

## 5. Summary Comparison

| Metric | Forward Register Slice (`axi_handshake_reg`) | Skid Buffer (`axi_skid_buffer`) |
| :--- | :--- | :--- |
| **Forward Path (`valid` / `data`)** | Registered (`valid_reg`, `data_reg`) | Registered (`main_valid`, `main_reg`) |
| **Backward Path (`s_ready`)** | **Combinational** (`m_ready \| ~valid_reg`) | **Sequential / Registered** (`~skid_valid`) |
| **Backward Register-to-Register?** | No (leaks `m_ready` delay upstream) | **Yes** (`skid_valid` flop $\rightarrow$ `s_ready`) |
| **Storage Elements** | 1 data + 1 valid flop | 2 data + 2 valid flops (`main` + `skid`) |
| **Peak Throughput** | 1 beat / cycle | 1 beat / cycle (0 bubble cycles) |
| **Best Use Case** | Breaking forward `data`/`valid` paths only | Full timing closure on both forward and backpressure paths |
