/*
 * slm_rmsnorm — RMSNorm / LayerNorm accelerator (SLB CSR slave, window
 * 0x3002_0000).
 *
 * RMSNorm mode:  ms = (sum x_i^2) / N + eps   (the /N is a right shift by
 *                log2(N), N in {8, 16} in this tile build; scratches depth-reduced), r = invsqrt(ms) in Q1.15,
 *                y_i = sat8( (x_i * r * g_i) >>> (21 - out_shift) )
 *                with gamma g_i signed INT8 interpreted Q1.6.
 * LayerNorm mode: pre-pass mu = (sum x_i) >>> log2(N) (arithmetic), then the
 *                same computation on x'_i = x_i - mu.
 *
 * The formula is bit-defined except for the invsqrt tolerance (this project's
 * slm_invsqrt is exactly rounded). Multiply-shift-saturate is done inline
 * (slm_requant instantiation optional per SPEC and not used here).
 *
 * Register map (window-relative):
 *   0x0000 CTRL   (W)  b0 start (ignored while busy), b1 clear_done
 *   0x0004 STATUS (R)  b0 busy, b1 done (sticky)
 *   0x0008 CFG    (RW) [6:0] N, b7 mode (0 = RMS / 1 = Layer), b8 irq_en,
 *                      [20:16] out_shift 0..15
 *   0x000C EPS    (RW) u32, reset 1
 *   0x0100-0x013C x scratch (16 words), signed INT8 per word [7:0]; result overwrites
 *                 byte 0 (upper bytes preserved as written)
 *   0x0200-0x023C gamma scratch (16 words), signed INT8 per word [7:0]
 * Undefined offsets read 0. irq = done & irq_en (level).
 * Caveat: out_shift > 21 is outside the contract (CFG documents 0..15) and
 * produces an implementation-defined shift amount.
 *
 * AI-assisted design.
 * Part of slm-soc. See docs/SPEC.md section 6.10.
 */
`default_nettype none

module slm_rmsnorm (
  input  wire        clk,          // clock
  input  wire        rst_n,        // synchronous active-low reset
  input  wire        s_req_valid,  // CSR request valid
  output wire        s_req_ready,  // CSR request accept (tied high)
  input  wire        s_req_write,  // 1 = write, 0 = read
  input  wire [15:0] s_req_addr,   // window-relative byte address
  input  wire [31:0] s_req_wdata,  // write data
  input  wire [3:0]  s_req_wstrb,  // byte strobes
  output wire        s_rsp_valid,  // 1-cycle response pulse
  output wire [31:0] s_rsp_rdata,  // read data
  output wire        irq           // level: done & irq_en
);

  localparam [2:0] S_IDLE = 3'd0;
  localparam [2:0] S_MEAN = 3'd1;
  localparam [2:0] S_SQ   = 3'd2;
  localparam [2:0] S_MS   = 3'd3;
  localparam [2:0] S_RS   = 3'd4;
  localparam [2:0] S_RW   = 3'd5;
  localparam [2:0] S_OUT  = 3'd6;

  reg [2:0]  state;
  reg        done_r;
  reg [6:0]  cfg_n;       // CFG[6:0]
  reg        cfg_mode;    // CFG[7]
  reg        cfg_irq_en;  // CFG[8]
  reg [4:0]  cfg_osh;     // CFG[20:16]
  reg [31:0] eps;         // EPS, reset 1

  reg [6:0]  n_r;         // N latched at start
  reg [4:0]  osh_r;       // out_shift latched at start
  reg signed [8:0]  mu;   // LayerNorm mean (0 in RMS mode)
  reg signed [15:0] sumx; // running sum of x for the mean pre-pass
  reg [31:0] acc;         // sum of squares
  reg [31:0] ms;          // mean square + eps
  reg [15:0] r_q;         // invsqrt result, Q1.15
  reg [6:0]  idx;         // element index

  reg [31:0] xmem [0:15]; // x / result scratch (tile: 16 words, N <= 16)
  reg [31:0] gmem [0:15]; // gamma scratch (tile: 16 words)

  integer k;
  initial begin
    for (k = 0; k < 16; k = k + 1) begin
      xmem[k] = 32'd0;
      gmem[k] = 32'd0;
    end
  end

  wire busy = (state != S_IDLE);

  // log2(N) for N in {8, 16, 32, 64}
  wire [2:0] lg = n_r[6] ? 3'd6 :
                  n_r[5] ? 3'd5 :
                  n_r[4] ? 3'd4 : 3'd3;

  // ------------------------------------------------------------------ CSR
  assign s_req_ready = 1'b1;

  wire        csr_wr = s_req_valid & s_req_write;
  wire        csr_rd = s_req_valid & ~s_req_write;
  wire        is_x   = (s_req_addr[15:8] == 8'h01);
  wire        is_g   = (s_req_addr[15:8] == 8'h02);
  wire        is_lo  = (s_req_addr[15:8] == 8'h00);
  wire [3:0]  scr_ix = s_req_addr[5:2];

  wire start_w = csr_wr & is_lo & (s_req_addr[7:2] == 6'd0) &
                 s_req_wstrb[0] & s_req_wdata[0];
  wire clrd_w  = csr_wr & is_lo & (s_req_addr[7:2] == 6'd0) &
                 s_req_wstrb[0] & s_req_wdata[1];

  reg [31:0] rd_mux;
  always @(*) begin
    rd_mux = 32'd0;
    if (is_x)
      rd_mux = xmem[scr_ix];
    else if (is_g)
      rd_mux = gmem[scr_ix];
    else if (is_lo) begin
      case (s_req_addr[7:2])
        6'd1:    rd_mux = {30'd0, done_r, busy};
        6'd2:    rd_mux = {11'd0, cfg_osh, 7'd0, cfg_irq_en, cfg_mode, cfg_n};
        6'd3:    rd_mux = eps;
        default: rd_mux = 32'd0;
      endcase
    end
  end

  reg        rsp_v;
  reg [31:0] rsp_d;
  always @(posedge clk) begin
    if (!rst_n) begin
      rsp_v <= 1'b0;
      rsp_d <= 32'd0;
    end else begin
      rsp_v <= s_req_valid;
      if (csr_rd)
        rsp_d <= rd_mux;
    end
  end

  assign s_rsp_valid = rsp_v;
  assign s_rsp_rdata = rsp_d;

  // -------------------------------------------------------------- datapath
  wire signed [7:0] x_cur = xmem[idx[3:0]][7:0];
  wire signed [9:0] xd    = {{2{x_cur[7]}}, x_cur} - {mu[8], mu};

  // mean pre-pass
  wire signed [15:0] sumx_n = sumx + {{8{x_cur[7]}}, x_cur};
  wire signed [15:0] mu_sh  = sumx_n >>> lg;

  // sum of squares
  wire signed [19:0] sq = xd * xd;

  // invsqrt
  wire        isq_go = (state == S_RS);
  wire        isq_busy;
  wire        isq_done;
  wire [15:0] isq_r;

  slm_invsqrt u_isq (
    .clk   (clk),
    .rst_n (rst_n),
    .start (isq_go),
    .in_v  (ms),
    .busy  (isq_busy),
    .done  (isq_done),
    .out_r (isq_r)
  );

  // output stage: y = sat8( (xd * r * g) >>> (21 - out_shift) )
  wire signed [7:0]  g_cur = gmem[idx[3:0]][7:0];
  wire signed [16:0] r_s   = {1'b0, r_q};
  wire signed [26:0] p1    = xd * r_s;
  wire signed [34:0] p2    = p1 * g_cur;
  wire [4:0]         shamt = 5'd21 - osh_r;
  wire signed [34:0] sh    = p2 >>> shamt;
  wire signed [7:0]  y_sat = (sh > 35'sd127)  ? 8'sd127 :
                             (sh < -35'sd128) ? -8'sd128 :
                                                sh[7:0];

  // ------------------------------------------------------------------ FSM
  always @(posedge clk) begin
    if (!rst_n) begin
      state      <= S_IDLE;
      done_r     <= 1'b0;
      cfg_n      <= 7'd8;
      cfg_mode   <= 1'b0;
      cfg_irq_en <= 1'b0;
      cfg_osh    <= 5'd0;
      eps        <= 32'd1;
      n_r        <= 7'd8;
      osh_r      <= 5'd0;
      mu         <= 9'sd0;
      sumx       <= 16'sd0;
      acc        <= 32'd0;
      ms         <= 32'd1;
      r_q        <= 16'd0;
      idx        <= 7'd0;
    end else begin
      // CSR writes
      if (csr_wr) begin
        if (is_x) begin
          if (s_req_wstrb[0]) xmem[scr_ix][7:0]   <= s_req_wdata[7:0];
          if (s_req_wstrb[1]) xmem[scr_ix][15:8]  <= s_req_wdata[15:8];
          if (s_req_wstrb[2]) xmem[scr_ix][23:16] <= s_req_wdata[23:16];
          if (s_req_wstrb[3]) xmem[scr_ix][31:24] <= s_req_wdata[31:24];
        end else if (is_g) begin
          if (s_req_wstrb[0]) gmem[scr_ix][7:0]   <= s_req_wdata[7:0];
          if (s_req_wstrb[1]) gmem[scr_ix][15:8]  <= s_req_wdata[15:8];
          if (s_req_wstrb[2]) gmem[scr_ix][23:16] <= s_req_wdata[23:16];
          if (s_req_wstrb[3]) gmem[scr_ix][31:24] <= s_req_wdata[31:24];
        end else if (is_lo && (s_req_addr[7:2] == 6'd2)) begin
          if (s_req_wstrb[0]) begin
            cfg_n    <= s_req_wdata[6:0];
            cfg_mode <= s_req_wdata[7];
          end
          if (s_req_wstrb[1]) cfg_irq_en <= s_req_wdata[8];
          if (s_req_wstrb[2]) cfg_osh    <= s_req_wdata[20:16];
        end else if (is_lo && (s_req_addr[7:2] == 6'd3)) begin
          if (s_req_wstrb[0]) eps[7:0]   <= s_req_wdata[7:0];
          if (s_req_wstrb[1]) eps[15:8]  <= s_req_wdata[15:8];
          if (s_req_wstrb[2]) eps[23:16] <= s_req_wdata[23:16];
          if (s_req_wstrb[3]) eps[31:24] <= s_req_wdata[31:24];
        end
        if (clrd_w)
          done_r <= 1'b0;
      end

      case (state)
        S_IDLE: begin
          if (start_w) begin
            state  <= cfg_mode ? S_MEAN : S_SQ;
            n_r    <= cfg_n;
            osh_r  <= cfg_osh;
            mu     <= 9'sd0;
            sumx   <= 16'sd0;
            acc    <= 32'd0;
            idx    <= 7'd0;
            done_r <= 1'b0;
          end
        end

        S_MEAN: begin
          sumx <= sumx_n;
          if (idx == n_r - 7'd1) begin
            mu    <= mu_sh[8:0];
            idx   <= 7'd0;
            state <= S_SQ;
          end else begin
            idx <= idx + 7'd1;
          end
        end

        S_SQ: begin
          acc <= acc + {12'd0, sq};
          if (idx == n_r - 7'd1) begin
            idx   <= 7'd0;
            state <= S_MS;
          end else begin
            idx <= idx + 7'd1;
          end
        end

        S_MS: begin
          ms    <= (acc >> lg) + eps;
          state <= S_RS;
        end

        S_RS: begin
          state <= S_RW;
        end

        S_RW: begin
          if (isq_done) begin
            r_q   <= isq_r;
            idx   <= 7'd0;
            state <= S_OUT;
          end
        end

        S_OUT: begin
          xmem[idx[3:0]][7:0] <= y_sat;
          if (idx == n_r - 7'd1) begin
            state  <= S_IDLE;
            done_r <= 1'b1;
          end else begin
            idx <= idx + 7'd1;
          end
        end

        default: begin
          state <= S_IDLE;
        end
      endcase
    end
  end

  assign irq = done_r & cfg_irq_en;

  wire _unused = &{1'b0, s_req_addr[1:0], isq_busy, mu_sh[15:9]};

endmodule

`default_nettype wire
