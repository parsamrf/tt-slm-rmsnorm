/*
 * slm_invsqrt — fixed-point reciprocal square root, Q1.15.
 *
 * out_r = round(32768 / sqrt(v)) for v in [1, 2^20] (spec accuracy +-4 LSB;
 * this implementation is exactly rounded, ties away from the floor).
 * Method: bit-serial search for the largest r with r^2 * v <= 2^30 (i.e.
 * r = floor(sqrt(2^30 / v))), then one rounding step: r += 1 when
 * (2r + 1)^2 * v <= 2^32, which is exactly the (r + 0.5)^2 * v <= 2^30
 * midpoint test. No LUT/Newton seed needed; the trial multiplies are
 * combinational per iteration. Latency ~19 cycles (1 start + 16 search +
 * 1 round + done). start is ignored while busy; in_v is captured at start;
 * out_r holds after done. v = 1 yields 32768 (fits in 16 bits unsigned).
 *
 * AI-assisted design.
 * Part of slm-soc. See docs/SPEC.md section 6.10.
 */
`default_nettype none

module slm_invsqrt (
  input  wire        clk,    // clock
  input  wire        rst_n,  // synchronous active-low reset
  input  wire        start,  // 1-cycle pulse; ignored while busy
  input  wire [31:0] in_v,   // v >= 1
  output wire        busy,   // high while computing
  output wire        done,   // 1-cycle pulse with out_r valid
  output wire [15:0] out_r   // Q1.15: round(32768 / sqrt(v))
);

  localparam [1:0] S_IDLE  = 2'd0;
  localparam [1:0] S_CALC  = 2'd1;
  localparam [1:0] S_ROUND = 2'd2;

  reg [1:0]  state;
  reg        done_r;
  reg [31:0] v_r;
  reg [15:0] r_r;
  reg [3:0]  bit_i;

  // search trial: t = r | (1 << bit_i); accept when t^2 * v <= 2^30
  wire [15:0] t    = r_r | (16'd1 << bit_i);
  wire [31:0] tt   = t * t;
  wire [63:0] p    = tt * v_r;
  wire        p_le = (p <= 64'h0000_0000_4000_0000);

  // rounding trial: (2r + 1)^2 * v <= 2^32
  wire [16:0] t2    = {r_r, 1'b1};
  wire [33:0] t2q   = t2 * t2;
  wire [65:0] p2    = t2q * v_r;
  wire        p2_le = (p2 <= 66'h0_0000_0001_0000_0000);

  always @(posedge clk) begin
    if (!rst_n) begin
      state  <= S_IDLE;
      done_r <= 1'b0;
      v_r    <= 32'd1;
      r_r    <= 16'd0;
      bit_i  <= 4'd0;
    end else begin
      done_r <= 1'b0;
      case (state)
        S_IDLE: begin
          if (start) begin
            v_r   <= in_v;
            r_r   <= 16'd0;
            bit_i <= 4'd15;
            state <= S_CALC;
          end
        end

        S_CALC: begin
          if (p_le)
            r_r <= t;
          if (bit_i == 4'd0)
            state <= S_ROUND;
          bit_i <= bit_i - 4'd1;
        end

        S_ROUND: begin
          if (p2_le)
            r_r <= r_r + 16'd1;
          done_r <= 1'b1;
          state  <= S_IDLE;
        end

        default: begin
          state <= S_IDLE;
        end
      endcase
    end
  end

  assign busy  = (state != S_IDLE);
  assign done  = done_r;
  assign out_r = r_r;

endmodule

`default_nettype wire
