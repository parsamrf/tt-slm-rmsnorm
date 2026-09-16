/*
 * SPI -> SLB register bridge for single-block TinyTapeout tiles.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Front end: the 32-bit-frame SPI register interface (spi_reg.sv, Apache-2.0
 * upstream IP) parameterized with ADDR_W=16, so one SPI frame is:
 *   32-bit command  {RW, width[1:0], 13 don't-care, addr[15:0]}  MSB first
 *   32-bit data     (MOSI on writes, MISO on reads)
 * Back end: one SLB CSR port (s_req_valid/ready/write/addr/wdata/wstrb ->
 * s_rsp_valid/rdata), the register interface every accelerator block of the
 * SoC exposes. Only 32-bit accesses are generated.
 *
 * Reads: spi_reg raises reg_addr_v and waits in its TX-load state until
 * reg_data_i_dv; the bridge issues the SLB read, captures s_rsp_rdata, and
 * holds data+dv until the load state ends. Writes: reg_data_o_dv is a
 * one-shot; the bridge issues the SLB write and holds req_valid until
 * accepted.
 */
`default_nettype none

module slb_spi_bridge (
    input  wire        clk,
    input  wire        rst_n,

    // SPI pins (already synchronized by the caller if needed; spi_reg
    // itself samples with the system clock, so 2-stage sync is included
    // here for safety)
    input  wire        spi_cs_n,
    input  wire        spi_clk,
    input  wire        spi_mosi,
    output wire        spi_miso,

    // SLB CSR master port
    output reg         s_req_valid,
    input  wire        s_req_ready,
    output reg         s_req_write,
    output reg  [15:0] s_req_addr,
    output reg  [31:0] s_req_wdata,
    output wire [3:0]  s_req_wstrb,
    input  wire        s_rsp_valid,
    input  wire [31:0] s_rsp_rdata
);

  // 2-stage input synchronizers
  reg [1:0] cs_q, ck_q, mo_q;
  always @(posedge clk) begin
    cs_q <= {cs_q[0], spi_cs_n};
    ck_q <= {ck_q[0], spi_clk};
    mo_q <= {mo_q[0], spi_mosi};
  end

  wire [15:0] reg_addr;
  wire [31:0] reg_data_o;
  wire        reg_addr_v, reg_data_o_dv, reg_rw;
  wire [1:0]  txn_width;
  reg  [31:0] rd_capture;
  reg         rd_dv;

  spi_reg #(
      .ADDR_W(16),
      .REG_W (32)
  ) u_spi_reg (
      .clk          (clk),
      .rstb         (rst_n),
      .ena          (1'b1),
      .spi_mosi     (mo_q[1]),
      .spi_miso     (spi_miso),
      .spi_clk      (ck_q[1]),
      .spi_cs_n     (cs_q[1]),
      .reg_addr     (reg_addr),
      .reg_data_i   (rd_capture),
      .reg_data_o   (reg_data_o),
      .reg_addr_v   (reg_addr_v),
      .reg_data_i_dv(rd_dv),
      .reg_data_o_dv(reg_data_o_dv),
      .reg_rw       (reg_rw),
      .txn_width    (txn_width)
  );

  assign s_req_wstrb = s_req_write ? 4'b1111 : 4'b0000;

  localparam [1:0] B_IDLE = 2'd0, B_REQ = 2'd1, B_WAIT = 2'd2;
  reg [1:0] bstate;
  reg       rd_issued;

  always @(posedge clk) begin
    if (!rst_n) begin
      bstate      <= B_IDLE;
      s_req_valid <= 1'b0;
      s_req_write <= 1'b0;
      s_req_addr  <= 16'h0;
      s_req_wdata <= 32'h0;
      rd_capture  <= 32'h0;
      rd_dv       <= 1'b0;
      rd_issued   <= 1'b0;
    end else begin
      // read handshake back to spi_reg: hold dv while it sits in TX-load
      if (!reg_addr_v) begin
        rd_dv     <= 1'b0;
        rd_issued <= 1'b0;
      end

      case (bstate)
        B_IDLE: begin
          if (reg_data_o_dv && reg_rw) begin           // SPI write frame done
            s_req_valid <= 1'b1;
            s_req_write <= 1'b1;
            s_req_addr  <= reg_addr;
            s_req_wdata <= reg_data_o;
            bstate      <= B_REQ;
          end else if (reg_addr_v && !reg_rw && !rd_issued) begin  // read
            s_req_valid <= 1'b1;
            s_req_write <= 1'b0;
            s_req_addr  <= reg_addr;
            rd_issued   <= 1'b1;
            bstate      <= B_REQ;
          end
        end
        B_REQ: begin
          if (s_req_ready) begin
            s_req_valid <= 1'b0;
            bstate      <= B_WAIT;
          end
        end
        B_WAIT: begin
          if (s_rsp_valid) begin
            if (!s_req_write) begin
              rd_capture <= s_rsp_rdata;
              rd_dv      <= 1'b1;
            end
            bstate <= B_IDLE;
          end
        end
        default: bstate <= B_IDLE;
      endcase
    end
  end

  wire _unused = &{txn_width, 1'b0};

endmodule
