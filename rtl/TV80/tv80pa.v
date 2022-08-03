`timescale 1ps / 1ps
//
// TV80 8-Bit Microprocessor Core
// Based on the VHDL T80 core by Daniel Wallner (jesus@opencores.org)
//
// Copyright (c) 2004 Guy Hutchison (ghutchis@opencores.org)
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
// CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
// SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

module tv80pa (/*AUTOARG*/
  // Outputs
  M1_n, MREQ_n, IORQ_n, RD_n, WR_n, RFSH_n, HALT_n, BUSAK_n, A, DO,
  // Inputs
  RESET_n, CLK, CEN_p, CEN_n, WAIT_n, INT_n, NMI_n, BUSRQ_n, DI
  );

  parameter Mode = 0;    // 0 => Z80, 1 => Fast Z80, 2 => 8080, 3 => GB
  parameter T2Write = 1; // 0 => WR_n active in T3, /=0 => WR_n active in T2
  parameter IOWait  = 1; // 0 => Single cycle I/O, 1 => Std I/O cycle


  input         RESET_n;
  input         CLK;
  input         CEN_p;
  input         CEN_n;
  input         WAIT_n;
  input         INT_n;
  input         NMI_n;
  input         BUSRQ_n;
  output        M1_n;
  output        MREQ_n;
  output        IORQ_n;
  output        RD_n;
  output        WR_n;
  output        RFSH_n;
  output        HALT_n;
  output        BUSAK_n;
  output reg [15:0] A;
  input [7:0]   DI;
  output [7:0]  DO;

  reg           MREQ_n;
  reg           IORQ_n;
  reg           RD_n;
  reg           WR_n;

  reg           CEN_pol;
  wire          intcycle_n;
  wire          no_read;
  wire          write;
  wire          iorq;
  reg [7:0]     di_reg;
  wire [6:0]    mcycle;
  wire [6:0]    tstate;

  reg [1:0]     intcycled_n;
  wire [15:0]   A_int;
  reg [15:0]    A_last;

  tv80_core #(Mode, IOWait) i_tv80_core
    (
     .cen (CEN_p && ~CEN_pol),
     .m1_n (M1_n),
     .iorq (iorq),
     .no_read (no_read),
     .write (write),
     .rfsh_n (RFSH_n),
     .halt_n (HALT_n),
     .wait_n (WAIT_n),
     .int_n (INT_n),
     .nmi_n (NMI_n),
     .reset_n (RESET_n),
     .busrq_n (BUSRQ_n),
     .busak_n (BUSAK_n),
     .clk (CLK),
     .IntE (),
     .stop (),
     .A (A_int),
     .dinst (DI),
     .di (di_reg),
     .dout (DO),
     .mc (mcycle),
     .ts (tstate),
     .intcycle_n (intcycle_n)
     );

always @(posedge CLK) begin

  if (no_read==1'b0 || write==1'b1) begin
        A <= A_int;
  end
  else begin
    A <= A_last;
  end

  if (~RESET_n) begin
    WR_n    <= 1'b1;
    RD_n    <= 1'b1;
    IORQ_n  <= 1'b1;
    MREQ_n  <= 1'b1;
    di_reg  <= 0;
    CEN_pol <= 1'b0;
    //intcycled_n <= 2'b11; /* RndMnkIII */
  end
  else if (CEN_p==1'b1 && CEN_pol==1'b0) begin
    CEN_pol <= 1'b1;
    if (mcycle=='b001) begin
      if (tstate==3'b010) begin
        IORQ_n <= 1'b1;
        MREQ_n <= 1'b1;
        RD_n   <= 1'b1;
      end
    end
    else begin
      if (tstate==3'b001 && iorq==1'b1) begin
        WR_n   <= ~write;
        RD_n   <= write;
        IORQ_n <= 1'b0;
      end
    end
  end
  else if (CEN_n==1'b1 && CEN_pol==1'b1) begin
    if (tstate==3'b010)
      CEN_pol <= ~WAIT_n;
    else
      CEN_pol <= 1'b0;
    if (tstate==3'b011 && BUSAK_n==1'b1) di_reg <= DI;
    if (mcycle==3'b001) begin
      if (tstate==3'b001) begin
        intcycled_n <= {intcycled_n[0],intcycle_n};
        RD_n   <= ~intcycle_n;
        MREQ_n <= ~intcycle_n;
        IORQ_n <= intcycled_n[1];
        A_last <= A_int;
      end
      if (tstate==3'b011) begin
        intcycled_n <= 2'b11;
        RD_n   <= 1'b1;
        MREQ_n <= 1'b0;
      end
      if (tstate==3'b100) MREQ_n <= 1'b1;
    end
    else begin
      if (no_read==1'b0 && iorq==1'b0) begin
        if (tstate==3'b001) begin
          RD_n   <= write;
          MREQ_n <= 1'b0;
          A_last <= A_int;
        end
      end
      if (tstate==3'b010) WR_n <= ~write;
      if (tstate==3'b011) begin
        WR_n   <= 1'b1;
        RD_n   <= 1'b1;
        IORQ_n <= 1'b1;
        MREQ_n <= 1'b1;
      end
    end
  end // if (CEN_n==1'b1 && CEN_pol==1'b1)
end // always @ (posedge CLK)

endmodule
