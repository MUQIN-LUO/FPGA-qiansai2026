// ==========================================================================
// PH1P 原语行为模型（仅 ModelSim 仿真用；综合使用 TD 的原语库）
// --------------------------------------------------------------------------
// TD 的加密 PH1P_LOGIC_ERAM 仿真模型在 ModelSim 10.3c 下无法绑定
// PH1P_PHY_GSR（vsim-3043）。这里按 ram_*.v 包装器实际使用的端口/参数
// 提供等价的行为模型，只用于功能仿真：
//   · PH1P_LOGIC_ERAM：双端口 RAM，NOREG 读为组合读，REG 读打一拍；
//     bea/beb/ECC 忽略（设计固定接 0/DISABLE）。
//   · PH1P_LOGIC_BUFG：时钟缓冲，直通。
//   · PH1P_PHY_GSR：全局复位占位，空模块。
// 本文件不参与综合。
// ==========================================================================
`timescale 1ps/1ps

module PH1P_LOGIC_ERAM #(
    parameter integer DATA_WIDTH_A = 20,
    parameter integer DATA_WIDTH_B = 20,
    parameter integer ADDR_WIDTH_A = 10,
    parameter integer ADDR_WIDTH_B = 10,
    parameter integer DATA_DEPTH_A = 1024,
    parameter integer DATA_DEPTH_B = 1024,
    parameter MODE   = "DP",
    parameter REGMODE_A = "NOREG",
    parameter REGMODE_B = "NOREG",
    parameter WRITEMODE_A = "NORMAL",
    parameter WRITEMODE_B = "NORMAL",
    parameter IMPLEMENT = "20K(FAST)",
    parameter ECC_ENCODE = "DISABLE",
    parameter ECC_DECODE = "DISABLE",
    parameter CLKMODE = "ASYNC",
    parameter SSROVERCE = "DISABLE",
    parameter OREGSET_A = "RESET",
    parameter OREGSET_B = "RESET",
    parameter RESETMODE_A = "ASYNC",
    parameter RESETMODE_B = "ASYNC",
    parameter ASYNC_RESET_RELEASE_A = "ASYNC",
    parameter ASYNC_RESET_RELEASE_B = "ASYNC",
    parameter INIT_FILE = "",
    parameter FILL_ALL = "NONE"
) (
    input  wire [DATA_WIDTH_A-1:0] dia,
    input  wire [DATA_WIDTH_B-1:0] dib,
    input  wire [ADDR_WIDTH_A-1:0] addra,
    input  wire [ADDR_WIDTH_B-1:0] addrb,
    input  wire cea, ceb, ocea, oceb,
    input  wire clka, clkb,
    input  wire wea, web,
    input  wire bea, beb,
    input  wire rsta, rstb,
    output reg  [DATA_WIDTH_A-1:0] doa,
    output reg  [DATA_WIDTH_B-1:0] dob,
    output wire ecc_sbiterr,
    output wire ecc_dbiterr,
    input  wire ecc_sbiterrinj,
    input  wire ecc_dbiterrinj
);
    assign ecc_sbiterr = 1'b0;
    assign ecc_dbiterr = 1'b0;
    reg [DATA_WIDTH_A-1:0] mem [0:DATA_DEPTH_A-1];
    integer i;

    initial begin
        for (i = 0; i < DATA_DEPTH_A; i = i + 1)
            mem[i] = {DATA_WIDTH_A{1'b0}};
    end

    // 写端口（bea/beb 忽略：设计固定接 0）
    always @(posedge clka) if (cea && wea) mem[addra] <= dia;
    always @(posedge clkb) if (ceb && web) mem[addrb] <= dib;

    generate
        if (REGMODE_A == "REG") begin : g_ra
            always @(posedge clka) begin
                if (!cea)       doa <= {DATA_WIDTH_A{1'b0}};
                else if (!wea)  doa <= mem[addra];
            end
        end
        else begin : g_ra_c
            always @(*) doa = mem[addra];
        end

        if (REGMODE_B == "REG") begin : g_rb
            always @(posedge clkb) begin
                if (!ceb)       dob <= {DATA_WIDTH_B{1'b0}};
                else if (!web)  dob <= mem[addrb];
            end
        end
        else begin : g_rb_c
            always @(*) dob = mem[addrb];
        end
    endgenerate

endmodule

module PH1P_LOGIC_BUFG (
    input  wire i,
    output wire o
);
    assign o = i;
endmodule

module PH1P_PHY_GSR ();
    // 全局复位占位（行为模型不依赖它）
endmodule
