// ==========================================================================
// 掩膜流跨时钟 FIFO（103.52 MHz MIPI 域 -> 25.00 MHz CCL 域）
// --------------------------------------------------------------------------
// 为什么需要：
//   CCL 的标签表更新路径在 9.66 ns 内无法收敛（27 级、净延迟占 69%，
//   slice 利用率 80.8%）。把 CCL 放到 50 MHz 系统时钟 /2 的 25.00 MHz 异步域
//   （40 ns），吞吐 25.0 M 拍/s 仍高于有效像素流 13.824 M 拍/s；
//   跨域只传 8 bit 掩膜事件，用标准格雷码指针 + 2FF 同步。
//
// 数据格式：{resync, err, hit[3:0], tuser, tlast, 2'b00}
//   err=1 表示此前发生过写丢失（满），本帧结果必须整帧丢弃。
//   x/y 坐标由 CCL 域的行/列计数器重建，不再跨域传递。
//
// 实现：深度 512（一行 320 拍 + 突发余量），同步写、寄存输出读，
//       读写各自时钟域，指针格雷码跨域。
// ==========================================================================
module isp_mask_fifo #(
    parameter integer AW = 9            // 深度 = 2^AW = 512
)(
    // ---------------- 写侧（MIPI 域，103.52 MHz） ----------------
    input  wire        I_wr_clk,
    input  wire        I_wr_rst_n,
    input  wire        I_wr_en,
    input  wire [9:0]  I_wr_data,
    output wire        O_full,
    // ---------------- 读侧（CCL 域，25.00 MHz） ----------------
    input  wire        I_rd_clk,
    input  wire        I_rd_rst_n,
    input  wire        I_rd_en,
    output reg  [9:0]  O_rd_data,
    output reg         O_rd_valid,
    output wire        O_empty
);

    localparam integer DEPTH = (1 << AW);

    reg [9:0]  mem [0:DEPTH-1];

    reg [AW:0] wr_bin, wr_gray;
    reg [AW:0] rd_bin, rd_gray;

    // 跨域同步（各 2 级）
    reg [AW:0] rd_gray_s1, rd_gray_s2;   // 读指针 -> 写域
    reg [AW:0] wr_gray_s1, wr_gray_s2;   // 写指针 -> 读域

    function [AW:0] bin2gray;
        input [AW:0] b;
        begin
            bin2gray = (b >> 1) ^ b;
        end
    endfunction

    // ---------------- 写侧 ----------------
    // 满标志必须用"如果本拍写"的下一次指针计算，且不能反馈 full 自身，
    // 否则 full = f(wr + ~full) 会形成组合环（仿真出现 vsim-3601）。
    wire [AW:0] wr_bin_nxt  = wr_bin + (I_wr_en ? {{AW{1'b0}}, 1'b1} : {(AW+1){1'b0}});
    wire [AW:0] wr_gray_nxt = bin2gray(wr_bin_nxt);

    assign O_full = (wr_gray_nxt == {~rd_gray_s2[AW:AW-1], rd_gray_s2[AW-2:0]});

    always @(posedge I_wr_clk or negedge I_wr_rst_n) begin
        if (!I_wr_rst_n) begin
            wr_bin  <= {(AW+1){1'b0}};
            wr_gray <= {(AW+1){1'b0}};
            rd_gray_s1 <= {(AW+1){1'b0}};
            rd_gray_s2 <= {(AW+1){1'b0}};
        end
        else begin
            rd_gray_s1 <= rd_gray;
            rd_gray_s2 <= rd_gray_s1;
            if (I_wr_en && !O_full) begin
                mem[wr_bin[AW-1:0]] <= I_wr_data;
                wr_bin  <= wr_bin_nxt;
                wr_gray <= wr_gray_nxt;
            end
        end
    end

    // ---------------- 读侧 ----------------
    assign O_empty = (rd_gray == wr_gray_s2);

    wire rd_allow = I_rd_en && !O_empty;
    wire [AW:0] rd_bin_nxt  = rd_bin + (rd_allow ? {{AW{1'b0}}, 1'b1} : {(AW+1){1'b0}});
    wire [AW:0] rd_gray_nxt = bin2gray(rd_bin_nxt);

    always @(posedge I_rd_clk or negedge I_rd_rst_n) begin
        if (!I_rd_rst_n) begin
            rd_bin     <= {(AW+1){1'b0}};
            rd_gray    <= {(AW+1){1'b0}};
            wr_gray_s1 <= {(AW+1){1'b0}};
            wr_gray_s2 <= {(AW+1){1'b0}};
            O_rd_data  <= 10'd0;
            O_rd_valid <= 1'b0;
        end
        else begin
            wr_gray_s1 <= wr_gray;
            wr_gray_s2 <= wr_gray_s1;
            O_rd_valid <= rd_allow;
            if (rd_allow) begin
                O_rd_data <= mem[rd_bin[AW-1:0]];
                rd_bin    <= rd_bin_nxt;
                rd_gray   <= rd_gray_nxt;
            end
        end
    end

endmodule
