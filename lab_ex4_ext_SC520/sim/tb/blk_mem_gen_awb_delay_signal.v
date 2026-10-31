// 行为模型：signal_delay 使用的简单双口 BRAM（仅仿真用，非设计逻辑）
// 与厂商 IP 一致的 1 拍同步读延迟。
module blk_mem_gen_awb_delay_signal (
    input  wire        clka,
    input  wire [0:0]  wea,
    input  wire [10:0] addra,
    input  wire [95:0] dia,
    input  wire        clkb,
    input  wire [10:0] addrb,
    output reg  [95:0] dob
);
    reg [95:0] mem [0:2047];
    always @(posedge clka) if (wea[0]) mem[addra] <= dia;
    always @(posedge clkb) dob <= mem[addrb];
endmodule
