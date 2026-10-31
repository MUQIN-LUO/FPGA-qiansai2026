// ==========================================================================
// AE 真实 I2C 集成测试：uicfgcs520 + uii2c + 受控 SDA 从机模型
// --------------------------------------------------------------------------
// 从机模型（仅仿真）：
//   · ACK 所有写字节；
//   · 传感器 ID 读（寄存器 0x3107/0x3108）返回 0x9E/0x4B，使 ID 检查通过；
//   · 解码写事务的 16 位寄存器地址与数据，记录到 log，供检查 AE 写入表。
//
// 检查：
//   1) 初始化表完成（O_cfg_done=1），ID 正确；
//   2) AE 请求 I_ae=1991、I_ag=96 后，真实 uicfgcs520 依次写：
//        0x3812=0x00（group hold 开始）
//        0x3e00/01/02 = 曝光 1991 的三段
//        0x3e09/0x3e06/0x3e07 = 增益（I_ag=96 → 0x09/0x00/0xC0）
//        0x3812=0x30（group hold 结束/生效）
//   3) O_ae_cfg_done 在序列期间为 0，最后一笔写完后才为 1；
//   4) 无 I2C 错误。
// 说明：这是 I2C 总线受控响应下的仿真证据；真实传感器/板上时序仍需上板确认。
// ==========================================================================
`timescale 1ns/1ps
module tb_ae_i2c;
    reg clk = 0;
    always #21 clk = ~clk;       // ~23.8 MHz

    reg rst_n = 0;
    reg ae_req = 0;
    reg [15:0] ae = 16'd0, ag = 16'd0;

    wire scl;
    wire sda;
    wire cfg_done, ae_cfg_done;
    wire [15:0] sensor_id;
    wire sensor_id_ok, iic_error;

    uicfgcs520 #(.CLK_DIV(23809524/100000 - 1)) u_cfg (
        .I_clk(clk), .I_rst_n(rst_n),
        .I_ae_req(ae_req), .I_ae(ae), .I_ag(ag),
        .O_cam_scl(scl), .IO_cam_sda(sda),
        .O_cfg_done(cfg_done), .O_ae_cfg_done(ae_cfg_done),
        .O_sensor_id(sensor_id), .O_sensor_id_ok(sensor_id_ok),
        .O_iic_error(iic_error)
    );

    // ---------------- 受控 I2C 从机模型 ----------------
    pullup (sda);
    reg sda_low;
    assign sda = sda_low ? 1'b0 : 1'bz;

    reg        started, phase;
    reg [3:0]  bitn;
    reg [7:0]  sh;
    reg [2:0]  byten;
    reg        rw;
    reg [15:0] regaddr;
    reg [7:0]  rd_shift;
    reg        drive_data;

    initial begin
        started    = 1'b0;
        phase      = 1'b0;
        bitn       = 4'd0;
        sh         = 8'd0;
        byten      = 3'd0;
        rw         = 1'b0;
        regaddr    = 16'd0;
        rd_shift   = 8'd0;
        drive_data = 1'b0;
        sda_low    = 1'b0;
    end

    integer log_n;
    reg [31:0] log_mem [0:255];
    reg        log_en;

    // 任意 SCL 高期间 SDA 下降都视为 START/重复 START（随机读需要）
    always @(negedge sda) if (scl) begin
        started  <= 1'b1;
        phase    <= 1'b0;
        bitn     <= 4'd0;
        byten    <= 3'd0;
        sda_low  <= 1'b0;
        drive_data <= 1'b0;
    end
    always @(posedge sda) if (scl) begin
        started    <= 1'b0;
        sda_low    <= 1'b0;
        drive_data <= 1'b0;
    end

    always @(posedge scl) if (started) begin
        if (!phase) begin
            sh <= {sh[6:0], sda};
            if (bitn == 4'd7) begin
                bitn  <= 4'd0;
                phase <= 1'b1;
            end
            else bitn <= bitn + 4'd1;
        end
        else begin
            // 第 9 个 SCL 上升沿：ACK 位处理
            phase <= 1'b0;
            if (byten == 3'd0) begin
                rw     <= sh[0];
                byten  <= 3'd1;
                if (sh[0]) begin
                    rd_shift   <= (regaddr == 16'h3107) ? 8'h9e :
                                  (regaddr == 16'h3108) ? 8'h4b : 8'h00;
                    drive_data <= 1'b1;
                end
            end
            else if (!rw) begin
                case (byten)
                    3'd1: regaddr[15:8] <= sh;
                    3'd2: regaddr[7:0]  <= sh;
                    3'd3: begin
                        if (log_en) begin
                            log_mem[log_n] <= {regaddr[15:0], sh};
                            log_n <= log_n + 1;
                        end
                    end
                    default: ;
                endcase
                if (byten != 3'd3) byten <= byten + 3'd1;
            end
            else begin
                drive_data <= 1'b0;   // 读字节结束（master ACK/NACK）
            end
        end
    end

    always @(negedge scl) if (started) begin
        // 地址字节 / 写数据字节：从机 ACK；读数据字节：释放给 master ACK/NACK
        if (phase && (byten == 3'd0 || !rw)) sda_low <= 1'b1;
        else if (phase)                      sda_low <= 1'b0;
        else if (drive_data) begin
            sda_low  <= ~rd_shift[7];
            rd_shift <= {rd_shift[6:0], 1'b0};
        end
        else sda_low <= 1'b0;
    end

    // ---- 短调试：前 400k 周期的状态/边沿 ----
    integer dbg_n = 0, dbg_edge = 0, dbg_cyc = 0;
    always @(posedge clk) begin
        dbg_cyc = dbg_cyc + 1;
        if (dbg_n < 20 && (dbg_cyc % 5000 == 0)) begin
            dbg_n = dbg_n + 1;
            $display("DBG t=%0t state=%0d iicS=%0d scl=%b sda=%b busy=%b started=%b bitn=%0d phase=%b byten=%0d",
                     $time, u_cfg.state, u_cfg.u_i2c.IIC_S, scl, sda,
                     u_cfg.u_i2c.O_iic_busy, started, bitn, phase, byten);
        end
    end
    always @(negedge sda) if (dbg_edge < 12) begin
        dbg_edge = dbg_edge + 1;
        $display("EDGE sda_fall t=%0t scl=%b started=%b", $time, scl, started);
    end
    always @(posedge scl) if (dbg_edge < 12) begin
        dbg_edge = dbg_edge + 1;
        $display("EDGE scl_rise t=%0t sda=%b started=%b bitn=%0d", $time, sda, started, bitn);
    end

    integer errors = 0;
    task check;
        input [1023:0] name;
        input cond;
        begin
            if (cond) $display("  [ OK ] %0s", name);
            else begin $display("  [FAIL] %0s", name); errors=errors+1; end
        end
    endtask

    integer i;
    initial begin
        log_en = 0; log_n = 0;
        repeat (10) @(negedge clk);
        rst_n = 1;
        // 等待初始化表完成（165 个寄存器 + 5ms reset wait）
        i = 0;
        while (!cfg_done && i < 4000000) begin
            @(negedge clk); i = i + 1;
        end
        $display("INIT: cfg_done=%0d id=%h id_ok=%0d err=%0d cycles=%0d",
                 cfg_done, sensor_id, sensor_id_ok, iic_error, i);
        check("init table completes", cfg_done == 1'b1);
        check("sensor ID = 9e4b", sensor_id == 16'h9e4b && sensor_id_ok == 1'b1);
        check("no I2C error during init", iic_error == 1'b0);

        // ---- AE 请求：曝光 1991、增益索引 96 ----
        //   初始化完成后 ae_cfg_done 已经是 1，必须先等它被拉低（请求被
        //   uicfgcs520 接受），再等它回到 1（8 个寄存器写完），否则会把
        //   初始化的 1 误当成 AE 完成。
        @(negedge clk); log_en = 1; log_n = 0;
        ae = 16'd1991; ag = 16'd96; ae_req = 1'b1;
        i = 0;
        while (ae_cfg_done !== 1'b0 && i < 100000) begin
            @(negedge clk); i = i + 1;
        end
        // 请求被接受（done 拉低）后立即释放，等 8 笔写完再看 done 回 1；
        // 若一直保持 req，uicfgcs520 回到 ST_IDLE 会再启动一轮。
        @(negedge clk); ae_req = 1'b0;
        i = 0;
        while (ae_cfg_done !== 1'b1 && i < 1000000) begin
            @(negedge clk); i = i + 1;
        end
        repeat (20) @(negedge clk);
        $display("AE: done=%0d err=%0d cycles=%0d writes=%0d",
                 ae_cfg_done, iic_error, i, log_n);
        for (i = 0; i < log_n; i = i + 1)
            $display("  WR[%0d] addr=%h data=%h", i,
                     log_mem[i][23:8], log_mem[i][7:0]);

        check("AE sequence completes", ae_cfg_done == 1'b1);
        check("no I2C error during AE", iic_error == 1'b0);
        check("AE writes exactly 8 registers", log_n == 8);
        if (log_n >= 8) begin
            check("hold start 0x3812=00", log_mem[0] == 32'h3812_00);
            check("exp 0x3e00=00",       log_mem[1] == 32'h3e00_00);
            check("exp 0x3e01=7c",       log_mem[2] == 32'h3e01_7c);
            check("exp 0x3e02=70",       log_mem[3] == 32'h3e02_70);
            check("gain 0x3e09=09",      log_mem[4] == 32'h3e09_09);
            check("gain 0x3e06=00",      log_mem[5] == 32'h3e06_00);
            check("gain 0x3e07=c0 (AG=96)", log_mem[6] == 32'h3e07_c0);
            check("hold end 0x3812=30",  log_mem[7] == 32'h3812_30);
        end

        $display("\n================================================");
        if (errors==0) $display("RESULT: PASS (0 errors)");
        else $display("RESULT: FAIL (%0d errors)", errors);
        $display("================================================");
        $finish;
    end
endmodule
