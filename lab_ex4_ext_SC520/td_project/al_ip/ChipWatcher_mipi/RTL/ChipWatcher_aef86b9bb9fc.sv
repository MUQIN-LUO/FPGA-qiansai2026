// synthesis syn_black_box 
module ChipWatcher_aef86b9bb9fc ( 
    input [15:0] probe0, 
    input [0:0] probe1, 
    input [0:0] probe2, 
    input [8:0] probe3, 
    input [10:0] probe4, 
    input [0:0] probe5, 
    input [0:0] probe6, 
    input [0:0] probe7, 
    input [0:0] probe8, 
    input       clk  
);
    localparam string IP_TYPE  = "ChipWatcher";
    localparam CWC_BUS_NUM     = 9;
    localparam INPUT_PIPE_NUM  = 0;
    localparam OUTPUT_PIPE_NUM = 0;
    localparam RAM_DATA_DEPTH  = 1024;
    localparam CAPTURE_CONTROL = 0;

    localparam integer CWC_BUS_WIDTH   [CWC_BUS_NUM-1:0] = {16,1,1,9,11,1,1,1,1};
    localparam integer CWC_DATA_ENABLE [CWC_BUS_NUM-1:0] = {1,1,1,1,1,1,1,1,1};    
    localparam integer CWC_TRIG_ENABLE [CWC_BUS_NUM-1:0] = {1,1,1,1,1,1,1,1,1};    
endmodule



