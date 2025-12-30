`include "lib/defines.vh"
// 回写模块（WB段）：流水线第五阶段，负责将运算结果/访存数据写回目标存储单元
// 核心功能：1. 向通用寄存器堆（regfile）回写数据；2. 向hilo寄存器回写乘除法结果；3. 输出调试信号

module WB(
    input wire clk,
    input wire rst,
    input wire flush,
    input wire [`StallBus-1:0] stall,

    input wire [`MEM_TO_WB_WD-1:0] mem_to_wb_bus,

    output wire [`WB_TO_RF_WD-1:0] wb_to_rf_bus,
    output wire [65:0] hilo_bus,

    output wire [31:0] debug_wb_pc, // 调试信号：当前回写指令的PC值
    output wire [3:0] debug_wb_rf_wen,  // 调试信号：寄存器堆写使能（4位扩展，便于观察）
    output wire [4:0] debug_wb_rf_wnum, // 调试信号：寄存器堆写地址
    output wire [31:0] debug_wb_rf_wdata    // 调试信号：寄存器堆写数据
);

    // 寄存MEM段传入的总线：同步时钟，避免亚稳态，处理暂停/冲洗逻辑
    reg [`MEM_TO_WB_WD-1:0] mem_to_wb_bus_r;

    always @ (posedge clk) begin
        if (rst) begin
            mem_to_wb_bus_r <= `MEM_TO_WB_WD'b0;
        end
        else if (flush) begin
            mem_to_wb_bus_r <= `MEM_TO_WB_WD'b0;
        end
        else if (stall[4]==`Stop && stall[5]==`NoStop) begin
            mem_to_wb_bus_r <= `MEM_TO_WB_WD'b0;
        end
        else if (stall[4]==`NoStop) begin
            mem_to_wb_bus_r <= mem_to_wb_bus;
        end
    end
    
    // 解析MEM段传入的总线信号：拆分得到回写所需的控制信号和数据    
    wire [31:0] wb_pc;
    wire rf_we;
    wire [4:0] rf_waddr;
    wire [31:0] rf_wdata;

    assign {
        hilo_bus,        // 135:70
        wb_pc,           // 69:38
        rf_we,           // 37
        rf_waddr,        // 36:32
        rf_wdata         // 31:0
    } = mem_to_wb_bus_r;

    // 打包回写寄存器堆的总线：将写使能、地址、数据组合为标准总线格式
    // assign wb_to_rf_bus = mem_to_wb_bus_r[`WB_TO_RF_WD-1:0];
    assign wb_to_rf_bus = {
        rf_we,           // 37
        rf_waddr,        // 36:32
        rf_wdata         // 31:0
    };

    // 调试信号赋值：直接映射核心信号，用于仿真/板级调试时观察回写状态
    assign debug_wb_pc = wb_pc;
    assign debug_wb_rf_wen = {4{rf_we}};// 写使能扩展为4位（便于波形观察）
    assign debug_wb_rf_wnum = rf_waddr;
    assign debug_wb_rf_wdata = rf_wdata;

endmodule