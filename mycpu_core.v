`include "lib/defines.vh"
// CPU核心模块：实例化五级流水线所有功能模块及控制模块，完成各模块信号连接
// 核心职责：1. 实例化取指、译码、执行、访存、回写模块；2. 实例化控制模块和hilo寄存器；3. 连接各模块的数据流和控制流

module mycpu_core(
    input wire clk,
    input wire rst,
    input wire [5:0] int,

    // 指令存储器接口
    output wire inst_sram_en,
    output wire [3:0] inst_sram_wen,
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input wire [31:0] inst_sram_rdata,

    // 数据存储器接口
    output wire data_sram_en,
    output wire [3:0] data_sram_wen,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata,
    input wire [31:0] data_sram_rdata,
    
    // 调试接口：输出回写阶段的关键信号，用于仿真/板级调试
    output wire [31:0] debug_wb_pc,
    output wire [3:0] debug_wb_rf_wen,
    output wire [4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata
);

    // 内部互连总线：各流水段之间的数据/控制信号传输
    wire [`IF_TO_ID_WD-1:0] if_to_id_bus;
    wire [`ID_TO_EX_WD-1:0] id_to_ex_bus;
    wire [`EX_TO_MEM_WD-1:0] ex_to_mem_bus;
    wire [`MEM_TO_WB_WD-1:0] mem_to_wb_bus;
    wire [`BR_WD-1:0] br_bus; 
    wire [`DATA_SRAM_WD-1:0] ex_dt_sram_bus;
    wire [`WB_TO_RF_WD-1:0] wb_to_rf_bus;
    
    // 流水线控制信号
    wire [`StallBus-1:0] stall;
    wire flush;
    wire [31:0] new_pc;
    wire stall_for_load;    // 加载指令导致的暂停请求（译码→控制模块）
    wire stall_for_ex;  // 乘除法多周期导致的暂停请求（执行→控制模块）

    // hilo寄存器相关信号：乘除法结果存储与读取
    wire [31:0] hi_data, lo_data;    // hilo寄存器读出的高32位/低32位数据（→执行模块）
    wire [65:0] hilo_bus;

    // 1. 实例化取指模块（IF）：从指令存储器读取指令，生成PC
    IF u_IF(
    	.clk             (clk             ),
        .rst             (rst             ),
        .stall           (stall           ),
        .br_bus          (br_bus          ),
        .if_to_id_bus    (if_to_id_bus    ),
        .inst_sram_en    (inst_sram_en    ),
        .inst_sram_wen   (inst_sram_wen   ),
        .inst_sram_addr  (inst_sram_addr  ),
        .inst_sram_wdata (inst_sram_wdata )
    );

    // 2. 实例化译码模块（ID）：解析指令，生成控制信号、操作数，处理跳转和数据前推
    ID u_ID(
    	.clk             (clk                 ),
        .rst             (rst                 ),
        .stall           (stall               ),
        .stallreq        (stallreq            ),
        .if_to_id_bus    (if_to_id_bus        ),

        // 前推相关信号：接收执行/访存模块的写寄存器信息，用于数据前推
        .ex_we           (ex_to_mem_bus[37]   ),
        .ex_waddr        (ex_to_mem_bus[36:32]),
        .ex_wdata        (ex_to_mem_bus[31:0] ),
        .ex_ram_read     (ex_to_mem_bus[38]   ),    // 执行模块的加载指令标记

        .mem_we          (mem_to_wb_bus[37]   ),
        .mem_waddr       (mem_to_wb_bus[36:32]),
        .mem_wdata       (mem_to_wb_bus[31:0] ),

        .inst_sram_rdata (inst_sram_rdata     ),
        .wb_to_rf_bus    (wb_to_rf_bus        ),

        .stall_for_load  (stall_for_load      ),
        .id_to_ex_bus    (id_to_ex_bus        ),
        .br_bus          (br_bus              )
    );

    // 3. 实例化执行模块（EX）：执行ALU运算、乘除法、加载/存储地址计算
    EX u_EX(
    	.clk             (clk             ),
        .rst             (rst             ),
        .stall           (stall           ),    // 接收控制模块的暂停信号
        .hi_data         (hi_data         ),    // 接收hilo寄存器的高32位数据
        .lo_data         (lo_data         ),    // 接收hilo寄存器的低32位数据
        .id_to_ex_bus    (id_to_ex_bus    ),
        .ex_to_mem_bus   (ex_to_mem_bus   ),
        .stall_for_ex    (stall_for_ex    ),    // 输出乘除法暂停请求（→控制模块）
        .data_sram_en    (data_sram_en    ),
        .data_sram_wen   (data_sram_wen   ),
        .data_sram_addr  (data_sram_addr  ),
        .data_sram_wdata (data_sram_wdata )
    );

    // 4. 实例化访存模块（MEM）：访问数据存储器，处理加载/存储数据
    MEM u_MEM(
    	.clk             (clk             ),
        .rst             (rst             ),
        .stall           (stall           ),

        .ex_to_mem_bus   (ex_to_mem_bus   ),
        .data_sram_rdata (data_sram_rdata ),     // 接收数据存储器的读取数据
        .mem_to_wb_bus   (mem_to_wb_bus   )
    );
    
    // 5. 实例化回写模块（WB）：将结果写回寄存器堆和hilo寄存器，输出调试信号
    WB u_WB(
    	.clk               (clk               ),
        .rst               (rst               ),
        .stall             (stall             ),
        .mem_to_wb_bus     (mem_to_wb_bus     ),
        .wb_to_rf_bus      (wb_to_rf_bus      ),
        .hilo_bus          (hilo_bus          ),
        .debug_wb_pc       (debug_wb_pc       ),
        .debug_wb_rf_wen   (debug_wb_rf_wen   ),
        .debug_wb_rf_wnum  (debug_wb_rf_wnum  ),
        .debug_wb_rf_wdata (debug_wb_rf_wdata )
    );

    // 6. 实例化控制模块（CTRL）：接收各模块暂停请求，生成流水线全局暂停/冲洗信号
    CTRL u_CTRL(
    	.rst               (rst               ),
        .stall_for_load    (stall_for_load    ),
        .stall_for_ex      (stall_for_ex      ),
        .flush             (flush             ),
        .stall             (stall             )
    );

    // 7. 实例化hilo寄存器模块：存储乘除法运算的高32位（余数）和低32位（商/积）
    hilo_reg u_hilo_reg(
        .clk                (clk                   ),
        .rst                (rst                   ),
        .stall              (stall                 ),   // 接收控制模块的暂停信号

        .ex_hi_we           (ex_to_mem_bus[146]    ),
        .ex_lo_we           (ex_to_mem_bus[145]    ),
        .ex_hi_in           (ex_to_mem_bus[144:113]),
        .ex_lo_in           (ex_to_mem_bus[112:81] ),

        .mem_hi_we          (mem_to_wb_bus[135]    ),
        .mem_lo_we          (mem_to_wb_bus[134]    ),
        .mem_hi_in          (mem_to_wb_bus[133:102]),
        .mem_lo_in          (mem_to_wb_bus[101:70] ),

        .hilo_bus           (hilo_bus              ),

        .hi_data            (hi_data               ),
        .lo_data            (lo_data               )
    );
    
endmodule