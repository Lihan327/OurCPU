`include "lib/defines.vh"
// 译码模块（ID段）：对IF段传入的指令进行解析，生成控制信号、操作数
// 核心功能：指令译码、跳转判断、数据前推处理、暂停请求生成
module ID(
    input wire clk,
    input wire rst, // 复位信号（高有效）
    input wire flush,   // 冲洗信号（高有效，清空译码结果）
    input wire [`StallBus-1:0] stall,   // 流水线暂停信号（控制是否插入气泡）   
    
    output wire stallreq,   // 译码阶段的暂停请求（输出给CTRL）

    input wire [`IF_TO_ID_WD-1:0] if_to_id_bus,

    input wire [31:0] inst_sram_rdata,  // 从指令存储器读取的指令内容

    input wire ex_we,
    input wire [4:0] ex_waddr,
    input wire [31:0] ex_wdata,
    input wire ex_ram_read,  // EX段指令是否为加载指令（用于判断load相关暂停）

    input wire mem_we,
    input wire [4:0] mem_waddr,
    input wire [31:0] mem_wdata,

    input wire [`WB_TO_RF_WD-1:0] wb_to_rf_bus,

    output wire stall_for_load, // 因加载指令导致的暂停请求（输出给CTRL）
    output wire [`ID_TO_EX_WD-1:0] id_to_ex_bus,
    output wire [`BR_WD-1:0] br_bus 
);
     // 寄存IF段传入的总线
    reg [`IF_TO_ID_WD-1:0] if_to_id_bus_r;
    wire [31:0] inst;
    wire [31:0] id_pc;
    wire ce;    // 指令有效信号（来自IF段）

    wire wb_rf_we;
    wire [4:0] wb_rf_waddr;
    wire [31:0] wb_rf_wdata;

    wire [4:0] mem_op;  // 访存指令类型标记（用于MEM段解析）
    wire [8:0] hilo_op; // 乘除法相关指令标记（mfhi/mflo等）

    reg is_stop;    // 暂停标记（用于流水线气泡插入时的指令屏蔽）

    // 同步寄存IF段总线：处理复位、冲洗、暂停逻辑
    always @ (posedge clk) begin
        if (rst) begin
            if_to_id_bus_r <= `IF_TO_ID_WD'b0;
            is_stop <= 1'b0;
        end
        else if (flush) begin
            if_to_id_bus_r <= `IF_TO_ID_WD'b0;
            is_stop <= 1'b0;
        end
        else if (stall[1]==`Stop && stall[2]==`NoStop) begin
            if_to_id_bus_r <= `IF_TO_ID_WD'b0;
            is_stop <= 1'b0;
        end
        else if (stall[1]==`NoStop) begin
            if_to_id_bus_r <= if_to_id_bus;
            is_stop <= 1'b0;
        end
        else if (stall[2]==`Stop) begin
            is_stop <= 1'b1;
        end
    end

    // 指令赋值：暂停时屏蔽指令（输出0）
    assign inst = is_stop ? inst : inst_sram_rdata;

    // 解析IF段传入的总线：指令有效信号、PC值
    assign {
        ce,
        id_pc
    } = if_to_id_bus_r;

    // 解析WB段传入的写寄存器总线
    assign {
        wb_rf_we,
        wb_rf_waddr,
        wb_rf_wdata
    } = wb_to_rf_bus;

    // 指令字段解析（MIPS32指令格式拆分）
    wire [5:0] opcode;
    wire [4:0] rs,rt,rd,sa;
    wire [5:0] func;
    wire [15:0] imm;
    wire [25:0] instr_index;    // 跳转目标索引（25~0bit，用于j/jal指令）
    wire [19:0] code;   // 扩展字段（25~6bit）
    wire [4:0] base;
    wire [15:0] offset;
    wire [2:0] sel; // 选择字段（2~0bit）

    // 译码器输出
    wire [63:0] op_d, func_d;
    // 寄存器地址译码结果,用于指令判断
    wire [31:0] rs_d, rt_d, rd_d, sa_d;

    // ALU操作数选择信号：sel_alu_src1（3bit）选择第一个操作数来源，sel_alu_src2（4bit）选择第二个
    wire [2:0] sel_alu_src1;
    wire [3:0] sel_alu_src2;
    wire [11:0] alu_op; // ALU运算控制信号（12bit，对应不同算术逻辑操作）

    // 数据存储器控制信号
    wire data_ram_en;
    wire [3:0] data_ram_wen;
    
    // 寄存器堆写控制信号
    wire rf_we;
    wire [4:0] rf_waddr;
    wire sel_rf_res;
    wire [2:0] sel_rf_dst;

    // 寄存器堆读出数据、hilo寄存器数据
    wire [31:0] rdata1, rdata2;
    wire [31:0] hi_data, lo_data;   // hilo寄存器数据（乘除法结果）
   
    // 数据前推判断信号：判断rs/rt是否需要从EX/MEM段前推数据 
    wire rs_ex_ok, rt_ex_ok;    // rs/rt是否与EX段写地址冲突
    wire rs_mem_ok, rt_mem_ok;  // rs/rt是否与MEM段写地址冲突
    wire sel_rs_forward;    // rs需要前推标记
    wire sel_rt_forward;    // rt需要前推标记
    wire [31:0] rs_forward_data;
    wire [31:0] rt_forward_data;

    // 前推处理后的寄存器数据
    wire [31:0] rf_rdata1;
    wire [31:0] rf_rdata2;

    // 最终传入EX段的操作数
    wire [31:0] rdata1_fd;
    wire [31:0] rdata2_fd;

    // 实例化通用寄存器堆：读取rs/rt对应的寄存器值
    regfile u_regfile(
    	.clk             (clk               ),
        .raddr1          (rs                ),
        .rdata1          (rdata1            ),
        .raddr2          (rt                ),
        .rdata2          (rdata2            ),
        
        .we              (wb_rf_we          ),
        .waddr           (wb_rf_waddr       ),
        .wdata           (wb_rf_wdata       )
    );

    // WB段数据前推判断：判断当前读取的rs/rt是否是WB段要写的寄存器
    wire sel_r1_wdata;
    wire sel_r2_wdata;
    assign sel_r1_wdata = wb_rf_we & (wb_rf_waddr == rs);
    assign sel_r2_wdata = wb_rf_we & (wb_rf_waddr == rt);

    // 前推处理：WB段数据直接替换寄存器读出数据（解决RAW相关）
    assign rf_rdata1 = sel_r1_wdata ? wb_rf_wdata : rdata1;
    assign rf_rdata2 = sel_r2_wdata ? wb_rf_wdata : rdata2;

    assign opcode      = inst[31:26];
    assign rs          = inst[25:21];
    assign rt          = inst[20:16];
    assign rd          = inst[15:11];
    assign sa          = inst[10:6];
    assign func        = inst[5:0];
    assign imm         = inst[15:0];
    assign instr_index = inst[25:0];
    assign code        = inst[25:6];
    assign base        = inst[25:21];
    assign offset      = inst[15:0];
    assign sel         = inst[2:0];

    // 指令类型标记：所有支持的指令对应的单bit标记（1表示当前指令是该类型）
    wire inst_add,   inst_addi,   inst_addu,   inst_addiu;
    wire inst_sub,   inst_subu,   inst_slt,    inst_slti;  
    wire inst_sltu,  inst_sltiu,  inst_div,    inst_divu;
    wire inst_mult,  inst_multu,  inst_and,    inst_andi;  
    wire inst_lui,   inst_nor,    inst_or,     inst_ori;
    wire inst_xor,   inst_xori,   inst_sll,    inst_sllv;
    wire inst_sra,   inst_srav,   inst_srl,    inst_srlv;
    wire inst_beq,   inst_bne,    inst_bgez,   inst_bgtz;
    wire inst_blez,  inst_bltz,   inst_bltzal, inst_bgezal;
    wire inst_j,     inst_jal,    inst_jr,     inst_jalr;  
    wire inst_mfhi,  inst_mflo,   inst_mthi,   inst_mtlo;
    wire inst_lb,    inst_lbu,    inst_lh,     inst_lhu;
    wire inst_lw,    inst_sb,     inst_sh,     inst_sw;
    wire inst_break, inst_syscall;
    wire inst_eret,  inst_mfc0,   inst_mtc0;
    wire inst_mul;

    // ALU运算类型标记：对应alu.v中的12种运算
    wire op_add, op_sub, op_slt, op_sltu;
    wire op_and, op_nor, op_or, op_xor;
    wire op_sll, op_srl, op_sra, op_lui;

    decoder_6_64 u0_decoder_6_64(
    	.in  (opcode),
        .out (op_d  )
    );

    decoder_6_64 u1_decoder_6_64(
    	.in  (func   ),
        .out (func_d )
    );
    
    decoder_5_32 u0_decoder_5_32(
    	.in  (rs   ),
        .out (rs_d )
    );

    decoder_5_32 u1_decoder_5_32(
    	.in  (rt   ),
        .out (rt_d )
    );

    
    assign inst_add     = op_d[6'b00_0000] & func_d[6'b10_0000];
    assign inst_addi    = op_d[6'b00_1000];
    assign inst_addu    = op_d[6'b00_0000] & func_d[6'b10_0001];
    assign inst_addiu   = op_d[6'b00_1001];
    assign inst_sub     = op_d[6'b00_0000] & func_d[6'b10_0010];
    assign inst_subu    = op_d[6'b00_0000] & func_d[6'b10_0011];
    assign inst_slt     = op_d[6'b00_0000] & func_d[6'b10_1010];
    assign inst_slti    = op_d[6'b00_1010];
    assign inst_sltu    = op_d[6'b00_0000] & func_d[6'b10_1011];
    assign inst_sltiu   = op_d[6'b00_1011];

    assign inst_div     = op_d[6'b00_0000] & func_d[6'b01_1010];
    assign inst_divu    = op_d[6'b00_0000] & func_d[6'b01_1011];
    assign inst_mul     = op_d[6'b01_1100] & func_d[6'b00_0010];
    assign inst_mult    = op_d[6'b00_0000] & func_d[6'b01_1000];
    assign inst_multu   = op_d[6'b00_0000] & func_d[6'b01_1001];
    
    assign inst_and     = op_d[6'b00_0000] & func_d[6'b10_0100];
    assign inst_andi    = op_d[6'b00_1100];
    assign inst_lui     = op_d[6'b00_1111];
    assign inst_nor     = op_d[6'b00_0000] & func_d[6'b10_0111];
    assign inst_or      = op_d[6'b00_0000] & func_d[6'b10_0101];
    assign inst_ori     = op_d[6'b00_1101];
    assign inst_xor     = op_d[6'b00_0000] & func_d[6'b10_0110];
    assign inst_xori    = op_d[6'b00_1110];
    
    assign inst_sllv    = op_d[6'b00_0000] & func_d[6'b00_0100];
    assign inst_sll     = op_d[6'b00_0000] & func_d[6'b00_0000];
    assign inst_srav    = op_d[6'b00_0000] & func_d[6'b00_0111];
    assign inst_sra     = op_d[6'b00_0000] & func_d[6'b00_0011];
    assign inst_srlv    = op_d[6'b00_0000] & func_d[6'b00_0110];
    assign inst_srl     = op_d[6'b00_0000] & func_d[6'b00_0010];

    assign inst_beq     = op_d[6'b00_0100];
    assign inst_bne     = op_d[6'b00_0101];
    assign inst_bgez    = op_d[6'b00_0001] & rt_d[5'b0_0001];
    assign inst_bgtz    = op_d[6'b00_0111];
    assign inst_blez    = op_d[6'b00_0110];
    assign inst_bltz    = op_d[6'b00_0001] & rt_d[5'b0_0000];
    assign inst_bgezal  = op_d[6'b00_0001] & rt_d[5'b1_0001];
    assign inst_bltzal  = op_d[6'b00_0001] & rt_d[5'b1_0000];
    assign inst_j       = op_d[6'b00_0010];
    assign inst_jal     = op_d[6'b00_0011];
    assign inst_jr      = op_d[6'b00_0000] & func_d[6'b00_1000];
    assign inst_jalr    = op_d[6'b00_0000] & func_d[6'b00_1001];

    assign inst_mfhi    = op_d[6'b00_0000] & func_d[6'b01_0000];
    assign inst_mflo    = op_d[6'b00_0000] & func_d[6'b01_0010];
    assign inst_mthi    = op_d[6'b00_0000] & func_d[6'b01_0001];
    assign inst_mtlo    = op_d[6'b00_0000] & func_d[6'b01_0011];

    assign inst_break   = op_d[6'b00_0000] & func_d[6'b00_1101];
    assign inst_syscall = op_d[6'b00_0000] & func_d[6'b00_1100];

    assign inst_lb      = op_d[6'b10_0000];
    assign inst_lbu     = op_d[6'b10_0100];
    assign inst_lh      = op_d[6'b10_0001];
    assign inst_lhu     = op_d[6'b10_0101];
    assign inst_lw      = op_d[6'b10_0011];
    assign inst_sb      = op_d[6'b10_1000];
    assign inst_sh      = op_d[6'b10_1001];
    assign inst_sw      = op_d[6'b10_1011];

    assign inst_eret    = op_d[6'b01_0000] & func_d[6'b01_1000];
    assign inst_mfc0    = op_d[6'b01_0000] & rs_d[5'b0_0000];
    assign inst_mtc0    = op_d[6'b01_0000] & rs_d[5'b0_0100];

    // 乘除法相关指令标记：打包为9bit信号传入EX段
    assign hilo_op = {
        inst_mfhi, inst_mflo , inst_mthi, inst_mtlo,
        inst_mult, inst_multu, inst_div , inst_divu,
        inst_mul
    };

    // 第一个ALU操作数来源选择：3bit控制（rs寄存器/PC值/sa零扩展）
    // rs to reg1  
    assign sel_alu_src1[0] = inst_add  | inst_addi  | inst_addu  | inst_addiu 
                           | inst_sub  | inst_subu  | inst_slt   | inst_slti 
                           | inst_sltu | inst_sltiu | inst_div   | inst_divu 
                           | inst_mul  | inst_mult  | inst_multu | inst_and 
                           | inst_andi | inst_nor   | inst_or    | inst_ori 
                           | inst_xor  | inst_xori  | inst_sllv  | inst_srav 
                           | inst_srlv | inst_mthi  | inst_mtlo  | inst_lb 
                           | inst_lbu  | inst_lh    | inst_lhu   | inst_lw 
                           | inst_sb   | inst_sh    | inst_sw; 
    // pc to reg1
    assign sel_alu_src1[1] = inst_jal | inst_bltzal | inst_bgezal | inst_jalr;
    // sa_zero_extend to reg1
    assign sel_alu_src1[2] = inst_sll | inst_sra | inst_srl;

    // 第二个ALU操作数来源选择：4bit控制（rt寄存器/立即数符号扩展/32'd8/立即数零扩展）
    // rt to reg2
    assign sel_alu_src2[0] = inst_add | inst_addu | inst_sub   | inst_subu 
                           | inst_slt | inst_sltu | inst_div   | inst_divu 
                           | inst_mul | inst_mult | inst_multu | inst_and 
                           | inst_nor | inst_or   | inst_xor   | inst_sllv 
                           | inst_sll | inst_srav | inst_sra   | inst_srlv 
                           | inst_srl;
    // imm_sign_extend to reg2
    assign sel_alu_src2[1] = inst_addi | inst_addiu | inst_lw  | inst_lb 
                           | inst_lbu  | inst_lh    | inst_lhu | inst_sw 
                           | inst_sh   | inst_sb    | inst_lui
                           | inst_slti | inst_sltiu ;
    // 32'd8 to reg2
    assign sel_alu_src2[2] = inst_jal | inst_bltzal | inst_bgezal | inst_jalr;
    // imm_zero_extend to reg2
    assign sel_alu_src2[3] = inst_ori | inst_andi | inst_xori;

    // ALU运算控制信号赋值：对应12种运算类型
    assign op_add  =  inst_add | inst_addu   | inst_addi   | inst_addiu 
                    | inst_lw  | inst_lb     | inst_lbu    | inst_lh 
                    | inst_lhu | inst_sw     | inst_sh     | inst_sb 
                    | inst_jal | inst_bltzal | inst_bgezal | inst_jalr;
    assign op_sub  =  inst_sub | inst_subu;
    assign op_slt  =  inst_slt | inst_slti;
    assign op_sltu =  inst_sltu | inst_sltiu;
    assign op_and  =  inst_and | inst_andi;
    assign op_nor  =  inst_nor;
    assign op_or   =  inst_or | inst_ori;
    assign op_xor  =  inst_xor | inst_xori;
    assign op_sll  =  inst_sllv | inst_sll;
    assign op_srl  =  inst_srlv | inst_srl;
    assign op_sra  = inst_srav | inst_sra;
    assign op_lui  =  inst_lui;

    // ALU控制信号打包：12bit传入EX段
    assign alu_op = {op_add, op_sub, op_slt, op_sltu,
                     op_and, op_nor, op_or, op_xor,
                     op_sll, op_srl, op_sra, op_lui};

    // 加载指令类型标记：5bit（lb/lbu/lh/lhu/lw）
    assign mem_op = {inst_lb, inst_lbu, inst_lh, inst_lhu, inst_lw};

    // 数据存储器使能：加载/存储指令时有效
    // load and store enable
    assign data_ram_en =  inst_lb | inst_lbu | inst_lh | inst_lhu 
                        | inst_lw | inst_sb  | inst_sh | inst_sw;

    // 数据存储器写使能：存储指令时有效（sb/sh/sw对应不同bit位）
    // write enable
    assign data_ram_wen = {1'b0, inst_sb, inst_sh, inst_sw};

    // 寄存器堆写使能：需要写结果到寄存器的指令有效
    // regfile store enable
    assign rf_we = inst_add    | inst_addu   | inst_addi  | inst_addiu 
                 | inst_sub    | inst_subu   | inst_lw    | inst_lb 
                 | inst_lbu    | inst_lh     | inst_lhu   | inst_jal 
                 | inst_bltzal | inst_bgezal | inst_jalr  | inst_slt 
                 | inst_slti   | inst_sltu   | inst_sltiu | inst_sllv 
                 | inst_sll    | inst_srlv   | inst_srl   | inst_srav 
                 | inst_sra    | inst_lui    | inst_and   | inst_andi
                 | inst_or     | inst_ori    | inst_xor   | inst_xori 
                 | inst_nor    | inst_mfhi   | inst_mflo  | inst_mfc0 
                 | inst_mul;

    // 写目标寄存器选择：3bit控制（rd/rt/$31）
    // store in [rd]
    assign sel_rf_dst[0] = inst_add  | inst_addu | inst_sub  | inst_subu 
                         | inst_slt  | inst_sltu | inst_sllv | inst_sll 
                         | inst_srlv | inst_srl  | inst_srav | inst_sra 
                         | inst_and  | inst_or   | inst_xor  | inst_nor 
                         | inst_mfhi | inst_mflo | inst_mul;
    // store in [rt] 
    assign sel_rf_dst[1] = inst_addi  | inst_addiu | inst_lw   | inst_lb 
                         | inst_lbu   | inst_lh    | inst_lhu  | inst_lui 
                         | inst_ori   | inst_andi  | inst_xori | inst_slti 
                         | inst_sltiu | inst_mfc0;
    // store in [31] PC
    assign sel_rf_dst[2] = inst_jal | inst_bltzal | inst_bgezal | inst_jalr;

    // 写目标寄存器地址：根据sel_rf_dst选择rd/rt/$31
    // sel for regfile address
    assign rf_waddr = {5{sel_rf_dst[0]}} & rd 
                    | {5{sel_rf_dst[1]}} & rt
                    | {5{sel_rf_dst[2]}} & 32'd31;

    // 写寄存器数据选择：0→ALU结果，1→加载数据
    assign sel_rf_res = inst_lw | inst_lb | inst_lbu | inst_lh | inst_lhu;

    // EX段数据前推判断：rs/rt是否与EX段写地址冲突
    assign rs_ex_ok =  (rs == ex_waddr)  &&  ex_we ? 1'b1 : 1'b0;
    assign rt_ex_ok =  (rt == ex_waddr)  &&  ex_we ? 1'b1 : 1'b0;

    // MEM段数据前推判断：rs/rt是否与MEM段写地址冲突
    assign rs_mem_ok = (rs == mem_waddr) && mem_we ? 1'b1 : 1'b0;
    assign rt_mem_ok = (rt == mem_waddr) && mem_we ? 1'b1 : 1'b0;

    // 前推选择：优先EX段，再MEM段
    assign sel_rs_forward = rs_ex_ok | rs_mem_ok;
    assign sel_rt_forward = rt_ex_ok | rt_mem_ok;
    
    // 前推数据赋值：EX段数据优先于MEM段
    assign rs_forward_data = rs_ex_ok  ? ex_wdata :
                             rs_mem_ok ? mem_wdata:
                             32'b0;

    assign rt_forward_data = rt_ex_ok  ? ex_wdata :
                             rt_mem_ok ? mem_wdata:
                             32'b0;

    // 最终操作数：前推数据优先于寄存器读出数据
    assign rdata1_fd = sel_rs_forward ? rs_forward_data : rf_rdata1;
    assign rdata2_fd = sel_rt_forward ? rt_forward_data : rf_rdata2;
    
    // load相关暂停请求：EX段是加载指令，且当前指令的rs/rt是加载指令的目标寄存器
    assign stall_for_load = ex_ram_read & (rs_ex_ok | rt_ex_ok);

    // 译码结果打包：传入EX段的总线
    assign id_to_ex_bus = {
        hilo_op,         // 172:164 乘除法指令标记
        mem_op,          // 163:159 加载指令类型标记
        id_pc,           // 158:127
        inst,            // 126:95
        alu_op,          // 94:83 ALU运算控制信号
        sel_alu_src1,    // 82:80
        sel_alu_src2,    // 79:76
        data_ram_en,     // 75
        data_ram_wen,    // 74:71
        rf_we,           // 70
        rf_waddr,        // 69:65
        sel_rf_res,      // 64 写寄存器数据选择
        rdata1_fd,       // 63:32 第一个操作数
        rdata2_fd        // 31:0 第二个操作数
    };

    // 跳转控制相关信号
    wire br_e;   // 跳转使能（1表示需要跳转）
    wire [31:0] br_addr;
    wire rs_eq_rt;  // rs == rt（用于beq指令）
    wire rs_ge_z;   // rs >= 0（用于bgez/bgezal指令）
    wire rs_gt_z;   // rs > 0（用于bgtz指令）
    wire rs_le_z;   // rs <= 0（用于blez指令）
    wire rs_lt_z;   // rs < 0（用于bltz/bltzal指令）
    wire [31:0] pc_plus_4;
    assign pc_plus_4 = id_pc + 32'h4;

    // 跳转条件判断
    assign rs_eq_rt = (rdata1_fd == rdata2_fd);
    assign rs_ge_z = ~rdata1_fd[31];
    assign rs_gt_z = ($signed(rdata1_fd) > 0);
    assign rs_le_z = (rdata1_fd[31]==1'b1 || rdata1_fd == 32'b0);
    assign rs_lt_z = (rdata1_fd[31]);

    // 跳转使能：满足对应跳转指令的条件时有效
    assign br_e = inst_beq & rs_eq_rt
                | inst_bne & ~rs_eq_rt
                | inst_bgez & rs_ge_z
                | inst_bgezal & rs_ge_z
                | inst_bgtz & rs_gt_z
                | inst_blez & rs_le_z
                | inst_bltz & rs_lt_z
                | inst_bltzal & rs_lt_z
                | inst_j
                | inst_jal
                | inst_jr
                | inst_jalr;

    // 跳转目标地址计算：根据不同跳转指令计算目标地址
    assign br_addr = inst_beq    ? (pc_plus_4 + {{14{inst[15]}}, inst[15:0], 2'b0}) :
                     inst_bne    ? (pc_plus_4 + {{14{inst[15]}}, inst[15:0], 2'b0}) :
                     inst_bgez   ? (pc_plus_4 + {{14{inst[15]}}, inst[15:0], 2'b0}) :
                     inst_bgezal ? (pc_plus_4 + {{14{inst[15]}}, inst[15:0], 2'b0}) :
                     inst_bgtz   ? (pc_plus_4 + {{14{inst[15]}}, inst[15:0], 2'b0}) :
                     inst_blez   ? (pc_plus_4 + {{14{inst[15]}}, inst[15:0], 2'b0}) :
                     inst_bltz   ? (pc_plus_4 + {{14{inst[15]}}, inst[15:0], 2'b0}) :
                     inst_bltzal ? (pc_plus_4 + {{14{inst[15]}}, inst[15:0], 2'b0}) :
                     inst_j      ? {pc_plus_4[31:28],inst[25:0], 2'b0}              :
                     inst_jal    ? {pc_plus_4[31:28],inst[25:0], 2'b0}              : 
                     inst_jr     ? rdata1_fd                                     :
                     inst_jalr   ? rdata1_fd                                     :
                     32'b0;

    // 跳转控制总线：打包跳转使能和目标地址，传入IF段
    assign br_bus = {
        br_e,
        br_addr
    };
 
endmodule