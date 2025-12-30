`include "lib/defines.vh"
// 执行模块（EX段）：接收ID段的控制信号和操作数，完成ALU运算、乘除法、访存地址计算
// 核心功能：ALU运算、乘除法指令实现、加载/存储指令的地址和数据处理、流水线暂停请求生成

module EX(
    input wire clk,
    input wire rst,
    input wire flush,
    input wire [`StallBus-1:0] stall,   // 流水线暂停信号
    input wire [31:0] hi_data,  // hilo寄存器高位数据（来自hilo_reg）
    input wire [31:0] lo_data,  // hilo寄存器低位数据（来自hilo_reg）

    input wire [`ID_TO_EX_WD-1:0] id_to_ex_bus,

    output wire [`EX_TO_MEM_WD-1:0] ex_to_mem_bus,
    output wire stall_for_ex,   // 执行阶段的暂停请求（乘除法多周期导致，输出给CTRL）

    output wire data_sram_en,
    output wire [3:0] data_sram_wen,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata
);

    // 寄存ID段传入的总线（同步时钟，避免亚稳态）
    reg [`ID_TO_EX_WD-1:0] id_to_ex_bus_r;

    // 同步寄存ID段总线：处理复位、冲洗、暂停逻辑
    always @ (posedge clk) begin
        if (rst) begin
            id_to_ex_bus_r <= `ID_TO_EX_WD'b0;
        end
        else if (flush) begin
            id_to_ex_bus_r <= `ID_TO_EX_WD'b0;
        end
        else if (stall[2]==`Stop && stall[3]==`NoStop) begin
            id_to_ex_bus_r <= `ID_TO_EX_WD'b0;
        end
        else if (stall[2]==`NoStop) begin
            id_to_ex_bus_r <= id_to_ex_bus;
        end
    end

    // 解析ID段传入的总线信号
    wire [31:0] ex_pc, inst;
    wire [8:0] hilo_op; // 乘除法相关指令标记
    wire [4:0] mem_op;
    wire [11:0] alu_op;
    wire [2:0] sel_alu_src1;
    wire [3:0] sel_alu_src2;
    wire data_ram_en;
    wire [3:0] data_ram_wen;
    wire rf_we;
    wire [4:0] rf_waddr;
    wire sel_rf_res;    // 写寄存器数据选择
    wire [31:0] rf_rdata1, rf_rdata2;   // 两个操作数（来自ID段前推处理后）
    reg is_in_delayslot;    // 延迟槽标记

    // 总线解析：从寄存后的总线中提取各信号
    assign {
        hilo_op,        // 172:164
        mem_op,         // 163:159 加载指令类型标记
        ex_pc,          // 158:127
        inst,           // 126:95
        alu_op,         // 94:83
        sel_alu_src1,   // 82:80
        sel_alu_src2,   // 79:76
        data_ram_en,    // 75
        data_ram_wen,   // 74:71
        rf_we,          // 70
        rf_waddr,       // 69:65
        sel_rf_res,     // 64
        rf_rdata1,      // 63:32
        rf_rdata2       // 31:0
    } = id_to_ex_bus_r;

    // 立即数扩展：符号扩展、零扩展、sa零扩展（用于ALU操作数）
    wire [31:0] imm_sign_extend, imm_zero_extend, sa_zero_extend;
    assign imm_sign_extend = {{16{inst[15]}},inst[15:0]};   // 立即数符号扩展到32bit
    assign imm_zero_extend = {16'b0, inst[15:0]};   // 立即数零扩展到32bit
    assign sa_zero_extend = {27'b0,inst[10:6]}; // sa字段零扩展到32bit

    // ALU相关信号：操作数、运算结果
    wire [31:0] alu_src1, alu_src2;
    wire [31:0] alu_result;
    wire [31:0] ex_result;
    wire [31:0] hilo_result;
    wire [65:0] hilo_bus;   // hilo寄存器写总线（传入MEM段）

    // 第一个ALU操作数选择：根据sel_alu_src1选择rs/PC/sa零扩展
    assign alu_src1 = sel_alu_src1[1] ? ex_pc :
                      sel_alu_src1[2] ? sa_zero_extend :
                      rf_rdata1;

    // 第二个ALU操作数选择：根据sel_alu_src2选择rt/立即数符号扩展/32'd8/立即数零扩展
    assign alu_src2 = sel_alu_src2[1] ? imm_sign_extend :
                      sel_alu_src2[2] ? 32'd8           :
                      sel_alu_src2[3] ? imm_zero_extend :
                      rf_rdata2;
    
    alu u_alu(
    	.alu_control (alu_op      ),
        .alu_src1    (alu_src1    ),
        .alu_src2    (alu_src2    ),
        .alu_result  (alu_result  )
    );

    //Store Part
    wire inst_sb, inst_sh, inst_sw;
    reg [3:0] data_sram_wen_r;
    reg [31:0] data_sram_wdata_r;

    // 解析存储指令标记（从data_ram_wen中提取）
    assign {
        inst_sb, 
        inst_sh,
        inst_sw
    } = data_ram_wen[2:0];

    // 存储指令数据处理：根据指令类型和地址低位，控制写使能和数据对齐
    always @ (*) begin
        case(1'b1)
            inst_sb:    // 字节存储（sb）：按地址低2bit选择写哪个字节
            begin
                data_sram_wdata_r <= {4{rf_rdata2[7:0]}};
                case(alu_result[1:0])
                    2'b00:
                    begin
                        data_sram_wen_r <= 4'b0001;
                    end
                    2'b01:
                    begin
                        data_sram_wen_r <= 4'b0010;
                    end
                    2'b10:
                    begin
                        data_sram_wen_r <= 4'b0100;
                    end
                    2'b11:
                    begin
                        data_sram_wen_r <= 4'b1000;
                    end
                    default:
                    begin
                        data_sram_wen_r <= 4'b0;
                    end
                endcase
            end
            inst_sh: // 半字存储（sh）：按地址低2bit选择写低半字或高半字
            begin
                data_sram_wdata_r <= {2{rf_rdata2[15:0]}};
                case(alu_result[1:0])
                    2'b00:
                    begin
                        data_sram_wen_r <= 4'b0011;
                    end
                    2'b10:
                    begin
                        data_sram_wen_r <= 4'b1100;
                    end
                    default:
                    begin
                        data_sram_wen_r <= 4'b0000;
                    end
                endcase
            end
            inst_sw: // 字存储（sw）：写4个字节
            begin
                data_sram_wdata_r <= rf_rdata2;
                data_sram_wen_r <= 4'b1111;
            end
            default:// 非存储指令：写使能0，数据0
            begin
                data_sram_wdata_r <= 32'b0;
                data_sram_wen_r <= 4'b0000;
            end
        endcase
    end

    // 数据存储器输出信号赋值：寄存后的写使能、地址、数据
    assign data_sram_en = data_ram_en;
    assign data_sram_wen = data_sram_wen_r;
    assign data_sram_addr = alu_result;     // 存储/加载地址（ALU计算结果）
    assign data_sram_wdata = data_sram_wdata_r;

    // EX段输出到MEM段的总线：打包所有结果和控制信号
    assign ex_to_mem_bus = {
        hilo_bus,       // 146:81
        mem_op,         // 80:76
        ex_pc,          // 75:44
        data_ram_en,    // 43
        data_ram_wen,   // 42:39
        sel_rf_res,     // 38 写寄存器数据选择
        rf_we,          // 37
        rf_waddr,       // 36:32
        ex_result       // 31:0 EX段最终结果
    };

    // HILO Part
    // 乘除法相关指令解析：从hilo_op中提取各指令标记
    wire inst_mfhi, inst_mflo,  inst_mthi,  inst_mtlo;
    wire inst_mult, inst_multu, inst_div,   inst_divu;
    wire inst_mul;

    assign {
        inst_mfhi, inst_mflo, inst_mthi, inst_mtlo,
        inst_mult, inst_multu, inst_div, inst_divu,
        inst_mul
    } = hilo_op;

    // 暂停请求信号：乘除法多周期导致的暂停（stall_for_div/stall_for_mul）
    reg stall_for_div;
    reg stall_for_mul;
    assign stall_for_ex = stall_for_div | stall_for_mul; // 总暂停请求
    
    // 乘法相关信号：结果（64bit）、有符号标记
    wire [63:0] mul_result;
    wire mul_signed; // 1表示有符号乘法（inst_mult），0表示无符号（inst_multu）
    
    // 除法相关信号：结果（64bit）、就绪信号
    wire [63:0] div_result;
    wire div_ready_i;   // 除法器就绪信号（1表示运算完成）

    // 除法器输入信号：操作数、开始信号、有符号标记
    reg [31:0] div_opdata1_o;
    reg [31:0] div_opdata2_o;
    reg div_start_o;
    reg signed_div_o;

    // hilo寄存器写控制信号：写使能、写数据
    wire hi_we, lo_we;
    wire [31:0] hi_result, lo_result;

    wire op_mul  = inst_mul | inst_mult | inst_multu;
    wire op_div  = inst_div | inst_divu;
    
    // hilo写使能：乘除法指令或mthi/mtlo指令时有效
    assign hi_we = inst_mthi | inst_div | inst_divu | inst_mult | inst_multu;
    assign lo_we = inst_mtlo | inst_div | inst_divu | inst_mult | inst_multu;
    
    // hilo写数据：根据指令类型选择数据
    assign hi_result = inst_mthi ? rf_rdata1         :  // mthi：写rs数据到hi
                       op_mul    ? mul_result[63:32] :  // 乘法：hi = 结果高32bit
                       op_div    ? div_result[63:32] :  // 除法：hi = 余数
                       32'b0;
    assign lo_result = inst_mtlo ? rf_rdata1        :   // mtlo：写rs数据到lo
                       op_mul    ? mul_result[31:0] :   // 乘法：lo = 结果低32bit
                       op_div    ? div_result[31:0] :   // 除法：lo = 商
                       32'b0;
                       
    // hilo读结果：mfhi读hi，mflo读lo
    assign hilo_result = inst_mfhi ? hi_data :
                         inst_mflo ? lo_data :
                         32'b0;

    // hilo写总线：打包写使能和写数据，传入MEM段
    assign hilo_bus = {
        hi_we, 
        lo_we,
        hi_result,
        lo_result
    };

    // EX段最终结果：mfhi/mflo指令取hilo结果，其他取ALU结果
    assign ex_result = (inst_mfhi | inst_mflo) ? hilo_result :
                       alu_result;
    
    // MUL part
    assign mul_signed = inst_mult;

    mul u_mul(
    	.clk        (clk            ),
        .resetn     (~rst           ),
        .mul_signed (mul_signed     ),
        .ina        (rf_rdata1      ), // 涔樻硶婧愭搷浣滄暟1
        .inb        (rf_rdata2      ), // 涔樻硶婧愭搷浣滄暟2
        .result     (mul_result     )  // 涔樻硶缁撴灉 64bit
    );

    // 乘法器暂停控制：单周期乘法（实际可扩展为多周期，此处用cnt简单控制）
    reg cnt;
    reg next_cnt;

    always @ (posedge clk) begin
        if (rst) begin
            cnt <= 1'b0;
        end
        else begin
            cnt <= next_cnt;
        end
    end

    always @ (*) begin
        if (rst) begin
            stall_for_mul <= 1'b0;
            next_cnt <= 1'b0;
        end
        else if ((inst_mult | inst_multu) & ~cnt) begin
            stall_for_mul <= 1'b1;  // 开始乘法，暂停流水线
            next_cnt <= 1'b1;
        end
        else if ((inst_mult | inst_multu) & cnt) begin
            stall_for_mul <= 1'b0;  // 乘法完成，恢复流水线
            next_cnt <= 1'b0;
        end
        else begin
            stall_for_mul <= 1'b0;
            next_cnt <= 1'b0;
        end
    end 
    
    // DIV part
    div u_div(
    	.rst          (rst           ),
        .clk          (clk           ),
        .signed_div_i (signed_div_o  ),
        .opdata1_i    (div_opdata1_o ), // 被除数（rs）
        .opdata2_i    (div_opdata2_o ), // 除数（rt）
        .start_i      (div_start_o   ), // 除法开始信号
        .annul_i      (1'b0          ), // 除法取消信号
        .result_o     (div_result    ), // 除法结果（64bit：商[31:0]，余数[63:32]）
        .ready_o      (div_ready_i   )  // 除法就绪信号
    );

    // 除法器控制逻辑：检测除法指令，启动除法，控制流水线暂停
    always @ (*) begin
        if (rst) begin
            stall_for_div <= `NoStop;
            div_opdata1_o <= `ZeroWord;
            div_opdata2_o <= `ZeroWord;
            div_start_o <= `DivStop;
            signed_div_o <= 1'b0;
        end
        else begin
            stall_for_div <= `NoStop;
            div_opdata1_o <= `ZeroWord;
            div_opdata2_o <= `ZeroWord;
            div_start_o <= `DivStop;
            signed_div_o <= 1'b0;
            case ({inst_div, inst_divu})
                2'b10:  // 有符号除法（inst_div）
                begin
                    if (div_ready_i == `DivResultNotReady) begin
                        div_opdata1_o <= rf_rdata1;
                        div_opdata2_o <= rf_rdata2;
                        div_start_o <= `DivStart;
                        signed_div_o <= 1'b1;
                        stall_for_div <= `Stop;  // 除法未完成，暂停流水线
                    end
                    else if (div_ready_i == `DivResultReady) begin
                        div_opdata1_o <= rf_rdata1;
                        div_opdata2_o <= rf_rdata2;
                        div_start_o <= `DivStop;
                        signed_div_o <= 1'b1;
                        stall_for_div <= `NoStop;   // 除法完成，恢复流水线
                    end
                    else begin
                        div_opdata1_o <= `ZeroWord;
                        div_opdata2_o <= `ZeroWord;
                        div_start_o <= `DivStop;
                        signed_div_o <= 1'b0;
                        stall_for_div <= `NoStop;
                    end
                end
                2'b01:  // 无符号除法（inst_divu）
                begin
                    if (div_ready_i == `DivResultNotReady) begin
                        div_opdata1_o <= rf_rdata1;
                        div_opdata2_o <= rf_rdata2;
                        div_start_o <= `DivStart;
                        signed_div_o <= 1'b0;
                        stall_for_div <= `Stop;
                    end
                    else if (div_ready_i == `DivResultReady) begin
                        div_opdata1_o <= rf_rdata1;
                        div_opdata2_o <= rf_rdata2;
                        div_start_o <= `DivStop;
                        signed_div_o <= 1'b0;
                        stall_for_div <= `NoStop;
                    end
                    else begin
                        div_opdata1_o <= `ZeroWord;
                        div_opdata2_o <= `ZeroWord;
                        div_start_o <= `DivStop;
                        signed_div_o <= 1'b0;
                        stall_for_div <= `NoStop;
                    end
                end
                default:    // 非除法指令，无操作
                begin
                end
            endcase
        end
    end

    // mul_result 鍜? div_result 鍙互鐩存帴浣跨敤*/
    
    
endmodule