`include "lib/defines.vh"
// 访存模块（MEM段）：接收EX段的结果，完成数据存储器访问（加载/存储）
// 核心功能：存储指令写数据到存储器、加载指令从存储器读数据、数据对齐处理

module MEM(
    input wire clk,
    input wire rst,
    input wire flush,
    input wire [`StallBus-1:0] stall,
    input wire [`EX_TO_MEM_WD-1:0] ex_to_mem_bus,
    input wire [31:0] data_sram_rdata,  // 从数据存储器读取的数据

    output wire [`MEM_TO_WB_WD-1:0] mem_to_wb_bus   // 访存结果传入WB段的总线
);

    // 寄存EX段传入的总线（同步时钟，避免亚稳态）
    reg [`EX_TO_MEM_WD-1:0] ex_to_mem_bus_r;

    // 同步寄存EX段总线：处理复位、冲洗、暂停逻辑
    always @ (posedge clk) begin
        if (rst) begin
            ex_to_mem_bus_r <= `EX_TO_MEM_WD'b0;
        end
        else if (flush) begin
            ex_to_mem_bus_r <= `EX_TO_MEM_WD'b0;
        end
        else if (stall[3]==`Stop && stall[4]==`NoStop) begin
            ex_to_mem_bus_r <= `EX_TO_MEM_WD'b0;
        end
        else if (stall[3]==`NoStop) begin
            ex_to_mem_bus_r <= ex_to_mem_bus;
        end
    end

    // 解析EX段传入的总线信号
    wire [65:0] hilo_bus;
    wire [31:0] mem_pc;
    wire data_ram_en;
    wire [3:0] data_ram_wen;
    wire sel_rf_res;    // 写寄存器数据选择（ALU结果/加载数据）
    
    wire [4:0] mem_op;
    wire rf_we;
    wire [4:0] rf_waddr;
    wire [31:0] rf_wdata;
    wire [31:0] ex_result;
    wire [31:0] mem_result; // 加载指令从存储器读出的数据（处理后）

    // 总线解析：从寄存后的总线中提取各信号
    assign {
        hilo_bus,       // 146:81
        mem_op,         // 80:76
        mem_pc,         // 75:44
        data_ram_en,    // 43
        data_ram_wen,   // 42:39
        sel_rf_res,     // 38
        rf_we,          // 37 寄存器堆写使能
        rf_waddr,       // 36:32
        ex_result       // 31:0
    } =  ex_to_mem_bus_r;

    // 加载指令标记：从mem_op中提取各加载指令类型
    wire inst_lb, inst_lbu, inst_lh, inst_lhu, inst_lw;

    assign {
        inst_lb,
        inst_lbu, 
        inst_lh, 
        inst_lhu, 
        inst_lw
    } = mem_op;

    // 加载指令数据处理：根据指令类型和地址低位，进行数据对齐和符号/零扩展
    reg [31:0] mem_result_r;
    always @ (*) begin
        case(1'b1)
            inst_lb:    // 字节加载（lb）：符号扩展到32bit
            begin
                case(ex_result[1:0])    // 根据地址低2bit选择读取哪个字节
                    2'b00:
                    begin
                        mem_result_r <= {{24{data_sram_rdata[7]}},data_sram_rdata[7:0]};
                    end
                    2'b01:
                    begin
                        mem_result_r <= {{24{data_sram_rdata[15]}},data_sram_rdata[15:8]};
                    end
                    2'b10:
                    begin
                        mem_result_r <= {{24{data_sram_rdata[23]}},data_sram_rdata[23:16]};
                    end
                    2'b11:
                    begin
                        mem_result_r <= {{24{data_sram_rdata[31]}},data_sram_rdata[31:24]};
                    end
                    default:
                    begin
                        mem_result_r <= 32'b0;
                    end
                endcase
            end
            inst_lbu:   // 无符号字节加载（lbu）：零扩展到32bit
            begin
                case(ex_result[1:0])
                    2'b00:
                    begin
                        mem_result_r <= {{24{1'b0}},data_sram_rdata[7:0]};
                    end
                    2'b01:
                    begin
                        mem_result_r <= {{24{1'b0}},data_sram_rdata[15:8]};
                    end
                    2'b10:
                    begin
                        mem_result_r <= {{24{1'b0}},data_sram_rdata[23:16]};
                    end
                    2'b11:
                    begin
                        mem_result_r <= {{24{1'b0}},data_sram_rdata[31:24]};
                    end
                    default:
                    begin
                        mem_result_r <= 32'b0;
                    end
                endcase
            end
            inst_lh:    // 半字加载（lh）：符号扩展到32bit
            begin
                case(ex_result[1:0])
                    2'b00:
                    begin
                        mem_result_r <= {{16{data_sram_rdata[15]}},data_sram_rdata[15:0]};
                    end
                    
                    2'b10:
                    begin
                        mem_result_r <= {{16{data_sram_rdata[31]}},data_sram_rdata[31:16]};
                    end
                    default:
                    begin
                        mem_result_r <= 32'b0;
                    end
                endcase
            end
            inst_lhu:   // 无符号半字加载（lhu）：零扩展到32bit
            begin
                case(ex_result[1:0])
                    2'b00:
                    begin
                        mem_result_r <= {{16{1'b0}},data_sram_rdata[15:0]};
                    end
                    2'b10:
                    begin
                        mem_result_r <= {{16{1'b0}},data_sram_rdata[31:16]};
                    end
                    default:
                    begin
                        mem_result_r <= 32'b0;
                    end
                endcase
            end
            inst_lw:    // 字加载（lw）：直接读取32bit数据
            begin
                mem_result_r <= data_sram_rdata;
            end
            default:    // 非加载指令：结果为0
            begin
                mem_result_r <= 32'b0;
            end
        endcase
    end

    // 加载数据赋值：处理后的存储器数据
    assign mem_result = mem_result_r;
   // 写寄存器数据选择：sel_rf_res=1→加载数据，0→EX段结果
    assign rf_wdata = sel_rf_res ? mem_result : ex_result;

    // 访存结果打包：传入WB段的总线（含hilo总线、写寄存器信号、数据等）
    assign mem_to_wb_bus = {
        hilo_bus,        // 135:70
        mem_pc,          // 69:38
        rf_we,           // 37
        rf_waddr,        // 36:32
        rf_wdata         // 31:0
    };

endmodule