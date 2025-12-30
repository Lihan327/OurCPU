`include "defines.vh"
// 通用寄存器堆模块：实现32个32位通用整数寄存器的读（双端口）写（单端口）操作
// 遵循MIPS32架构，$0寄存器恒为0，不可写入
module regfile(
    input wire clk, // 时钟信号，写操作在上升沿触发
    input wire [4:0] raddr1,
    output wire [31:0] rdata1,
    input wire [4:0] raddr2,
    output wire [31:0] rdata2,
    
    input wire we,
    input wire [4:0] waddr,
    input wire [31:0] wdata
);
    reg [31:0] reg_array [31:0];    // 定义32个32位通用寄存器（reg_array[0]为$0，恒为0）
    // write
    always @ (posedge clk) begin
        if (we && waddr != 5'b0) begin
            reg_array[waddr] <= wdata;
        end
    end

    // read out 1
    assign rdata1 = (raddr1 == 5'b0) ? 32'b0 :
                    reg_array[raddr1];
    // read out2
    assign rdata2 = (raddr2 == 5'b0) ? 32'b0 :
                    reg_array[raddr2];
endmodule