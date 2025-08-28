`timescale 1ns / 1ps

// ------------------- RAM Module -------------------
module ram(
    input clk,
    input we,
    input rst,
    input [5:0] addr,
    input [7:0] din,
    output reg [7:0] dout
);
    reg [7:0] mem [63:0];

    initial begin
        mem[0] = 8'd85;  // IBI1 = 850ms
        mem[1] = 8'd86;  // IBI2 = 860ms
        mem[2] = 8'd94;
        mem[3] = 8'd78;
    end

    always @(posedge clk) begin
        if (rst)
            dout <= 8'h00;
        else if (we)
            mem[addr] <= din;
        else
            dout <= mem[addr];
    end
endmodule


// ------------------- Pipelined ALU -------------------
module pipelined_alu (
    input wire clk,
    input wire rst,
    input wire [7:0] a, b,
    input wire [1:0] opcode,
    output reg [7:0] result,
    output reg flag
);
    localparam OP_BPM = 2'b00,
               OP_HRV = 2'b01,
               OP_AVG = 2'b10,
               OP_IRR = 2'b11;

    reg [7:0] a_reg, b_reg;
    reg [1:0] opcode_reg;
    reg [7:0] alu_result_stage2;
    reg flag_stage2;
    reg [7:0] diff, temp;

    // Stage 1
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            a_reg <= 0; b_reg <= 0; opcode_reg <= 0;
        end else begin
            a_reg <= a;
            b_reg <= b;
            opcode_reg <= opcode;
        end
    end

    // Stage 2
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            alu_result_stage2 <= 0;
            flag_stage2 <= 0;
        end else begin
            case (opcode_reg)
                OP_BPM: begin
                    if (a_reg != 0) begin
                        alu_result_stage2 <= 6000 / a_reg;
                        flag_stage2 <= (6000 / a_reg < 50 || 6000 / a_reg > 100);
                    end else begin
                        alu_result_stage2 <= 0;
                        flag_stage2 <= 1;
                    end
                end
                OP_HRV: begin
                    temp = (a_reg > b_reg) ? (a_reg - b_reg) : (b_reg - a_reg);
                    alu_result_stage2 <= temp;
                    flag_stage2 <= (temp < 8);
                end
                OP_AVG: begin
                    alu_result_stage2 <= (a_reg + b_reg) >> 1;
                    flag_stage2 <= 0;
                end
                OP_IRR: begin
                    diff = (a_reg > b_reg) ? (a_reg - b_reg) : (b_reg - a_reg);
                    alu_result_stage2 <= diff;
                    flag_stage2 <= (diff > 15);
                end
                default: begin
                    alu_result_stage2 <= 0;
                    flag_stage2 <= 0;
                end
            endcase
        end
    end

    // Stage 3
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            result <= 0;
            flag <= 0;
        end else begin
            result <= alu_result_stage2;
            flag <= flag_stage2;
        end
    end
endmodule


// ------------------- Interrupt Controller -------------------
module interrupt_controller(
    input wire ext_int,
    input wire int_int,
    output reg [1:0] irq_vector,
    output reg irq_valid
);
    always @(*) begin
        if (ext_int) begin
            irq_vector = 2'b01;
            irq_valid = 1;
        end else if (int_int) begin
            irq_vector = 2'b10;
            irq_valid = 1;
        end else begin
            irq_vector = 2'b00;
            irq_valid = 0;
        end
    end
endmodule


// ------------------- FSM with Latched Interrupt Logic -------------------
module beat(
    input clk,
    input rst,
    input ext_int,
    input int_int,
    output [4:0] state_out,
    output fsm_busy_out,
    output interrupt_pending_out,
    output [1:0] irq_vector_out,
    output reg [7:0] bpm, hrv, avg, irr,
    output reg done
);
    reg [4:0] state, saved_state;
    reg [5:0] addr, index;
    reg [7:0] a, b;
    reg [1:0] opcode;
    reg [7:0] alu_a, alu_b;
    reg we = 0;
    reg fsm_busy;
    reg interrupt_pending;
    reg [1:0] active_irq_vector;
    reg interrupt_pending_ext, interrupt_pending_int;

    wire [7:0] ram_dout;
    wire [7:0] alu_result;
    wire alu_flag;
    wire [1:0] irq_vector;
    wire irq_valid;

    ram mem_inst (
        .clk(clk), .we(we), .rst(rst), .addr(addr), .din(8'd0), .dout(ram_dout)
    );

    pipelined_alu alu_inst (
        .clk(clk), .rst(rst), .a(alu_a), .b(alu_b), .opcode(opcode), .result(alu_result), .flag(alu_flag)
    );

    interrupt_controller irq_ctrl (
        .ext_int(ext_int), .int_int(int_int), .irq_vector(irq_vector), .irq_valid(irq_valid)
    );

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= 0; addr <= 0; index <= 0;
            a <= 0; b <= 0; opcode <= 0; alu_a <= 0; alu_b <= 0;
            bpm <= 0; hrv <= 0; avg <= 0; irr <= 0;
            fsm_busy <= 0; interrupt_pending <= 0; done <= 0;
            interrupt_pending_ext <= 0; interrupt_pending_int <= 0;
        end else begin
            // Latch interrupts
            if (ext_int) interrupt_pending_ext <= 1;
            if (int_int) interrupt_pending_int <= 1;

            if (!interrupt_pending && !fsm_busy && (interrupt_pending_ext || interrupt_pending_int)) begin
                interrupt_pending <= 1;
                saved_state <= state;
                if (interrupt_pending_ext) begin
                    active_irq_vector <= 2'b01;
                    state <= 5'd30;
                end else if (interrupt_pending_int) begin
                    active_irq_vector <= 2'b10;
                    state <= 5'd31;
                end
            end else begin
                case (state)
                    5'd0: begin addr <= index; state <= 1; irr <= alu_result; fsm_busy <= 0; end
                    5'd1: state <= 2;
                    5'd2: begin a <= ram_dout; addr <= index + 1; state <= 3; end
                    5'd3: state <= 4;
                    5'd4: begin b <= ram_dout; state <= 5; end
                    5'd5: begin opcode <= 2'b00; alu_a <= a; alu_b <= b; fsm_busy <= 1; state <= 6; end
                    5'd6: state <= 7;
                    5'd7: state <= 8;
                    5'd8: begin opcode <= 2'b01; alu_a <= a; alu_b <= b; state <= 9; end
                    5'd9: state <= 10;
                    5'd10: begin bpm <= alu_result; state <= 11; end
                    5'd11: begin opcode <= 2'b10; alu_a <= a; alu_b <= b; state <= 12; end
                    5'd12: state <= 13;
                    5'd13: begin hrv <= alu_result; state <= 14; end
                    5'd14: begin opcode <= 2'b11; alu_a <= a; alu_b <= b; state <= 15; end
                    5'd15: state <= 16;
                    5'd16: begin avg <= alu_result; state <= 17; end
                    5'd17: begin
                        if (index >= 6'd3) begin done <= 1; state <= 17;
                        end else begin index <= index + 2; state <= 0; end
                    end
                    5'd30: begin bpm <= 0; interrupt_pending <= 0; interrupt_pending_ext <= 0; state <= saved_state; end
                    5'd31: begin hrv <= hrv + 1; interrupt_pending <= 0; interrupt_pending_int <= 0; state <= saved_state; end
                endcase
            end
        end
    end

    assign state_out = state;
    assign fsm_busy_out = fsm_busy;
    assign interrupt_pending_out = interrupt_pending;
    assign irq_vector_out = irq_vector;

endmodule
