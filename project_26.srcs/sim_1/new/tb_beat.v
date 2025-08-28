`timescale 1ns / 1ps

module beat_tb;

    reg clk = 0;
    reg rst = 1;
    reg ext_int = 0;
    reg int_int = 0;

    wire [7:0] bpm, hrv, avg, irr;
    wire done;
    wire [4:0] state_out;
    wire fsm_busy_out;
    wire interrupt_pending_out;
    wire [1:0] irq_vector_out;

    // Clock generation: 10ns period (100MHz)
    always #5 clk = ~clk;

    // DUT instance
    beat uut (
        .clk(clk),
        .rst(rst),
        .ext_int(ext_int),
        .int_int(int_int),
        .bpm(bpm),
        .hrv(hrv),
        .avg(avg),
        .irr(irr),
        .done(done),
        .state_out(state_out),
        .fsm_busy_out(fsm_busy_out),
        .interrupt_pending_out(interrupt_pending_out),
        .irq_vector_out(irq_vector_out)
    );

    initial begin
        $display("Starting simulation...");
        $dumpfile("beat_tb.vcd");     // For GTKWave
        $dumpvars(0, beat_tb);

        // Reset
        #20 rst = 0;

        // Let FSM operate a bit
        #380;

        // Trigger external interrupt (bpm reset)
        $display(">>> External interrupt triggered at %t", $time);
        ext_int = 1;
        #10;
        ext_int = 0;

        // Wait more and trigger internal interrupt (hrv increment)
        #0;
        $display(">>> Internal interrupt triggered at %t", $time);
        int_int = 1;
        #10;
        int_int = 0;

        // Wait for FSM to finish
        #300;

        $display("Final BPM = %d", bpm);
        $display("Final HRV = %d", hrv);
        $display("Final AVG = %d", avg);
        $display("Final IRR = %d", irr);

        $finish;
    end

endmodule
