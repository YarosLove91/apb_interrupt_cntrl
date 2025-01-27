`timescale 1ns/1ps

module interrupt_apb_pulp_pwm_tb;

    // DUT input/output REG
    logic                   test_mode_i;
    logic                   event_fifo_valid_i;
    logic                   event_fifo_fulln_o;
    logic [EVT_ID_WIDTH-1:0] event_fifo_data_i;
    logic [31:0]            events_i;
    logic [4:0]             core_irq_id_o;
    logic                   core_irq_req_o;
    logic                   core_irq_ack_i;
    logic                   core_irq_sec_o;
    logic [4:0]             core_irq_id_i;
    logic                   core_secure_mode_i;
    logic                   core_clock_en_o;
    logic                   fetch_en_o;

    // APB signals
    reg         PCLK;
    reg         PRESETn;
    reg  [31:0] PADDR;
    reg         PWRITE;
    reg         PSEL;
    reg         PENABLE;
    reg  [31:0] PWDATA;
    wire [31:0] PRDATA;
    wire        PREADY;

    `include "params_pulp.vh"
    `include "apb_tasks.vh"

    // Instantiate the APB slave interface
    APB #(.ADDR_WIDTH(32), .DATA_WIDTH(32)) apb_slave_if();

    // Instantiate the DUT
    interrupt_controller_apb #(.PER_ID_WIDTH(5),
                               .EVT_ID_WIDTH(8),
                               .ENA_SEC_IRQ(1)
    ) DUT (
        .clk_i              (PCLK              ),
        .rst_ni             (PRESETn           ),
        .test_mode_i        (test_mode_i       ),
        .event_fifo_valid_i (event_fifo_valid_i),
        .event_fifo_data_i  (event_fifo_data_i ),
        .events_i           (events_i          ),
        .core_irq_ack_i     (core_irq_ack_i    ),
        .core_irq_id_i      (core_irq_id_i     ),
        .core_secure_mode_i (core_secure_mode_i),
        .apb_slave          (apb_slave_if.Slave)
    );

    // APB master drives
    assign apb_slave_if.paddr = PADDR;
    assign apb_slave_if.pwrite = PWRITE;
    assign apb_slave_if.psel = PSEL;
    assign apb_slave_if.penable = PENABLE;
    assign apb_slave_if.pwdata = PWDATA;

    // APB slave drives
    assign PREADY = apb_slave_if.pready;
    assign PRDATA = apb_slave_if.prdata;

    // Dump the signals
    initial begin
        $dumpfile("Interrupt_cntrl.vcd");
        $dumpvars(0, interrupt_apb_pulp_pwm_tb);
    end

    // Clock and reset generator
    initial begin
        PCLK = 1'b0;
        PRESETn = 1'b0;
        #25;
        PRESETn = 1'b1; // Release reset
    end

    initial begin
        PCLK <= 1'bx;
        PRESETn <= 1'bx;
        // Power ON
        #25;
        -> power_on;
        PSEL <= 0;
        PENABLE <= 0;
    end

    always #(CLK_PERIOD/2) PCLK <= ~PCLK;

    initial begin
        @(power_on);
        PRESETn <= 1'b0;
        PCLK <= 1'b0;
        #10;
        @(posedge PCLK);
        PRESETn <= 1'b1;
        -> reset_done;
    end

    // External Events
    event extern_clk_start;
    initial begin
        ctr_in = 0;
        @(extern_clk_start);
        repeat(50)
            #(CLK_PERIOD*17.3/2) ctr_in = !ctr_in;
    end

    always #(CLK_PERIOD/2) PCLK = ~PCLK;

    // Test Cases
    initial begin
        // Test case 1: Basic interrupt handling
        test_basic_interrupt_handling();

        // test_case_2();
        // test_case_3();
        // test_case_4();
    end

    // Task for basic interrupt handling
    task test_basic_interrupt_handling;
        begin
            @(posedge PRESETn);                    // Wait for reset to be released
            // Configure the APB registers for interrupt handling
            apb_w_wr(REG_MASK_ADDR, 32'hFFFFFFFF); // Enable all interrupts
            apb_w_wr(REG_INT_SET_ADDR, 32'h1);     // Set interrupt for event 0

            // Simulate an event
            events_i = 32'h0000_0001;               // Trigger event 0
            event_fifo_valid_i = 1;
            event_fifo_data_i = 8'h00;             // Event ID
            #10; // Wait for a short time
            event_fifo_valid_i = 0;                // Clear valid signal

            // Wait for the interrupt request
            @(posedge core_irq_req_o);
            $display("Test 1: Basic interrupt handling passed.");
        end
    endtask

endmodule