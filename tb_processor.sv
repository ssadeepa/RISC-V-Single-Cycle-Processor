module tb_processor();

  logic        clk;
  logic        rst;

  logic [31:0] write_data, address;
  logic        mem_write;

  // Instantiate the processor to be tested
  processor dut(clk, rst, write_data, address, mem_write);
  
  // Initialize test
  initial begin
    rst <= 1; 
    #22; 
    rst <= 0;
  end

  // Generate clock signal to sequence tests
  always begin
    clk <= 1; 
    #5; 
    clk <= 0;
    #5;
  end

  // Check results
  always @(negedge clk) begin
    if (mem_write) begin
      if (address === 108 && write_data === 32'hABCDE7D5) begin
        $display("Simulation succeeded");
        $stop;
      end
    end
  end
endmodule
