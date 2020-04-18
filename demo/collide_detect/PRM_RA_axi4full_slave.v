//////////////////////////////////////////////////////////////////////////////////
// Company:    
// Engineer: 29505
// Create Date: 2019-07-14 19:06:08
// Last Modified by:   29505
// Last Modified time: 2019-07-17 22:53:32
// Email: 295054118@whut.edu.cn
// page:  
// Design Name: PRM_RA_axi4full_slave.v  
// Module Name: PRM_RA_axi4full_slave
// Project Name:  
// Target Devices:  
// Tool Versions:  
// Description:  
// 
// Dependencies:   
// 
// Revision:  
// Revision  
// Additional Comments:   
// 
// 
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// Company:   
// Engineer: Ruige_Lee
// Create Date: 2019-07-14 15:55:37
// Last Modified by:   Ruige_Lee
// Last Modified time: 2019-07-14 16:49:37
// Email: 295054118@whut.edu.cn
// page: https://whutddk.github.io/
// Design Name: PRM_RA_axi4full_slave.v  
// Module Name: PRM_RA_axi4full_slave
// Project Name:   
// Target Devices:   
// Tool Versions:   
// Description:   
// 
// Dependencies:   
// 
// Revision:  
// Revision:    -   
// Additional Comments:  
// 
//
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// Company:   
// Engineer: Ruige_Lee
// Create Date: 2019-04-01 17:02:49
// Last Modified by:   Ruige_Lee
// Last Modified time: 2019-04-02 19:36:05
// Email: 295054118@whut.edu.cn
// Design Name:   
// Module Name: PRM_RA_axi4full_slave
// Project Name:   
// Target Devices:   
// Tool Versions:   
// Description:   
// 
// Dependencies:   
// 
// Revision:  
// Revision:    -   
// Additional Comments:  
// 
//////////////////////////////////////////////////////////////////////////////////

// generate from vivado 2018.3


`timescale 1 ns / 1 ps

module PRM_RA_axi4full_slave #
	(
		// Width of ID for for write address, write data, read address and read data
		parameter integer C_S_AXI_ID_WIDTH	= 1,
		// Width of S_AXI data bus
		parameter integer C_S_AXI_DATA_WIDTH	= 32,
		// Width of S_AXI address bus
		parameter integer C_S_AXI_ADDR_WIDTH	= 32
	)
	(
		input wire  S_AXI_ACLK,// Global Clock Signal
		input wire  S_AXI_ARESETN,// Global Reset Signal. This Signal is Active LOW
		input wire [C_S_AXI_ID_WIDTH-1 : 0] S_AXI_AWID,// Write Address ID
		input wire [C_S_AXI_ADDR_WIDTH-1 : 0] S_AXI_AWADDR,// Write address
		input wire [7 : 0] S_AXI_AWLEN,// Burst length. The burst length gives the exact number of transfers in a burst
		input wire [2 : 0] S_AXI_AWSIZE,// Burst size. This signal indicates the size of each transfer in the burst
		input wire [1 : 0] S_AXI_AWBURST,// Burst type. The burst type and the size information, determine how the address for each transfer within the burst is calculated.
		input wire  S_AXI_AWLOCK,// Lock type. Provides additional information about the atomic characteristics of the transfer.
		input wire [3 : 0] S_AXI_AWCACHE,// Memory type. This signal indicates how transactions are required to progress through a system.
		input wire [2 : 0] S_AXI_AWPROT,// Protection type. This signal indicates the privilege and security level of the transaction, and whether the transaction is a data access or an instruction access.
		input wire [3 : 0] S_AXI_AWQOS,// Quality of Service, QoS identifier sent for each	write transaction.
		// input wire [3 : 0] S_AXI_AWREGION,// Region identifier. Permits a single physical interface on a slave to be used for multiple logical interfaces.
		input wire  S_AXI_AWVALID,// Write address valid. This signal indicates that the channel is signaling valid write address and control information.
		output wire  S_AXI_AWREADY,// Write address ready. This signal indicates that the slave is ready to accept an address and associated control signals.
		input wire [C_S_AXI_DATA_WIDTH-1 : 0] S_AXI_WDATA,// Write Data
		input wire [(C_S_AXI_DATA_WIDTH/8)-1 : 0] S_AXI_WSTRB,// Write strobes. This signal indicates which byte lanes hold valid data. There is one write strobe bit for each eight bits of the write data bus.
		input wire  S_AXI_WLAST,// Write last. This signal indicates the last transfer in a write burst.
		input wire  S_AXI_WVALID,// Write valid. This signal indicates that valid write data and strobes are available.
		output wire  S_AXI_WREADY,// Write ready. This signal indicates that the slave can accept the write data.
		output wire [C_S_AXI_ID_WIDTH-1 : 0] S_AXI_BID,// Response ID tag. This signal is the ID tag of the write response.
		output wire [1 : 0] S_AXI_BRESP,// Write response. This signal indicates the status of the write transaction.
		output wire  S_AXI_BVALID,// Write response valid. This signal indicates that the channel is signaling a valid write response.
		input wire  S_AXI_BREADY,// Response ready. This signal indicates that the master can accept a write response.
		input wire [C_S_AXI_ID_WIDTH-1 : 0] S_AXI_ARID,// Read address ID. This signal is the identification tag for the read address group of signals.
		input wire [C_S_AXI_ADDR_WIDTH-1 : 0] S_AXI_ARADDR,// Read address. This signal indicates the initial address of a read burst transaction.
		input wire [7 : 0] S_AXI_ARLEN,// Burst length. The burst length gives the exact number of transfers in a burst
		input wire [2 : 0] S_AXI_ARSIZE,// Burst size. This signal indicates the size of each transfer in the burst
		input wire [1 : 0] S_AXI_ARBURST,// Burst type. The burst type and the size information, determine how the address for each transfer within the burst is calculated.
		input wire  S_AXI_ARLOCK,// Lock type. Provides additional information about the atomic characteristics of the transfer.
		input wire [3 : 0] S_AXI_ARCACHE,// Memory type. This signal indicates how transactions are required to progress through a system.
		input wire [2 : 0] S_AXI_ARPROT,// Protection type. This signal indicates the privilege and security level of the transaction, and whether the transaction is a data access or an instruction access.
		input wire [3 : 0] S_AXI_ARQOS,// Quality of Service, QoS identifier sent for each read transaction.
		// input wire [3 : 0] S_AXI_ARREGION,// Region identifier. Permits a single physical interface on a slave to be used for multiple logical interfaces.
		input wire  S_AXI_ARVALID,// Write address valid. This signal indicates that the channel is signaling valid read address and control information.
		output wire  S_AXI_ARREADY,// Read address ready. This signal indicates that the slave is ready to accept an address and associated control signals.
		output wire [C_S_AXI_ID_WIDTH-1 : 0] S_AXI_RID,// Read ID tag. This signal is the identification tag for the read data group of signals generated by the slave.
		output wire [C_S_AXI_DATA_WIDTH-1 : 0] S_AXI_RDATA,// Read Data
		output wire [1 : 0] S_AXI_RRESP,// Read response. This signal indicates the status of the read transfer.
		output wire  S_AXI_RLAST,// Read last. This signal indicates the last transfer in a read burst.
		output wire  S_AXI_RVALID,// Read valid. This signal indicates that the channel is signaling the required read data.
		input wire  S_AXI_RREADY,// Read ready. This signal indicates that the master can accept the read data and response information.
	


		// user 
		input clear,
		output [8191:0] edgeWire


	);

	// AXI4FULL signals
	reg [C_S_AXI_ADDR_WIDTH-1 : 0] 	axi_awaddr;
	reg  	axi_awready;
	reg  	axi_wready;
	reg [1 : 0] 	axi_bresp;
	reg  	axi_bvalid;
	reg [C_S_AXI_ADDR_WIDTH-1 : 0] 	axi_araddr;
	reg  	axi_arready;
	reg [C_S_AXI_DATA_WIDTH-1 : 0] 	axi_rdata;
	reg [1 : 0] 	axi_rresp;
	reg  	axi_rlast;
	reg  	axi_rvalid;
	// aw_wrap_en determines wrap boundary and enables wrapping
	wire aw_wrap_en;
	// ar_wrap_en determines wrap boundary and enables wrapping
	wire ar_wrap_en;
	// aw_wrap_size is the size of the write transfer, the
	// write address wraps to a lower address if upper address
	// limit is reached
	wire [31:0]  aw_wrap_size ; 
	// ar_wrap_size is the size of the read transfer, the
	// read address wraps to a lower address if upper address
	// limit is reached
	wire [31:0]  ar_wrap_size ; 
	// The axi_awv_awr_flag flag marks the presence of write address valid
	reg axi_awv_awr_flag;
	//The axi_arv_arr_flag flag marks the presence of read address valid
	reg axi_arv_arr_flag; 
	// The axi_awlen_cntr internal write address counter to keep track of beats in a burst transaction
	reg [7:0] axi_awlen_cntr;
	//The axi_arlen_cntr internal read address counter to keep track of beats in a burst transaction
	reg [7:0] axi_arlen_cntr;
	reg [1:0] axi_arburst;
	reg [1:0] axi_awburst;
	reg [7:0] axi_arlen;
	reg [7:0] axi_awlen;
	//local parameter for addressing 32 bit / 64 bit C_S_AXI_DATA_WIDTH
	//ADDR_LSB is used for addressing 32/64 bit registers/memories
	//ADDR_LSB = 2 for 32 bits (n downto 2) 
	//ADDR_LSB = 3 for 42 bits (n downto 3)

	localparam integer ADDR_LSB = (C_S_AXI_DATA_WIDTH/32)+ 1;
	localparam integer OPT_MEM_ADDR_BITS = 7;
	localparam integer USER_NUM_MEM = 1;
	//----------------------------------------------
	//-- Signals for user logic memory space example
	//------------------------------------------------
	wire [OPT_MEM_ADDR_BITS:0] mem_address;
	wire [USER_NUM_MEM-1:0] mem_select;
	reg [C_S_AXI_DATA_WIDTH-1:0] mem_data_out[0 : USER_NUM_MEM-1];

	genvar i;
	genvar j;
	genvar mem_byte_index;

	// I/O Connections assignments

	assign S_AXI_AWREADY	= axi_awready;
	assign S_AXI_WREADY	= axi_wready;
	assign S_AXI_BRESP	= axi_bresp;
	assign S_AXI_BVALID	= axi_bvalid;
	assign S_AXI_ARREADY	= axi_arready;
	assign S_AXI_RDATA	= axi_rdata;
	assign S_AXI_RRESP	= axi_rresp;
	assign S_AXI_RLAST	= axi_rlast;
	assign S_AXI_RVALID	= axi_rvalid;
	assign S_AXI_BID = S_AXI_AWID;
	assign S_AXI_RID = S_AXI_ARID;
	assign  aw_wrap_size = (C_S_AXI_DATA_WIDTH/8 * (axi_awlen)); 
	assign  ar_wrap_size = (C_S_AXI_DATA_WIDTH/8 * (axi_arlen)); 
	assign  aw_wrap_en = ((axi_awaddr & aw_wrap_size) == aw_wrap_size)? 1'b1: 1'b0;
	assign  ar_wrap_en = ((axi_araddr & ar_wrap_size) == ar_wrap_size)? 1'b1: 1'b0;

	// Implement axi_awready generation

	// axi_awready is asserted for one S_AXI_ACLK clock cycle when both
	// S_AXI_AWVALID and S_AXI_WVALID are asserted. axi_awready is
	// de-asserted when reset is low.

	always @( posedge S_AXI_ACLK ) begin
		if ( S_AXI_ARESETN == 1'b0 ) begin
			axi_awready <= 1'b0;
			axi_awv_awr_flag <= 1'b0;
		end
		else begin
			if ( ~axi_awready && S_AXI_AWVALID && ~axi_awv_awr_flag && ~axi_arv_arr_flag ) begin
				// slave is ready to accept an address and
				// associated control signals
				axi_awready <= 1'b1;
				axi_awv_awr_flag  <= 1'b1; 
				// used for generation of bresp() and bvalid
			end
			else if ( S_AXI_WLAST && axi_wready ) begin
			// preparing to accept next address after current write burst tx completion
				axi_awv_awr_flag  <= 1'b0;
			end
			else begin
				axi_awready <= 1'b0;
			end
		end
	end
	// Implement axi_awaddr latching

	// This process is used to latch the address when both 
	// S_AXI_AWVALID and S_AXI_WVALID are valid. 

	always @( posedge S_AXI_ACLK ) begin
		if ( S_AXI_ARESETN == 1'b0 ) begin
			axi_awaddr <= 0;
			axi_awlen_cntr <= 0;
			axi_awburst <= 0;
			axi_awlen <= 0;
		end
		else begin
			if ( ~axi_awready && S_AXI_AWVALID && ~axi_awv_awr_flag ) begin
				// address latching
				axi_awaddr <= S_AXI_AWADDR[C_S_AXI_ADDR_WIDTH - 1:0];
				axi_awburst <= S_AXI_AWBURST;
				axi_awlen <= S_AXI_AWLEN;
				// start address of transfer
				axi_awlen_cntr <= 0;
			end
			else if( ( axi_awlen_cntr <= axi_awlen ) && axi_wready && S_AXI_WVALID ) begin
				axi_awlen_cntr <= axi_awlen_cntr + 1;
				case ( axi_awburst )
				2'b00: begin// fixed burst
				// The write address for all the beats in the transaction are fixed
					axi_awaddr <= axi_awaddr;
					//for awsize = 4 bytes (010)
				end
				2'b01: begin//incremental burst
				// The write address for all the beats in the transaction are increments by awsize
					axi_awaddr[C_S_AXI_ADDR_WIDTH - 1:ADDR_LSB] <= axi_awaddr[C_S_AXI_ADDR_WIDTH - 1:ADDR_LSB] + 1;
					//awaddr aligned to 4 byte boundary
					axi_awaddr[ADDR_LSB-1:0]  <= {ADDR_LSB{1'b0}};
					//for awsize = 4 bytes (010)
				end
				2'b10: begin//Wrapping burst
				// The write address wraps when the address reaches wrap boundary 
					if (aw_wrap_en) begin
						axi_awaddr <= (axi_awaddr - aw_wrap_size);
					end
					else begin
						axi_awaddr[C_S_AXI_ADDR_WIDTH - 1:ADDR_LSB] <= axi_awaddr[C_S_AXI_ADDR_WIDTH - 1:ADDR_LSB] + 1;
						axi_awaddr[ADDR_LSB-1:0]  <= {ADDR_LSB{1'b0}};
					end
				end
				default: begin//reserved (incremental burst for example)
					axi_awaddr <= axi_awaddr[C_S_AXI_ADDR_WIDTH - 1:ADDR_LSB] + 1;
					//for awsize = 4 bytes (010)
				end
				endcase
			end
		end
	end
	// Implement axi_wready generation

	// axi_wready is asserted for one S_AXI_ACLK clock cycle when both
	// S_AXI_AWVALID and S_AXI_WVALID are asserted. axi_wready is 
	// de-asserted when reset is low. 

	always @( posedge S_AXI_ACLK ) begin
		if ( S_AXI_ARESETN == 1'b0 ) begin
			axi_wready <= 1'b0;
		end
		else begin
			if ( ~axi_wready && S_AXI_WVALID && axi_awv_awr_flag) begin
				// slave can accept the write data
				axi_wready <= 1'b1;
			end
			else if ( S_AXI_WLAST && axi_wready ) begin
				axi_wready <= 1'b0;
			end
		end
	end
	// Implement write response logic generation

	// The write response and response valid signals are asserted by the slave 
	// when axi_wready, S_AXI_WVALID, axi_wready and S_AXI_WVALID are asserted.  
	// This marks the acceptance of address and indicates the status of 
	// write transaction.

	always @( posedge S_AXI_ACLK ) begin
		if ( S_AXI_ARESETN == 1'b0 ) begin
			axi_bvalid <= 0;
			axi_bresp <= 2'b0;
		end
		else begin
			if ( axi_awv_awr_flag && axi_wready && S_AXI_WVALID && ~axi_bvalid && S_AXI_WLAST ) begin
				axi_bvalid <= 1'b1;
				axi_bresp  <= 2'b0;
				// 'OKAY' response
			end
			else begin
				if ( S_AXI_BREADY && axi_bvalid ) begin
				//check if bready is asserted while bvalid is high) 
				//(there is a possibility that bready is always asserted high)   
					axi_bvalid <= 1'b0;
				end
			end
		end
	end   
	// Implement axi_arready generation

	// axi_arready is asserted for one S_AXI_ACLK clock cycle when
	// S_AXI_ARVALID is asserted. axi_awready is 
	// de-asserted when reset (active low) is asserted. 
	// The read address is also latched when S_AXI_ARVALID is 
	// asserted. axi_araddr is reset to zero on reset assertion.

	always @( posedge S_AXI_ACLK ) begin
		if ( S_AXI_ARESETN == 1'b0 ) begin
			axi_arready <= 1'b0;
			axi_arv_arr_flag <= 1'b0;
		end
		else begin
			if (~axi_arready && S_AXI_ARVALID && ~axi_awv_awr_flag && ~axi_arv_arr_flag) begin
				axi_arready <= 1'b1;
				axi_arv_arr_flag <= 1'b1;
			end
			else if (axi_rvalid && S_AXI_RREADY && axi_arlen_cntr == axi_arlen) begin
				// preparing to accept next address after current read completion
				axi_arv_arr_flag  <= 1'b0;
			end
			else begin
				axi_arready <= 1'b0;
			end
		end
	end
	// Implement axi_araddr latching

	//This process is used to latch the address when both 
	//S_AXI_ARVALID and S_AXI_RVALID are valid. 
	always @( posedge S_AXI_ACLK ) begin
		if ( S_AXI_ARESETN == 1'b0 ) begin
			axi_araddr <= 0;
			axi_arlen_cntr <= 0;
			axi_arburst <= 0;
			axi_arlen <= 0;
			axi_rlast <= 1'b0;
		end 
		else begin
			if (~axi_arready && S_AXI_ARVALID && ~axi_arv_arr_flag) begin
			  // address latching
				axi_araddr <= S_AXI_ARADDR[C_S_AXI_ADDR_WIDTH - 1:0]; 
				axi_arburst <= S_AXI_ARBURST; 
				axi_arlen <= S_AXI_ARLEN;
				// start address of transfer
				axi_arlen_cntr <= 0;
				axi_rlast <= 1'b0;
			end
			else if((axi_arlen_cntr <= axi_arlen) && axi_rvalid && S_AXI_RREADY) begin
				axi_arlen_cntr <= axi_arlen_cntr + 1;
				axi_rlast <= 1'b0;
			
				case (axi_arburst)
				2'b00: begin// fixed burst
					// The read address for all the beats in the transaction are fixed
					axi_araddr       <= axi_araddr;
					//for arsize = 4 bytes (010)
				end   
				2'b01: begin//incremental burst
				// The read address for all the beats in the transaction are increments by awsize
					axi_araddr[C_S_AXI_ADDR_WIDTH - 1:ADDR_LSB] <= axi_araddr[C_S_AXI_ADDR_WIDTH - 1:ADDR_LSB] + 1;
					//araddr aligned to 4 byte boundary
					axi_araddr[ADDR_LSB-1:0]  <= {ADDR_LSB{1'b0}};
					//for awsize = 4 bytes (010)
				end
				2'b10: begin//Wrapping burst
				// The read address wraps when the address reaches wrap boundary 
					if (ar_wrap_en) begin
					  axi_araddr <= (axi_araddr - ar_wrap_size);
					end
					else begin
					axi_araddr[C_S_AXI_ADDR_WIDTH - 1:ADDR_LSB] <= axi_araddr[C_S_AXI_ADDR_WIDTH - 1:ADDR_LSB] + 1;
					//araddr aligned to 4 byte boundary
					axi_araddr[ADDR_LSB-1:0]  <= {ADDR_LSB{1'b0}};
					end
				end
				default: begin//reserved (incremental burst for example)
					axi_araddr <= axi_araddr[C_S_AXI_ADDR_WIDTH - 1:ADDR_LSB]+1;
					//for arsize = 4 bytes (010)
				end
				endcase
			end
			else if((axi_arlen_cntr == axi_arlen) && ~axi_rlast && axi_arv_arr_flag ) begin
				axi_rlast <= 1'b1;
			end
			else if (S_AXI_RREADY) begin
				axi_rlast <= 1'b0;
			end
		end
	end
	// Implement axi_arvalid generation

	// axi_rvalid is asserted for one S_AXI_ACLK clock cycle when both 
	// S_AXI_ARVALID and axi_arready are asserted. The slave registers 
	// data are available on the axi_rdata bus at this instance. The 
	// assertion of axi_rvalid marks the validity of read data on the 
	// bus and axi_rresp indicates the status of read transaction.axi_rvalid 
	// is deasserted on reset (active low). axi_rresp and axi_rdata are 
	// cleared to zero on reset (active low).  

	always @( posedge S_AXI_ACLK ) begin
		if ( S_AXI_ARESETN == 1'b0 ) begin
			axi_rvalid <= 0;
			axi_rresp  <= 0;
		end
		else begin
			if (axi_arv_arr_flag && ~axi_rvalid) begin
				axi_rvalid <= 1'b1;
				axi_rresp  <= 2'b0;
			  // 'OKAY' response
			end
			else if (axi_rvalid && S_AXI_RREADY) begin
				axi_rvalid <= 1'b0;
			end
		end
	end
	




	// ------------------------------------------
	// -- Example code to access user logic memory region
	// ------------------------------------------
/*
	generate
		if ( USER_NUM_MEM >= 1 ) begin
			assign mem_select = 1;
			// 												Read 												write 																neither
			assign mem_address = ( axi_arv_arr_flag ? axi_araddr[ADDR_LSB + OPT_MEM_ADDR_BITS : ADDR_LSB] : ( axi_awv_awr_flag ? axi_awaddr[ADDR_LSB + OPT_MEM_ADDR_BITS : ADDR_LSB] : 0 ) );
		end
	endgenerate

	// implement Block RAM(s)
	generate
		for ( i = 0; i <= USER_NUM_MEM - 1; i = i + 1 ) begin: BRAM_GEN
			wire mem_rden;
			wire mem_wren;

			assign mem_wren = axi_wready && S_AXI_WVALID ;
			assign mem_rden = axi_arv_arr_flag ;
		
			for ( mem_byte_index = 0; mem_byte_index <= ( C_S_AXI_DATA_WIDTH / 8 - 1 ); mem_byte_index = mem_byte_index + 1 ) begin:BYTE_BRAM_GEN
				wire [7 : 0] data_in;
				wire [7 : 0] data_out;
				reg  [7 : 0] byte_ram [0 : 31];
				integer  j;
		 
				//assigning 8 bit data
				assign data_in  = S_AXI_WDATA[(mem_byte_index*8+7) -: 8];
				assign data_out = byte_ram[mem_address];
		 
				always @ ( posedge S_AXI_ACLK ) begin
					if ( mem_wren && S_AXI_WSTRB[mem_byte_index] ) begin
						byte_ram[mem_address] <= data_in;
					end
				end
		  
				always @( posedge S_AXI_ACLK ) begin
					if ( mem_rden ) begin
						mem_data_out[i][(mem_byte_index*8+7) -: 8] <= data_out;
					end   
				end
			end
		end       
	endgenerate
	//Output register or memory read data

	always @( mem_data_out, axi_rvalid) begin
		if ( axi_rvalid ) begin
		  // Read address mux
			axi_rdata <= mem_data_out[0];
		end
		else begin
			axi_rdata <= 32'h00000000;
		end       
	end    

*/



assign mem_address = ( ( axi_awv_awr_flag ? axi_awaddr[ADDR_LSB + OPT_MEM_ADDR_BITS : ADDR_LSB] : 0 ) );


reg [31:0] edgeReg_0;
reg [31:0] edgeReg_1;
reg [31:0] edgeReg_2;
reg [31:0] edgeReg_3;
reg [31:0] edgeReg_4;
reg [31:0] edgeReg_5;
reg [31:0] edgeReg_6;
reg [31:0] edgeReg_7;
reg [31:0] edgeReg_8;
reg [31:0] edgeReg_9;
reg [31:0] edgeReg_10;
reg [31:0] edgeReg_11;
reg [31:0] edgeReg_12;
reg [31:0] edgeReg_13;
reg [31:0] edgeReg_14;
reg [31:0] edgeReg_15;
reg [31:0] edgeReg_16;
reg [31:0] edgeReg_17;
reg [31:0] edgeReg_18;
reg [31:0] edgeReg_19;
reg [31:0] edgeReg_20;
reg [31:0] edgeReg_21;
reg [31:0] edgeReg_22;
reg [31:0] edgeReg_23;
reg [31:0] edgeReg_24;
reg [31:0] edgeReg_25;
reg [31:0] edgeReg_26;
reg [31:0] edgeReg_27;
reg [31:0] edgeReg_28;
reg [31:0] edgeReg_29;
reg [31:0] edgeReg_30;
reg [31:0] edgeReg_31;
reg [31:0] edgeReg_32;
reg [31:0] edgeReg_33;
reg [31:0] edgeReg_34;
reg [31:0] edgeReg_35;
reg [31:0] edgeReg_36;
reg [31:0] edgeReg_37;
reg [31:0] edgeReg_38;
reg [31:0] edgeReg_39;
reg [31:0] edgeReg_40;
reg [31:0] edgeReg_41;
reg [31:0] edgeReg_42;
reg [31:0] edgeReg_43;
reg [31:0] edgeReg_44;
reg [31:0] edgeReg_45;
reg [31:0] edgeReg_46;
reg [31:0] edgeReg_47;
reg [31:0] edgeReg_48;
reg [31:0] edgeReg_49;
reg [31:0] edgeReg_50;
reg [31:0] edgeReg_51;
reg [31:0] edgeReg_52;
reg [31:0] edgeReg_53;
reg [31:0] edgeReg_54;
reg [31:0] edgeReg_55;
reg [31:0] edgeReg_56;
reg [31:0] edgeReg_57;
reg [31:0] edgeReg_58;
reg [31:0] edgeReg_59;
reg [31:0] edgeReg_60;
reg [31:0] edgeReg_61;
reg [31:0] edgeReg_62;
reg [31:0] edgeReg_63;
reg [31:0] edgeReg_64;
reg [31:0] edgeReg_65;
reg [31:0] edgeReg_66;
reg [31:0] edgeReg_67;
reg [31:0] edgeReg_68;
reg [31:0] edgeReg_69;
reg [31:0] edgeReg_70;
reg [31:0] edgeReg_71;
reg [31:0] edgeReg_72;
reg [31:0] edgeReg_73;
reg [31:0] edgeReg_74;
reg [31:0] edgeReg_75;
reg [31:0] edgeReg_76;
reg [31:0] edgeReg_77;
reg [31:0] edgeReg_78;
reg [31:0] edgeReg_79;
reg [31:0] edgeReg_80;
reg [31:0] edgeReg_81;
reg [31:0] edgeReg_82;
reg [31:0] edgeReg_83;
reg [31:0] edgeReg_84;
reg [31:0] edgeReg_85;
reg [31:0] edgeReg_86;
reg [31:0] edgeReg_87;
reg [31:0] edgeReg_88;
reg [31:0] edgeReg_89;
reg [31:0] edgeReg_90;
reg [31:0] edgeReg_91;
reg [31:0] edgeReg_92;
reg [31:0] edgeReg_93;
reg [31:0] edgeReg_94;
reg [31:0] edgeReg_95;
reg [31:0] edgeReg_96;
reg [31:0] edgeReg_97;
reg [31:0] edgeReg_98;
reg [31:0] edgeReg_99;
reg [31:0] edgeReg_100;
reg [31:0] edgeReg_101;
reg [31:0] edgeReg_102;
reg [31:0] edgeReg_103;
reg [31:0] edgeReg_104;
reg [31:0] edgeReg_105;
reg [31:0] edgeReg_106;
reg [31:0] edgeReg_107;
reg [31:0] edgeReg_108;
reg [31:0] edgeReg_109;
reg [31:0] edgeReg_110;
reg [31:0] edgeReg_111;
reg [31:0] edgeReg_112;
reg [31:0] edgeReg_113;
reg [31:0] edgeReg_114;
reg [31:0] edgeReg_115;
reg [31:0] edgeReg_116;
reg [31:0] edgeReg_117;
reg [31:0] edgeReg_118;
reg [31:0] edgeReg_119;
reg [31:0] edgeReg_120;
reg [31:0] edgeReg_121;
reg [31:0] edgeReg_122;
reg [31:0] edgeReg_123;
reg [31:0] edgeReg_124;
reg [31:0] edgeReg_125;
reg [31:0] edgeReg_126;
reg [31:0] edgeReg_127;
reg [31:0] edgeReg_128;
reg [31:0] edgeReg_129;
reg [31:0] edgeReg_130;
reg [31:0] edgeReg_131;
reg [31:0] edgeReg_132;
reg [31:0] edgeReg_133;
reg [31:0] edgeReg_134;
reg [31:0] edgeReg_135;
reg [31:0] edgeReg_136;
reg [31:0] edgeReg_137;
reg [31:0] edgeReg_138;
reg [31:0] edgeReg_139;
reg [31:0] edgeReg_140;
reg [31:0] edgeReg_141;
reg [31:0] edgeReg_142;
reg [31:0] edgeReg_143;
reg [31:0] edgeReg_144;
reg [31:0] edgeReg_145;
reg [31:0] edgeReg_146;
reg [31:0] edgeReg_147;
reg [31:0] edgeReg_148;
reg [31:0] edgeReg_149;
reg [31:0] edgeReg_150;
reg [31:0] edgeReg_151;
reg [31:0] edgeReg_152;
reg [31:0] edgeReg_153;
reg [31:0] edgeReg_154;
reg [31:0] edgeReg_155;
reg [31:0] edgeReg_156;
reg [31:0] edgeReg_157;
reg [31:0] edgeReg_158;
reg [31:0] edgeReg_159;
reg [31:0] edgeReg_160;
reg [31:0] edgeReg_161;
reg [31:0] edgeReg_162;
reg [31:0] edgeReg_163;
reg [31:0] edgeReg_164;
reg [31:0] edgeReg_165;
reg [31:0] edgeReg_166;
reg [31:0] edgeReg_167;
reg [31:0] edgeReg_168;
reg [31:0] edgeReg_169;
reg [31:0] edgeReg_170;
reg [31:0] edgeReg_171;
reg [31:0] edgeReg_172;
reg [31:0] edgeReg_173;
reg [31:0] edgeReg_174;
reg [31:0] edgeReg_175;
reg [31:0] edgeReg_176;
reg [31:0] edgeReg_177;
reg [31:0] edgeReg_178;
reg [31:0] edgeReg_179;
reg [31:0] edgeReg_180;
reg [31:0] edgeReg_181;
reg [31:0] edgeReg_182;
reg [31:0] edgeReg_183;
reg [31:0] edgeReg_184;
reg [31:0] edgeReg_185;
reg [31:0] edgeReg_186;
reg [31:0] edgeReg_187;
reg [31:0] edgeReg_188;
reg [31:0] edgeReg_189;
reg [31:0] edgeReg_190;
reg [31:0] edgeReg_191;
reg [31:0] edgeReg_192;
reg [31:0] edgeReg_193;
reg [31:0] edgeReg_194;
reg [31:0] edgeReg_195;
reg [31:0] edgeReg_196;
reg [31:0] edgeReg_197;
reg [31:0] edgeReg_198;
reg [31:0] edgeReg_199;
reg [31:0] edgeReg_200;
reg [31:0] edgeReg_201;
reg [31:0] edgeReg_202;
reg [31:0] edgeReg_203;
reg [31:0] edgeReg_204;
reg [31:0] edgeReg_205;
reg [31:0] edgeReg_206;
reg [31:0] edgeReg_207;
reg [31:0] edgeReg_208;
reg [31:0] edgeReg_209;
reg [31:0] edgeReg_210;
reg [31:0] edgeReg_211;
reg [31:0] edgeReg_212;
reg [31:0] edgeReg_213;
reg [31:0] edgeReg_214;
reg [31:0] edgeReg_215;
reg [31:0] edgeReg_216;
reg [31:0] edgeReg_217;
reg [31:0] edgeReg_218;
reg [31:0] edgeReg_219;
reg [31:0] edgeReg_220;
reg [31:0] edgeReg_221;
reg [31:0] edgeReg_222;
reg [31:0] edgeReg_223;
reg [31:0] edgeReg_224;
reg [31:0] edgeReg_225;
reg [31:0] edgeReg_226;
reg [31:0] edgeReg_227;
reg [31:0] edgeReg_228;
reg [31:0] edgeReg_229;
reg [31:0] edgeReg_230;
reg [31:0] edgeReg_231;
reg [31:0] edgeReg_232;
reg [31:0] edgeReg_233;
reg [31:0] edgeReg_234;
reg [31:0] edgeReg_235;
reg [31:0] edgeReg_236;
reg [31:0] edgeReg_237;
reg [31:0] edgeReg_238;
reg [31:0] edgeReg_239;
reg [31:0] edgeReg_240;
reg [31:0] edgeReg_241;
reg [31:0] edgeReg_242;
reg [31:0] edgeReg_243;
reg [31:0] edgeReg_244;
reg [31:0] edgeReg_245;
reg [31:0] edgeReg_246;
reg [31:0] edgeReg_247;
reg [31:0] edgeReg_248;
reg [31:0] edgeReg_249;
reg [31:0] edgeReg_250;
reg [31:0] edgeReg_251;
reg [31:0] edgeReg_252;
reg [31:0] edgeReg_253;
reg [31:0] edgeReg_254;
reg [31:0] edgeReg_255;


assign edgeWire = {edgeReg_255, edgeReg_254, edgeReg_253, edgeReg_252, edgeReg_251, edgeReg_250, edgeReg_249, edgeReg_248, edgeReg_247, edgeReg_246, edgeReg_245, edgeReg_244, edgeReg_243, edgeReg_242, edgeReg_241, edgeReg_240, edgeReg_239, edgeReg_238, edgeReg_237, edgeReg_236, edgeReg_235, edgeReg_234, edgeReg_233, edgeReg_232, edgeReg_231, edgeReg_230, edgeReg_229, edgeReg_228, edgeReg_227, edgeReg_226, edgeReg_225, edgeReg_224, edgeReg_223, edgeReg_222, edgeReg_221, edgeReg_220, edgeReg_219, edgeReg_218, edgeReg_217, edgeReg_216, edgeReg_215, edgeReg_214, edgeReg_213, edgeReg_212, edgeReg_211, edgeReg_210, edgeReg_209, edgeReg_208, edgeReg_207, edgeReg_206, edgeReg_205, edgeReg_204, edgeReg_203, edgeReg_202, edgeReg_201, edgeReg_200, edgeReg_199, edgeReg_198, edgeReg_197, edgeReg_196, edgeReg_195, edgeReg_194, edgeReg_193, edgeReg_192, edgeReg_191, edgeReg_190, edgeReg_189, edgeReg_188, edgeReg_187, edgeReg_186, edgeReg_185, edgeReg_184, edgeReg_183, edgeReg_182, edgeReg_181, edgeReg_180, edgeReg_179, edgeReg_178, edgeReg_177, edgeReg_176, edgeReg_175, edgeReg_174, edgeReg_173, edgeReg_172, edgeReg_171, edgeReg_170, edgeReg_169, edgeReg_168, edgeReg_167, edgeReg_166, edgeReg_165, edgeReg_164, edgeReg_163, edgeReg_162, edgeReg_161, edgeReg_160, edgeReg_159, edgeReg_158, edgeReg_157, edgeReg_156, edgeReg_155, edgeReg_154, edgeReg_153, edgeReg_152, edgeReg_151, edgeReg_150, edgeReg_149, edgeReg_148, edgeReg_147, edgeReg_146, edgeReg_145, edgeReg_144, edgeReg_143, edgeReg_142, edgeReg_141, edgeReg_140, edgeReg_139, edgeReg_138, edgeReg_137, edgeReg_136, edgeReg_135, edgeReg_134, edgeReg_133, edgeReg_132, edgeReg_131, edgeReg_130, edgeReg_129, edgeReg_128, edgeReg_127, edgeReg_126, edgeReg_125, edgeReg_124, edgeReg_123, edgeReg_122, edgeReg_121, edgeReg_120, edgeReg_119, edgeReg_118, edgeReg_117, edgeReg_116, edgeReg_115, edgeReg_114, edgeReg_113, edgeReg_112, edgeReg_111, edgeReg_110, edgeReg_109, edgeReg_108, edgeReg_107, edgeReg_106, edgeReg_105, edgeReg_104, edgeReg_103, edgeReg_102, edgeReg_101, edgeReg_100, edgeReg_99, edgeReg_98, edgeReg_97, edgeReg_96, edgeReg_95, edgeReg_94, edgeReg_93, edgeReg_92, edgeReg_91, edgeReg_90, edgeReg_89, edgeReg_88, edgeReg_87, edgeReg_86, edgeReg_85, edgeReg_84, edgeReg_83, edgeReg_82, edgeReg_81, edgeReg_80, edgeReg_79, edgeReg_78, edgeReg_77, edgeReg_76, edgeReg_75, edgeReg_74, edgeReg_73, edgeReg_72, edgeReg_71, edgeReg_70, edgeReg_69, edgeReg_68, edgeReg_67, edgeReg_66, edgeReg_65, edgeReg_64, edgeReg_63, edgeReg_62, edgeReg_61, edgeReg_60, edgeReg_59, edgeReg_58, edgeReg_57, edgeReg_56, edgeReg_55, edgeReg_54, edgeReg_53, edgeReg_52, edgeReg_51, edgeReg_50, edgeReg_49, edgeReg_48, edgeReg_47, edgeReg_46, edgeReg_45, edgeReg_44, edgeReg_43, edgeReg_42, edgeReg_41, edgeReg_40, edgeReg_39, edgeReg_38, edgeReg_37, edgeReg_36, edgeReg_35, edgeReg_34, edgeReg_33, edgeReg_32, edgeReg_31, edgeReg_30, edgeReg_29, edgeReg_28, edgeReg_27, edgeReg_26, edgeReg_25, edgeReg_24, edgeReg_23, edgeReg_22, edgeReg_21, edgeReg_20, edgeReg_19, edgeReg_18, edgeReg_17, edgeReg_16, edgeReg_15, edgeReg_14, edgeReg_13, edgeReg_12, edgeReg_11, edgeReg_10, edgeReg_9, edgeReg_8, edgeReg_7, edgeReg_6, edgeReg_5, edgeReg_4, edgeReg_3, edgeReg_2, edgeReg_1, edgeReg_0};

always @( posedge S_AXI_ACLK ) begin 
    axi_rdata <= 32'd0;
	if ( S_AXI_ARESETN == 1'b0 || clear == 1'b1 ) begin

		 edgeReg_0 <= 32'b0;
		 edgeReg_1 <= 32'b0;
		 edgeReg_2 <= 32'b0;
		 edgeReg_3 <= 32'b0;
		 edgeReg_4 <= 32'b0;
		 edgeReg_5 <= 32'b0;
		 edgeReg_6 <= 32'b0;
		 edgeReg_7 <= 32'b0;
		 edgeReg_8 <= 32'b0;
		 edgeReg_9 <= 32'b0;
		 edgeReg_10 <= 32'b0;
		 edgeReg_11 <= 32'b0;
		 edgeReg_12 <= 32'b0;
		 edgeReg_13 <= 32'b0;
		 edgeReg_14 <= 32'b0;
		 edgeReg_15 <= 32'b0;
		 edgeReg_16 <= 32'b0;
		 edgeReg_17 <= 32'b0;
		 edgeReg_18 <= 32'b0;
		 edgeReg_19 <= 32'b0;
		 edgeReg_20 <= 32'b0;
		 edgeReg_21 <= 32'b0;
		 edgeReg_22 <= 32'b0;
		 edgeReg_23 <= 32'b0;
		 edgeReg_24 <= 32'b0;
		 edgeReg_25 <= 32'b0;
		 edgeReg_26 <= 32'b0;
		 edgeReg_27 <= 32'b0;
		 edgeReg_28 <= 32'b0;
		 edgeReg_29 <= 32'b0;
		 edgeReg_30 <= 32'b0;
		 edgeReg_31 <= 32'b0;
		 edgeReg_32 <= 32'b0;
		 edgeReg_33 <= 32'b0;
		 edgeReg_34 <= 32'b0;
		 edgeReg_35 <= 32'b0;
		 edgeReg_36 <= 32'b0;
		 edgeReg_37 <= 32'b0;
		 edgeReg_38 <= 32'b0;
		 edgeReg_39 <= 32'b0;
		 edgeReg_40 <= 32'b0;
		 edgeReg_41 <= 32'b0;
		 edgeReg_42 <= 32'b0;
		 edgeReg_43 <= 32'b0;
		 edgeReg_44 <= 32'b0;
		 edgeReg_45 <= 32'b0;
		 edgeReg_46 <= 32'b0;
		 edgeReg_47 <= 32'b0;
		 edgeReg_48 <= 32'b0;
		 edgeReg_49 <= 32'b0;
		 edgeReg_50 <= 32'b0;
		 edgeReg_51 <= 32'b0;
		 edgeReg_52 <= 32'b0;
		 edgeReg_53 <= 32'b0;
		 edgeReg_54 <= 32'b0;
		 edgeReg_55 <= 32'b0;
		 edgeReg_56 <= 32'b0;
		 edgeReg_57 <= 32'b0;
		 edgeReg_58 <= 32'b0;
		 edgeReg_59 <= 32'b0;
		 edgeReg_60 <= 32'b0;
		 edgeReg_61 <= 32'b0;
		 edgeReg_62 <= 32'b0;
		 edgeReg_63 <= 32'b0;
		 edgeReg_64 <= 32'b0;
		 edgeReg_65 <= 32'b0;
		 edgeReg_66 <= 32'b0;
		 edgeReg_67 <= 32'b0;
		 edgeReg_68 <= 32'b0;
		 edgeReg_69 <= 32'b0;
		 edgeReg_70 <= 32'b0;
		 edgeReg_71 <= 32'b0;
		 edgeReg_72 <= 32'b0;
		 edgeReg_73 <= 32'b0;
		 edgeReg_74 <= 32'b0;
		 edgeReg_75 <= 32'b0;
		 edgeReg_76 <= 32'b0;
		 edgeReg_77 <= 32'b0;
		 edgeReg_78 <= 32'b0;
		 edgeReg_79 <= 32'b0;
		 edgeReg_80 <= 32'b0;
		 edgeReg_81 <= 32'b0;
		 edgeReg_82 <= 32'b0;
		 edgeReg_83 <= 32'b0;
		 edgeReg_84 <= 32'b0;
		 edgeReg_85 <= 32'b0;
		 edgeReg_86 <= 32'b0;
		 edgeReg_87 <= 32'b0;
		 edgeReg_88 <= 32'b0;
		 edgeReg_89 <= 32'b0;
		 edgeReg_90 <= 32'b0;
		 edgeReg_91 <= 32'b0;
		 edgeReg_92 <= 32'b0;
		 edgeReg_93 <= 32'b0;
		 edgeReg_94 <= 32'b0;
		 edgeReg_95 <= 32'b0;
		 edgeReg_96 <= 32'b0;
		 edgeReg_97 <= 32'b0;
		 edgeReg_98 <= 32'b0;
		 edgeReg_99 <= 32'b0;
		 edgeReg_100 <= 32'b0;
		 edgeReg_101 <= 32'b0;
		 edgeReg_102 <= 32'b0;
		 edgeReg_103 <= 32'b0;
		 edgeReg_104 <= 32'b0;
		 edgeReg_105 <= 32'b0;
		 edgeReg_106 <= 32'b0;
		 edgeReg_107 <= 32'b0;
		 edgeReg_108 <= 32'b0;
		 edgeReg_109 <= 32'b0;
		 edgeReg_110 <= 32'b0;
		 edgeReg_111 <= 32'b0;
		 edgeReg_112 <= 32'b0;
		 edgeReg_113 <= 32'b0;
		 edgeReg_114 <= 32'b0;
		 edgeReg_115 <= 32'b0;
		 edgeReg_116 <= 32'b0;
		 edgeReg_117 <= 32'b0;
		 edgeReg_118 <= 32'b0;
		 edgeReg_119 <= 32'b0;
		 edgeReg_120 <= 32'b0;
		 edgeReg_121 <= 32'b0;
		 edgeReg_122 <= 32'b0;
		 edgeReg_123 <= 32'b0;
		 edgeReg_124 <= 32'b0;
		 edgeReg_125 <= 32'b0;
		 edgeReg_126 <= 32'b0;
		 edgeReg_127 <= 32'b0;
		 edgeReg_128 <= 32'b0;
		 edgeReg_129 <= 32'b0;
		 edgeReg_130 <= 32'b0;
		 edgeReg_131 <= 32'b0;
		 edgeReg_132 <= 32'b0;
		 edgeReg_133 <= 32'b0;
		 edgeReg_134 <= 32'b0;
		 edgeReg_135 <= 32'b0;
		 edgeReg_136 <= 32'b0;
		 edgeReg_137 <= 32'b0;
		 edgeReg_138 <= 32'b0;
		 edgeReg_139 <= 32'b0;
		 edgeReg_140 <= 32'b0;
		 edgeReg_141 <= 32'b0;
		 edgeReg_142 <= 32'b0;
		 edgeReg_143 <= 32'b0;
		 edgeReg_144 <= 32'b0;
		 edgeReg_145 <= 32'b0;
		 edgeReg_146 <= 32'b0;
		 edgeReg_147 <= 32'b0;
		 edgeReg_148 <= 32'b0;
		 edgeReg_149 <= 32'b0;
		 edgeReg_150 <= 32'b0;
		 edgeReg_151 <= 32'b0;
		 edgeReg_152 <= 32'b0;
		 edgeReg_153 <= 32'b0;
		 edgeReg_154 <= 32'b0;
		 edgeReg_155 <= 32'b0;
		 edgeReg_156 <= 32'b0;
		 edgeReg_157 <= 32'b0;
		 edgeReg_158 <= 32'b0;
		 edgeReg_159 <= 32'b0;
		 edgeReg_160 <= 32'b0;
		 edgeReg_161 <= 32'b0;
		 edgeReg_162 <= 32'b0;
		 edgeReg_163 <= 32'b0;
		 edgeReg_164 <= 32'b0;
		 edgeReg_165 <= 32'b0;
		 edgeReg_166 <= 32'b0;
		 edgeReg_167 <= 32'b0;
		 edgeReg_168 <= 32'b0;
		 edgeReg_169 <= 32'b0;
		 edgeReg_170 <= 32'b0;
		 edgeReg_171 <= 32'b0;
		 edgeReg_172 <= 32'b0;
		 edgeReg_173 <= 32'b0;
		 edgeReg_174 <= 32'b0;
		 edgeReg_175 <= 32'b0;
		 edgeReg_176 <= 32'b0;
		 edgeReg_177 <= 32'b0;
		 edgeReg_178 <= 32'b0;
		 edgeReg_179 <= 32'b0;
		 edgeReg_180 <= 32'b0;
		 edgeReg_181 <= 32'b0;
		 edgeReg_182 <= 32'b0;
		 edgeReg_183 <= 32'b0;
		 edgeReg_184 <= 32'b0;
		 edgeReg_185 <= 32'b0;
		 edgeReg_186 <= 32'b0;
		 edgeReg_187 <= 32'b0;
		 edgeReg_188 <= 32'b0;
		 edgeReg_189 <= 32'b0;
		 edgeReg_190 <= 32'b0;
		 edgeReg_191 <= 32'b0;
		 edgeReg_192 <= 32'b0;
		 edgeReg_193 <= 32'b0;
		 edgeReg_194 <= 32'b0;
		 edgeReg_195 <= 32'b0;
		 edgeReg_196 <= 32'b0;
		 edgeReg_197 <= 32'b0;
		 edgeReg_198 <= 32'b0;
		 edgeReg_199 <= 32'b0;
		 edgeReg_200 <= 32'b0;
		 edgeReg_201 <= 32'b0;
		 edgeReg_202 <= 32'b0;
		 edgeReg_203 <= 32'b0;
		 edgeReg_204 <= 32'b0;
		 edgeReg_205 <= 32'b0;
		 edgeReg_206 <= 32'b0;
		 edgeReg_207 <= 32'b0;
		 edgeReg_208 <= 32'b0;
		 edgeReg_209 <= 32'b0;
		 edgeReg_210 <= 32'b0;
		 edgeReg_211 <= 32'b0;
		 edgeReg_212 <= 32'b0;
		 edgeReg_213 <= 32'b0;
		 edgeReg_214 <= 32'b0;
		 edgeReg_215 <= 32'b0;
		 edgeReg_216 <= 32'b0;
		 edgeReg_217 <= 32'b0;
		 edgeReg_218 <= 32'b0;
		 edgeReg_219 <= 32'b0;
		 edgeReg_220 <= 32'b0;
		 edgeReg_221 <= 32'b0;
		 edgeReg_222 <= 32'b0;
		 edgeReg_223 <= 32'b0;
		 edgeReg_224 <= 32'b0;
		 edgeReg_225 <= 32'b0;
		 edgeReg_226 <= 32'b0;
		 edgeReg_227 <= 32'b0;
		 edgeReg_228 <= 32'b0;
		 edgeReg_229 <= 32'b0;
		 edgeReg_230 <= 32'b0;
		 edgeReg_231 <= 32'b0;
		 edgeReg_232 <= 32'b0;
		 edgeReg_233 <= 32'b0;
		 edgeReg_234 <= 32'b0;
		 edgeReg_235 <= 32'b0;
		 edgeReg_236 <= 32'b0;
		 edgeReg_237 <= 32'b0;
		 edgeReg_238 <= 32'b0;
		 edgeReg_239 <= 32'b0;
		 edgeReg_240 <= 32'b0;
		 edgeReg_241 <= 32'b0;
		 edgeReg_242 <= 32'b0;
		 edgeReg_243 <= 32'b0;
		 edgeReg_244 <= 32'b0;
		 edgeReg_245 <= 32'b0;
		 edgeReg_246 <= 32'b0;
		 edgeReg_247 <= 32'b0;
		 edgeReg_248 <= 32'b0;
		 edgeReg_249 <= 32'b0;
		 edgeReg_250 <= 32'b0;
		 edgeReg_251 <= 32'b0;
		 edgeReg_252 <= 32'b0;
		 edgeReg_253 <= 32'b0;
		 edgeReg_254 <= 32'b0;
		 edgeReg_255 <= 32'b0;
	end

	else if ( axi_awv_awr_flag ) begin
		case(mem_address)
			8'd0: edgeReg_0 <= S_AXI_WDATA | edgeReg_0;
			8'd1: edgeReg_1 <= S_AXI_WDATA | edgeReg_1;
			8'd2: edgeReg_2 <= S_AXI_WDATA | edgeReg_2;
			8'd3: edgeReg_3 <= S_AXI_WDATA | edgeReg_3;
			8'd4: edgeReg_4 <= S_AXI_WDATA | edgeReg_4;
			8'd5: edgeReg_5 <= S_AXI_WDATA | edgeReg_5;
			8'd6: edgeReg_6 <= S_AXI_WDATA | edgeReg_6;
			8'd7: edgeReg_7 <= S_AXI_WDATA | edgeReg_7;
			8'd8: edgeReg_8 <= S_AXI_WDATA | edgeReg_8;
			8'd9: edgeReg_9 <= S_AXI_WDATA | edgeReg_9;
			8'd10: edgeReg_10 <= S_AXI_WDATA | edgeReg_10;
			8'd11: edgeReg_11 <= S_AXI_WDATA | edgeReg_11;
			8'd12: edgeReg_12 <= S_AXI_WDATA | edgeReg_12;
			8'd13: edgeReg_13 <= S_AXI_WDATA | edgeReg_13;
			8'd14: edgeReg_14 <= S_AXI_WDATA | edgeReg_14;
			8'd15: edgeReg_15 <= S_AXI_WDATA | edgeReg_15;
			8'd16: edgeReg_16 <= S_AXI_WDATA | edgeReg_16;
			8'd17: edgeReg_17 <= S_AXI_WDATA | edgeReg_17;
			8'd18: edgeReg_18 <= S_AXI_WDATA | edgeReg_18;
			8'd19: edgeReg_19 <= S_AXI_WDATA | edgeReg_19;
			8'd20: edgeReg_20 <= S_AXI_WDATA | edgeReg_20;
			8'd21: edgeReg_21 <= S_AXI_WDATA | edgeReg_21;
			8'd22: edgeReg_22 <= S_AXI_WDATA | edgeReg_22;
			8'd23: edgeReg_23 <= S_AXI_WDATA | edgeReg_23;
			8'd24: edgeReg_24 <= S_AXI_WDATA | edgeReg_24;
			8'd25: edgeReg_25 <= S_AXI_WDATA | edgeReg_25;
			8'd26: edgeReg_26 <= S_AXI_WDATA | edgeReg_26;
			8'd27: edgeReg_27 <= S_AXI_WDATA | edgeReg_27;
			8'd28: edgeReg_28 <= S_AXI_WDATA | edgeReg_28;
			8'd29: edgeReg_29 <= S_AXI_WDATA | edgeReg_29;
			8'd30: edgeReg_30 <= S_AXI_WDATA | edgeReg_30;
			8'd31: edgeReg_31 <= S_AXI_WDATA | edgeReg_31;
			8'd32: edgeReg_32 <= S_AXI_WDATA | edgeReg_32;
			8'd33: edgeReg_33 <= S_AXI_WDATA | edgeReg_33;
			8'd34: edgeReg_34 <= S_AXI_WDATA | edgeReg_34;
			8'd35: edgeReg_35 <= S_AXI_WDATA | edgeReg_35;
			8'd36: edgeReg_36 <= S_AXI_WDATA | edgeReg_36;
			8'd37: edgeReg_37 <= S_AXI_WDATA | edgeReg_37;
			8'd38: edgeReg_38 <= S_AXI_WDATA | edgeReg_38;
			8'd39: edgeReg_39 <= S_AXI_WDATA | edgeReg_39;
			8'd40: edgeReg_40 <= S_AXI_WDATA | edgeReg_40;
			8'd41: edgeReg_41 <= S_AXI_WDATA | edgeReg_41;
			8'd42: edgeReg_42 <= S_AXI_WDATA | edgeReg_42;
			8'd43: edgeReg_43 <= S_AXI_WDATA | edgeReg_43;
			8'd44: edgeReg_44 <= S_AXI_WDATA | edgeReg_44;
			8'd45: edgeReg_45 <= S_AXI_WDATA | edgeReg_45;
			8'd46: edgeReg_46 <= S_AXI_WDATA | edgeReg_46;
			8'd47: edgeReg_47 <= S_AXI_WDATA | edgeReg_47;
			8'd48: edgeReg_48 <= S_AXI_WDATA | edgeReg_48;
			8'd49: edgeReg_49 <= S_AXI_WDATA | edgeReg_49;
			8'd50: edgeReg_50 <= S_AXI_WDATA | edgeReg_50;
			8'd51: edgeReg_51 <= S_AXI_WDATA | edgeReg_51;
			8'd52: edgeReg_52 <= S_AXI_WDATA | edgeReg_52;
			8'd53: edgeReg_53 <= S_AXI_WDATA | edgeReg_53;
			8'd54: edgeReg_54 <= S_AXI_WDATA | edgeReg_54;
			8'd55: edgeReg_55 <= S_AXI_WDATA | edgeReg_55;
			8'd56: edgeReg_56 <= S_AXI_WDATA | edgeReg_56;
			8'd57: edgeReg_57 <= S_AXI_WDATA | edgeReg_57;
			8'd58: edgeReg_58 <= S_AXI_WDATA | edgeReg_58;
			8'd59: edgeReg_59 <= S_AXI_WDATA | edgeReg_59;
			8'd60: edgeReg_60 <= S_AXI_WDATA | edgeReg_60;
			8'd61: edgeReg_61 <= S_AXI_WDATA | edgeReg_61;
			8'd62: edgeReg_62 <= S_AXI_WDATA | edgeReg_62;
			8'd63: edgeReg_63 <= S_AXI_WDATA | edgeReg_63;
			8'd64: edgeReg_64 <= S_AXI_WDATA | edgeReg_64;
			8'd65: edgeReg_65 <= S_AXI_WDATA | edgeReg_65;
			8'd66: edgeReg_66 <= S_AXI_WDATA | edgeReg_66;
			8'd67: edgeReg_67 <= S_AXI_WDATA | edgeReg_67;
			8'd68: edgeReg_68 <= S_AXI_WDATA | edgeReg_68;
			8'd69: edgeReg_69 <= S_AXI_WDATA | edgeReg_69;
			8'd70: edgeReg_70 <= S_AXI_WDATA | edgeReg_70;
			8'd71: edgeReg_71 <= S_AXI_WDATA | edgeReg_71;
			8'd72: edgeReg_72 <= S_AXI_WDATA | edgeReg_72;
			8'd73: edgeReg_73 <= S_AXI_WDATA | edgeReg_73;
			8'd74: edgeReg_74 <= S_AXI_WDATA | edgeReg_74;
			8'd75: edgeReg_75 <= S_AXI_WDATA | edgeReg_75;
			8'd76: edgeReg_76 <= S_AXI_WDATA | edgeReg_76;
			8'd77: edgeReg_77 <= S_AXI_WDATA | edgeReg_77;
			8'd78: edgeReg_78 <= S_AXI_WDATA | edgeReg_78;
			8'd79: edgeReg_79 <= S_AXI_WDATA | edgeReg_79;
			8'd80: edgeReg_80 <= S_AXI_WDATA | edgeReg_80;
			8'd81: edgeReg_81 <= S_AXI_WDATA | edgeReg_81;
			8'd82: edgeReg_82 <= S_AXI_WDATA | edgeReg_82;
			8'd83: edgeReg_83 <= S_AXI_WDATA | edgeReg_83;
			8'd84: edgeReg_84 <= S_AXI_WDATA | edgeReg_84;
			8'd85: edgeReg_85 <= S_AXI_WDATA | edgeReg_85;
			8'd86: edgeReg_86 <= S_AXI_WDATA | edgeReg_86;
			8'd87: edgeReg_87 <= S_AXI_WDATA | edgeReg_87;
			8'd88: edgeReg_88 <= S_AXI_WDATA | edgeReg_88;
			8'd89: edgeReg_89 <= S_AXI_WDATA | edgeReg_89;
			8'd90: edgeReg_90 <= S_AXI_WDATA | edgeReg_90;
			8'd91: edgeReg_91 <= S_AXI_WDATA | edgeReg_91;
			8'd92: edgeReg_92 <= S_AXI_WDATA | edgeReg_92;
			8'd93: edgeReg_93 <= S_AXI_WDATA | edgeReg_93;
			8'd94: edgeReg_94 <= S_AXI_WDATA | edgeReg_94;
			8'd95: edgeReg_95 <= S_AXI_WDATA | edgeReg_95;
			8'd96: edgeReg_96 <= S_AXI_WDATA | edgeReg_96;
			8'd97: edgeReg_97 <= S_AXI_WDATA | edgeReg_97;
			8'd98: edgeReg_98 <= S_AXI_WDATA | edgeReg_98;
			8'd99: edgeReg_99 <= S_AXI_WDATA | edgeReg_99;
			8'd100: edgeReg_100 <= S_AXI_WDATA | edgeReg_100;
			8'd101: edgeReg_101 <= S_AXI_WDATA | edgeReg_101;
			8'd102: edgeReg_102 <= S_AXI_WDATA | edgeReg_102;
			8'd103: edgeReg_103 <= S_AXI_WDATA | edgeReg_103;
			8'd104: edgeReg_104 <= S_AXI_WDATA | edgeReg_104;
			8'd105: edgeReg_105 <= S_AXI_WDATA | edgeReg_105;
			8'd106: edgeReg_106 <= S_AXI_WDATA | edgeReg_106;
			8'd107: edgeReg_107 <= S_AXI_WDATA | edgeReg_107;
			8'd108: edgeReg_108 <= S_AXI_WDATA | edgeReg_108;
			8'd109: edgeReg_109 <= S_AXI_WDATA | edgeReg_109;
			8'd110: edgeReg_110 <= S_AXI_WDATA | edgeReg_110;
			8'd111: edgeReg_111 <= S_AXI_WDATA | edgeReg_111;
			8'd112: edgeReg_112 <= S_AXI_WDATA | edgeReg_112;
			8'd113: edgeReg_113 <= S_AXI_WDATA | edgeReg_113;
			8'd114: edgeReg_114 <= S_AXI_WDATA | edgeReg_114;
			8'd115: edgeReg_115 <= S_AXI_WDATA | edgeReg_115;
			8'd116: edgeReg_116 <= S_AXI_WDATA | edgeReg_116;
			8'd117: edgeReg_117 <= S_AXI_WDATA | edgeReg_117;
			8'd118: edgeReg_118 <= S_AXI_WDATA | edgeReg_118;
			8'd119: edgeReg_119 <= S_AXI_WDATA | edgeReg_119;
			8'd120: edgeReg_120 <= S_AXI_WDATA | edgeReg_120;
			8'd121: edgeReg_121 <= S_AXI_WDATA | edgeReg_121;
			8'd122: edgeReg_122 <= S_AXI_WDATA | edgeReg_122;
			8'd123: edgeReg_123 <= S_AXI_WDATA | edgeReg_123;
			8'd124: edgeReg_124 <= S_AXI_WDATA | edgeReg_124;
			8'd125: edgeReg_125 <= S_AXI_WDATA | edgeReg_125;
			8'd126: edgeReg_126 <= S_AXI_WDATA | edgeReg_126;
			8'd127: edgeReg_127 <= S_AXI_WDATA | edgeReg_127;
			8'd128: edgeReg_128 <= S_AXI_WDATA | edgeReg_128;
			8'd129: edgeReg_129 <= S_AXI_WDATA | edgeReg_129;
			8'd130: edgeReg_130 <= S_AXI_WDATA | edgeReg_130;
			8'd131: edgeReg_131 <= S_AXI_WDATA | edgeReg_131;
			8'd132: edgeReg_132 <= S_AXI_WDATA | edgeReg_132;
			8'd133: edgeReg_133 <= S_AXI_WDATA | edgeReg_133;
			8'd134: edgeReg_134 <= S_AXI_WDATA | edgeReg_134;
			8'd135: edgeReg_135 <= S_AXI_WDATA | edgeReg_135;
			8'd136: edgeReg_136 <= S_AXI_WDATA | edgeReg_136;
			8'd137: edgeReg_137 <= S_AXI_WDATA | edgeReg_137;
			8'd138: edgeReg_138 <= S_AXI_WDATA | edgeReg_138;
			8'd139: edgeReg_139 <= S_AXI_WDATA | edgeReg_139;
			8'd140: edgeReg_140 <= S_AXI_WDATA | edgeReg_140;
			8'd141: edgeReg_141 <= S_AXI_WDATA | edgeReg_141;
			8'd142: edgeReg_142 <= S_AXI_WDATA | edgeReg_142;
			8'd143: edgeReg_143 <= S_AXI_WDATA | edgeReg_143;
			8'd144: edgeReg_144 <= S_AXI_WDATA | edgeReg_144;
			8'd145: edgeReg_145 <= S_AXI_WDATA | edgeReg_145;
			8'd146: edgeReg_146 <= S_AXI_WDATA | edgeReg_146;
			8'd147: edgeReg_147 <= S_AXI_WDATA | edgeReg_147;
			8'd148: edgeReg_148 <= S_AXI_WDATA | edgeReg_148;
			8'd149: edgeReg_149 <= S_AXI_WDATA | edgeReg_149;
			8'd150: edgeReg_150 <= S_AXI_WDATA | edgeReg_150;
			8'd151: edgeReg_151 <= S_AXI_WDATA | edgeReg_151;
			8'd152: edgeReg_152 <= S_AXI_WDATA | edgeReg_152;
			8'd153: edgeReg_153 <= S_AXI_WDATA | edgeReg_153;
			8'd154: edgeReg_154 <= S_AXI_WDATA | edgeReg_154;
			8'd155: edgeReg_155 <= S_AXI_WDATA | edgeReg_155;
			8'd156: edgeReg_156 <= S_AXI_WDATA | edgeReg_156;
			8'd157: edgeReg_157 <= S_AXI_WDATA | edgeReg_157;
			8'd158: edgeReg_158 <= S_AXI_WDATA | edgeReg_158;
			8'd159: edgeReg_159 <= S_AXI_WDATA | edgeReg_159;
			8'd160: edgeReg_160 <= S_AXI_WDATA | edgeReg_160;
			8'd161: edgeReg_161 <= S_AXI_WDATA | edgeReg_161;
			8'd162: edgeReg_162 <= S_AXI_WDATA | edgeReg_162;
			8'd163: edgeReg_163 <= S_AXI_WDATA | edgeReg_163;
			8'd164: edgeReg_164 <= S_AXI_WDATA | edgeReg_164;
			8'd165: edgeReg_165 <= S_AXI_WDATA | edgeReg_165;
			8'd166: edgeReg_166 <= S_AXI_WDATA | edgeReg_166;
			8'd167: edgeReg_167 <= S_AXI_WDATA | edgeReg_167;
			8'd168: edgeReg_168 <= S_AXI_WDATA | edgeReg_168;
			8'd169: edgeReg_169 <= S_AXI_WDATA | edgeReg_169;
			8'd170: edgeReg_170 <= S_AXI_WDATA | edgeReg_170;
			8'd171: edgeReg_171 <= S_AXI_WDATA | edgeReg_171;
			8'd172: edgeReg_172 <= S_AXI_WDATA | edgeReg_172;
			8'd173: edgeReg_173 <= S_AXI_WDATA | edgeReg_173;
			8'd174: edgeReg_174 <= S_AXI_WDATA | edgeReg_174;
			8'd175: edgeReg_175 <= S_AXI_WDATA | edgeReg_175;
			8'd176: edgeReg_176 <= S_AXI_WDATA | edgeReg_176;
			8'd177: edgeReg_177 <= S_AXI_WDATA | edgeReg_177;
			8'd178: edgeReg_178 <= S_AXI_WDATA | edgeReg_178;
			8'd179: edgeReg_179 <= S_AXI_WDATA | edgeReg_179;
			8'd180: edgeReg_180 <= S_AXI_WDATA | edgeReg_180;
			8'd181: edgeReg_181 <= S_AXI_WDATA | edgeReg_181;
			8'd182: edgeReg_182 <= S_AXI_WDATA | edgeReg_182;
			8'd183: edgeReg_183 <= S_AXI_WDATA | edgeReg_183;
			8'd184: edgeReg_184 <= S_AXI_WDATA | edgeReg_184;
			8'd185: edgeReg_185 <= S_AXI_WDATA | edgeReg_185;
			8'd186: edgeReg_186 <= S_AXI_WDATA | edgeReg_186;
			8'd187: edgeReg_187 <= S_AXI_WDATA | edgeReg_187;
			8'd188: edgeReg_188 <= S_AXI_WDATA | edgeReg_188;
			8'd189: edgeReg_189 <= S_AXI_WDATA | edgeReg_189;
			8'd190: edgeReg_190 <= S_AXI_WDATA | edgeReg_190;
			8'd191: edgeReg_191 <= S_AXI_WDATA | edgeReg_191;
			8'd192: edgeReg_192 <= S_AXI_WDATA | edgeReg_192;
			8'd193: edgeReg_193 <= S_AXI_WDATA | edgeReg_193;
			8'd194: edgeReg_194 <= S_AXI_WDATA | edgeReg_194;
			8'd195: edgeReg_195 <= S_AXI_WDATA | edgeReg_195;
			8'd196: edgeReg_196 <= S_AXI_WDATA | edgeReg_196;
			8'd197: edgeReg_197 <= S_AXI_WDATA | edgeReg_197;
			8'd198: edgeReg_198 <= S_AXI_WDATA | edgeReg_198;
			8'd199: edgeReg_199 <= S_AXI_WDATA | edgeReg_199;
			8'd200: edgeReg_200 <= S_AXI_WDATA | edgeReg_200;
			8'd201: edgeReg_201 <= S_AXI_WDATA | edgeReg_201;
			8'd202: edgeReg_202 <= S_AXI_WDATA | edgeReg_202;
			8'd203: edgeReg_203 <= S_AXI_WDATA | edgeReg_203;
			8'd204: edgeReg_204 <= S_AXI_WDATA | edgeReg_204;
			8'd205: edgeReg_205 <= S_AXI_WDATA | edgeReg_205;
			8'd206: edgeReg_206 <= S_AXI_WDATA | edgeReg_206;
			8'd207: edgeReg_207 <= S_AXI_WDATA | edgeReg_207;
			8'd208: edgeReg_208 <= S_AXI_WDATA | edgeReg_208;
			8'd209: edgeReg_209 <= S_AXI_WDATA | edgeReg_209;
			8'd210: edgeReg_210 <= S_AXI_WDATA | edgeReg_210;
			8'd211: edgeReg_211 <= S_AXI_WDATA | edgeReg_211;
			8'd212: edgeReg_212 <= S_AXI_WDATA | edgeReg_212;
			8'd213: edgeReg_213 <= S_AXI_WDATA | edgeReg_213;
			8'd214: edgeReg_214 <= S_AXI_WDATA | edgeReg_214;
			8'd215: edgeReg_215 <= S_AXI_WDATA | edgeReg_215;
			8'd216: edgeReg_216 <= S_AXI_WDATA | edgeReg_216;
			8'd217: edgeReg_217 <= S_AXI_WDATA | edgeReg_217;
			8'd218: edgeReg_218 <= S_AXI_WDATA | edgeReg_218;
			8'd219: edgeReg_219 <= S_AXI_WDATA | edgeReg_219;
			8'd220: edgeReg_220 <= S_AXI_WDATA | edgeReg_220;
			8'd221: edgeReg_221 <= S_AXI_WDATA | edgeReg_221;
			8'd222: edgeReg_222 <= S_AXI_WDATA | edgeReg_222;
			8'd223: edgeReg_223 <= S_AXI_WDATA | edgeReg_223;
			8'd224: edgeReg_224 <= S_AXI_WDATA | edgeReg_224;
			8'd225: edgeReg_225 <= S_AXI_WDATA | edgeReg_225;
			8'd226: edgeReg_226 <= S_AXI_WDATA | edgeReg_226;
			8'd227: edgeReg_227 <= S_AXI_WDATA | edgeReg_227;
			8'd228: edgeReg_228 <= S_AXI_WDATA | edgeReg_228;
			8'd229: edgeReg_229 <= S_AXI_WDATA | edgeReg_229;
			8'd230: edgeReg_230 <= S_AXI_WDATA | edgeReg_230;
			8'd231: edgeReg_231 <= S_AXI_WDATA | edgeReg_231;
			8'd232: edgeReg_232 <= S_AXI_WDATA | edgeReg_232;
			8'd233: edgeReg_233 <= S_AXI_WDATA | edgeReg_233;
			8'd234: edgeReg_234 <= S_AXI_WDATA | edgeReg_234;
			8'd235: edgeReg_235 <= S_AXI_WDATA | edgeReg_235;
			8'd236: edgeReg_236 <= S_AXI_WDATA | edgeReg_236;
			8'd237: edgeReg_237 <= S_AXI_WDATA | edgeReg_237;
			8'd238: edgeReg_238 <= S_AXI_WDATA | edgeReg_238;
			8'd239: edgeReg_239 <= S_AXI_WDATA | edgeReg_239;
			8'd240: edgeReg_240 <= S_AXI_WDATA | edgeReg_240;
			8'd241: edgeReg_241 <= S_AXI_WDATA | edgeReg_241;
			8'd242: edgeReg_242 <= S_AXI_WDATA | edgeReg_242;
			8'd243: edgeReg_243 <= S_AXI_WDATA | edgeReg_243;
			8'd244: edgeReg_244 <= S_AXI_WDATA | edgeReg_244;
			8'd245: edgeReg_245 <= S_AXI_WDATA | edgeReg_245;
			8'd246: edgeReg_246 <= S_AXI_WDATA | edgeReg_246;
			8'd247: edgeReg_247 <= S_AXI_WDATA | edgeReg_247;
			8'd248: edgeReg_248 <= S_AXI_WDATA | edgeReg_248;
			8'd249: edgeReg_249 <= S_AXI_WDATA | edgeReg_249;
			8'd250: edgeReg_250 <= S_AXI_WDATA | edgeReg_250;
			8'd251: edgeReg_251 <= S_AXI_WDATA | edgeReg_251;
			8'd252: edgeReg_252 <= S_AXI_WDATA | edgeReg_252;
			8'd253: edgeReg_253 <= S_AXI_WDATA | edgeReg_253;
			8'd254: edgeReg_254 <= S_AXI_WDATA | edgeReg_254;
			8'd255: edgeReg_255 <= S_AXI_WDATA | edgeReg_255;
		endcase
	end


end

// generate

// for ( i = 0; i < 8; i = i + 1 ) begin
// 	always @( posedge S_AXI_ACLK ) begin 
// 		if ( S_AXI_ARESETN == 1'b0 || clear == 1'b1 ) begin
			
// 				// oneGrid_Reg[i] <= 32'b0;
	
// 		end

// 		else begin
// 			if ( axi_awv_awr_flag ) begin
// 				oneGrid_Reg[ mem_address ] <= S_AXI_WDATA;
// 			end
// 		end
// 	end
// end


// 	for ( i = 0; i < 8; i = i + 1 ) begin
// 		assign oneGridState[1024*i+1023 : 1024*i] = oneGrid_Reg[ i ];
// 	end


// endgenerate


endmodule
