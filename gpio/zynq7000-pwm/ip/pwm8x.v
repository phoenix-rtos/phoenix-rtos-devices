/*
 * Phoenix-RTOS
 *
 * Xilinx Zynq 7000 PWM8X IP core
 *
 * Copyright 2022 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

`timescale 1 ns / 1 ps

module pwm8x_v0_1 #
(
	// Users to add parameters here

	// User parameters ends
	// Do not modify the parameters beyond this line


	// Parameters of Axi Slave Bus Interface S00_AXI
	parameter integer C_S00_AXI_DATA_WIDTH	= 32,
	parameter integer C_S00_AXI_ADDR_WIDTH	= 6
)
(
	// Users to add ports here

	output reg [7:0] pwm,

	// User ports ends
	// Do not modify the ports beyond this line


	// Ports of Axi Slave Bus Interface S00_AXI
	input wire  s00_axi_aclk,
	input wire  s00_axi_aresetn,
	input wire [C_S00_AXI_ADDR_WIDTH-1 : 0] s00_axi_awaddr,
	input wire [2 : 0] s00_axi_awprot,
	input wire  s00_axi_awvalid,
	output wire  s00_axi_awready,
	input wire [C_S00_AXI_DATA_WIDTH-1 : 0] s00_axi_wdata,
	input wire [(C_S00_AXI_DATA_WIDTH/8)-1 : 0] s00_axi_wstrb,
	input wire  s00_axi_wvalid,
	output wire  s00_axi_wready,
	output wire [1 : 0] s00_axi_bresp,
	output wire  s00_axi_bvalid,
	input wire  s00_axi_bready,
	input wire [C_S00_AXI_ADDR_WIDTH-1 : 0] s00_axi_araddr,
	input wire [2 : 0] s00_axi_arprot,
	input wire  s00_axi_arvalid,
	output wire  s00_axi_arready,
	output wire [C_S00_AXI_DATA_WIDTH-1 : 0] s00_axi_rdata,
	output wire [1 : 0] s00_axi_rresp,
	output wire  s00_axi_rvalid,
	input wire  s00_axi_rready
);

wire clk;
wire rstn;

wire [31:0] comp0;
wire [31:0] comp1;
wire [31:0] comp2;
wire [31:0] comp3;
wire [31:0] comp4;
wire [31:0] comp5;
wire [31:0] comp6;
wire [31:0] comp7;
wire [31:0] reload;

reg [31:0] counter;

reg [31:0] comp0_buff;
reg [31:0] comp1_buff;
reg [31:0] comp2_buff;
reg [31:0] comp3_buff;
reg [31:0] comp4_buff;
reg [31:0] comp5_buff;
reg [31:0] comp6_buff;
reg [31:0] comp7_buff;

// Instantiation of Axi Bus Interface S00_AXI
pwm8x_v0_1_S00_AXI # (
	.C_S_AXI_DATA_WIDTH(C_S00_AXI_DATA_WIDTH),
	.C_S_AXI_ADDR_WIDTH(C_S00_AXI_ADDR_WIDTH)
) pwm8x_v0_1_S00_AXI_inst (
	.reg0(comp0),
	.reg1(comp1),
	.reg2(comp2),
	.reg3(comp3),
	.reg4(comp4),
	.reg5(comp5),
	.reg6(comp6),
	.reg7(comp7),
	.reg8(reload),
	.S_AXI_ACLK(s00_axi_aclk),
	.S_AXI_ARESETN(s00_axi_aresetn),
	.S_AXI_AWADDR(s00_axi_awaddr),
	.S_AXI_AWPROT(s00_axi_awprot),
	.S_AXI_AWVALID(s00_axi_awvalid),
	.S_AXI_AWREADY(s00_axi_awready),
	.S_AXI_WDATA(s00_axi_wdata),
	.S_AXI_WSTRB(s00_axi_wstrb),
	.S_AXI_WVALID(s00_axi_wvalid),
	.S_AXI_WREADY(s00_axi_wready),
	.S_AXI_BRESP(s00_axi_bresp),
	.S_AXI_BVALID(s00_axi_bvalid),
	.S_AXI_BREADY(s00_axi_bready),
	.S_AXI_ARADDR(s00_axi_araddr),
	.S_AXI_ARPROT(s00_axi_arprot),
	.S_AXI_ARVALID(s00_axi_arvalid),
	.S_AXI_ARREADY(s00_axi_arready),
	.S_AXI_RDATA(s00_axi_rdata),
	.S_AXI_RRESP(s00_axi_rresp),
	.S_AXI_RVALID(s00_axi_rvalid),
	.S_AXI_RREADY(s00_axi_rready)
);

// Add user logic here

assign clk = s00_axi_aclk;
assign rstn = s00_axi_aresetn;

always @(posedge clk, negedge rstn) begin
	if (~rstn) begin
		counter <= 0;
		pwm <= 0;
		comp0_buff <= 0;
		comp1_buff <= 0;
		comp2_buff <= 0;
		comp3_buff <= 0;
		comp4_buff <= 0;
		comp5_buff <= 0;
		comp6_buff <= 0;
		comp7_buff <= 0;
	end else begin
		if (counter >= reload) begin
			counter <= 0;
			comp0_buff <= comp0;
			comp1_buff <= comp1;
			comp2_buff <= comp2;
			comp3_buff <= comp3;
			comp4_buff <= comp4;
			comp5_buff <= comp5;
			comp6_buff <= comp6;
			comp7_buff <= comp7;
		end else begin
			counter <= counter + 1;
			comp0_buff <= comp0_buff;
			comp1_buff <= comp1_buff;
			comp2_buff <= comp2_buff;
			comp3_buff <= comp3_buff;
			comp4_buff <= comp4_buff;
			comp5_buff <= comp5_buff;
			comp6_buff <= comp6_buff;
			comp7_buff <= comp7_buff;
		end

		pwm[0] <= (comp0_buff > counter);
		pwm[1] <= (comp1_buff > counter);
		pwm[2] <= (comp2_buff > counter);
		pwm[3] <= (comp3_buff > counter);
		pwm[4] <= (comp4_buff > counter);
		pwm[5] <= (comp5_buff > counter);
		pwm[6] <= (comp6_buff > counter);
		pwm[7] <= (comp7_buff > counter);
	end
end

// User logic ends

endmodule
