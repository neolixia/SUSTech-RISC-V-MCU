/***************************************************
*	Module Name:I2C		   
*	Description:I2C控制器，兼容1字节和2字节地址段器件
**************************************************/
module I2C(
	Clk,
	Rst_n,
	
	Rddata_num,
	Wrdata_num,
	Wdaddr_num,
	
	Device_addr,
	Word_addr,
	
	Wr,
	Wr_data,
	Wr_data_vaild,
	Rd,
	Rd_data,
	Rd_data_vaild,
	
	Scl,
	Sda,
	Done
);

//系统时钟采用50MHz
parameter SYS_CLOCK = 50_000_000;
//SCL总线时钟采用400kHz
parameter SCL_CLOCK = 400_000;
//产生时钟SCL计数器最大值
localparam SCL_CNT_M = SYS_CLOCK/SCL_CLOCK;

	input             Clk;          //系统时钟
	input             Rst_n;        //系统复位信号
	
	input     [5:0]   Rddata_num;   //I2C总线连续读取数据字节数
	input     [5:0]   Wrdata_num;   //I2C总线连续读取数据字节数
	input     [1:0]   Wdaddr_num;   //I2C器件数据地址字节数
	
	input     [2:0]   Device_addr;  //I2C器件地址
	input     [15:0]  Word_addr;    //I2C寄存器地址
	
	input             Wr;           //I2C器件写使能
	input     [7:0]   Wr_data;      //I2C器件写数据
	output            Wr_data_vaild;//I2C器件写数据有效标志位
	input             Rd;           //I2C器件读使能
	output reg[7:0]   Rd_data;      //I2C器件读数据
	output reg        Rd_data_vaild;//I2C器件读数据有效标志位
	
	output  reg       Scl;          //I2C时钟线
	inout             Sda;          //I2C数据线
	output  reg       Done;         //对I2C器件读写完成标识位
	
  
	
	//主状态机状态
	localparam  IDLE     = 9'b0_0000_0001,//空闲状态
               WR_START = 9'b0_0000_0010,//写开始状态
               WR_CTRL  = 9'b0_0000_0100,//写控制状态
               WR_WADDR = 9'b0_0000_1000,//写地址状态
               WR_DATA  = 9'b0_0001_0000,//写数据状态
               RD_START = 9'b0_0010_0000,//读开始状态
               RD_CTRL  = 9'b0_0100_0000,//读控制状态
               RD_DATA  = 9'b0_1000_0000,//读数据状态
               STOP     = 9'b1_0000_0000;//停止状态
				
	reg [8:0] main_state;	   //主状态机状态寄存器
	reg       sda_en;          //sda数据总线控制位
	reg       sda_reg;         //sda数据输出寄存器
	reg       W_flag;          //IIC写操作标志位
	reg       R_flag;          //IIC读操作标志位
	reg       FF;              //串行输出输入任务执行标志位
	wire[7:0] wr_ctrl_word;    //写控制数据寄存器
	wire[7:0] rd_ctrl_word;    //读控制数据寄存器
	reg [15:0]scl_cnt;         //SCL时钟计数器
	reg       scl_vaild;       //IIC非空闲时期
	reg       scl_high;        //SCL时钟高电平中部标志位
	reg       scl_low;         //SCL时钟低电平中部标志位	
	reg [7:0] halfbit_cnt;     //串行数据传输计数器
	reg       ack;             //串行输出输入高低电平计数完成标志位
	reg [1:0] waddr_cnt;       //地址字节数计数器
	reg [7:0] wdata_cnt;       //写数据字节数计数器
	reg [7:0] rdata_cnt;       //读数据字节数计数器
	reg [7:0] sda_data_out;    //待输出SDA串行数据
	reg [7:0] sda_data_in;     //SDA串行输入后数据
	wire      rdata_vaild_r; 
	
	//读写控制字
	assign wr_ctrl_word = {4'b1010, Device_addr,1'b0};
	assign rd_ctrl_word = {4'b1010, Device_addr,1'b1};
	
	//I2C数据线采用三态传输
	assign Sda = sda_en ? sda_reg : 1'bz;
	
	//I2C非空闲时期scl_vaild的产生
	always@(posedge Clk or negedge Rst_n)
	begin
		if(!Rst_n)
			scl_vaild <= 1'b0;
		else if(Wr | Rd)
			scl_vaild <= 1'b1;
		else if(Done)
			scl_vaild <= 1'b0;
		else
			scl_vaild <= scl_vaild;
	end

	//scl时钟计数器
	always@(posedge Clk or negedge Rst_n)
	begin
		if(!Rst_n)
			scl_cnt <= 16'd0;
		else if(scl_vaild)begin
			if(scl_cnt == SCL_CNT_M - 1)
				scl_cnt <= 16'd0;
			else
				scl_cnt <= scl_cnt + 16'd1;
		end
		else
			scl_cnt <= 16'd0;
	end

	//scl时钟,在计数器值到达最大值一半和0时翻转
	always@(posedge Clk or negedge Rst_n)
	begin
		if(!Rst_n)
			Scl <= 1'b1;
		else if(scl_cnt == SCL_CNT_M >>1)
			Scl <= 1'b0;
		else if(scl_cnt == 16'd0)
			Scl <= 1'b1;
		else
			Scl <= Scl;
	end	

	//scl时钟高低电平中部标志位
	always@(posedge Clk or negedge Rst_n)
	begin
		if(!Rst_n)
			scl_high <= 1'b0;
		else if(scl_cnt == (SCL_CNT_M>>2))
			scl_high <= 1'b1;
		else
			scl_high <= 1'b0;
	end
	//scl时钟低电平中部标志位
	always@(posedge Clk or negedge Rst_n)
	begin
		if(!Rst_n)
			scl_low <= 1'b0;
		else if(scl_cnt == (SCL_CNT_M>>1)+(SCL_CNT_M>>2))
			scl_low <= 1'b1;
		else
			scl_low <= 1'b0;
	end	

	//主状态机
	always@(posedge Clk or negedge Rst_n)
	begin
		if(!Rst_n)begin
			main_state <= IDLE;
			sda_reg    <= 1'b1;
			W_flag     <= 1'b0;       
			R_flag     <= 1'b0;			
			Done       <= 1'b0;
			waddr_cnt  <= 2'd1;
			wdata_cnt  <= 8'd1;
			rdata_cnt  <= 8'd1;
		end
		else begin
			case(main_state)
				IDLE:begin
					sda_reg   <= 1'b1;
					W_flag    <= 1'b0;
					R_flag    <= 1'b0;
					Done      <= 1'b0;
					waddr_cnt <= 2'd1;
					wdata_cnt <= 8'd1;
					rdata_cnt <= 8'd1;
					if(Wr)begin
						main_state <= WR_START;
						W_flag     <= 1'b1;
					end
					else if(Rd)begin
						main_state <= WR_START;
						R_flag     <= 1'b1;
					end
					else
						main_state <= IDLE;
				end

				WR_START:begin
					if(scl_low)begin
						main_state   <= WR_CTRL;
						sda_data_out <= wr_ctrl_word;
						FF           <= 1'b0;
					end
					else if(scl_high)begin
						sda_reg    <= 1'b0;
						main_state <= WR_START;
					end
					else
						main_state <= WR_START;
				end

				WR_CTRL:begin
					if(FF == 1'b0)
						send_8bit_data;
					else begin
						if(ack == 1'b1) begin//收到响应
							if(scl_low)begin
								main_state   <= WR_WADDR;
								FF           <= 1'b0;
								if(Wdaddr_num == 2'b1)
									sda_data_out <= Word_addr[7:0];
								else
									sda_data_out <= Word_addr[15:8];
							end
							else
								main_state   <= WR_CTRL;
						end
						else//未收到响应
							main_state      <= IDLE;
					end
				end

				WR_WADDR:begin
					if(FF == 1'b0)
						send_8bit_data;
					else begin
						if(ack == 1'b1) begin//收到响应
							if(waddr_cnt == Wdaddr_num)begin
								if(W_flag && scl_low)begin
									main_state   <= WR_DATA;
									sda_data_out <= Wr_data;
									waddr_cnt    <= 2'd1;
									FF           <= 1'b0;
								end
								else if(R_flag && scl_low)begin
									main_state <= RD_START;
									sda_reg    <= 1'b1;
								end
								else
									main_state <= WR_WADDR;
							end
							else begin
								if(scl_low)begin
									waddr_cnt    <= waddr_cnt + 2'd1;
									main_state   <= WR_WADDR;
									sda_data_out <= Word_addr[7:0];
									FF           <= 1'b0;
								end
								else
									main_state   <= WR_WADDR;
							end
						end
						else//未收到响应
							main_state <= IDLE;
					end
				end

				WR_DATA:begin
					if(FF == 1'b0)
						send_8bit_data;
					else begin
						if(ack == 1'b1) begin//收到响应
							if(wdata_cnt == Wrdata_num)begin
								if(scl_low)begin
									main_state <= STOP;
									sda_reg    <= 1'b0;
									wdata_cnt  <= 8'd1;
								end
								else
									main_state <= WR_DATA;
							end
							else begin
								if(scl_low)begin
									wdata_cnt    <= wdata_cnt + 8'd1;
									main_state   <= WR_DATA;
									sda_data_out <= Wr_data;
									FF           <= 1'b0;
								end
								else
									main_state   <= WR_DATA;
							end
						end
						else//未收到响应
							main_state <= IDLE;
					end
				end

				RD_START:begin
					if(scl_low)begin
						main_state   <= RD_CTRL;
						sda_data_out <= rd_ctrl_word;
						FF           <= 1'b0;
					end
					else if(scl_high)begin
						main_state <= RD_START;
						sda_reg    <= 1'b0;
					end
					else
						main_state <= RD_START;
				end

				RD_CTRL:begin
					if(FF == 1'b0)
						send_8bit_data;
					else begin
						if(ack == 1'b1) begin//收到响应
							if(scl_low)begin
								main_state <= RD_DATA;
								FF         <= 1'b0;
							end
							else
								main_state <= RD_CTRL;
						end
						else//未收到响应
							main_state    <= IDLE;
					end
				end
				
				RD_DATA:begin
					if(FF == 1'b0)
						receive_8bit_data;
					else begin
						if(rdata_cnt == Rddata_num)begin
							sda_reg <= 1'b1;
							if(scl_low)begin
								main_state <= STOP;
								sda_reg    <= 1'b0;
							end
							else
								main_state <= RD_DATA;
						end
						else begin
							sda_reg <= 1'b0;
							if(scl_low)begin
								rdata_cnt  <= rdata_cnt + 8'd1;
								main_state <= RD_DATA;
								FF         <= 1'b0;
							end
							else
								main_state <= RD_DATA;
						end
					end
				end

				STOP:begin//结束操作
					if(scl_high)begin
						sda_reg    <= 1'b1;
						main_state <= IDLE;
						Done       <= 1'b1;
					end
					else
						main_state <= STOP;
				end
				
				default:
					main_state <= IDLE;
			endcase
		end
	end
		
	//sda串行接收与发送时scl高低电平计数器
	always@(posedge Clk or negedge Rst_n)
	begin
		if(!Rst_n)
			halfbit_cnt <= 8'd0;
		else if((main_state == WR_CTRL)||
		        (main_state == WR_WADDR)||
				  (main_state == WR_DATA)||
				  (main_state == RD_CTRL)||
				  (main_state == RD_DATA))begin
			if(scl_low | scl_high)begin
				if(halfbit_cnt == 8'd17)
					halfbit_cnt <= 8'd0;
				else
					halfbit_cnt <= halfbit_cnt + 8'd1;
			end
			else
				halfbit_cnt <= halfbit_cnt;
		end
		else
			halfbit_cnt <= 8'd0;
	end

	//数据接收方对发送的响应检测标志位
	always@(posedge Clk or negedge Rst_n)
	begin
		if(!Rst_n)
			ack <= 1'b0;
		else if((halfbit_cnt == 8'd16)&&scl_high&&(Sda==1'b0))
			ack <= 1'b1;
		else if((halfbit_cnt == 8'd17)&&scl_low)
			ack <= 1'b0;
		else
			ack <= ack;
	end
	
   //输出串行数据任务
	task send_8bit_data;
	if(scl_high && (halfbit_cnt == 8'd16))
		FF <= 1;
	else if(halfbit_cnt < 8'd17)begin
		sda_reg <= sda_data_out[7];
		if(scl_low)
			sda_data_out <= {sda_data_out[6:0],1'b0};
		else
			sda_data_out <= sda_data_out;
	end
	else
		;
	endtask
	
	//串行数据输入任务
	task receive_8bit_data;
	if(scl_low && (halfbit_cnt == 8'd15))
		FF <= 1;
	else if((halfbit_cnt < 8'd15))begin
		if(scl_high)
			sda_data_in <= {sda_data_in[6:0],Sda};
		else begin
			sda_data_in <= sda_data_in;
		end
	end
	else
		;
	endtask

	//sda三态使能信号sda_en
	always@(*)
	begin
		case(main_state)
		IDLE:
			sda_en = 1'b0;

		WR_START,RD_START,STOP:
			sda_en = 1'b1;

		WR_CTRL,WR_WADDR,WR_DATA,RD_CTRL:
			if(halfbit_cnt < 16)
				sda_en = 1'b1;
			else
				sda_en = 1'b0;

		RD_DATA:
			if(halfbit_cnt < 16)
				sda_en = 1'b0;
			else
				sda_en = 1'b1;		
		default:

			sda_en = 1'b0;		
		endcase
	end

	//写数据有效标志位
	assign Wr_data_vaild = ((main_state==WR_WADDR)&&
	                       (waddr_cnt==Wdaddr_num)&&
								  (W_flag && scl_low)&&
								  (ack == 1'b1))||
                     	  ((main_state == WR_DATA)&&
								  (ack == 1'b1)&&(scl_low)&&
								  (wdata_cnt != Wrdata_num));

	//读数据有效标志位前寄存器
	assign rdata_vaild_r = (main_state == RD_DATA)
	                        &&(halfbit_cnt == 8'd15)&&scl_low;

	//读出数据有效标志位
	always@(posedge Clk or negedge Rst_n)
	begin
	if(!Rst_n)
		Rd_data_vaild <= 1'b0;
	else if(rdata_vaild_r)
		Rd_data_vaild <= 1'b1;
	else
		Rd_data_vaild <= 1'b0;
	end

	//读出的有效数据
	always@(posedge Clk or negedge Rst_n)
	begin
	if(!Rst_n)
		Rd_data <= 8'd0;
	else if(rdata_vaild_r)
		Rd_data <= sda_data_in;
	else
		Rd_data <= Rd_data;
	end

endmodule 