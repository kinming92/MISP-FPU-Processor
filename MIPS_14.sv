module MIPS_14(
    input logic clk, rst,
    output logic y
    );
logic [7:0] instr_mem [0:63];   
logic [31:0] regi [0:31]; // MIPS Processor Register
logic [7:0] RAM [0:63];
logic [31:0] NPC, PC;
logic [5:0] op, funct;
logic [4:0] rs,rt,rd,shamt;
logic [15:0] imm;
logic [25:0] addr;
logic [31:0] instr;
logic [2:0] alu_op;
logic [31:0] a,b,c;
logic [5:0] PCi;

logic[2:0] state;
logic[2:0] count;
logic start, init_count;

//////////////FPU////////////////////////////////////////
logic [31:0] regfp [0:31]; // FPU Coprocessor Register
logic fpu_op;
logic [31:0] a1,b1, c1;
logic [4:0] ft, fs, fd;
logic startCP1;

parameter S0= 3'd0, S1 = 3'd1,S2=3'd2, S3=3'd3,S4=3'd4, WAIT_STATE = 3'd6;


parameter  ALU_FCN= 6'b000_000, ADDI= 6'b001_000,
BEQ=6'b000_100, J=6'b000_010, LW=6'b100_011, SW=6'b101_011;

//floating pointing coprocessor parameter
parameter ADD_S = 6'b010_001, SWC1 = 6'b111_001, LWC1 = 6'b110_001;

parameter MEM_INIT = "C:/Users/kmloh/Desktop/Project_6/project_6/Project6_ASM/float_add_8.txt";
parameter RAM_INIT = "C:/Users/kmloh/Desktop/Project_6/project_6/Project6_ASM/float_add_data_8.txt";
initial begin  


//regi[0] = 32'h0;
//    integer i;
//    for(i = 0; i<32;i=i+1) begin
//    regi[i] = 32'h0;
//    regfp[i] = 32'h0;
//    end
    
    //assume $t0 is a constant zero 
    regi[8] = 32'h0;
    regfp[8] = 32'h0;
    $readmemh(MEM_INIT, instr_mem); // initialize the instruction memory
    $readmemh(RAM_INIT, RAM);
end

always_comb begin //control logic
  

    PCi=PC[5:0];
    instr= {instr_mem[PCi],instr_mem[PCi+1],instr_mem[PCi+2],instr_mem[PCi+3]};
    
    op= instr[31:26]; // braking the instruction into chunks
    addr= instr[25:0];
    rs= instr [25:21];
    rt= instr[20:16];
    imm= instr[15:0];
    rd= instr[15:11];
    shamt= instr[10:6];
    funct= instr[5:0];
	
	ft = instr[20:16];
	fs = instr[15:11];
	fd = instr[10:6];
    
    if(instr==32'b0)begin
         NPC=32'bX;
         PC=32'bX;
    end     
    else begin
    NPC=PC+4; // program counter

    case(op) 
        ALU_FCN: begin
            a= regi[rs];
            b= regi[rt]; 
            if(funct==6'b100_000) alu_op=3'd2; // add
            else if(funct==6'b100_010) alu_op=3'd3; //sub
            else if(funct==6'b100_100) alu_op=3'd4; // and
            else if(funct==6'b100_101) alu_op=3'd5; //or
			else if(funct==6'b100_110) alu_op=3'd6; //xor
        end
        ADDI: begin
            a = regi[rs];
            b= {{16{imm[15]}},imm};
            alu_op=3'd2; // add
        end
        BEQ: begin
//            op= 3'd3;
            a= regi[rs];
            b= regi[rt];
            if(a==b) NPC= PC +{imm,2'b00}+4;
        end
        J: begin
            NPC= {PC[31:28],addr,2'b00};
           //NPC = ((PC & 32'hf0000000) | (addr << 2));
        end
        LW: begin
            alu_op=3'd2;
            a=regi[rs];
            b=imm;
        end
        SW: begin
            alu_op=3'd2;
            a=regi[rs];
            b=imm;
        end
		ADD_S: begin
			fpu_op = 1'b0;
		    a1 = regfp[ft];
            b1 = regfp[fs];
		end
		LWC1: begin
            alu_op=3'd2;
            a=regi[rs];
            b=imm;
		end
		
		SWC1: begin
			alu_op=3'd2;
            a=regi[rs];
            b=imm;
		
		end
    endcase 
    end
end   
always_ff@(posedge clk, posedge rst) begin
    if (rst) begin
        PC<='b0;
        state<=S0;
        count<=S0;
    end
    else begin 
    case(state)
        S0: begin
            if (~(op==ADD_S))PC<=NPC; 
            case (op)
                  //MIPS processor
                  ALU_FCN: regi[rd] <=c;
                  ADDI: regi[rt] <=c; 
                  LW: regi[rt]<= {RAM[c], RAM[c+1], RAM[c+2], RAM[c+3]};
                  SW: {RAM[c], RAM[c+1], RAM[c+2], RAM[c+3]}<= regi[rt];
                  
                  //FPU Coprocessor
                  ADD_S: begin
                         case(count) 
                            S0: begin
                                start<=1'b1;
                                count<=S1;
                            end
                            S1: begin
                                start<=1'b0;
                                count<=S2;
                            end
                            S2: count<=S3;
                            S3: count<=S4;        
                            S4: begin
                                count<=S0;
                                PC<=NPC;
                                regfp[fd]<=c1;
                            end
                         endcase
                  end
                  LWC1: regfp[ft]<= {RAM[c], RAM[c+1], RAM[c+2], RAM[c+3]};
                  SWC1: {RAM[c], RAM[c+1], RAM[c+2], RAM[c+3]}<= regfp[ft];
            endcase
        end
    endcase
    end
end

assign y = ^{RAM[c], RAM[c+1], RAM[c+2], RAM[c+3]};
ALU brain (clk, rst ,a, b, alu_op, c); // ALU module
fp_14 fpu (clk, rst,start, fpu_op, a1, b1, c1); //fpu module
    
endmodule

module fa(
	input	a, b, cin,
	output	cout, sum
);
  
	assign cout = (a&b)|(b&cin)|(cin&a);
	assign sum = a^b^cin;
  
endmodule

module ALU(
    input clk,rst,
	input			[31:0]	a, b,
	input			[2:0]	op,
	output	logic	[31:0]	c
);

	logic	[31:0]	a1, b1, c1, cin, cout, sum;
	logic			ov;
	


	fa fa0 (.a(a1[0]), .b(b1[0]), .cin(cin[0]), .cout(cout[0]), .sum(sum[0]));
	fa fa1 (.a(a1[1]), .b(b1[1]), .cin(cin[1]), .cout(cout[1]), .sum(sum[1]));
	fa fa2 (.a(a1[2]), .b(b1[2]), .cin(cin[2]), .cout(cout[2]), .sum(sum[2]));
	fa fa3 (.a(a1[3]), .b(b1[3]), .cin(cin[3]), .cout(cout[3]), .sum(sum[3]));
	fa fa4 (.a(a1[4]), .b(b1[4]), .cin(cin[4]), .cout(cout[4]), .sum(sum[4]));
	fa fa5 (.a(a1[5]), .b(b1[5]), .cin(cin[5]), .cout(cout[5]), .sum(sum[5]));
	fa fa6 (.a(a1[6]), .b(b1[6]), .cin(cin[6]), .cout(cout[6]), .sum(sum[6]));
	fa fa7 (.a(a1[7]), .b(b1[7]), .cin(cin[7]), .cout(cout[7]), .sum(sum[7]));
	fa fa8 (.a(a1[8]), .b(b1[8]), .cin(cin[8]), .cout(cout[8]), .sum(sum[8]));
    fa fa9 (.a(a1[9]), .b(b1[9]), .cin(cin[9]), .cout(cout[9]), .sum(sum[9]));
    fa fa10 (.a(a1[10]), .b(b1[10]), .cin(cin[10]), .cout(cout[10]), .sum(sum[10]));
    fa fa11 (.a(a1[11]), .b(b1[11]), .cin(cin[11]), .cout(cout[11]), .sum(sum[11]));
    fa fa12 (.a(a1[12]), .b(b1[12]), .cin(cin[12]), .cout(cout[12]), .sum(sum[12]));
    fa fa13 (.a(a1[13]), .b(b1[13]), .cin(cin[13]), .cout(cout[13]), .sum(sum[13]));
    fa fa14 (.a(a1[14]), .b(b1[14]), .cin(cin[14]), .cout(cout[14]), .sum(sum[14]));
    fa fa15 (.a(a1[15]), .b(b1[15]), .cin(cin[15]), .cout(cout[15]), .sum(sum[15]));
	fa fa16 (.a(a1[16]), .b(b1[16]), .cin(cin[16]), .cout(cout[16]), .sum(sum[16]));
    fa fa17 (.a(a1[17]), .b(b1[17]), .cin(cin[17]), .cout(cout[17]), .sum(sum[17]));
    fa fa18 (.a(a1[18]), .b(b1[18]), .cin(cin[18]), .cout(cout[18]), .sum(sum[18]));
    fa fa19 (.a(a1[19]), .b(b1[19]), .cin(cin[19]), .cout(cout[19]), .sum(sum[19]));
    fa fa20 (.a(a1[20]), .b(b1[20]), .cin(cin[20]), .cout(cout[20]), .sum(sum[20]));
    fa fa21 (.a(a1[21]), .b(b1[21]), .cin(cin[21]), .cout(cout[21]), .sum(sum[21]));
    fa fa22 (.a(a1[22]), .b(b1[22]), .cin(cin[22]), .cout(cout[22]), .sum(sum[22]));
    fa fa23 (.a(a1[23]), .b(b1[23]), .cin(cin[23]), .cout(cout[23]), .sum(sum[23]));
	fa fa24 (.a(a1[24]), .b(b1[24]), .cin(cin[24]), .cout(cout[24]), .sum(sum[24]));
    fa fa25 (.a(a1[25]), .b(b1[25]), .cin(cin[25]), .cout(cout[25]), .sum(sum[25]));
    fa fa26 (.a(a1[26]), .b(b1[26]), .cin(cin[26]), .cout(cout[26]), .sum(sum[26]));
    fa fa27 (.a(a1[27]), .b(b1[27]), .cin(cin[27]), .cout(cout[27]), .sum(sum[27]));
    fa fa28 (.a(a1[28]), .b(b1[28]), .cin(cin[28]), .cout(cout[28]), .sum(sum[28]));
    fa fa29 (.a(a1[29]), .b(b1[29]), .cin(cin[29]), .cout(cout[29]), .sum(sum[29]));
    fa fa30 (.a(a1[30]), .b(b1[30]), .cin(cin[30]), .cout(cout[30]), .sum(sum[30]));
    fa fa31 (.a(a1[31]), .b(b1[31]), .cin(cin[31]), .cout(cout[31]), .sum(sum[31]));        	
	
	assign ov  = a1[31]^b1[31]? sum[31] : cout[31];
	
	always @(posedge clk or posedge rst)
		if(rst) c1 <= 'b0;
		else 	c1 <= c;
	
	always_comb begin
		case(op)
			'd0: begin
				a1  = a;
				b1  = 'h0;
				cin = 'h0;
				c	=  sum; //sign extension
			end
			'd1: begin
				a1  = 'h0;
				b1  = b;
				cin = 'h0;
				c	= sum;  //sign extension
			end
			'd2: begin
				a1  = a;
				b1  = b;
				cin = {cout[30:0], 1'b0};				
				c	= {ov, sum[30:0]};
			end
			'd3: begin
				a1  = a;
				b1  = ~b;
				cin = {cout[30:0], 1'b1};
				c	= {ov, sum[30:0]};
			end
			'd4: begin //bitwise AND
				a1  = a;
				b1  = b;
				cin = 'h0;
				c	= cout;
			end
			'd5: begin //bitwise OR
				a1  = a;
				b1  = b;
				cin = 'hFFFF;
				c	= cout;
			end
			'd6: begin //bitwise XOR
				a1  = a;
				b1  = b;
				cin = 'h0;
				c	=  sum;
			end
			'd7: begin
				c	= c1;
			end
		endcase
	end
endmodule

/////////////////////////FPU///////////////////////
module fp_14(
    input logic clk,
    input logic rst,
    input logic start,
    input logic op,
    input logic [31:0] a,
    input logic [31:0] b,
    output logic   [31:0] c
    );

logic [7:0] ea, eb,shift; // exp of a,b,c
logic gt,eq,ma,meq; //true if exp of a is bigger than b. eq true if it equal
logic [23:0] placesmall, placebig; // placements for the shift
logic [2:0]state;
logic [24:0] ult;
logic [4:0] q, subshift; //q= output of primary encoder which dtermines the shift in s4
logic basic; //true if the operation is basically addition, false if is subtraction

// always need rst=0 when puting in the input
parameter s0=3'd0, s1=3'd1, s2=3'd2, s3=3'd3, s4=3'd4, s5=3'd5;

always_comb begin // determine how much to shift 
 
    ea= a[30:23];
    eb= b[30:23]; 
   
    gt=(a[30:23]>b[30:23]); // true if exp of a is greater than b
    eq=(a[30:23]==b[30:23]); // true if exp of a is equal to b
    ma=(a[22:0]>b[22:0]); // true if the mantissa of a is greater than b
    meq=(a[22:0]==b[22:0]); // true if mantissa of a is equal to b
    
    if(gt) shift=ea-eb;
    else shift=eb-ea;
    
    basic=(~a[31]&~b[31]&~op)||(~a[31]&b[31]&op)||(a[31]&b[31]&~op)||(a[31]&~b[31]&op);
    
    if(gt) begin //  sets the placebig to a because a has bigger exp
        placesmall={1'b1, b[22:0]} >> shift; 
        placebig= {1'b1, a[22:0]}; 
    end 
    else if(eq) begin // if the exp are the same
        if(ma)begin //sets the placebig to a because a has bigger mantissa
            placebig={1'b1, a[22:0]};
            placesmall= {1'b1, b[22:0]};
        end    
        else begin // sets the placebig to b beccause either b has bigger mantissa or has the same 
            placebig={1'b1, b[22:0]};
            placesmall= {1'b1, a[22:0]};           
        end
    end   
    else begin //sets place big to b because b has bigger exp
        placesmall = {1'b1, a[22:0]} >> shift;
        placebig= {1'b1, b[22:0]};
    end
    subshift=5'd25-q;
end 


always_ff@(posedge clk or posedge rst) begin // sets up conditions for the adder
     if(rst) state<=s0;
     else if (start) state<=s1;
     else begin
         casex(state) 
            s0: ;
             s1: begin 
                   if(basic)begin
                        ult<=placebig+placesmall;
                        state<=2;
                   end     
                   else begin
                        ult<=placebig-placesmall;
                        state<=s3;
                   end    
                   c<=32'bX; 
               end       
            s2: begin // basically addition
                if(ult[24])begin
                    if(gt)c[30:23]<= ea+1'b1;   
                    else begin
                        c[30:23]<= eb+1'b1; 
                    end    
                    c[22:0]<=ult[23:1];      
                end
                else begin
                    if(gt) c[30:23]<= ea;
                    else c[30:23]<=eb; 
                    c[22:0]<= ult[22:0];
                end
                c[31]<=a[31];
            end
            s3: begin
                c[22:0]<=ult[24:2]<<subshift;
                if(gt)begin
                    c[31]<=a[31];
                    c[30:23]<=a[30:23]-subshift+2'b10;
                end    
                else if(eq) begin
                    if(ma) begin
                        c[31]<=a[31];
                        c[30:23]<=a[30:23]-subshift+2'b10;
                    end
                    else if(meq) c<=31'd0;
                    else begin
                        c[31]<=b[31];
                        c[30:23]<=b[30:23]-subshift+2'b10;
                    end
                end    
                else begin
                     c[31]<=b[31];
                     c[30:23]<=eb-subshift+2'b10;  
                end
            end
            default: c=32'd5;
         endcase
         end
 end
 
 priority_encoder_fpu p (ult,q);

    
endmodule

module priority_encoder_fpu (
 input [24:0] D,
 output logic [4:0] Q 
);
always @(D) 
begin 
casex(D)
     25'b0_0000_0000_0000_0000_0000_0001 :  Q=5'd0;
     25'b0_0000_0000_0000_0000_0000_001x :  Q=5'd1;
     25'b0_0000_0000_0000_0000_0000_01xx :  Q=5'd2;
     25'b0_0000_0000_0000_0000_0000_1xxx :   Q=5'd3; 
     25'b0_0000_0000_0000_0000_0001_xxxx :   Q=5'd4; 
     25'b0_0000_0000_0000_0000_001x_xxxx :   Q=5'd5; 
     25'b0_0000_0000_0000_0000_01xx_xxxx :   Q=5'd6; 
     25'b0_0000_0000_0000_0000_1xxx_xxxx :   Q=5'd7; 
     25'b0_0000_0000_0000_0001_xxxx_xxxx :   Q=5'd8; 
     25'b0_0000_0000_0000_001x_xxxx_xxxx :   Q=5'd9; 
     25'b0_0000_0000_0000_01xx_xxxx_xxxx :   Q=5'd10; 
     25'b0_0000_0000_0000_1xxx_xxxx_xxxx :   Q=5'd11; 
     25'b0_0000_0000_0001_xxxx_xxxx_xxxx :   Q=5'd12; 
     25'b0_0000_0000_001x_xxxx_xxxx_xxxx :   Q=5'd13; 
     25'b0_0000_0000_01xx_xxxx_xxxx_xxxx :   Q=5'd14; 
     25'b0_0000_0000_1xxx_xxxx_xxxx_xxxx :   Q=5'd15; 
     25'b0_0000_0001_xxxx_xxxx_xxxx_xxxx :   Q=5'd16; 
     25'b0_0000_001x_xxxx_xxxx_xxxx_xxxx :   Q=5'd17; 
     25'b0_0000_01xx_xxxx_xxxx_xxxx_xxxx :   Q=5'd18; 
     25'b0_0000_1xxx_xxxx_xxxx_xxxx_xxxx :   Q=5'd19; 
     25'b0_0001_xxxx_xxxx_xxxx_xxxx_xxxx :   Q=5'd20; 
     25'b0_001x_xxxx_xxxx_xxxx_xxxx_xxxx :   Q=5'd21; 
     25'b0_01xx_xxxx_xxxx_xxxx_xxxx_xxxx :   Q=5'd22; 
     25'b0_1xxx_xxxx_xxxx_xxxx_xxxx_xxxx :   Q=5'd23;
     25'b1_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx :   Q=5'd24;
 default Q=5'd0;
 endcase
end 
endmodule


