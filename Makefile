all:
	iverilog -o gshare.out gshare_tb.v 
	vvp gshare.out
	gtkwave gshare.vcd

clean:
	 rm gshare.out gshare.vcd
	 
