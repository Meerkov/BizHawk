
// Nes_Emu 0.7.0. http://www.slack.net/~ant/nes-emu/

// TODO: remove
#if !defined (NDEBUG) && 0
	#pragma peephole on
	#pragma global_optimizer on
	#pragma optimization_level 4
	#pragma scheduling 604
	#undef BLARGG_ENABLE_OPTIMIZER
#endif

#include <iostream>
#include <fstream>
#include <chrono>
#include <string>

#include "Nes_Cpu.h"

#include <string.h>
#include <limits.h>
#include "blargg_endian.h"

#include "nes_cpu_io.h"

/* Copyright (C) 2003-2006 Shay Green. This module is free software; you
can redistribute it and/or modify it under the terms of the GNU Lesser
General Public License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version. This
module is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
more details. You should have received a copy of the GNU Lesser General
Public License along with this module; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA */

#include "blargg_source.h"

#ifdef BLARGG_ENABLE_OPTIMIZER
	#include BLARGG_ENABLE_OPTIMIZER
#endif

inline void Nes_Cpu::set_code_page( int i, uint8_t const* p )
{
	code_map [i] = p - (unsigned) i * page_size;
}

void Nes_Cpu::reset( void const* unmapped_page )
{
	r.status = 0;
	r.sp = 0;
	r.pc = 0;
	r.a = 0;
	r.x = 0;
	r.y = 0;
	
	error_count_ = 0;
	clock_count = 0;
	clock_limit = 0;
	irq_time_ = LONG_MAX / 2 + 1;
	end_time_ = LONG_MAX / 2 + 1;
	
	assert( page_size == 0x800 ); // assumes this
	set_code_page( 0, low_mem );
	set_code_page( 1, low_mem );
	set_code_page( 2, low_mem );
	set_code_page( 3, low_mem );
	for ( int i = 4; i < page_count + 1; i++ )
		set_code_page( i, (uint8_t*) unmapped_page );
	
	#ifndef NDEBUG
		blargg_verify_byte_order();
	#endif
}

void Nes_Cpu::map_code( nes_addr_t start, unsigned size, const void* data )
{
	// address range must begin and end on page boundaries
	require( start % page_size == 0 );
	require( size % page_size == 0 );
	require( start + size <= 0x10000 );
	
	unsigned first_page = start / page_size;
	for ( unsigned i = size / page_size; i--; )
		set_code_page( first_page + i, (uint8_t*) data + i * page_size );
}

// Note: 'addr' is evaulated more than once in the following macros, so it
// must not contain side-effects.

//static void log_read( int opcode ) { LOG_FREQ( "read", 256, opcode ); }

#define READ_LIKELY_PPU( addr ) (NES_CPU_READ_PPU( this, (addr), (clock_count) ))
#define READ( addr )            (NES_CPU_READ( this, (addr), (clock_count) ))
#define READ_NO_RETURN( addr ) (NES_CPU_READ_CLOCK_ONLY( this, (addr), (clock_count) ))
#define WRITE( addr, data )     {NES_CPU_WRITE( this, (addr), (data), (clock_count) );}

#define READ_LOW( addr )        (low_mem [int (addr)])
#define WRITE_LOW( addr, data ) (void) (READ_LOW( addr ) = (data))

#define READ_PROG( addr )   (code_map [(addr) >> page_bits] [addr])
#define READ_PROG16( addr ) GET_LE16( &READ_PROG( addr ) )

#define SET_SP( v )     (sp = ((v) + 1) | 0x100)
#define GET_SP()        ((sp - 1) & 0xFF)
#define PUSH( v )       ((sp = (sp - 1) | 0x100), WRITE_LOW( sp, v ))

#ifdef BLARGG_ENABLE_OPTIMIZER
	#include BLARGG_ENABLE_OPTIMIZER
#endif

int Nes_Cpu::read( nes_addr_t addr )
{
	return READ( addr );
}

void Nes_Cpu::write( nes_addr_t addr, int value )
{
	WRITE( addr, value );
}

void Nes_Cpu::set_tracecb(void (*cb)(unsigned int *data))
{
	tracecb = cb;
}

#ifndef NES_CPU_GLUE_ONLY

static const unsigned char clock_table [256] = {
//  0 1 2 3 4 5 6 7 8 9 A B C D E F
	7,6,2,8,3,3,5,5,3,2,2,2,4,4,6,6,// 0
	3,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,// 1
	6,6,2,8,3,3,5,5,4,2,2,2,4,4,6,6,// 2
	3,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,// 3
	6,6,2,8,3,3,5,5,3,2,2,2,3,4,6,6,// 4
	3,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,// 5
	6,6,2,8,3,3,5,5,4,2,2,2,5,4,6,6,// 6
	3,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,// 7
	2,6,2,6,3,3,3,3,2,2,2,2,4,4,4,4,// 8
	3,6,2,6,4,4,4,4,2,5,2,5,5,5,5,5,// 9
	2,6,2,6,3,3,3,3,2,2,2,2,4,4,4,4,// A
	3,5,2,5,4,4,4,4,2,4,2,4,4,4,4,4,// B
	2,6,2,8,3,3,5,5,2,2,2,2,4,4,6,6,// C
	3,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,// D
	2,6,2,8,3,3,5,5,2,2,2,2,4,4,6,6,// E
	3,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7 // F
};

/*
// TEST STUFF:
static unsigned long long int call_table[256] = {
	//  0 1 2 3 4 5 6 7 8 9 A B C D E F
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// 0
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// 1
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// 2
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// 3
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// 4
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// 5
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// 6
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// 7
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// 8
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// 9
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// A
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// B
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// C
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// D
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// E
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 // F
};
static unsigned long long time_table[256] = {
	//  0 1 2 3 4 5 6 7 8 9 A B C D E F
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// 0
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// 1
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// 2
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// 3
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// 4
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// 5
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// 6
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// 7
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// 8
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// 9
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// A
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// B
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// C
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// D
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,// E
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 // F
};
static int test_call_nums = 0;
static int previous_op_code = -1;
static std::chrono::time_point<std::chrono::steady_clock> previous_start;*/

// END TEST STUFF


// status flags

static const int  st_n = 0x80;
static const int  st_v = 0x40;
static const int  st_r = 0x20;
static const int  st_b = 0x10;
static const int  st_d = 0x08;
static const int  st_i = 0x04;
static const int  st_z = 0x02;
static const int  st_c = 0x01;

Nes_Cpu::result_t Nes_Cpu::run( nes_time_t end )
{
	set_end_time_( end );
	clock_count = 0;
	
	volatile result_t result = result_cycles;
	
#if !BLARGG_CPU_CISC
	long clock_count = this->clock_count;
	uint8_t* const low_mem = this->low_mem;
#endif

	// registers
	unsigned pc = r.pc;
	int sp;
	SET_SP( r.sp );
	int a = r.a;
	int x = r.x;
	int y = r.y;
	
	#define IS_NEG (nz & 0x880)
	
	#define CALC_STATUS( out ) do {             \
		out = status & (st_v | st_d | st_i);    \
		out |= (c >> 8) & st_c;                 \
		if ( IS_NEG ) out |= st_n;              \
		if ( !(nz & 0xFF) ) out |= st_z;        \
	} while ( 0 )

	#define SET_STATUS( in ) do {               \
		status = in & (st_v | st_d | st_i);     \
		c = in << 8;                            \
		nz = (in << 4) & 0x800;                 \
		nz |= ~in & st_z;                       \
	} while ( 0 )

	unsigned data;
	int status;
	int c;  // carry set if (c & 0x100) != 0
	int nz; // Z set if (nz & 0xFF) == 0, N set if (nz & 0x880) != 0
	{
		int temp = r.status;
		SET_STATUS( temp );
	}

	uint8_t opcode;
loop:

#if !defined (NDEBUG)
	assert( (unsigned) GET_SP() < 0x100 );
	assert( (unsigned) a < 0x100 );
	assert( (unsigned) x < 0x100 );
	assert( (unsigned) y < 0x100 );
#endif



	if ( clock_count >= clock_limit )
		goto end;

	uint8_t const* page = code_map [pc >> page_bits];
	opcode = page [pc];
	/*
	// TEST ZONE
	if (previous_op_code >= 0) {
		auto t2 = std::chrono::high_resolution_clock::now();
		call_table[previous_op_code]++;
		time_table[previous_op_code] += (t2 - previous_start).count();
	}
	previous_op_code = opcode;
	{
		if (test_call_nums > 1000000) {
			std::ofstream myfile;
			std::string s = "";
			for (int i = 0; i < 256; i += 16) {
				for (int j = 0; j < 16; j++) {
					s += std::to_string(time_table[i + j]);
					if (j != 15) {
						s.append("\t");
					}
					else {
						s.append("\n");
					}
				}
			}
			myfile.open("example.txt");
			myfile << "Call Nums: \n";
			myfile << s;
			myfile.close();

			test_call_nums = 0;
		}
		else {
			test_call_nums++;
		}
	}
	previous_start = std::chrono::high_resolution_clock::now();
	
	// END TEST ZONE*/

#if !defined (NDEBUG)
	if (tracecb)
	{
		unsigned int scratch[7];
		scratch[0] = a;
		scratch[1] = x;
		scratch[2] = y;
		scratch[3] = sp;
		scratch[4] = pc;
		scratch[5] = status;
		scratch[6] = opcode;
		tracecb(scratch);
	}
#endif
	
	clock_count += clock_table [opcode];

	switch (opcode)
	{

		// Macros

#define GET_OPERAND( addr )   page [addr]
#define GET_OPERAND16( addr ) GET_LE16( &page [addr] )

//#define GET_OPERAND( addr )   READ_PROG( addr )
//#define GET_OPERAND16( addr ) READ_PROG16( addr )

#define ADD_PAGE        (pc++, data += GET_OPERAND( pc )<<8);
#define GET_ADDR()      GET_OPERAND16( pc )

#define HANDLE_PAGE_CROSSING( lsb ) clock_count += (lsb) >> 8;

#define INC_DEC_XY( reg, n ) reg = uint8_t (nz = reg + n); goto loop;

#define IND_Y(r,c) {                                            \
		int temp = READ_LOW( data ) + y;                        \
		data = temp + 0x100 * READ_LOW( uint8_t (data + 1) );   \
		if (c) HANDLE_PAGE_CROSSING( temp );                    \
		if (!(r) || (temp & 0x100))                             \
			READ_NO_RETURN( data - ( temp & 0x100 ) );          \
	}

#define IND_X {                                                 \
		int temp = data + x;                                    \
		data = 0x100 * READ_LOW( uint8_t (temp + 1) ) + READ_LOW( uint8_t (temp) ); \
	}

// Often-Used

	case 0xB5: // LDA zp,x
		nz = a = low_mem[uint8_t (page[pc+1] + x)];
		pc+=2;
		goto loop;

	case 0xA5: // LDA zp
		nz = a = low_mem[ page[pc+1] ];
		pc+=2;
		goto loop;

	case 0xD0: // BNE
		{
			pc+=2;
			if (!((uint8_t)nz)) { 
				clock_count--; 
				goto loop; 
			}
			data = page [pc-1];
			int offset = (BOOST::int8_t) data;
			int extra_clock = (pc & 0xFF) + offset;
			pc = BOOST::uint16_t(pc + offset);
			clock_count += (extra_clock >> 8) & 1;
			goto loop;
		}
	
	case 0x20: { // JSR
		int temp = pc + 2;
		pc = GET_OPERAND16( pc+1 );
		WRITE_LOW( 0x100 | (sp - 1), temp >> 8 );
		sp = (sp - 2) | 0x100;
		WRITE_LOW( sp, temp );
		goto loop;
	}
	
	case 0x4C: // JMP abs
		pc = GET_OPERAND16( pc+1 );
		goto loop;
	
	case 0xE8: // INX
		pc++; 
		INC_DEC_XY( x, 1 )
	
	case 0x10: // BPL
		{
			pc+=2;
			if (IS_NEG) { 
				clock_count--; 
				goto loop; 
			}
			int offset = (BOOST::int8_t) page [pc-1];
			int extra_clock = (pc & 0xFF) + offset;
			pc = BOOST::uint16_t(pc + offset);
			clock_count += (extra_clock >> 8) & 1;
			goto loop;
		}
	
// ARITH_ADDR_MODES( 0xC5 )          
case 0xC5 - 0x04: /* (ind,x) */   
	pc++;        
	data = page [pc];                   
	IND_X                               
		data = READ(data);
	// CMP
	nz = a - data;
	c = ~nz;
	nz &= 0xFF;
	pc++;
	goto loop;

case 0xC5 + 0x0C: /* (ind),y */ 
	pc++;          
	data = page [pc];                   
	IND_Y(true,true)                    
		data = READ(data);
	// CMP
	nz = a - data;
	c = ~nz;
	nz &= 0xFF;
	pc++;
	goto loop;

case 0xC5 + 0x10: /* zp,X */          
	pc++;    
	data = uint8_t (page [pc] + x);     
	data = READ_LOW( data );            
	// CMP
	nz = a - data;
	c = ~nz;
	nz &= 0xFF;
	pc++;
	goto loop;

case 0xC5 + 0x00: /* zp */
	// CMP
	nz = a - READ_LOW( page [pc+1] );   
	c = ~nz;
	nz &= 0xFF;
	pc+=2;
	goto loop;

case 0xC5 + 0x14: /* abs,Y */      
	pc++;       
	data = page [pc] + y;               
	HANDLE_PAGE_CROSSING(data);
	{int temp = data;
	ADD_PAGE
		if (temp & 0x100)
			READ_NO_RETURN(data - 0x100);
	data = READ(data);
	// CMP
	nz = a - data;
	c = ~nz;
	nz &= 0xFF;
	pc++;
	goto loop; }

case 0xC5 + 0x18: /* abs,X */    
	pc++;         
	data = page [pc] + x;               
{                              
	HANDLE_PAGE_CROSSING( data );       
	int temp = data;                    
	ADD_PAGE                            
	if ( temp & 0x100 )                 
		 READ_NO_RETURN( data - 0x100 );      
	data = READ(data);
	// CMP
	nz = a - data;
	c = ~nz;
	nz &= 0xFF;
	pc++;
	goto loop;
}          

case 0xC5 + 0x08: /* abs */        
	pc++;       
	data = page [pc];                   
	ADD_PAGE                                                          
	data = READ( data );                
	// CMP
	nz = a - data;
	c = ~nz;
	nz &= 0xFF;
	pc++;
	goto loop;

case 0xC5 + 0x04: /* imm */                           
	// CMP
	nz = a - page [pc+1];   
	c = ~nz;
	nz &= 0xFF;
	pc+=2;
	goto loop;
	
	case 0x30: // BMI
		{
			pc+=2;
			if (!(IS_NEG)) {
				clock_count--; 
				goto loop;
			}
			data = page [pc-1];
			int offset = (BOOST::int8_t) data;
			int extra_clock = (pc & 0xFF) + offset;
			pc = BOOST::uint16_t(pc + offset);
			clock_count += (extra_clock >> 8) & 1;
			goto loop;
		}
	
	case 0xF0: // BEQ
		{
			pc+=2;
			if (!(!(uint8_t)nz)) {
				clock_count--;
				goto loop;
			}
			data = page [pc-1];
			int offset = (BOOST::int8_t) data;
			int extra_clock = (pc & 0xFF) + offset;
			pc = BOOST::uint16_t(pc + offset);
			clock_count += (extra_clock >> 8) & 1;
			goto loop;
		}
	
	case 0x95: // STA zp,x
		WRITE_LOW(uint8_t (page[pc+1] + x), a);
		pc+=2;
		goto loop;

	case 0x85: // STA zp
		WRITE_LOW( page [pc+1], a );
		pc+=2;
		goto loop;
	
	case 0xC8: // INY
		pc++; 
		INC_DEC_XY( y, 1 ) 

	case 0xA8: // TAY
		pc++;
		nz = y = a;
		goto loop;

	case 0x98: // TYA
		pc++;
		nz = a = y;
		goto loop;
	
	case 0xAD:{// LDA abs
		pc++;
		unsigned addr = GET_ADDR();
		a = nz = READ_LIKELY_PPU( addr );
		pc += 2;
		goto loop;
	}
	
	case 0x60: // RTS
		pc++;
		pc = 1 + READ_LOW( sp ) + READ_LOW( 0x100 | (sp - 0xFF) ) * 0x100;
		sp = (sp - 0xFE) | 0x100;
		goto loop;

	case 0x99: // STA abs,Y
		pc++;
		data = page [pc] + y; 
		{
			int temp = data;
			ADD_PAGE
			READ_NO_RETURN(data - (temp & 0x100));
		}
		WRITE(data, a);
		pc++;
		goto loop;
	
	case 0x9D: // STA abs,X
		pc++;
		data = page [pc] + x;
	{
		int temp = data & 0x100;
		ADD_PAGE
		READ_NO_RETURN( data - temp );
		WRITE(data, a);
	}
		pc++;
		goto loop;

	case 0x8D: // STA abs
		pc++;
		data = page [pc];
		ADD_PAGE
		WRITE( data, a );
		pc++;
		goto loop;
	
	case 0xA9: // LDA #imm
		nz = a = page [pc+1];
		pc+=2;
		goto loop;
	
#if 0
	case 0xA1: // LDA (ind,X)
		IND_X
		goto lda_ptr;
	
	case 0xB1: // LDA (ind),Y
		IND_Y(true,true)
		goto lda_ptr;
	
	case 0xB9: // LDA abs,Y
		data += y;
		goto lda_ind_common;
	
	case 0xBD: // LDA abs,X
		data += x;
	lda_ind_common: {
		HANDLE_PAGE_CROSSING( data );
		int temp = data;
		ADD_PAGE
		if ( temp & 0x100 )
			READ( data - 0x100 );
	}
	lda_ptr:
		a = nz = READ( data );
		pc++;
		goto loop;
#else
	// optimization of most commonly used memory read instructions
	
	case 0xB9:// LDA abs,Y
		pc++;
		data = page [pc] + y;
		{
			unsigned msb = GET_OPERAND(pc + 1);
			// indexed common
			pc += 2;
			HANDLE_PAGE_CROSSING(data);
			unsigned temp = data + msb*0x100;
			if ((unsigned)(temp - 0x2000) < 0x6000) {
				if (data & 0x100)
					READ_NO_RETURN(temp - 0x100);
				a = nz = READ(temp);
			}
			else {
				a = nz = READ_PROG(BOOST::uint16_t(temp));
			}
		}
		goto loop;

	case 0xBD:{// LDA abs,X
		pc++;
		data = page [pc] + x;
		unsigned msb = GET_OPERAND( pc+1 );
		// indexed common
		pc+=2;
		HANDLE_PAGE_CROSSING( data );
		int temp = data + msb * 0x100;
		if ((unsigned)(temp - 0x2000) < 0x6000) {
			if (data & 0x100)
				READ_NO_RETURN(temp - 0x100);
			a = nz = READ(temp);
		}
		else {
			a = nz = READ_PROG(BOOST::uint16_t(temp));
		}
		goto loop;
	}
	
	case 0xB1:{// LDA (ind),Y
		pc++;
		data = page [pc];
		unsigned msb = READ_LOW( (uint8_t) (data + 1) );
		data = READ_LOW( data ) + y;
		// indexed common
		pc++;
		HANDLE_PAGE_CROSSING( data );
		int temp = data + msb * 0x100;
		if ((unsigned)(temp - 0x2000) < 0x6000) {
			if (data & 0x100)
				READ_NO_RETURN(temp - 0x100);
			a = nz = READ(temp);
		}
		else {
			a = nz = READ_PROG(BOOST::uint16_t(temp));
		}
		goto loop;
	}
	
	case 0xA1: // LDA (ind,X)
		pc++;
		data = page [pc];
		IND_X
		a = nz = READ( data );
		pc++;
		goto loop;
	
#endif

// Branch

	case 0x50: // BVC
		{
			pc+=2;
			if (!(!(status & st_v))) {
				clock_count--;
				goto loop;
			}
			data = page [pc-1];
			int offset = (BOOST::int8_t) data;
			int extra_clock = (pc & 0xFF) + offset;
			pc = BOOST::uint16_t(pc + offset);
			clock_count += (extra_clock >> 8) & 1;
			goto loop;
		}
	
	case 0x70: // BVS
		{
			pc+=2;
			if (!(status & st_v )) {
				clock_count--;
				goto loop;
			}
			int offset = (BOOST::int8_t) page [pc-1];
			int extra_clock = (pc & 0xFF) + offset;
			pc = BOOST::uint16_t(pc + offset);
			clock_count += (extra_clock >> 8) & 1;
			goto loop;
		}
	
	case 0xB0: // BCS
		{
			pc+=2;
			if (!(c & 0x100)) {
				clock_count--;
				goto loop;
			}
			int offset = (BOOST::int8_t) page [pc-1];
			int extra_clock = (pc & 0xFF) + offset;
			pc = BOOST::uint16_t(pc+offset);
			clock_count += (extra_clock >> 8) & 1;
			goto loop;
		}
	
	case 0x90: // BCC
		{
			pc += 2;
			if (c & 0x100) {
				clock_count--;
				goto loop;
			}
			int offset = (BOOST::int8_t) page[pc-1];
			int extra_clock = (pc & 0xFF) + offset;
			pc = BOOST::uint16_t(pc+offset);
			clock_count += (extra_clock >> 8) & 1;
		}
		goto loop;
	
// Load/store
	
	case 0x94: // STY zp,x
		WRITE_LOW( uint8_t(page[pc+1] + x) , y);
		pc+=2;
		goto loop;

	case 0x84: // STY zp
		WRITE_LOW(page[pc+1], y );
		pc+=2;
		goto loop;
	
	case 0x96: // STX zp,y
		WRITE_LOW(uint8_t (page[pc+1] + y), x);
		pc+=2;
		goto loop;

	case 0x86: // STX zp
		WRITE_LOW(page[pc+1], x );
		pc+=2;
		goto loop;
	
	case 0xB6: // LDX zp,y
		nz = x = READ_LOW(uint8_t (page[pc+1] + y));
		pc+=2;
		goto loop;

	case 0xA6: // LDX zp
		nz = x = READ_LOW( page[pc+1] );
		pc+=2;
		goto loop;

	case 0xA2: // LDX #imm
		nz = x = page[pc+1];
		pc+=2;
		goto loop;
	
	case 0xB4: // LDY zp,x
		nz = y = READ_LOW(uint8_t (page[pc+1] + x));
		pc+=2;
		goto loop;

	case 0xA4: // LDY zp
		nz = y = READ_LOW(page [pc+1]);
		pc+=2;
		goto loop;

	case 0xA0: // LDY #imm
		nz = y = page[pc+1];
		pc+=2;
		goto loop;
	
	case 0x91: // STA (ind),Y
		pc++;
		data = page [pc];
		IND_Y(false,false)
		WRITE(data, a);
		pc++;
		goto loop;
	
	case 0x81: // STA (ind,X)
		pc++;
		data = page [pc];
		IND_X
		WRITE(data, a);
		pc++;
		goto loop;
	
	case 0xBC: // LDY abs,X
		pc++;
		data = page [pc] + x;
		HANDLE_PAGE_CROSSING( data );
		{
			unsigned addr = data + 0x100 * GET_OPERAND(pc+1);
			if (data & 0x100)
				READ_NO_RETURN(addr - 0x100);
			y = nz = READ(addr);
			pc+=2;
			goto loop;
		}

	case 0xAC:{// LDY abs
		pc++;
		data = page [pc];
		unsigned addr = data + 0x100 * GET_OPERAND( pc+1 );
		if ( data & 0x100 )
			READ_NO_RETURN( addr - 0x100 );
		y = nz = READ( addr );
		pc+=2;
		goto loop;
	}
	
	case 0xBE: // LDX abs,y
		pc++;
		data = page [pc] + y;
		HANDLE_PAGE_CROSSING( data );
		{
			unsigned addr = data + 0x100 * GET_OPERAND(pc+1);
			if (data & 0x100)
				READ_NO_RETURN(addr - 0x100);
			x = nz = READ(addr);
			pc+=2;
		}
		goto loop;

	case 0xAE:{// LDX abs
		pc++;
		data = page [pc];
		unsigned addr = data + 0x100 * GET_OPERAND( pc+1 );
		if ( data & 0x100 )
			READ_NO_RETURN( addr - 0x100 );
		x = nz = READ( addr );
		pc+=2;
		}
		goto loop;
	
	case 0x8C: {// STY abs
		pc++;
		unsigned addr = GET_ADDR();
		WRITE(addr, y);
		pc += 2;
		goto loop;
	}
	
	case 0x8E: {// STX abs
		pc++;
		unsigned addr = GET_ADDR();
		WRITE( addr, x );
		pc += 2;
		goto loop;
	}

// Compare

	case 0xEC:{// CPX abs
		pc++;
		unsigned addr = GET_ADDR();
		data = READ( addr );
		nz = x - data;
		c = ~nz;
		nz &= 0xFF;
		pc+=2;
		goto loop;
	}
	
	case 0xE4: // CPX zp
		pc++;
		data = READ_LOW( page[pc] );
		nz = x - data;
		c = ~nz;
		nz &= 0xFF;
		pc++;
		goto loop;

	case 0xE0: // CPX #imm
		pc++;
		data = page [pc];
		nz = x - data;
		c = ~nz;
		nz &= 0xFF;
		pc++;
		goto loop;
	
	case 0xCC:{// CPY abs
		pc++;
		unsigned addr = GET_ADDR();
		data = READ( addr );
		nz = y - data;
		c = ~nz;
		nz &= 0xFF;
		pc+=2;
		goto loop;
	}
	
	case 0xC4: // CPY zp
		pc++;
		data = READ_LOW( page [pc] );
		nz = y - data;
		c = ~nz;
		nz &= 0xFF;
		pc++;
		goto loop;

	case 0xC0: // CPY #imm
		nz = y - page [pc+1];
		c = ~nz;
		nz &= 0xFF;
		pc+=2;
		goto loop;
	
// Logical
// ARITH_ADDR_MODES( 0x25 )          
case 0x25 - 0x04: /* (ind,x) */    
		pc++;       
	data = page [pc];                   
	IND_X                               
		data = READ(data);
	// AND
	nz = (a &= data);
	pc++;
	goto loop;

case 0x25 + 0x0C: /* (ind),y */       
		pc++;    
	data = page [pc];                   
	IND_Y(true,true)                    
		data = READ(data);
	// AND
	nz = (a &= data);
	pc++;
	goto loop;

case 0x25 + 0x10: /* zp,X */       
		pc++;       
	data = uint8_t (page [pc] + x);     
	data = READ_LOW( data );            
	// AND
	nz = (a &= data);
	pc++;
	goto loop;

case 0x25 + 0x00: /* zp */        
		pc++;        
	data = READ_LOW( page [pc] );       
	// AND
	nz = (a &= data);
	pc++;
	goto loop;

case 0x25 + 0x14: /* abs,Y */      
		pc++;       
	data = page [pc] + y;               
	{
		HANDLE_PAGE_CROSSING(data);
		int temp = data;
		ADD_PAGE
			if (temp & 0x100)
				READ_NO_RETURN(data - 0x100);
		data = READ(data);
		// AND
		nz = (a &= data);
		pc++;
		goto loop;
	}

case 0x25 + 0x18: /* abs,X */   
		pc++;          
	data = page [pc] + x;               
 {                              
	HANDLE_PAGE_CROSSING( data );       
	int temp = data;                    
	ADD_PAGE                            
	if ( temp & 0x100 )                 
		 READ_NO_RETURN( data - 0x100 );      
	data = READ(data);
	// AND
	nz = (a &= data);
	pc++;
	goto loop;
}          

case 0x25 + 0x08: /* abs */    
		pc++;           
	data = page [pc];                   
	ADD_PAGE                                                          
	data = READ( data );                
	// AND
	nz = (a &= data);
	pc++;
	goto loop;

case 0x25 + 0x04: /* imm */    
		pc++;           
	data = page [pc];                                                 
	// AND
	nz = (a &= data);
	pc++;
	goto loop;

// ARITH_ADDR_MODES( 0x45 )          
case 0x45 - 0x04: /* (ind,x) */    
		pc++;                  
	data = page [pc];                   
	IND_X                               
		data = READ(data);
	// EOR
	nz = (a ^= data);
	pc++;
	goto loop;

case 0x45 + 0x0C: /* (ind),y */    
		pc++;                  
	data = page [pc];                   
	IND_Y(true,true)                    
		data = READ(data);
	// EOR
	nz = (a ^= data);
	pc++;
	goto loop;

case 0x45 + 0x10: /* zp,X */         
		pc++;                
	data = uint8_t (page [pc] + x);     
	data = READ_LOW( data );            
	// EOR
	nz = (a ^= data);
	pc++;
	goto loop;

case 0x45 + 0x00: /* zp */         
		pc++;                  
	data = READ_LOW( page [pc] );       
	// EOR
	nz = (a ^= data);
	pc++;
	goto loop;

case 0x45 + 0x14: /* abs,Y */    
		pc++;                    
	data = page [pc] + y;               
	{
		HANDLE_PAGE_CROSSING(data);
		int temp = data;
		ADD_PAGE
			if (temp & 0x100)
				READ_NO_RETURN(data - 0x100);
		data = READ(data);
		// EOR
		nz = (a ^= data);
		pc++;
		goto loop;
	}

case 0x45 + 0x18: /* abs,X */     
		pc++;                   
	data = page [pc] + x;               
{                              
	HANDLE_PAGE_CROSSING( data );       
	int temp = data;                    
	ADD_PAGE                            
	if ( temp & 0x100 )                 
		 READ_NO_RETURN( data - 0x100 );      
	data = READ(data);
	// EOR
	nz = (a ^= data);
	pc++;
	goto loop;
}           

case 0x45 + 0x08: /* abs */   
		pc++;                       
	data = page [pc];                   
	ADD_PAGE                           
	data = READ( data );                
	// EOR
	nz = (a ^= data);
	pc++;
	goto loop;

case 0x45 + 0x04: /* imm */     
		pc++;                     
	data = page [pc];                             
	// EOR
	nz = (a ^= data);
	pc++;
	goto loop;

// ARITH_ADDR_MODES( 0x05 )          
case 0x05 - 0x04: /* (ind,x) */   
		pc++;                   
	data = page [pc];                   
	IND_X                               
		data = READ(data);
	// ORA
	nz = (a |= data);
	pc++;
	goto loop;

case 0x05 + 0x0C: /* (ind),y */      
		pc++;                
	data = page [pc];                   
	IND_Y(true,true)                    
		data = READ(data);
	// ORA
	nz = (a |= data);
	pc++;
	goto loop;

case 0x05 + 0x10: /* zp,X */         
		pc++;                
	data = uint8_t (page [pc] + x);     
	data = READ_LOW( data );            
	// ORA
	nz = (a |= data);
	pc++;
	goto loop;

case 0x05 + 0x00: /* zp */       
		pc++;                    
	data = READ_LOW( page [pc] );       
	// ORA
	nz = (a |= data);
	pc++;
	goto loop;

case 0x05 + 0x14: /* abs,Y */     
		pc++;                   
	data = page [pc] + y;               
	{
		HANDLE_PAGE_CROSSING(data);
		int temp = data;
		ADD_PAGE
			if (temp & 0x100)
				READ_NO_RETURN(data - 0x100);
		data = READ(data);
		// ORA
		nz = (a |= data);
		pc++;
		goto loop;
	}

case 0x05 + 0x18: /* abs,X */      
		pc++;                  
	data = page [pc] + x;               
{                              
	HANDLE_PAGE_CROSSING( data );       
	int temp = data;                    
	ADD_PAGE                            
	if ( temp & 0x100 )                 
		 READ_NO_RETURN( data - 0x100 );      
	data = READ(data);
	// ORA
	nz = (a |= data);
	pc++;
	goto loop;
}         

case 0x05 + 0x08: /* abs */         
		pc++;                 
	data = page [pc];                   
	ADD_PAGE                         
	data = READ( data );                
	// ORA
	nz = (a |= data);
	pc++;
	goto loop;

case 0x05 + 0x04: /* imm */       
		pc++;                   
	data = page [pc];                    
	// ORA
	nz = (a |= data);
	pc++;
	goto loop;
	
	case 0x2C:{// BIT abs
		pc++;           
		unsigned addr = GET_ADDR();
		pc += 2;
		status &= ~st_v;
		nz = READ_LIKELY_PPU( addr );
		status |= nz & st_v;
		if ( a & nz )
			goto loop;
		// result must be zero, even if N bit is set
		nz = nz << 4 & 0x800;
	}
		goto loop;
	
	case 0x24: // BIT zp
		pc++;
		data = page [pc];
		nz = READ_LOW( data );
		pc++;
		status &= ~st_v;
		status |= nz & st_v;
		if ( a & nz )
			goto loop;
		// result must be zero, even if N bit is set
		nz = nz << 4 & 0x800;
		goto loop;
		
// Add/subtract
// ARITH_ADDR_MODES( 0xE5 )          
case 0xE5 - 0x04: /* (ind,x) */
		pc++;                      
	data = page [pc];                   
	IND_X                               
		data = READ(data);
	// SBC
	data ^= 0xFF;
	{
		int carry = (c >> 8) & 1;
		int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
		status &= ~st_v;
		status |= (ov >> 2) & 0x40;
		c = nz = a + data + carry;
		pc++;
		a = (uint8_t)nz;
		goto loop;
	}

case 0xE5 + 0x0C: /* (ind),y */   
		pc++;                   
	data = page [pc];                   
	IND_Y(true,true)                    
		data = READ(data);
	// SBC
	data ^= 0xFF;
	{
		int carry = (c >> 8) & 1;
		int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
		status &= ~st_v;
		status |= (ov >> 2) & 0x40;
		c = nz = a + data + carry;
		pc++;
		a = (uint8_t)nz;
		goto loop;
	}

case 0xE5 + 0x10: /* zp,X */             
		pc++;            
	data = uint8_t (page [pc] + x);     
	data = READ_LOW( data );            
	// SBC
	data ^= 0xFF;
	{
		int carry = (c >> 8) & 1;
		int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
		status &= ~st_v;
		status |= (ov >> 2) & 0x40;
		c = nz = a + data + carry;
		pc++;
		a = (uint8_t)nz;
		goto loop;
	}

case 0xE5 + 0x00: /* zp */
	data = (READ_LOW( page [pc+1] ))^0xFF;       
	// SBC
	{
		int carry = (c >> 8) & 1;
		int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
		status &= ~st_v;
		status |= (ov >> 2) & 0x40;
		c = nz = a + data + carry;
		pc+=2;
		a = (uint8_t)nz;
		goto loop;
	}

case 0xE5 + 0x14: /* abs,Y */          
		pc++;              
	data = page [pc] + y;               
	{
		HANDLE_PAGE_CROSSING(data);
		int temp = data;
		ADD_PAGE
			if (temp & 0x100)
				READ_NO_RETURN(data - 0x100);
		data = READ(data);
		// SBC
		data ^= 0xFF;
		{
			int carry = (c >> 8) & 1;
			int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
			status &= ~st_v;
			status |= (ov >> 2) & 0x40;
			c = nz = a + data + carry;
			pc++;
			a = (uint8_t)nz;
			goto loop;
		}
	}

case 0xE5 + 0x18: /* abs,X */      
		pc++;                  
	data = page [pc] + x;               
{                              
	HANDLE_PAGE_CROSSING( data );       
	int temp = data;                    
	ADD_PAGE                            
	if ( temp & 0x100 )                 
		 READ_NO_RETURN( data - 0x100 );      
	data = READ(data);
	// SBC
	data ^= 0xFF;
	{
		int carry = (c >> 8) & 1;
		int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
		status &= ~st_v;
		status |= (ov >> 2) & 0x40;
		c = nz = a + data + carry;
		pc++;
		a = (uint8_t)nz;
		goto loop;
	}
}      

case 0xE5 + 0x08: /* abs */              
		pc++;            
	data = page [pc];                   
	ADD_PAGE                                                         
	data = READ( data );                
	// SBC
	data ^= 0xFF;
	{
		int carry = (c >> 8) & 1;
		int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
		status &= ~st_v;
		status |= (ov >> 2) & 0x40;
		c = nz = a + data + carry;
		pc++;
		a = (uint8_t)nz;
		goto loop;
	}

case 0xE5 + 0x04: /* imm */               
		pc++;           
	data = page [pc];                   
  // SBC
		data ^= 0xFF;
		{
			int carry = (c >> 8) & 1;
			int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
			status &= ~st_v;
			status |= (ov >> 2) & 0x40;
			c = nz = a + data + carry;
			pc++;
			a = (uint8_t)nz;
			goto loop;
		}


	case 0xEB: // unofficial equivalent
		pc++;
		data = page [pc] ^ 0xFF; 
		{
			int carry = (c >> 8) & 1;
			int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
			status &= ~st_v;
			status |= (ov >> 2) & 0x40;
			c = nz = a + data + carry;
			pc++;
			a = (uint8_t)nz;
			goto loop;
		}

// ARITH_ADDR_MODES( 0x65 )          
case 0x65 - 0x04: /* (ind,x) */         
		pc++;             
	data = page [pc];                   
	IND_X                               
		data = READ(data);
	// ADC
	{
		int carry = (c >> 8) & 1;
		int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
		status &= ~st_v;
		status |= (ov >> 2) & 0x40;
		c = nz = a + data + carry;
		pc++;
		a = (uint8_t)nz;
		goto loop;
	}

case 0x65 + 0x0C: /* (ind),y */           
		pc++;           
	data = page [pc];                   
	IND_Y(true,true)                    
		data = READ(data);
	// ADC
	{
		int carry = (c >> 8) & 1;
		int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
		status &= ~st_v;
		status |= (ov >> 2) & 0x40;
		c = nz = a + data + carry;
		pc++;
		a = (uint8_t)nz;
		goto loop;
	}

case 0x65 + 0x10: /* zp,X */              
		pc++;           
	data = uint8_t (page [pc] + x);     
	data = READ_LOW( data );            
	// ADC
	{
		int carry = (c >> 8) & 1;
		int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
		status &= ~st_v;
		status |= (ov >> 2) & 0x40;
		c = nz = a + data + carry;
		pc++;
		a = (uint8_t)nz;
		goto loop;
	}

case 0x65 + 0x00: /* zp */                
		pc++;           
	data = READ_LOW( page [pc] );       
	// ADC
	{
		int carry = (c >> 8) & 1;
		int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
		status &= ~st_v;
		status |= (ov >> 2) & 0x40;
		c = nz = a + data + carry;
		pc++;
		a = (uint8_t)nz;
		goto loop;
	}

case 0x65 + 0x14: /* abs,Y */             
		pc++;           
	data = page [pc] + y;               
	{
		HANDLE_PAGE_CROSSING(data);
		int temp = data;
		ADD_PAGE
			if (temp & 0x100)
				READ_NO_RETURN(data - 0x100);
		data = READ(data);
		// ADC
		{
			int carry = (c >> 8) & 1;
			int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
			status &= ~st_v;
			status |= (ov >> 2) & 0x40;
			c = nz = a + data + carry;
			pc++;
			a = (uint8_t)nz;
			goto loop;
		}
	}

case 0x65 + 0x18: /* abs,X */      
		pc++;                  
	data = page [pc] + x;               
 {                              
	HANDLE_PAGE_CROSSING( data );       
	int temp = data;                    
	ADD_PAGE                            
	if ( temp & 0x100 )                 
		 READ_NO_RETURN( data - 0x100 );      
	data = READ(data);
	// ADC
	{
		int carry = (c >> 8) & 1;
		int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
		status &= ~st_v;
		status |= (ov >> 2) & 0x40;
		c = nz = a + data + carry;
		pc++;
		a = (uint8_t)nz;
		goto loop;
	}
}        

case 0x65 + 0x08: /* abs */           
		pc++;               
	data = page [pc];                   
	ADD_PAGE                          
	data = READ( data );                
	// ADC
	{
		int carry = (c >> 8) & 1;
		int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
		status &= ~st_v;
		status |= (ov >> 2) & 0x40;
		c = nz = a + data + carry;
		pc++;
		a = (uint8_t)nz;
		goto loop;
	}

case 0x65 + 0x04: /* imm */           
		pc++;               
	data = page [pc];                   
// ADC
	{
		int carry = (c >> 8) & 1;
		int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
		status &= ~st_v;
		status |= (ov >> 2) & 0x40;
		c = nz = a + data + carry;
		pc++;
		a = (uint8_t) nz;
		goto loop;
	}                             
	
// Shift/rotate

	case 0x4A: // LSR A
		pc++;
		c = a << 8;
		a = nz = a >> 1;
		goto loop;

	case 0x6A: // ROR A
		pc++;
		nz = (c >> 1) & 0x80; // could use bit insert macro here
		c = a << 8;
		nz |= a >> 1;
		a = nz;
		goto loop;

	case 0x0A: // ASL A
		pc++;
		c = nz = a << 1;
		a = (uint8_t) nz;
		goto loop;

	case 0x2A: { // ROL A
		pc++;           
		nz = a << 1;
		int temp = (c >> 8) & 1;
		c = nz;
		nz |= temp;
		a = (uint8_t) nz;
		goto loop;
	}
	
	case 0x3E: // ROL abs,X
		pc++;
		data = page [pc]+ x;
		{
			int temp = data;
			ADD_PAGE
			READ_NO_RETURN(data - (temp & 0x100));
			WRITE(data, temp = READ(data));
			nz = (c >> 8) & 1;
			nz |= (c = temp << 1);
		}
		WRITE(data, (uint8_t)nz);
		pc++;
		goto loop;
	
	case 0x1E: // ASL abs,X
		pc++;
		data = page [pc] + x;
		{
			int temp = data;
			ADD_PAGE
			READ_NO_RETURN(data - (temp & 0x100));
			WRITE(data, temp = READ(data));
			nz = c = temp << 1;
		}
		pc++;
		WRITE(data, (uint8_t)nz);
		goto loop;

	case 0x0E: // ASL abs
		pc++;
		data = page [pc];
		{
			int temp = data;
			ADD_PAGE
			WRITE(data, temp = READ(data));
			nz = c = temp << 1;
		}
		WRITE(data, (uint8_t)nz);
		pc++;
		goto loop;

	case 0x2E: // ROL abs
		pc++;
		data = page [pc];
	{
		int temp = data;
		ADD_PAGE
		WRITE( data, temp = READ( data ) );
		nz = (c >> 8) & 1;
		nz |= (c = temp << 1);
    }
		pc++;
		WRITE( data, (uint8_t) nz );
		goto loop;

	case 0x7E: // ROR abs,X
		pc++;
		data = page [pc] + x; 
		{
			int temp = data;
			ADD_PAGE
			READ_NO_RETURN(data - (temp & 0x100));
			WRITE(data, temp = READ(data));
			nz = ((c >> 1) & 0x80) | (temp >> 1);
			c = temp << 8;
			WRITE(data, (uint8_t)nz);
			pc++;
			goto loop;
		}
	
	case 0x5E: // LSR abs,X
		pc++;
		data = page [pc] + x;
		{
			int temp = data;
			ADD_PAGE
			READ_NO_RETURN(data - (temp & 0x100));
			WRITE(data, temp = READ(data));
			nz = (temp >> 1);
			c = temp << 8;
			pc++;
			WRITE(data, (uint8_t)nz);
			goto loop;
		}

	case 0x4E: // LSR abs
		pc++;
		data = page [pc];
		{
			int temp = data;
			ADD_PAGE
			WRITE(data, temp = READ(data));
			nz = (temp >> 1);
			c = temp << 8;
			pc++;
			WRITE(data, (uint8_t)nz);
			goto loop;
		}

	case 0x6E: // ROR abs
		pc++;
		data = page [pc];
	{
		int temp = data;
		ADD_PAGE
		WRITE( data, temp = READ( data ) );
		nz = ((c >> 1) & 0x80) | (temp >> 1);
		c = temp << 8;
		pc++;
		WRITE(data, (uint8_t)nz);
		goto loop;
	}
	
	case 0x76: // ROR zp,x
		pc++;
		data = uint8_t (page [pc] + x);
		{
			int temp = READ_LOW(data);
			nz = ((c >> 1) & 0x80) | (temp >> 1);
			c = temp << 8;
			pc++;
			WRITE_LOW(data, nz);
			goto loop;
		}
	
	case 0x56: // LSR zp,x
		pc++;
		data = uint8_t (page [pc] + x);
		{
			int temp = READ_LOW(data);
			nz = (temp >> 1);
			c = temp << 8;
			pc++;
			WRITE_LOW(data, nz);
			goto loop;
		}

	case 0x46: // LSR zp
		pc++;
		data = page [pc];
		{
			int temp = READ_LOW(data);
			nz = (temp >> 1);
			c = temp << 8;
			pc++;
			WRITE_LOW(data, nz);
			goto loop;
		}

	case 0x66: // ROR zp
		pc++;
		data = page [pc];
	{
		int temp = READ_LOW( data );
		nz = ((c >> 1) & 0x80) | (temp >> 1);
		c = temp << 8;
		pc++;
		WRITE_LOW(data, nz);
		goto loop;
	}
	
	case 0x36: // ROL zp,x
		pc++;
		data = page [pc];
		data = uint8_t (data + x);
		nz = (c >> 8) & 1;
		nz |= (c = READ_LOW(data) << 1);
		WRITE_LOW(data, nz);
		pc++;
		goto loop;
	
	case 0x16: // ASL zp,x
		pc++;
		data = uint8_t (page [pc] + x);
		nz = (c = READ_LOW(data) << 1);
		pc++;
		WRITE_LOW(data, nz);
		goto loop;

	case 0x06: // ASL zp
		pc++;
		data = page [pc];
		nz = (c = READ_LOW(data) << 1);
		WRITE_LOW(data, nz);
		pc++;
		goto loop;

	case 0x26: // ROL zp
		pc++;
		data = page [pc];
		nz = (c >> 8) & 1;
		nz |= (c = READ_LOW( data ) << 1);
		WRITE_LOW(data, nz);
		pc++;
		goto loop;
	
// Increment/decrement

	case 0xCA:
		pc++;           
		INC_DEC_XY( x, -1 ) // DEX
	
	case 0x88:
		pc++;           
		INC_DEC_XY( y, -1 ) // DEY
	
	case 0xF6: // INC zp,x
		pc++;
		data = uint8_t (page[pc] + x);
		nz = READ_LOW(data) + 1;
		WRITE_LOW(data, nz);
		pc++;
		goto loop;

	case 0xE6: // INC zp
		pc++;
		data = page [pc];
		nz = READ_LOW(data) + 1;
		WRITE_LOW(data, nz);
		pc++;
		goto loop;
	
	case 0xD6: // DEC zp,x
		pc++;
		data = page [pc];
		data = uint8_t (data + x);
		nz =  READ_LOW(data) - 1;
		WRITE_LOW(data, nz);
		pc++;
		goto loop;

	case 0xC6: // DEC zp
		pc++;
		data = page [pc];
		nz = READ_LOW( data ) - 1;
		pc++;
		WRITE_LOW( data, nz );
		goto loop;
	
	case 0xFE: { // INC abs,x
		pc++;           
		data = page [pc];
		int temp = data + x;
		data = x + GET_ADDR();
		READ_NO_RETURN( data - ( temp & 0x100 ) );
		{
			int temp;
			WRITE(data, temp = READ(data));
			nz = temp + 1;
			WRITE(data, (uint8_t)nz);
		}
		pc += 2;
		goto loop;
	}
	
	case 0xEE: // INC abs
		pc++;
		data = GET_ADDR();
		{
			int temp;
			WRITE(data, temp = READ(data));
			nz = temp + 1;
			WRITE(data, (uint8_t)nz);
		}
		pc += 2;
		goto loop;
	
	case 0xDE: { // DEC abs,x
		pc++;           
		data = page [pc];
		int temp = data + x;
		data = x + GET_ADDR();
		READ_NO_RETURN( data - ( temp & 0x100 ) );
		{
			int temp;
			WRITE(data, temp = READ(data));
			nz = temp - 1;
			WRITE(data, (uint8_t)nz);
		}
		pc += 2;
		goto loop;
	}
	
	case 0xCE: // DEC abs
		pc++;
		data = GET_ADDR();
	{
		int temp;
		WRITE( data, temp = READ( data ) );
		nz = temp -1;
		WRITE( data, (uint8_t) nz );
	}
		pc += 2;
		goto loop;
		
// Transfer

	case 0xAA: // TAX
		pc++;
		nz = x = a;
		goto loop;

	case 0x8A: // TXA
		pc++;
		nz = a = x;
		goto loop;

	case 0x9A: // TXS
		pc++;
		SET_SP( x ); // verified (no flag change)
		goto loop;
	
	case 0xBA: // TSX
		pc++;
		x = nz = GET_SP();
		goto loop;
	
// Stack
	
	case 0x48: // PHA
		pc++;
		PUSH( a ); // verified
		goto loop;
		
	case 0x68: // PLA
		pc++;
		a = nz = READ_LOW( sp );
		sp = (sp - 0xFF) | 0x100;
		goto loop;
		
	case 0x40: // RTI
		{
			int temp = READ_LOW( sp );
			pc   = READ_LOW( 0x100 | (sp - 0xFF) );
			pc  |= READ_LOW( 0x100 | (sp - 0xFE) ) * 0x100;
			sp = (sp - 0xFD) | 0x100;
			data = status;
			SET_STATUS( temp );
		}
		if ( !((data ^ status) & st_i) )
			goto loop; // I flag didn't change
	i_flag_changed:
		//dprintf( "%6d %s\n", time(), (status & st_i ? "SEI" : "CLI") );
		this->r.status = status; // update externally-visible I flag
		// update clock_limit based on modified I flag
		clock_limit = end_time_;
		if ( end_time_ <= irq_time_ || status & st_i)
			goto loop;
		clock_limit = irq_time_;
		goto loop;
	
	case 0x28:{// PLP
		pc++;           
		int temp = READ_LOW( sp );
		sp = (sp - 0xFF) | 0x100;
		data = status;
		SET_STATUS( temp );
		if ( !((data ^ status) & st_i) )
			goto loop; // I flag didn't change
		if ( !(status & st_i) )
			goto handle_cli;
		goto handle_sei;
	}
	
	case 0x08: { // PHP
		pc++;           
		int temp;
		CALC_STATUS( temp );
		PUSH( temp | st_b | st_r );
		goto loop;
	}
	
	case 0x6C: // JMP (ind)
		pc++;
		data = GET_ADDR();
		pc = READ( data );
		pc |= READ( (data & 0xFF00) | ((data + 1) & 0xFF) ) << 8;
		goto loop;
	
	case 0x00: { // BRK
		pc+=2;
		WRITE_LOW( 0x100 | (sp - 1), pc >> 8 );
		WRITE_LOW( 0x100 | (sp - 2), pc );
		int temp;
		CALC_STATUS( temp );
		sp = (sp - 3) | 0x100;
		WRITE_LOW( sp, temp | st_b | st_r );
		pc = GET_LE16( &code_map [0xFFFE >> page_bits] [0xFFFE] );
		status |= st_i;
		goto i_flag_changed;
	}
	
// Flags

	case 0x38: // SEC
		pc++;
		c = ~0;
		goto loop;
	
	case 0x18: // CLC
		pc++;
		c = 0;
		goto loop;
		
	case 0xB8: // CLV
		pc++;
		status &= ~st_v;
		goto loop;
	
	case 0xD8: // CLD
		pc++;
		status &= ~st_d;
		goto loop;
	
	case 0xF8: // SED
		pc++;
		status |= st_d;
		goto loop;
	
	case 0x58: // CLI
		pc++;
		if ( !(status & st_i) )
			goto loop;
		status &= ~st_i;
	handle_cli:
		//dprintf( "%6d CLI\n", time() );
		this->r.status = status; // update externally-visible I flag
		if ( clock_count < end_time_ )
		{
			assert( clock_limit == end_time_ );
			if ( end_time_ <= irq_time_ )
				goto loop; // irq is later
			if ( clock_count >= irq_time_ )
				irq_time_ = clock_count + 1; // delay IRQ until after next instruction
			clock_limit = irq_time_;
			goto loop;
		}
		// execution is stopping now, so delayed CLI must be handled by caller
		result = result_cli;
		goto end;
		
	case 0x78: // SEI
		pc++;
		if ( status & st_i )
			goto loop;
		status |= st_i;
	handle_sei:
		//dprintf( "%6d SEI\n", time() );
		this->r.status = status; // update externally-visible I flag
		clock_limit = end_time_;
		if ( clock_count < irq_time_ )
			goto loop;
		result = result_sei; // IRQ will occur now, even though I flag is set
		goto end;

// Unofficial
	case 0x1C: case 0x3C: case 0x5C: case 0x7C: case 0xDC: case 0xFC: { // SKW
		pc++;
		data = page [pc] + x;
		HANDLE_PAGE_CROSSING( data );
		int addr = GET_ADDR() + x;
		if ( data & 0x100 )
			READ_NO_RETURN( addr - 0x100 );
		READ_NO_RETURN( addr );
	}
	case 0x0C: // SKW
		pc+=3;
		goto loop;

	case 0x74: case 0x04: case 0x14: case 0x34: case 0x44: case 0x54: case 0x64: // SKB
	case 0x80: case 0x82: case 0x89: case 0xC2: case 0xD4: case 0xE2: case 0xF4:
		pc+=2;
		goto loop;

	case 0xEA: case 0x1A: case 0x3A: case 0x5A: case 0x7A: case 0xDA: case 0xFA: // NOP
		pc++;
		goto loop;

	// ARITH_ADDR_MODES_PTR
	case 0xC7 - 0x04: /* (ind,x) */
		pc++;
		data = page [pc];
		IND_X
		WRITE(data, nz = READ(data));
		nz = uint8_t(nz - 1);
		WRITE(data, nz);
		pc++;
		nz = a - nz;
		c = ~nz;
		nz &= 0xFF;
		goto loop;

	case 0xC7 + 0x0C:
		pc++;
		data = page [pc];
		IND_Y(false,false)
		WRITE(data, nz = READ(data));
		nz = uint8_t(nz - 1);
		WRITE(data, nz);
		pc++;
		nz = a - nz;
		c = ~nz;
		nz &= 0xFF;
		goto loop;

	case 0xC7 + 0x10: /* zp,X */
		pc++;
		data = uint8_t (page [pc] + x);
		WRITE(data, nz = READ(data));
		nz = uint8_t(nz - 1);
		WRITE(data, nz);
		pc++;
		nz = a - nz;
		c = ~nz;
		nz &= 0xFF;
		goto loop;

	case 0xC7 + 0x14: /* abs,Y */
		pc++;
		data = page [pc] + y;
		{
		int temp = data;
		ADD_PAGE
		READ_NO_RETURN(data - (temp & 0x100));
		WRITE(data, nz = READ(data));
		nz = uint8_t(nz - 1);
		WRITE(data, nz);
		pc++;
		nz = a - nz;
		c = ~nz;
		nz &= 0xFF;
		goto loop;
		}

	case 0xC7 + 0x18: /* abs,X */
		pc++;
		data = page [pc] + x;
		{
		int temp = data;
		ADD_PAGE
		READ_NO_RETURN( data - ( temp & 0x100 ) );
		WRITE(data, nz = READ(data));
		nz = uint8_t(nz - 1);
		WRITE(data, nz);
		pc++;
		nz = a - nz;
		c = ~nz;
		nz &= 0xFF;
		goto loop;
		}

	case 0xC7 + 0x08: /* abs */
		pc++;
		data = page [pc];
		ADD_PAGE
		WRITE(data, nz = READ(data));
		nz = uint8_t(nz - 1);
		WRITE(data, nz);
		pc++;
		nz = a - nz;
		c = ~nz;
		nz &= 0xFF;
		goto loop;

	case 0xC7 + 0x00: /* zp */
		pc++;
		data = page [pc]; // DCP
		WRITE( data, nz = READ( data ) );
		nz = uint8_t( nz - 1 );
		WRITE( data, nz );
		pc++;
		nz = a - nz;
		c = ~nz;
		nz &= 0xFF;
		goto loop;


// ARITH_ADDR_MODES_PTR( 0xE7 )      
	case 0xE7 - 0x04: /* (ind,x) */    
		pc++;       
		data = page [pc];                   
		IND_X 
		WRITE(data, nz = READ(data));
		nz = uint8_t(nz + 1);
		WRITE(data, nz);
		data = nz ^ 0xFF;
		{
			int carry = (c >> 8) & 1;
			int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
			status &= ~st_v;
			status |= (ov >> 2) & 0x40;
			c = nz = a + data + carry;
			pc++;
			a = (uint8_t)nz;
			goto loop;
		}

	case 0xE7 + 0x0C:
		pc++;
		data = page [pc];                   
		IND_Y(false,false)
		WRITE(data, nz = READ(data));
		nz = uint8_t(nz + 1);
		WRITE(data, nz);
		data = nz ^ 0xFF;
		{
			int carry = (c >> 8) & 1;
			int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
			status &= ~st_v;
			status |= (ov >> 2) & 0x40;
			c = nz = a + data + carry;
			pc++;
			a = (uint8_t)nz;
			goto loop;
		}

	case 0xE7 + 0x10: /* zp,X */   
		pc++;           
		data = uint8_t (page [pc] + x);     
		WRITE(data, nz = READ(data));
		nz = uint8_t(nz + 1);
		WRITE(data, nz);
		data = nz ^ 0xFF;
		{
			int carry = (c >> 8) & 1;
			int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
			status &= ~st_v;
			status |= (ov >> 2) & 0x40;
			c = nz = a + data + carry;
			pc++;
			a = (uint8_t)nz;
			goto loop;
		}

	case 0xE7 + 0x14: /* abs,Y */    
		pc++;         
		data = page [pc] + y;               
		{
			int temp = data;
			ADD_PAGE
				READ_NO_RETURN(data - (temp & 0x100));
			WRITE(data, nz = READ(data));
			nz = uint8_t(nz + 1);
			WRITE(data, nz);
			data = nz ^ 0xFF;
			{
				int carry = (c >> 8) & 1;
				int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
				status &= ~st_v;
				status |= (ov >> 2) & 0x40;
				c = nz = a + data + carry;
				pc++;
				a = (uint8_t)nz;
				goto loop;
			}
		}

	case 0xE7 + 0x18: /* abs,X */      
		pc++;       
		data = page [pc] + x;               
	{                              
		int temp = data;                    
		ADD_PAGE                            
		READ_NO_RETURN( data - ( temp & 0x100 ) );    
		WRITE(data, nz = READ(data));
		nz = uint8_t(nz + 1);
		WRITE(data, nz);
		data = nz ^ 0xFF;
		{
			int carry = (c >> 8) & 1;
			int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
			status &= ~st_v;
			status |= (ov >> 2) & 0x40;
			c = nz = a + data + carry;
			pc++;
			a = (uint8_t)nz;
			goto loop;
		}
	}                                       
	case 0xE7 + 0x08: /* abs */         
		pc++;      
		data = page [pc];                   
		ADD_PAGE                            
			WRITE(data, nz = READ(data));
		nz = uint8_t(nz + 1);
		WRITE(data, nz);
		data = nz ^ 0xFF;
		{
			int carry = (c >> 8) & 1;
			int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
			status &= ~st_v;
			status |= (ov >> 2) & 0x40;
			c = nz = a + data + carry;
			pc++;
			a = (uint8_t)nz;
			goto loop;
		}

	case 0xE7 + 0x00: /* zp */              
		pc++;  
		data = page [pc]; // ISC
		WRITE(data, nz = READ(data));
		nz = uint8_t(nz + 1);
		WRITE(data, nz);
		data = nz ^ 0xFF; 
		{
			int carry = (c >> 8) & 1;
			int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
			status &= ~st_v;
			status |= (ov >> 2) & 0x40;
			c = nz = a + data + carry;
			pc++;
			a = (uint8_t)nz;
			goto loop;
		}


	// ARITH_ADDR_MODES_PTR( 0x27 )      
	case 0x27 - 0x04: /* (ind,x) */         
		pc++;  
		data = page [pc];                   
		IND_X
		{ // RLA
			WRITE(data, nz = READ(data));
		int temp = c;
		c = nz << 1;
		nz = uint8_t(c) | ((temp >> 8) & 0x01);
		WRITE(data, nz);
		pc++;
		nz = a &= nz;
		goto loop;
		}

	case 0x27 + 0x0C:
		pc++;
		data = page [pc];                   
		IND_Y(false,false) 
		{ // RLA
			WRITE(data, nz = READ(data));
			int temp = c;
			c = nz << 1;
			nz = uint8_t(c) | ((temp >> 8) & 0x01);
			WRITE(data, nz);
			pc++;
			nz = a &= nz;
			goto loop;
		}

	case 0x27 + 0x10: /* zp,X */              
		pc++;
		data = uint8_t (page [pc] + x);     
		{ // RLA
			WRITE(data, nz = READ(data));
			int temp = c;
			c = nz << 1;
			nz = uint8_t(c) | ((temp >> 8) & 0x01);
			WRITE(data, nz);
			pc++;
			nz = a &= nz;
			goto loop;
		}

	case 0x27 + 0x14: /* abs,Y */             
		pc++;
		data = page [pc] + y;               
		{
			int temp = data;
			ADD_PAGE
				READ_NO_RETURN(data - (temp & 0x100));
			{ // RLA
				WRITE(data, nz = READ(data));
				int temp = c;
				c = nz << 1;
				nz = uint8_t(c) | ((temp >> 8) & 0x01);
				WRITE(data, nz);
				pc++;
				nz = a &= nz;
				goto loop;
			}
		}

	case 0x27 + 0x18: /* abs,X */             
		pc++;
		data = page [pc] + x;               
	{                              
		int temp = data;                    
		ADD_PAGE                            
		READ_NO_RETURN( data - ( temp & 0x100 ) );    
		{ // RLA
			WRITE(data, nz = READ(data));
			int temp = c;
			c = nz << 1;
			nz = uint8_t(c) | ((temp >> 8) & 0x01);
			WRITE(data, nz);
			pc++;
			nz = a &= nz;
			goto loop;
		}
	}      

	case 0x27 + 0x08: /* abs */               
		pc++;
		data = page [pc];                   
		ADD_PAGE                            
		{ // RLA
			WRITE(data, nz = READ(data));
		int temp = c;
		c = nz << 1;
		nz = uint8_t(c) | ((temp >> 8) & 0x01);
		WRITE(data, nz);
		pc++;
		nz = a &= nz;
		goto loop;
		}

	case 0x27 + 0x00: /* zp */                
		pc++;
		data = page [pc];                   
	{ // RLA
		WRITE(data, nz = READ(data));
		int temp = c;
		c = nz << 1;
		nz = uint8_t(c) | ((temp >> 8) & 0x01);
		WRITE(data, nz);
		pc++;
		nz = a &= nz;
		goto loop;
		}


	// ARITH_ADDR_MODES_PTR( 0x67 )      
	case 0x67 - 0x04: /* (ind,x) */          
		pc++; 
		data = page [pc];                   
		IND_X                               
		{ // RRA
			int temp;
		WRITE(data, temp = READ(data));
		nz = ((c >> 1) & 0x80) | (temp >> 1);
		WRITE(data, nz);
		data = nz;
		c = temp << 8;
		{
			int carry = (c >> 8) & 1;
			int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
			status &= ~st_v;
			status |= (ov >> 2) & 0x40;
			c = nz = a + data + carry;
			pc++;
			a = (uint8_t)nz;
			goto loop;
		}
		}

	case 0x67 + 0x0C:
		pc++;
		data = page [pc];                   
		IND_Y(false,false)                  
		{ // RRA
			int temp;
			WRITE(data, temp = READ(data));
			nz = ((c >> 1) & 0x80) | (temp >> 1);
			WRITE(data, nz);
			data = nz;
			c = temp << 8;
			{
				int carry = (c >> 8) & 1;
				int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
				status &= ~st_v;
				status |= (ov >> 2) & 0x40;
				c = nz = a + data + carry;
				pc++;
				a = (uint8_t)nz;
				goto loop;
			}
		}

	case 0x67 + 0x10: /* zp,X */        
		pc++;      
		data = uint8_t (page [pc] + x);     
		{ // RRA
			int temp;
			WRITE(data, temp = READ(data));
			nz = ((c >> 1) & 0x80) | (temp >> 1);
			WRITE(data, nz);
			data = nz;
			c = temp << 8;
			{
				int carry = (c >> 8) & 1;
				int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
				status &= ~st_v;
				status |= (ov >> 2) & 0x40;
				c = nz = a + data + carry;
				pc++;
				a = (uint8_t)nz;
				goto loop;
			}
		}

	case 0x67 + 0x14: /* abs,Y */        
		pc++;     
		data = page [pc] + y;               
		{
			int temp = data;
			ADD_PAGE
			READ_NO_RETURN(data - (temp & 0x100));
			{ // RRA
				int temp;
				WRITE(data, temp = READ(data));
				nz = ((c >> 1) & 0x80) | (temp >> 1);
				WRITE(data, nz);
				data = nz;
				c = temp << 8;
				{
					int carry = (c >> 8) & 1;
					int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
					status &= ~st_v;
					status |= (ov >> 2) & 0x40;
					c = nz = a + data + carry;
					pc++;
					a = (uint8_t)nz;
					goto loop;
				}
			}
		}

	case 0x67 + 0x18: /* abs,X */          
		pc++;   
		data = page [pc] + x;               
		{                              
		int temp = data;                    
		ADD_PAGE                            
		READ_NO_RETURN( data - ( temp & 0x100 ) );    
		{ // RRA
			int temp;
			WRITE(data, temp = READ(data));
			nz = ((c >> 1) & 0x80) | (temp >> 1);
			WRITE(data, nz);
			data = nz;
			c = temp << 8;
			{
				int carry = (c >> 8) & 1;
				int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
				status &= ~st_v;
				status |= (ov >> 2) & 0x40;
				c = nz = a + data + carry;
				pc++;
				a = (uint8_t)nz;
				goto loop;
			}
		}
	}     

	case 0x67 + 0x08: /* abs */        
		pc++;       
		data = page [pc];                   
		ADD_PAGE                            
		{ // RRA
			int temp;
		WRITE(data, temp = READ(data));
		nz = ((c >> 1) & 0x80) | (temp >> 1);
		WRITE(data, nz);
		data = nz;
		c = temp << 8;
		{
			int carry = (c >> 8) & 1;
			int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
			status &= ~st_v;
			status |= (ov >> 2) & 0x40;
			c = nz = a + data + carry;
			pc++;
			a = (uint8_t)nz;
			goto loop;
		}
		}

	case 0x67 + 0x00: /* zp */         
		pc++;       
		data = page [pc];                   
		{ // RRA
		int temp;
		WRITE(data, temp = READ(data));
		nz = ((c >> 1) & 0x80) | (temp >> 1);
		WRITE(data, nz);
		data = nz;
		c = temp << 8; 
		{
			int carry = (c >> 8) & 1;
			int ov = (a ^ 0x80) + carry + (BOOST::int8_t) data; // sign-extend
			status &= ~st_v;
			status |= (ov >> 2) & 0x40;
			c = nz = a + data + carry;
			pc++;
			a = (uint8_t)nz;
			goto loop;
		}
		}

	// ARITH_ADDR_MODES_PTR( 0x07 )      
	case 0x07 - 0x04: /* (ind,x) */       
		pc++;    
		data = page [pc];                   
		IND_X                               
			// SLO
			WRITE(data, nz = READ(data));
		c = nz << 1;
		nz = uint8_t(c);
		WRITE(data, nz);
		nz = (a |= nz);
		pc++;
		goto loop;

	case 0x07 + 0x0C:
		pc++;
		data = page [pc];                   
		IND_Y(false,false)                  
			// SLO
			WRITE(data, nz = READ(data));
		c = nz << 1;
		nz = uint8_t(c);
		WRITE(data, nz);
		nz = (a |= nz);
		pc++;
		goto loop;

	case 0x07 + 0x10: /* zp,X */           
		pc++;   
		data = uint8_t (page [pc] + x);     
		// SLO
		WRITE(data, nz = READ(data));
		c = nz << 1;
		nz = uint8_t(c);
		WRITE(data, nz);
		nz = (a |= nz);
		pc++;
		goto loop;

	case 0x07 + 0x14: /* abs,Y */     
		pc++;        
		data = page [pc] + y;               
		{
			int temp = data;
			ADD_PAGE
				READ_NO_RETURN(data - (temp & 0x100));
			// SLO
			WRITE(data, nz = READ(data));
			c = nz << 1;
			nz = uint8_t(c);
			WRITE(data, nz);
			nz = (a |= nz);
			pc++;
			goto loop;
		}

	case 0x07 + 0x18: /* abs,X */       
		pc++;      
		data = page [pc] + x;               
		{                              
		int temp = data;                    
		ADD_PAGE                            
		READ_NO_RETURN( data - ( temp & 0x100 ) );    
		// SLO
		WRITE(data, nz = READ(data));
		c = nz << 1;
		nz = uint8_t(c);
		WRITE(data, nz);
		nz = (a |= nz);
		pc++;
		goto loop;
	}         

	case 0x07 + 0x08: /* abs */     
		pc++;          
		data = page [pc];                   
		ADD_PAGE                            
			// SLO
			WRITE(data, nz = READ(data));
		c = nz << 1;
		nz = uint8_t(c);
		WRITE(data, nz);
		nz = (a |= nz);
		pc++;
		goto loop;

	case 0x07 + 0x00: /* zp */     
		pc++;           
		data = page [pc];                   
		// SLO
			WRITE(data, nz = READ(data));
			c = nz << 1;
			nz = uint8_t(c);
			WRITE(data, nz);
			nz = (a |= nz);
			pc++;
			goto loop;

	// ARITH_ADDR_MODES_PTR( 0x47 )      
	case 0x47 - 0x04: /* (ind,x) */        
		pc++;   
		data = page [pc];                   
		IND_X                               
			// SRE
			WRITE(data, nz = READ(data));
		c = nz << 8;
		nz >>= 1;
		WRITE(data, nz);
		nz = a ^= nz;
		pc++;
		goto loop;

	case 0x47 + 0x0C:
		pc++;
		data = page [pc];                   
		IND_Y(false,false)                  
			// SRE
			WRITE(data, nz = READ(data));
		c = nz << 8;
		nz >>= 1;
		WRITE(data, nz);
		nz = a ^= nz;
		pc++;
		goto loop;

	case 0x47 + 0x10: /* zp,X */             
		pc++; 
		data = uint8_t (page [pc] + x);     
		// SRE
		WRITE(data, nz = READ(data));
		c = nz << 8;
		nz >>= 1;
		WRITE(data, nz);
		nz = a ^= nz;
		pc++;
		goto loop;

	case 0x47 + 0x14: /* abs,Y */           
		pc++;  
		data = page [pc] + y;               
		{
			int temp = data;
			ADD_PAGE
				READ_NO_RETURN(data - (temp & 0x100));
			// SRE
			WRITE(data, nz = READ(data));
			c = nz << 8;
			nz >>= 1;
			WRITE(data, nz);
			nz = a ^= nz;
			pc++;
			goto loop;
		}

	case 0x47 + 0x18: /* abs,X */         
		pc++;    
		data = page [pc] + x;               
		{                              
		int temp = data;                    
		ADD_PAGE                            
		READ_NO_RETURN( data - ( temp & 0x100 ) );    
		// SRE
		WRITE(data, nz = READ(data));
		c = nz << 8;
		nz >>= 1;
		WRITE(data, nz);
		nz = a ^= nz;
		pc++;
		goto loop;
	}      

	case 0x47 + 0x08: /* abs */              
		pc++; 
		data = page [pc];                   
		ADD_PAGE                            
			// SRE
			WRITE(data, nz = READ(data));
		c = nz << 8;
		nz >>= 1;
		WRITE(data, nz);
		nz = a ^= nz;
		pc++;
		goto loop;

	case 0x47 + 0x00: /* zp */         
		pc++;       
		data = page [pc];                   
		// SRE
			WRITE( data, nz = READ( data ) );
			c = nz << 8;
			nz >>= 1;
			WRITE( data, nz );
			nz = a ^= nz;
			pc++;
			goto loop;

	case 0x4B: // ALR
		pc++;
		a &= page [pc];
		c = a << 8;
		a = nz = a >> 1;
		pc++;
		goto loop;

	case 0x0B: // ANC
	case 0x2B:
		pc++;
		nz = a &= page[pc];
		c = a << 1;
		pc++;
		goto loop;

	case 0x6B: // ARR
		pc++;
		nz = a = uint8_t( ( ( page[pc] & a ) >> 1 ) | ( ( c >> 1 ) & 0x80 ) );
		c = a << 2;
		status = ( status & ~st_v ) | ( ( a ^ a << 1 ) & st_v );
		pc++;
		goto loop;

	case 0xAB: // LXA
		pc++;
		nz = x = a = page [pc];
		pc++;
		goto loop;

	case 0xA3: // LAX
		pc++;
		data = page [pc];
		IND_X
		nz = x = a = READ(data);
		pc++;
		goto loop;

	case 0xB3:
		pc++;
		data = page[pc];
		IND_Y(true,true)
		nz = x = a = READ(data);
		pc++;
		goto loop;

	case 0xB7:
		pc++;
		data = uint8_t (page[pc] + y);
		nz = x = a = READ_LOW(data);
		pc++;
		goto loop;

	case 0xA7:
		pc++;
		nz = x = a = READ_LOW( page[pc] );
		pc++;
		goto loop;

	case 0xBF: {
		pc++;
		data = page[pc] + y;
		HANDLE_PAGE_CROSSING( data );
		int temp = data;
		ADD_PAGE;
		if ( temp & 0x100 )
			READ_NO_RETURN( data - 0x100 );
		nz = x = a = READ(data);
		pc++;
		goto loop;
	}

	case 0xAF:
		pc++;
		data = page[pc];
		ADD_PAGE
		nz = x = a = READ( data );
		pc++;
		goto loop;

	case 0x83: // SAX
		pc++;
		data = page[pc];
		IND_X
		WRITE(data, a & x);
		pc++;
		goto loop;

	case 0x97:
		pc++;
		data = page[pc];
		data = uint8_t (data + y);
		WRITE(data, a & x);
		pc++;
		goto loop;

	case 0x8F:
		pc++;
		data = page[pc];
		ADD_PAGE
		WRITE(data, a & x);
		pc++;
		goto loop;

	case 0x87:
		pc++;
		data = page[pc];
		WRITE( data, a & x );
		pc++;
		goto loop;

	case 0xCB: // SBX
		pc++;
		data = ( a & x ) - page[pc];
		c = ( data <= 0xFF ) ? 0x100 : 0;
		nz = x = uint8_t( data );
		pc++;
		goto loop;

	case 0x93: // SHA (ind),Y
		pc++;
		data = page[pc];
		IND_Y(false,false)
		pc++;
		WRITE( data, uint8_t( a & x & ( ( data >> 8 ) + 1 ) ) );
		goto loop;

	case 0x9F: { // SHA abs,Y
		pc++;
		data = page[pc] + y;
		int temp = data;
		ADD_PAGE
		READ_NO_RETURN( data - ( temp & 0x100 ) );
		pc++;
		WRITE( data, uint8_t( a & x & ( ( data >> 8 ) + 1 ) ) );
		goto loop;
	}

	case 0x9E: { // SHX abs,Y
		pc++;
		data = page[pc] + y;
		int temp = data;
		ADD_PAGE
		READ_NO_RETURN( data - ( temp & 0x100 ) );
		pc++;
		if ( !( temp & 0x100 ) )
			WRITE( data, uint8_t( x & ( ( data >> 8 ) + 1 ) ) );
		goto loop;
	}

	case 0x9C: { // SHY abs,X
		pc++;
		data = page[pc] + x;
		int temp = data;
		ADD_PAGE
		READ_NO_RETURN( data - ( temp & 0x100 ) );
		pc++;
		if ( !( temp & 0x100) )
			WRITE( data, uint8_t( y & ( ( data >> 8 ) + 1 ) ) );
		goto loop;
	}

	case 0x9B: { // SHS abs,Y
		pc++;
		data = page[pc] + y;
		int temp = data;
		ADD_PAGE
		READ_NO_RETURN( data - ( temp & 0x100 ) );
		pc++;
		SET_SP( a & x );
		WRITE( data, uint8_t( a & x & ( ( data >> 8 ) + 1 ) ) );
		goto loop;
	}

	case 0xBB: { // LAS abs,Y
		pc++;
		data = page[pc] + y;
		HANDLE_PAGE_CROSSING( data );
		int temp = data;
		ADD_PAGE
		if ( temp & 0x100 )
			READ_NO_RETURN( data - 0x100 );
		pc++;
		a = GET_SP();
		x = a &= READ( data );
		SET_SP( a );
		goto loop;
	}

// Unimplemented
	
	case page_wrap_opcode: // HLT
		pc++;
		if ( pc > 0x10000 )
		{
			// handle wrap-around (assumes caller has put page of HLT at 0x10000)
			pc = (pc - 1) & 0xFFFF;
			clock_count -= 2;
			goto loop;
		}
		// fall through
		{
		// skip over proper number of bytes
		static unsigned char const row[8] = { 0x95, 0x95, 0x95, 0xd5, 0x95, 0x95, 0xd5, 0xf5 };
		int len = row[opcode >> 2 & 7] >> (opcode << 1 & 6) & 3;
		if (opcode == 0x9C)
			len = 3;
		pc += len - 1;
		error_count_++;
		goto loop;
		}

	default:
		pc++;
		// skip over proper number of bytes
		static unsigned char const row [8] = { 0x95, 0x95, 0x95, 0xd5, 0x95, 0x95, 0xd5, 0xf5 };
		int len = row [opcode >> 2 & 7] >> (opcode << 1 & 6) & 3;
		if ( opcode == 0x9C )
			len = 3;
		pc += len - 1;
		error_count_++;
		goto loop;
		
		//result = result_badop; // TODO: re-enable
		//pc--;
		//goto end;
	}
	
	// If this fails then the case above is missing an opcode
	assert( false );
	
end:
	
	{
		int temp;
		CALC_STATUS( temp );
		r.status = temp;
	}
	
	this->clock_count = clock_count;
	r.pc = pc;
	r.sp = GET_SP();
	r.a = a;
	r.x = x;
	r.y = y;
	irq_time_ = LONG_MAX / 2 + 1;
	
	return result;
}

#endif

