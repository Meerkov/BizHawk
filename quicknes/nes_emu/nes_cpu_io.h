
#include "Nes_Core.h"
#include "Nes_Mapper.h"

#include "blargg_source.h"

int Nes_Core::cpu_read(nes_addr_t addr, nes_time_t time)
{
	//LOG_FREQ( "cpu_read", 16, addr >> 12 );

	{
		if (!(addr & 0xE000))
			return cpu::low_mem[addr & 0x7FF];
	}

	{
		if (addr > 0x7FFF)
			return *cpu::get_code(addr);
	}

	time += cpu_time_offset;
	if (addr < 0x4000)
		return ppu.read(addr, time);

	clock_ = time;
	if (data_reader_mapped[addr >> page_bits])
	{
		int result = mapper->read(time, addr);
		if (result >= 0)
			return result;
	}

	if (addr < 0x6000)
		return read_io(addr);

	if (addr < sram_readable)
		return impl->sram[addr & (impl_t::sram_size - 1)];

	if (addr < lrom_readable)
		return *cpu::get_code(addr);

#ifndef NDEBUG
	log_unmapped(addr);
#endif

	return addr >> 8; // simulate open bus
}

void Nes_Core::cpu_read_clock_only(nes_addr_t addr, nes_time_t time)
{
	if (!(addr & 0xE000))
		return;

	if (addr > 0x7FFF)
		return;

	time += cpu_time_offset;
	if (addr < 0x4000) {
		ppu.read(addr, time);
		return;
	}

	clock_ = time;
	if (data_reader_mapped[addr >> page_bits])
	{
		if (mapper->read(time, addr) >= 0)
			return;
	}

	if (addr < 0x6000) {
		read_io(addr);
		return;
	}

	if (addr < sram_readable) {
		impl->sram[addr & (impl_t::sram_size - 1)];
		return;
	}

	if (addr < lrom_readable)
		return;

#ifndef NDEBUG
	log_unmapped(addr);
#endif

	return;
}

inline int Nes_Core::cpu_read_ppu( nes_addr_t addr, nes_time_t time )
{
	//LOG_FREQ( "cpu_read_ppu", 16, addr >> 12 );
	
	// Read of status register (0x2002) is heavily optimized since many games
	// poll it hundreds of times per frame.
	nes_time_t next = ppu_2002_time;
	int result = ppu.r2002;
	if ( addr == 0x2002 )
	{
		ppu.second_write = false;
		if ( time >= next )
			result = ppu.read_2002( time + cpu_time_offset );
	}
	else
	{
		result = cpu::low_mem [addr & 0x7FF];
		if ( addr >= 0x2000 )
			result = cpu_read( addr, time );
	}
	
	return result;
}

void Nes_Core::cpu_write_2007( int data )
{
	// ppu.write_2007() is inlined
	if ( ppu.write_2007( data ) & Nes_Ppu::vaddr_clock_mask )
		mapper->a12_clocked();
}

void Nes_Core::cpu_write( nes_addr_t addr, int data, nes_time_t time )
{
	//LOG_FREQ( "cpu_write", 16, addr >> 12 );
	
	if ( !(addr & 0xE000) )
	{
		cpu::low_mem [addr & 0x7FF] = data;
		return;
	}
	
	time += cpu_time_offset;
	if ( addr < 0x4000 )
	{
		if ( (addr & 7) == 7 )
			cpu_write_2007( data );
		else
			ppu.write( time, addr, data );
		return;
	}
	
	clock_ = time;
	if ( data_writer_mapped [addr >> page_bits] && mapper->write_intercepted( time, addr, data ) )
		return;
	
	if ( addr < 0x6000 )
	{
		write_io( addr, data );
		return;
	}
	
	if ( addr < sram_writable )
	{
		impl->sram [addr & (impl_t::sram_size - 1)] = data;
		return;
	}
	
	if ( addr > 0x7FFF )
	{
		mapper->write( clock_, addr, data );
		return;
	}
	
	#ifndef NDEBUG
		log_unmapped( addr, data );
	#endif
}

#define NES_CPU_READ_PPU( cpu, addr, time ) \
	STATIC_CAST(Nes_Core&,*cpu).cpu_read_ppu( addr, time )

#define NES_CPU_READ( cpu, addr, time ) \
	STATIC_CAST(Nes_Core&,*cpu).cpu_read( addr, time )

#define NES_CPU_READ_CLOCK_ONLY( cpu, addr, time ) \
	STATIC_CAST(Nes_Core&,*cpu).cpu_read_clock_only( addr, time )

#define NES_CPU_WRITEX( cpu, addr, data, time ){\
	STATIC_CAST(Nes_Core&,*cpu).cpu_write( addr, data, time );\
}

#define NES_CPU_WRITE( cpu, addr, data, time ){\
	if ( addr < 0x800 ) cpu->low_mem [addr] = data;\
	else if ( addr == 0x2007 ) STATIC_CAST(Nes_Core&,*cpu).cpu_write_2007( data );\
	else STATIC_CAST(Nes_Core&,*cpu).cpu_write( addr, data, time );\
}

