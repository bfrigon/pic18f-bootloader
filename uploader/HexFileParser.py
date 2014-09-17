import serial
import os, sys
import re

devices = {
	#devname:[mem_program_size, mem_eeprom_size]
	'18f2520':[0x8000, 0x80]
}

class ExceptionParserError(Exception): pass
class ExceptionUploadError(Exception): pass


class HexFileParser:
	blocks = {}
	device = None
	
	
	def __init__(self, filename, device = None, logger = None, silent = False):

		self.device = device
		self.filename = filename
		self.logger = logger
		self.silent = silent
		
		linenum = 0
		addrh = 0
		fptr = open(self.filename, "r");
	

		while True:
			line = fptr.readline().upper()
			if len(line) == 0:
				break
		
			line = line.strip()	
			if line[0] == ';':
				line = line.lstrip(';')
				self.device = line.replace('PIC', '')

				continue
		
			linenum += 1
		
			length = int(line[1:3], 16)
			addr = addrh + int(line[3:7], 16)
			rtype = int(line[7:9], 16)
			checksum = self.calc_checksum(line[1:-2])
		
			if checksum != int(line[-2:], 16):
				raise ExceptionParserError('Invalid checksum in "%s" at line %d' % (self.filename, linenum))

		
			if rtype == 0x00:
				self.store_block(addr, line[9:-2])
			
			elif rtype == 0x01:
				break
					
			elif rtype == 0x04:
				if length < 2:
					raise ExceptionParserError('Invalid record data in "%s" at line %d' % (self.filename, linenum))
				
				addrh = int(line[9:13], 16) << 16
			
			else:
				raise ExceptionParserError('Unexpected record type in "%s" at line %d' % (self.filename, linenum))
	


	def calc_checksum(self, data):
		checksum = 0
	
		for i in range(0, len(data), 2):
			checksum = (checksum - int(data[i:i+2], 16)) & 0xFF
	
		return checksum



	def store_block(self, addr, data):
		if (addr >= 0x200000):
			self.blocks[addr] = data;
			return
	
		blk_start = addr & 0xFFFFFFC0
		blk_offset = (addr & 0x3F)

		try:
			block = self.blocks[blk_start]
		except KeyError:
			block = 'FF' * 64
	
		if blk_offset + (len(data) / 2) > 64:
			block = block[:blk_offset * 2] + data[:128-(blk_offset * 2)]
		
			# Store the remaining bytes in the next block #
			self.store_block((addr + 64) & 0xFFFFFFC0, data[128-(blk_offset * 2):])
		
		else:
			block = block[:blk_offset * 2] + data + block[(blk_offset * 2) + len(data):]

		self.blocks[blk_start] = block
	
	
	
	def validate(self):
		try:
			specs = devices[self.device]
		except KeyError:
			specs = [0x1FFFFF, 0x400]
		
		
		#for block in sorted(hexfile.blocks.iterkeys()):
		
		
		return True

	
	def send_command(self, sio, cmd, data = ''):
		output = cmd + data
		
		if len(data) > 0:
			output = output + ('%02X' % self.calc_checksum(data))

		if self.logger:
			self.logger.debug('Sending >> ' + output)
					
		sio.write(output)
		
		response = sio.read(3)

		if len(response) == 0:
			raise ExceptionUploadError('Timeout. No response from target')
	
		if self.logger:
			self.logger.debug('Receiving << ' + str(response))
		
		if response == 'SF0':
			raise ExceptionUploadError('(F0) Attemps to write over protected memory area')
		elif response == 'SF1':
			raise ExceptionUploadError('(F1) Buffer overflow, data block length is grater than 64 bytes')
		elif response == 'SF2':
			raise ExceptionUploadError('(F2) Attemps to write to an invalid location')
		elif response == 'SF3':
			raise ExceptionUploadError('(F3) Invalid checksum, data block corrupted')
		elif response == 'SF5':
			raise ExceptionUploadError('(F5) Write failed')
		
		
	
	def upload(self, port, baud=38400, skip=[]):
		
		### Open serial port ###
		if self.logger and not self.silent:
			self.logger.info('Using serial port: "%s"' % port)
		
		sio = serial.Serial(port, baud, timeout=5)

		### Send ping command to bootloader ###
		self.send_command(sio, '?')
		
		if self.logger and not self.silent:
			self.logger.info('Connected to target')

		if self.logger and not self.silent:
			self.logger.info('Uploading to target...')
			
		p_block_addr = 0
		p_block_len = 0
		for block in sorted(self.blocks.iterkeys()):
		
			try:
				if len(skip) > 1 and block >= skip[0] and block <= skip[1]:
				
					if self.logger and not self.silent:
						self.logger.info('Skipping block at 0x%06X' % block)			
						
					continue

				
				if block > p_block_addr + p_block_len:
					self.send_command(sio, '@', '%06X' % block)
	
				data = self.blocks[block]
				p_block_addr = block

				p_block_len = len(data) / 2
			
				self.send_command(sio, '+', '%02X%s' % (p_block_len, data))
			
			except ExceptionUploadError as e:
				raise ExceptionUploadError(str(e) + ' (block: 0x%06X)' % block)			
	
		self.send_command(sio, '!')
		
		if self.logger and not self.silent:
			self.logger.info('Done, target has been programmed.')					

		sio.close()

