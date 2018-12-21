/*
amigafloppy - driver for the USB floppy controller for amiga disks
Copyright (C) 2018  John Tsiombikas <nuclear@member.fsf.org>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <stdio.h>
#include <stdlib.h>
#include "dev.h"
#include "opt.h"
#include "adf.h"

#define TRACK_SIZE		(0x1900 * 2 + 0x440)
#define NUM_TRACKS		80

static void print_progress(int cyl, int head);

int main(int argc, char **argv)
{
	int i, j, num_tries, res, status = 1;
	unsigned char buf[TRACK_SIZE];

	if(init_options(argc, argv) == -1) {
		return 1;
	}

	if(init_device(opt.devfile, opt.baudrate) == -1) {
		return 1;
	}

	if(adf_open(opt.fname) == -1) {
		shutdown_device();
		return 1;
	}

	begin_read();
	for(i=0; i<NUM_TRACKS; i++) {
		move_head(i);
		for(j=0; j<2; j++) {
			select_head(j);

			num_tries = opt.retries;
			while((res = read_track(buf)) == -1 && num_tries-- > 0);
			if(res == -1) {
				fprintf(stderr, "failed to read track %d side %d\n", i, j);
				goto done;
			}

			if(adf_write_track(buf) == -1) {
				fprintf(stderr, "failed to write track %d side %d to ADF image\n", i, j);
				goto done;
			}
			if(opt.verbose) {
				print_progress(i, j);
			}
		}
	}
	putchar('\n');
	status = 0;

done:
	end_access();
	adf_close();
	shutdown_device();
	if(status != 0) {
		remove(opt.fname);
	}
	return status;
}

static void print_progress(int cyl, int head)
{
	int i, p, count;

	p = ((cyl << 1) | head) * 100 / ((NUM_TRACKS - 1) * 2);
	count = p / 2;

	printf("Reading (C:%02d H:%d) [", cyl, head);

	for(i=0; i<50; i++) {
		if(i < count || count == 50) {
			putchar('=');
		} else if(i == count) {
			putchar('>');
		} else {
			putchar(' ');
		}
	}

	printf("] %d%%  \r", p);
	fflush(stdout);
}
