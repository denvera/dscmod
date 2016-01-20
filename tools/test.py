#!/usr/bin/env python

import io
import sys

f = io.open("/dev/dsc_txt", 'r')

#f.write(u'\x05\x0f\x0f\x1c')
#f.flush()
#f.close()

while True:
    try:
        d = f.readline()        
        #print d
        #line = "".join(d.split('\n'))
        bytes_all = d.split(' ')
        if bytes_all[0] == 'C':
            continue

        bytes = bytes_all[:-1]
        cmd = int(bytes[0],2)
        crc = int(bytes[-1], 2)
        ccrc = 0
        for b in bytes[:-1]:
            if len(b) > 1:
                ccrc += int(b,2)
        ccrc &= 0xff 
        if cmd != 0x05 and cmd != 0x11:
            print  "{} [{}|{}] - {}".format(["{:02X}".format(int(b,2)) for b in bytes_all], hex(ccrc), hex(crc), (ccrc == crc) and "OK" or "BAD")
        elif cmd == 0x11:
            print  "{}".format(["{:02X}".format(int(b,2)) for b in bytes_all])
        sys.stdout.flush()
    except KeyboardInterrupt, e:
        break
    except Exception, e:
        print "Exception: {}".format(str(e))
