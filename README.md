## DSC KeyBus kernel module
Creates /dev/dsc_txt and /dev/dsc_bin device nodes which can be used to read and write from the KeyBus of DSC alarm systems. 
To test: ```cat /dev/dsc_txt```

If you see all 1's repeating, you likely have CLK and DATA swapped, or DATA is disconnected and floating high.

Simply ```insmod dscmod.ko``` to get started once setting the GPIO pins as below.

The binary (/dev/dsc_bin) dev node just emits length prefixed messages as a byte stream rather than new line terminated text.

[dsc-node](https://github.com/denvera/dsc-node) provides a front-end to communicate with DSC alarm systems.

Tested on Raspberry Pi (2) and pcDuino (sunxi/sun4i). It's recommended that the Linux PREEMPT_RT patches be applied when using single core Raspberry Pis. Should work fine on NextThingCo's C.H.I.P too.

KeyBus runs at 12V, so ensure that a level shifter is used. Any level shifter should work, I've used:

* https://www.sparkfun.com/products/12009
* http://www.aliexpress.com/item/1Pcs-5V-to-3-3V-IIC-I2C-Logic-Level-Converter-Bi-Directional-Module-for-Arduino/32307221398.html

Set the GPIO pins used by modifying the keybus gpio struct, replacing 7 and 8 below with whatever GPIO pins correspond to your connections to the KeyBus.

```static struct gpio keybus[] = {
        { 7, GPIOF_IN, "DSC CLK" },    // KeyBus Clock (Yellow)
        { 8, GPIOF_IN, "DSC DATA" },   // KeyBus Data (Green)
};```


Example data:

```denver@pcDuino:~$ sudo cat /dev/dsc_txt
00000101 0 10010001 00000001 00010000 11000111 00010000 11000111 00010000 11000111 1    // 0x05 Status messages
00000101 0 10010001 00000001 00010000 11000111 00010000 11000111 00010000 11000111 1
...
00000101 0 10010001 00000001 00010000 11000111 00010000 11000111 00010000 11000111 1
00000101 0 10010001 00000001 00010000 11000111 00010000 11000111 00010000 11000111 1
00010001 0 10101010 10101010 10101010 10101010 10101010 10101010 10101010 10                 // 0x11 Supervision Enquiry
00000101 0 10010001 00000001 00010000 11000111 00010000 11000111 00010000 11000111 1
...
00000101 0 10010001 00000001 00010000 11000111 00010000 11000111 00010000 11000111 1
```

The test.py script parses the above and prints the messages in hex, along with [rcv'd checksum | calculated checksum] - OK if these match, or BAD if they don't.
```denver@pcDuino:~/dscmod$ sudo python tools/test.py
['27', '00', '91', '01', '10', 'C7', '01', '91', '00'] [0x91|0x91] - OK
['27', '00', '90', '03', '10', 'C7', '00', '91', '00'] [0x91|0x91] - OK
['E6', '00', '2C', '40', '00', '00', '00', '00', '52', '00'] [0x52|0x52] - OK
['27', '00', '91', '01', '10', 'C7', '01', '91', '00'] [0x91|0x91] - OK
```

This project was largely based off the great work of the folks at http://www.avrfreaks.net/forum/dsc-keybus-protocol - Particularly the posts by 'mrboard'.

The KeyBus can easily be connected to GPIO pins as below:
![KeyBus to Raspberry Pi](hardware/keybus_dscmod.png?raw=true "KeyBus to GPIO")