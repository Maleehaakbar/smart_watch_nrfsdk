
SMP -> simple management protocol in mcumgr
SMP ble -> security manager protocol in ble stack
There are 2 different security layers , bonding and pairing at device level
and to encrypt specific service, we enable, encrypt ,authentic in that APIs

Incnreae the MTU size to send more data in single packet, so data can be send in fewer packets
as if data is send in chunks, then with each pacet , 3 byte header will be send
that increas overhead

1. Key signing mcuboot:

signing with keys-> mcuboot contains public key , and developer sign with corresponding private key 
and then mcuboot verifies the key.

to generate key with imgtool.py , open nrf terminal and run following command
python3 /home/mod/ncs/v3.3.1/bootloader/mcuboot/scripts/imgtool.py keygen -k my_private_key.pem -t ecdsa-p256
SB_CONFIG_BOOT_SIGNATURE_KEY_FILE  //give the path of key file

this key is propagated to the MCUboot image as CONFIG_BOOT_SIGNATURE_KEY_FILE.
During the MCUboot build, imgtool getpub is run on that private key file.
 It extracts the public half and generates a C source file (keys.c) containing it as a byte array.
That file is compiled into the bootloader, so MCUboot has the public key embedded in its binary.
The application image is signed by imgtool sign using the same private key, via CONFIG_MCUBOOT_SIGNATURE_KEY_FILE.
At boot, MCUboot verifies the image signature against the embedded public key.

So the single private key file does both jobs: 
it signs the app, and the bootloader build extracts the matching public key from it.

2. MCUboot and mcumgr

   mcuboot only implements the small part of mcumgr needed for serial recovery over uart or cdc acm.
   full” MCUmgr (file system, shell, stats, logs, etc.) lives in the application, not the bootloader

   the reason is to keep bootloade size small and ota over ble and udp etc is done with application.
   The running application includes the MCUmgr SMP server over BLE.
   So MCUboot handles verification, swap, and rollback, while the application handles transport

   SMP Simple Management Protocol
   SMP is an application layer protocol, see sample SMP for details

3.   MCUmgr
   mcumgr layout contains
   command handler, mgmt, transfer encoding,transport

   command handler(img_mgmt, fs_mgmt , os_mgmt etc)
   transfer encoding (SMP)
   transport(BLE,Shell)

   command def -> header, CBOR key-value map

   mcumgr client -> mobile or cli
   mcumgr server -> application/bootloader with mcumgr service

   differet lib/client to use mcumgr on device -> Auterm, smp client, mcumgr client etc

   4. CBOR
   CBOR Encoder: Takes a program's data structure (such as numbers, strings, or arrays) 
   and serializes it into a compact binary byte stream

   Because it is binary rather than plain text, CBOR messages are much smaller and faster to transmit,
    used in (IoT) , BLE etc

    FOr ble ota, group 0(OS group , img group) mainly used

   4. SDU (Service Data Unit)
   the raw packet of data passed b/w an upper-layer application or protocol and the L2CAP layer
   The L2CAP MTU defines the maximum size (in octets) of an SDU that an upper-layer entity can receive on a specific channel

   5. L2CAP
   The L2CAP layer sits on top of the HCI layer and transfers data between the upper layers of the host (GAP, GATT, application) 
   and the lower layer protocol stack.
   ATT PDU is accomodated in L2CAP SDU and then L2CAP SDU is accomadoted in HCI ACL data packet
    
   L2CAP terminologies:

   (a)channel identifier (CID)

Phone                              Sensor
 ATT   <--------channel 1-------->  ATT
 SMP   <--------channel 2-------->  SMP
 App   <--------channel 3-------->  App (LE CoC)
        \_______ one radio link _______/

   Each end of a channel is identified by a CID (Channel Identifier), a number. For example, the ATT channel always uses CID 0x0004.
   It’s like a house number: it tells L2CAP which upper layer should receive the data.

  (b)SDU (Service Data Unit)

The SDU is the actual data the upper layer wants to send. L2CAP doesn’t care what’s inside; it just carries it.
Example: your app wants to send the sensor reading "temp=25.4" (9 bytes). Those 9 bytes are the SDU.

   (c)PDU
   PDU = header + SDU.
   If the SDU is too big for one radio packet,
   L2CAP can split it into multiple PDUs (segmentation) and the receiver puts it back together (reassembly)

   (d)MTU
   The MTU is the biggest SDU a receiver is willing to accept on that channel
Example : 

Putting it all together

Say the MTU is 247 and you send a 100-byte message:

Upper layer gives:   SDU = 100 bytes          (≤ 247, so allowed)
L2CAP wraps it:      PDU = 4 header + 100 = 104 bytes
Sent on channel:     CID = 0x0004 (ATT)
Receiver strips the header, hands the 100-byte SDU up

If you tried to send a 300-byte SDU, it would be rejected, because it exceeds the MTU of 247.

additional overhead:
BLE packet structure:
Preamble: 2 byte
Access Address: 4 bytes
LL: 2 bytes   
MTU : 247 bytes (actual data)
CRC  : 3 bytes

Total 258 bytes on-air

6. Swap using move algorithm is used


7. USB CDC DFU
//minicom access dvttyascm1
//CDC ACM provided by USB stack emulates serial lines/UART over USB

//design pattern: instead of using global variables, use a usb struct , init it with all information like gpio , timer etc
   install cargo,rustup and mcumgr cli

   CONFIG_MCUMGR_GRP_OS_MCUMGR_PARAMS=y  //add in prj.conf
   mcumgr commands
   cargo install mcumgrctl --no-default-features
   mcumgrctl --usb-serial
   mcumgrctl --usb-serial "2fe3:.*"    
   mcumgrctl -u . firmware update -b mcuboot zephyr.signed.bin  //upload firmware to slot 1
   
   mcumgrctl -u . image get-state

   mcumgrctl -u . image set-state hash id  //make pending true , active=false  and confirm = false
   reset baord // make active = true
    mcumgrctl -u . image get-state
    mcumgrctl -u . image set-state --confirm //currently active image gets confirmed
    reset board now to check again // now confirm and active both are true
    
    
    to avoid conflicts with terminal, temporarily comment out cargo path from shell env, edit bashrc
