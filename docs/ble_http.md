to connect with gadget bridge app

Go to Settings → Discover and pair options.
Enable Discover unsupported devices 
Go back to the main screen and press the + (plus) button.
GadgetBridge will scan. You should see a device 
Long press on it.
 select Bangle.js as the device type 


 for gps data , also enable the "Send GPS data" option and allow internet access in the settings of the watch device when nrf device is shown in app.
  enabled the global permissions of app, also enable specific options for device /watch in that device settings
 update gps data interval in app 


  difference in received response of gps data and http data

  gps -> receives a JSON object containing GPS values directly
  http -> receives a JSON object containing an HTTP response body encoded as a string inside another JSON object.
          a json object from gadget app is received that contain http  id, response etc 
            now further reposne is a string that further contains data as json


when using callback in application, decide what you will provide to callback so the user/application can ,infer info from it, (data , statuscode etc)
//register the user callback means store the function address in function pointer , just a different name for same concept
//and then using function pointer and arguments , we call the function/callback
//when some event occur (ble data received)so call the user callback function if user want notification in app


TODO: check how bt_rcv_sb or ble thread and main run side by side in zpehyr rtos


dynamic allocation
use heap for temporary buffers, deallocate them after use

parsing
when send and recv data , parsing strings, json , js strings is done through decoding , escape seqences, null terminator,
double quote marks, etc are decoded....so parse each and every format .

http over ble integration
send http request with request ID over ble NUS service
register a callback that execute when http response received
implement time out handler that execute when timeout occurs
receive the ble data from app and search for GB 
if gadget bridge msg receive then check if json received is gps/http/ etc by searching string
if http reposnse receive, extract response, parse and decode it into utf-8 bytes
pass the parsed resposne to callback


global , local variables and passing pointers as arguments to infer data
for buffers global array used  
local variables are used mostly and pointer is passed an argument to fetch the value


Hexadecimal numbers and characters

hex numbers:Used when doing math, base conversions, or bit-shifting (e.g., each hex digit = 4 bits)
hex characters : Used when processing strings, text, or formatting codes like HTML colors (#FF0000 uses 6 hex characters)

unicode 
A code point is a unique numerical value assigned to represent an abstract character, symbol, or control function in the Unicode 



TODOs: UTF-8 encodind decoding , from hex characters int 
proceesing of strings from http request and response


BLE attt and data link layer
MTU controls the size of the data packet at the ATT layer; 
Data Length controls how much of that packet can fit into a single radio-level Link Layer packet.

To send an entire 247-byte ATT packet in one LL packet, you need 251 bytes of LL payload.
But if your LL Data Length is still 27 bytes, the BLE stack fragments that mtu larger packet into multiple LL packets.
imp : A connection event can contain multiple LL packets, depending on available time, connection parameters, and controller scheduling.