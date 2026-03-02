reduced logging , if uart messages drops
(as i2c is lower so reduced delay/ increase sampling can overwhelm the CPU)
or 
CONFIG_LOG_BUFFER_SIZE=1024  (default, change it to 2KB)