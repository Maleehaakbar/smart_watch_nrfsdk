reduced logging , if uart messages drops
(as i2c is lower so reduced delay/ increase sampling can overwhelm the CPU)
or 
CONFIG_LOG_BUFFER_SIZE=1024  (default, change it to 2KB)


manage the log module,so all logs displays on or off with prj.conf
lvgl mem monitor implement first in native sim using instructions of lvgl documentation
track the size of main.c/ main stack using memory report