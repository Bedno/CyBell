rem ffmpeg:  -acodec pcm_s8  and  -f s8  outputs signed 8bit RAW (no header).  -ac 1 forces mono.  Freq must be integer divisor of 80M and 100.
ffmpeg -i %1.wav -acodec pcm_s8 -f s8 -ac 1 -ar 12800 -y %1.raw
rem xxd:  byte to hex coverter
xxd -i %1.raw >%1.hex
rem Format .h adding altered code header.
rem Formerly: const signed char PROGMEM wav_%1[] = {
echo const uint8_t PROGMEM wav_%1[] = { >%1.h
tail +2 %1.hex >>%1.h
tail -1 %1.h >>sizes.lst
pause
