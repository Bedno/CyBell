rem Create wav file of single word using text to speech.
set FNAME=%1
if not "%FNAME%" == "naught" goto NOTNAUGHT
set FNAME=0
:NOTNAUGHT
echo %FNAME%
speak -w %FNAME%.wav -z -a 200 -v en-us+f2 -s 200 %1
rem ffmpeg:  -acodec pcm_s8  and  -f s8  outputs signed 8bit RAW (no header).  -ac 1 forces mono.
ffmpeg -i %FNAME%.wav -acodec pcm_s8 -f s8 -ac 1 -ar 12800 -y %FNAME%.raw
rem xxd:  byte to hex coverter
xxd -i %FNAME%.raw >%FNAME%.hex
rem Format .h adding altered code header.
echo const uint8_t PROGMEM wav_%FNAME%[] = { >%FNAME%.h
tail +2 %FNAME%.hex >>%FNAME%.h

:out
