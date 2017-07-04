@echo off
del /a /f /q Objects\*.crf
del /a /f /q Objects\*.d
del /a /f /q Objects\*.o

del /a /f /q Listings\*.i
del /a /f /q Listings\*.lst
del /a /f /q Listings\*.txt

@echo on
copy Objects\btsb.hex ..\output_btsb_205.hex
dir ..\output_btsb_205.hex