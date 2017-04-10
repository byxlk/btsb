@echo off
del /a /f /q Objects\*.crf
del /a /f /q Objects\*.d
del /a /f /q Objects\*.o

del /a /f /q Listings\*.i
del /a /f /q Listings\*.lst
del /a /f /q Listings\*.txt

@echo on
copy Objects\btsbapp.hex ..\..\output_btsbapp.hex
dir ..\..\output_btsbapp.hex
copy Objects\btsbapp.bin ..\..\output_btsbapp.bin
dir ..\..\output_btsbapp.bin