#line 1 "..\\User\\emWinTask\\MainTask.c"














 
#line 1 "..\\User\\emWinTask\\MainTask.h"













 




#line 1 "..\\STemWin\\inc\\GUI.h"
































 


















 
  



#line 1 "..\\STemWin\\inc\\GUI_ConfDefaults.h"




































 


















 
  



#line 1 "..\\STemWin\\Config\\GUIConf.h"
































 


















 
 






 





 









 







 





 





#line 62 "..\\STemWin\\inc\\GUI_ConfDefaults.h"

#line 70 "..\\STemWin\\inc\\GUI_ConfDefaults.h"



#line 79 "..\\STemWin\\inc\\GUI_ConfDefaults.h"






 







 







 







 
















































 
#line 166 "..\\STemWin\\inc\\GUI_ConfDefaults.h"

 





 











 




 






 
#line 58 "..\\STemWin\\inc\\GUI.h"
#line 1 "..\\STemWin\\inc\\GUI_Type.h"




































 


















 
  



#line 1 "..\\STemWin\\inc\\LCD.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\LCD.h"
#line 1 "..\\STemWin\\inc\\Global.h"
































 


















 
  








 
#line 81 "..\\STemWin\\inc\\Global.h"



 
#line 59 "..\\STemWin\\inc\\LCD.h"















 












 








 







 








 
typedef int LCD_DRAWMODE;
typedef unsigned long LCD_COLOR;




 
typedef struct { signed short x,y; } GUI_POINT;
typedef struct { signed short x0,y0,x1,y1; } LCD_RECT;

typedef struct {
  int              NumEntries;
  char             HasTrans;
  const LCD_COLOR * pPalEntries;
} LCD_LOGPALETTE;

 
typedef struct {
  int x,y;
  unsigned char KeyStat;
} LCD_tMouseState;

typedef struct {
  int               NumEntries;
  const LCD_COLOR * pPalEntries;
} LCD_PHYSPALETTE;




 
typedef LCD_COLOR      tLCDDEV_Index2Color  (unsigned Index);
typedef unsigned int   tLCDDEV_Color2Index  (LCD_COLOR Color);
typedef unsigned int   tLCDDEV_GetIndexMask (void);

typedef void tLCDDEV_Index2ColorBulk(void * pIndex, LCD_COLOR * pColor, unsigned long NumItems, unsigned char SizeOfIndex);
typedef void tLCDDEV_Color2IndexBulk(LCD_COLOR * pColor, void * pIndex, unsigned long NumItems, unsigned char SizeOfIndex);




 
typedef struct {
  tLCDDEV_Color2Index  * pfColor2Index;
  tLCDDEV_Index2Color  * pfIndex2Color;
  tLCDDEV_GetIndexMask * pfGetIndexMask;
  int NoAlpha;
  tLCDDEV_Color2IndexBulk * pfColor2IndexBulk;
  tLCDDEV_Index2ColorBulk * pfIndex2ColorBulk;
} LCD_API_COLOR_CONV;

extern const LCD_API_COLOR_CONV LCD_API_ColorConv_0;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_1;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_1_2;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_1_4;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_1_5;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_1_8;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_1_16;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_1_24;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_2;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_4;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_5;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_6;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_8;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_16;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_1616I;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_111;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_222;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_233;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_323;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_332;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_444_12;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_444_12_1;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_444_16;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_555;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_565;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_556;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_655;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_666;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_666_9;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_822216;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_84444;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_8666;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_8666_1;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_88666I;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_888;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_8888;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M111;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M1555I;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M222;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M233;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M323;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M332;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M4444I;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M444_12;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M444_12_1;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M444_16;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M555;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M565;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M556;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M655;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M666;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M666_9;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M8565;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M888;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M8888;
extern const LCD_API_COLOR_CONV LCD_API_ColorConv_M8888I;

#line 278 "..\\STemWin\\inc\\LCD.h"

void GUICC_M1555I_SetCustColorConv(tLCDDEV_Color2IndexBulk * pfColor2IndexBulk, tLCDDEV_Index2ColorBulk * pfIndex2ColorBulk);
void GUICC_M565_SetCustColorConv  (tLCDDEV_Color2IndexBulk * pfColor2IndexBulk, tLCDDEV_Index2ColorBulk * pfIndex2ColorBulk);
void GUICC_M4444I_SetCustColorConv(tLCDDEV_Color2IndexBulk * pfColor2IndexBulk, tLCDDEV_Index2ColorBulk * pfIndex2ColorBulk);
void GUICC_M888_SetCustColorConv  (tLCDDEV_Color2IndexBulk * pfColor2IndexBulk, tLCDDEV_Index2ColorBulk * pfIndex2ColorBulk);
void GUICC_M8888I_SetCustColorConv(tLCDDEV_Color2IndexBulk * pfColor2IndexBulk, tLCDDEV_Index2ColorBulk * pfIndex2ColorBulk);




 
#line 297 "..\\STemWin\\inc\\LCD.h"












 
typedef void         tLCDDEV_DrawPixel    (int x, int y);
typedef void         tLCDDEV_DrawHLine    (int x0, int y0,  int x1);
typedef void         tLCDDEV_DrawVLine    (int x , int y0,  int y1);
typedef void         tLCDDEV_FillRect     (int x0, int y0, int x1, int y1);
typedef unsigned int tLCDDEV_GetPixelIndex(int x, int y);
typedef void         tLCDDEV_SetPixelIndex(int x, int y, int ColorIndex);
typedef void         tLCDDEV_XorPixel     (int x, int y);
typedef void         tLCDDEV_FillPolygon  (const GUI_POINT * pPoints, int NumPoints, int x0, int y0);
typedef void         tLCDDEV_FillPolygonAA(const GUI_POINT * pPoints, int NumPoints, int x0, int y0);
typedef void         tLCDDEV_GetRect      (LCD_RECT * pRect);
typedef int          tLCDDEV_Init         (void);
typedef void         tLCDDEV_On           (void);
typedef void         tLCDDEV_Off          (void);
typedef void         tLCDDEV_SetLUTEntry  (unsigned char Pos, LCD_COLOR color);
typedef void *       tLCDDEV_GetDevFunc   (int Index);
typedef signed long          tLCDDEV_GetDevProp   (int Index);
typedef void         tLCDDEV_SetOrg       (int x, int y);




 
typedef struct GUI_DEVICE     GUI_DEVICE;
typedef struct GUI_DEVICE_API GUI_DEVICE_API;

typedef void tLCDDEV_DrawBitmap   (int x0, int y0, int xsize, int ysize,
                       int BitsPerPixel, int BytesPerLine,
                       const unsigned char * pData, int Diff,
                       const void * pTrans);    


















 

#line 374 "..\\STemWin\\inc\\LCD.h"

int LCD_GetXSizeMax(void);
int LCD_GetYSizeMax(void);
int LCD_GetVXSizeMax(void);
int LCD_GetVYSizeMax(void);
int LCD_GetBitsPerPixelMax(void);
void LCD_SetDisplaySize(int xSizeDisplay, int ySizeDisplay);
int LCD_GetXSizeDisplay(void);
int LCD_GetYSizeDisplay(void);

int LCD_GetXSizeEx          (int LayerIndex);
int LCD_GetYSizeEx          (int LayerIndex);
int LCD_GetVXSizeEx         (int LayerIndex);
int LCD_GetVYSizeEx         (int LayerIndex);
int LCD_GetBitsPerPixelEx   (int LayerIndex);
unsigned long LCD_GetNumColorsEx      (int LayerIndex);
int LCD_GetXMagEx           (int LayerIndex);
int LCD_GetYMagEx           (int LayerIndex);
int LCD_GetMirrorXEx        (int LayerIndex);
int LCD_GetMirrorYEx        (int LayerIndex);
int LCD_GetSwapXYEx         (int LayerIndex);
int LCD_GetReversLUTEx      (int LayerIndex);
int LCD_GetPhysColorsInRAMEx(int LayerIndex);

int LCD_GetXSize            (void);
int LCD_GetYSize            (void);
int LCD_GetVXSize           (void);
int LCD_GetVYSize           (void);
int LCD_GetBitsPerPixel     (void);
unsigned long LCD_GetNumColors        (void);
int LCD_GetXMag             (void);
int LCD_GetYMag             (void);
int LCD_GetMirrorX          (void);
int LCD_GetMirrorY          (void);
int LCD_GetSwapXY           (void);
int LCD_GetReversLUT        (void);
int LCD_GetPhysColorsInRAM  (void);

signed long LCD__GetBPP      (unsigned long IndexMask);
signed long LCD__GetBPPDevice(unsigned long IndexMask);

tLCDDEV_Index2Color * LCD_GetpfIndex2ColorEx(int LayerIndex);
tLCDDEV_Color2Index * LCD_GetpfColor2IndexEx(int LayerIndex);

tLCDDEV_Color2Index * LCD_GetpfColor2Index(void);

int LCD_GetNumLayers(void);

LCD_COLOR * LCD_GetPalette  (void);
LCD_COLOR * LCD_GetPaletteEx(int LayerIndex);

void (* LCD_GetDevFunc(int LayerIndex, int Item))(void);




 
                                        
#line 456 "..\\STemWin\\inc\\LCD.h"
                                        
#line 467 "..\\STemWin\\inc\\LCD.h"




 
                                           





 
                                        






 
typedef struct {
  void * pVRAM;
} LCD_X_SETVRAMADDR_INFO;

typedef struct {
  int xPos, yPos;
} LCD_X_SETORG_INFO;

typedef struct {
  LCD_COLOR Color;
  unsigned char Pos;
} LCD_X_SETLUTENTRY_INFO;

typedef struct {
  int xSize, ySize;
} LCD_X_SETSIZE_INFO;

typedef struct {
  int xPos, yPos;
  int xLen, yLen;
  int BytesPerPixel;
  unsigned long Off;
} LCD_X_SETPOS_INFO;

typedef struct {
  int Alpha;
} LCD_X_SETALPHA_INFO;

typedef struct {
  int OnOff;
} LCD_X_SETVIS_INFO;

typedef struct {
  int AlphaMode;
} LCD_X_SETALPHAMODE_INFO;

typedef struct {
  int ChromaMode;
} LCD_X_SETCHROMAMODE_INFO;

typedef struct {
  LCD_COLOR ChromaMin;
  LCD_COLOR ChromaMax;
} LCD_X_SETCHROMA_INFO;

typedef struct {
  int Index;
} LCD_X_SHOWBUFFER_INFO;




 
#line 554 "..\\STemWin\\inc\\LCD.h"

int  LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void * pData);
void LCD_X_Config(void);




 
int LCD_SetAlphaEx     (int LayerIndex, int Alpha);
int LCD_SetPosEx       (int LayerIndex, int xPos, int yPos);
int LCD_SetSizeEx      (int LayerIndex, int xSize, int ySize);
int LCD_SetVisEx       (int LayerIndex, int OnOff);
int LCD_SetVRAMAddrEx  (int LayerIndex, void * pVRAM);
int LCD_SetVSizeEx     (int LayerIndex, int xSize, int ySize);
int LCD_SetAlphaModeEx (int LayerIndex, int AlphaMode);
int LCD_SetChromaModeEx(int LayerIndex, int ChromaMode);
int LCD_SetChromaEx    (int LayerIndex, LCD_COLOR ChromaMin, LCD_COLOR ChromaMax);

int LCD_SetAlpha     (int Alpha);
int LCD_SetVRAMAddr  (void * pVRAM);
int LCD_SetVSize     (int xSize, int ySize);
int LCD_SetSize      (int xSize, int ySize);
int LCD_SetVis       (int OnOff);
int LCD_SetPos       (int xPos, int yPos);
int LCD_SetAlphaMode (int AlphaMode);
int LCD_SetChromaMode(int ChromaMode);
int LCD_SetChroma    (LCD_COLOR ChromaMin, LCD_COLOR ChromaMax);
int LCD_SetLUTEntry  (unsigned char Pos, LCD_COLOR Color);
int LCD_SetDevFunc   (int LayerIndex, int IdFunc, void (* pDriverFunc)(void));




 
int LCD_GetPosEx(int LayerIndex, int * pxPos, int * pyPos);

int LCD_GetPos  (int * pxPos, int * pyPos);




 
int LCD_Refresh  (void);
int LCD_RefreshEx(int LayerIndex);




 
typedef struct {
  int  (* pfStart)   (int x0, int y0, int x1, int y1);
  void (* pfSetPixel)(int PixelIndex);
  void (* pfNextLine)(void);
  void (* pfEnd)     (void);
} LCD_API_NEXT_PIXEL;

LCD_API_NEXT_PIXEL * LCD_GetNextPixelAPI(void);




 
typedef void tLCD_HL_DrawHLine    (int x0, int y0,  int x1);
typedef void tLCD_HL_DrawPixel    (int x0, int y0);

typedef struct {
  tLCD_HL_DrawHLine * pfDrawHLine;
  tLCD_HL_DrawPixel * pfDrawPixel;
} tLCD_HL_APIList;

void LCD_DrawHLine(int x0, int y0,  int x1);
void LCD_DrawPixel(int x0, int y0);
void LCD_DrawVLine(int x,  int y0,  int y1);





 
void LCD_SetClipRectEx(const LCD_RECT * pRect);
void LCD_SetClipRectMax(void);

 
signed long  LCD_GetDevCap  (int Index);
signed long  LCD_GetDevCapEx(int LayerIndex, int Index);

 
int emWin_LCD_Init(void);
int LCD_InitColors(void);

void LCD_SetBkColor   (LCD_COLOR Color);  
void LCD_SetColor     (LCD_COLOR Color);  
void LCD_SetPixelIndex(int x, int y, int ColorIndex);

 
void LCD_InitLUT(void);
int  LCD_SetLUTEntryEx(int LayerIndex, unsigned char Pos, LCD_COLOR Color);
void LCD_SetLUTEx(int LayerIndex, const LCD_PHYSPALETTE * pPalette);
void LCD_SetLUT  (const LCD_PHYSPALETTE * pPalette);

LCD_DRAWMODE LCD_SetDrawMode  (LCD_DRAWMODE dm);
void LCD_SetColorIndex(unsigned PixelIndex);
void LCD_SetBkColorIndex(unsigned PixelIndex);
void LCD_FillRect(int x0, int y0, int x1, int y1);
typedef void tLCD_SetPixelAA(int x, int y, unsigned char Intens);

void LCD_SetPixelAA4_Trans  (int x, int y, unsigned char Intens);
void LCD_SetPixelAA4_NoTrans(int x, int y, unsigned char Intens);

void LCD_SetPixelAA8_Trans  (int x, int y, unsigned char Intens);
void LCD_SetPixelAA8_NoTrans(int x, int y, unsigned char Intens);

LCD_COLOR    LCD_AA_MixColors16 (LCD_COLOR Color, LCD_COLOR BkColor, unsigned char Intens);
LCD_COLOR    LCD_AA_MixColors256(LCD_COLOR Color, LCD_COLOR BkColor, unsigned char Intens);
LCD_COLOR    LCD_MixColors256   (LCD_COLOR Color, LCD_COLOR BkColor, unsigned Intens);
LCD_COLOR    LCD_GetPixelColor(int x, int y);      
unsigned int LCD_GetPixelIndex(int x, int y);
int          LCD_GetBkColorIndex (void);
int          LCD_GetColorIndex (void);



unsigned long          LCD_AA_SetAndMask(unsigned long AndMask);


 
int LCD_SetMaxNumColors(unsigned MaxNumColors);




 


typedef void tLCD_DrawBitmap(int x0, int y0, int xsize, int ysize,
                             int xMul, int yMul, int BitsPerPixel, int BytesPerLine,
                             const unsigned char * pPixel, const void * pTrans);
typedef void tRect2TextRect (LCD_RECT * pRect);

struct tLCD_APIList_struct {
  tLCD_DrawBitmap   * pfDrawBitmap;
  tRect2TextRect    * pfRect2TextRect;
  tRect2TextRect    * pfTransformRect;
};

typedef struct tLCD_APIList_struct tLCD_APIList;

extern tLCD_APIList LCD_APIListCCW;
extern tLCD_APIList LCD_APIListCW;
extern tLCD_APIList LCD_APIList180;







tLCD_SetPixelAA * LCD__GetPfSetPixel(int BitsPerPixel);






 
void LCD__SetPhysColor(unsigned char Pos, LCD_COLOR Color);




 




int LCD_ControlCache  (int Cmd);
int LCD_ControlCacheEx(int LayerIndex, int Cmd);




 
unsigned         LCD_Color2Index     (LCD_COLOR Color);
LCD_COLOR        LCD_Index2Color     (int Index);
LCD_COLOR        LCD_Index2ColorEx   (int i, unsigned LayerIndex);




 
unsigned char LCD_X_Read00(void);
unsigned char LCD_X_Read01(void);
void LCD_X_Write00 (unsigned char c);
void LCD_X_Write01 (unsigned char c);
void LCD_X_WriteM01(unsigned char * pData, int NumBytes);







 
#line 62 "..\\STemWin\\inc\\GUI_Type.h"
#line 63 "..\\STemWin\\inc\\GUI_Type.h"




 
typedef const char *  GUI_ConstString;

typedef LCD_COLOR       GUI_COLOR;
typedef LCD_LOGPALETTE  GUI_LOGPALETTE;
typedef LCD_DRAWMODE    GUI_DRAWMODE;
typedef LCD_RECT        GUI_RECT;

typedef struct {
  void      (* pfDraw)  (int x0,
                         int y0,
                         int xsize, 
                         int ysize, 
                         const unsigned char * pPixel, 
                         const LCD_LOGPALETTE * pLogPal, 
                         int xMag, 
                         int yMag);
  GUI_COLOR (* pfIndex2Color)(unsigned Index);
  void      (* pfDrawHW)(int x0,
                         int y0,
                         int xsize, 
                         int ysize, 
                         const unsigned char * pPixel, 
                         const LCD_LOGPALETTE * pLogPal, 
                         int xMag, 
                         int yMag);
  const LCD_API_COLOR_CONV * pColorConvAPI;
} GUI_BITMAP_METHODS;

typedef struct {
  unsigned short XSize;
  unsigned short YSize;
  unsigned short BytesPerLine;
  unsigned short BitsPerPixel;
  const unsigned char * pData;
  const GUI_LOGPALETTE * pPal;
  const GUI_BITMAP_METHODS * pMethods;
} GUI_BITMAP;





 
typedef struct {
  unsigned short ID;
  unsigned short Format;
  unsigned short XSize;
  unsigned short YSize;
  unsigned short BytesPerLine;
  unsigned short BitsPerPixel;
  unsigned short NumColors;
  unsigned short HasTrans;
} GUI_BITMAP_STREAM;

typedef struct {
  int    Cmd;
  unsigned long    v;
  void * p;
} GUI_BITMAPSTREAM_PARAM;

typedef struct {
  int XSize;
  int YSize;
  int BitsPerPixel;
  int NumColors;
  int HasTrans;
} GUI_BITMAPSTREAM_INFO;

typedef void * (* GUI_BITMAPSTREAM_CALLBACK)(GUI_BITMAPSTREAM_PARAM * pParam);

typedef struct {
  int x,y;
  unsigned char  Pressed;
  unsigned char  Layer;
} GUI_PID_STATE;

typedef struct {
  int Key;
  int Pressed;
} GUI_KEY_STATE;

typedef struct {
  int xPos;
  int yPos;
  int xSize;
  int ySize;
  int Delay;
} GUI_GIF_IMAGE_INFO;

typedef struct {
  int xSize;
  int ySize;
  int NumImages;
} GUI_GIF_INFO;

typedef struct GUI_REGISTER_EXIT GUI_REGISTER_EXIT;

struct GUI_REGISTER_EXIT {
  void (* pfVoid)(void);
  GUI_REGISTER_EXIT * pNext;
};

typedef struct {
  void (* cbBegin)(void);
  void (* cbEnd)  (void);
} GUI_MULTIBUF_API;

typedef struct {
  void (* cbBeginEx)(int LayerIndex);
  void (* cbEndEx)  (int LayerIndex);
} GUI_MULTIBUF_API_EX;




 



 
typedef struct {
  signed short c0;
  signed short c1;
} GUI_FONT_TRANSLIST;

typedef struct {
  unsigned short FirstChar;
  unsigned short LastChar;
  const GUI_FONT_TRANSLIST * pList;
} GUI_FONT_TRANSINFO;

typedef struct {
  unsigned char XSize;
  unsigned char XDist;
  unsigned char BytesPerLine;
  const unsigned char * pData;
} GUI_CHARINFO;

typedef struct {
  unsigned char XSize;
  unsigned char YSize;
  signed char XPos;
  signed char YPos;
  unsigned char XDist;
  const unsigned char * pData;
} GUI_CHARINFO_EXT;

typedef struct GUI_FONT_PROP {
  unsigned short First;                                   
  unsigned short Last;                                    
  const GUI_CHARINFO         * paCharInfo;      
  const struct GUI_FONT_PROP * pNext;           
} GUI_FONT_PROP;

typedef struct GUI_FONT_PROP_EXT {
  unsigned short First;                                   
  unsigned short Last;                                    
  const GUI_CHARINFO_EXT         * paCharInfo;  
  const struct GUI_FONT_PROP_EXT * pNext;       
} GUI_FONT_PROP_EXT;

typedef struct {
  const unsigned char      * pData;
  const unsigned char                 * pTransData;
  const GUI_FONT_TRANSINFO * pTrans;
  unsigned short                       FirstChar;
  unsigned short                       LastChar;
  unsigned char                         XSize;
  unsigned char                         XDist;
  unsigned char                         BytesPerLine;
} GUI_FONT_MONO;







 
typedef struct {
  unsigned short Flags;
  unsigned char Baseline;
  unsigned char LHeight;      
  unsigned char CHeight;      
} GUI_FONTINFO;

#line 260 "..\\STemWin\\inc\\GUI_Type.h"




 
typedef unsigned short  tGUI_GetCharCode   (const char * s);
typedef int  tGUI_GetCharSize   (const char * s);
typedef int  tGUI_CalcSizeOfChar(unsigned short Char);
typedef int  tGUI_Encode        (char * s, unsigned short Char);

typedef struct {
  tGUI_GetCharCode    * pfGetCharCode;
  tGUI_GetCharSize    * pfGetCharSize;
  tGUI_CalcSizeOfChar * pfCalcSizeOfChar;
  tGUI_Encode         * pfEncode;
} GUI_UC_ENC_APILIST;




 
typedef int  tGUI_GetLineDistX(const char * s, int Len);
typedef int  tGUI_GetLineLen  (const char * s, int MaxLen);
typedef void tGL_DispLine     (const char * s, int Len);

typedef struct {
  tGUI_GetLineDistX * pfGetLineDistX;
  tGUI_GetLineLen   * pfGetLineLen;
  tGL_DispLine      * pfDispLine;
} tGUI_ENC_APIList;

extern const tGUI_ENC_APIList GUI_ENC_APIList_SJIS;
extern const tGUI_ENC_APIList GUI_ENC_APIList_EXT;




 
typedef struct GUI_FONT GUI_FONT;

typedef void GUI_DISPCHAR    (unsigned short c);
typedef int  GUI_GETCHARDISTX(unsigned short c, int * pSizeX);
typedef void GUI_GETFONTINFO (const GUI_FONT * pFont, GUI_FONTINFO * pfi);
typedef char GUI_ISINFONT    (const GUI_FONT * pFont, unsigned short c);
typedef int  GUI_GETCHARINFO (unsigned short c, GUI_CHARINFO_EXT * pInfo);

#line 312 "..\\STemWin\\inc\\GUI_Type.h"





void GUIMONO_DispChar (unsigned short c); int GUIMONO_GetCharDistX(unsigned short c, int * pSizeX); void GUIMONO_GetFontInfo (const GUI_FONT * pFont, GUI_FONTINFO * pfi); char GUIMONO_IsInFont (const GUI_FONT * pFont, unsigned short c); int GUIMONO_GetCharInfo (unsigned short c, GUI_CHARINFO_EXT * pInfo);
void GUIPROP_DispChar (unsigned short c); int GUIPROP_GetCharDistX(unsigned short c, int * pSizeX); void GUIPROP_GetFontInfo (const GUI_FONT * pFont, GUI_FONTINFO * pfi); char GUIPROP_IsInFont (const GUI_FONT * pFont, unsigned short c); int GUIPROP_GetCharInfo (unsigned short c, GUI_CHARINFO_EXT * pInfo);
void GUIPROP_EXT_DispChar (unsigned short c); int GUIPROP_EXT_GetCharDistX(unsigned short c, int * pSizeX); void GUIPROP_EXT_GetFontInfo (const GUI_FONT * pFont, GUI_FONTINFO * pfi); char GUIPROP_EXT_IsInFont (const GUI_FONT * pFont, unsigned short c); int GUIPROP_EXT_GetCharInfo (unsigned short c, GUI_CHARINFO_EXT * pInfo);
void GUIPROP_FRM_DispChar (unsigned short c); int GUIPROP_FRM_GetCharDistX(unsigned short c, int * pSizeX); void GUIPROP_FRM_GetFontInfo (const GUI_FONT * pFont, GUI_FONTINFO * pfi); char GUIPROP_FRM_IsInFont (const GUI_FONT * pFont, unsigned short c); int GUIPROP_FRM_GetCharInfo (unsigned short c, GUI_CHARINFO_EXT * pInfo);
void GUIPROPAA_DispChar (unsigned short c); int GUIPROPAA_GetCharDistX(unsigned short c, int * pSizeX); void GUIPROPAA_GetFontInfo (const GUI_FONT * pFont, GUI_FONTINFO * pfi); char GUIPROPAA_IsInFont (const GUI_FONT * pFont, unsigned short c); int GUIPROPAA_GetCharInfo (unsigned short c, GUI_CHARINFO_EXT * pInfo);
void GUIPROP_AA2_DispChar (unsigned short c); int GUIPROP_AA2_GetCharDistX(unsigned short c, int * pSizeX); void GUIPROP_AA2_GetFontInfo (const GUI_FONT * pFont, GUI_FONTINFO * pfi); char GUIPROP_AA2_IsInFont (const GUI_FONT * pFont, unsigned short c); int GUIPROP_AA2_GetCharInfo (unsigned short c, GUI_CHARINFO_EXT * pInfo);
void GUIPROP_AA2_EXT_DispChar (unsigned short c); int GUIPROP_AA2_EXT_GetCharDistX(unsigned short c, int * pSizeX); void GUIPROP_AA2_EXT_GetFontInfo (const GUI_FONT * pFont, GUI_FONTINFO * pfi); char GUIPROP_AA2_EXT_IsInFont (const GUI_FONT * pFont, unsigned short c); int GUIPROP_AA2_EXT_GetCharInfo (unsigned short c, GUI_CHARINFO_EXT * pInfo);
void GUIPROP_AA4_DispChar (unsigned short c); int GUIPROP_AA4_GetCharDistX(unsigned short c, int * pSizeX); void GUIPROP_AA4_GetFontInfo (const GUI_FONT * pFont, GUI_FONTINFO * pfi); char GUIPROP_AA4_IsInFont (const GUI_FONT * pFont, unsigned short c); int GUIPROP_AA4_GetCharInfo (unsigned short c, GUI_CHARINFO_EXT * pInfo);
void GUIPROP_AA4_EXT_DispChar (unsigned short c); int GUIPROP_AA4_EXT_GetCharDistX(unsigned short c, int * pSizeX); void GUIPROP_AA4_EXT_GetFontInfo (const GUI_FONT * pFont, GUI_FONTINFO * pfi); char GUIPROP_AA4_EXT_IsInFont (const GUI_FONT * pFont, unsigned short c); int GUIPROP_AA4_EXT_GetCharInfo (unsigned short c, GUI_CHARINFO_EXT * pInfo);

 
#line 335 "..\\STemWin\\inc\\GUI_Type.h"

 
#line 344 "..\\STemWin\\inc\\GUI_Type.h"

 
#line 353 "..\\STemWin\\inc\\GUI_Type.h"

 
#line 362 "..\\STemWin\\inc\\GUI_Type.h"

 
#line 371 "..\\STemWin\\inc\\GUI_Type.h"

 
#line 380 "..\\STemWin\\inc\\GUI_Type.h"

 
#line 389 "..\\STemWin\\inc\\GUI_Type.h"

 
#line 398 "..\\STemWin\\inc\\GUI_Type.h"

 
#line 407 "..\\STemWin\\inc\\GUI_Type.h"

 
#line 416 "..\\STemWin\\inc\\GUI_Type.h"

 
#line 425 "..\\STemWin\\inc\\GUI_Type.h"

 
#line 434 "..\\STemWin\\inc\\GUI_Type.h"





struct GUI_FONT {
  GUI_DISPCHAR     * pfDispChar; 
  GUI_GETCHARDISTX * pfGetCharDistX; 
  GUI_GETFONTINFO  * pfGetFontInfo; 
  GUI_ISINFONT     * pfIsInFont;
  GUI_GETCHARINFO  * pfGetCharInfo;
  const tGUI_ENC_APIList* pafEncode;
  unsigned char YSize;
  unsigned char YDist;
  unsigned char XMag;
  unsigned char YMag;
  union {
    const void              * pFontData;
    const GUI_FONT_MONO     * pMono;
    const GUI_FONT_PROP     * pProp;
    const GUI_FONT_PROP_EXT * pPropExt;
  } p;
  unsigned char Baseline;
  unsigned char LHeight;      
  unsigned char CHeight;      
};




 
typedef void GUI_CALLBACK_VOID_U8_P(unsigned char Data, void * p);




 
typedef struct {
  unsigned long ID;            
  unsigned short YSize;         
  unsigned short YDist;         
  unsigned short Baseline;      
  unsigned short LHeight;       
  unsigned short CHeight;       
  unsigned short NumAreas;      
} GUI_SI_FONT;

typedef struct {
  unsigned short First;         
  unsigned short Last;          
} GUI_SIF_CHAR_AREA;

typedef struct {
  unsigned short XSize;         
  unsigned short XDist;         
  unsigned short BytesPerLine;  
  unsigned short Dummy;
  unsigned long OffData;       
} GUI_SIF_CHARINFO;

typedef struct {
  unsigned short XSize;         
  unsigned short YSize;         
  signed short XOff;          
  signed short YOff;          
  unsigned short XDist;         
  unsigned short Dummy;
  unsigned long OffData;       
} GUI_SIF_CHARINFO_EXT;

typedef struct tGUI_SIF_APIList_struct {
  GUI_DISPCHAR          * pfDispChar;
  GUI_GETCHARDISTX      * pfGetCharDistX;
  GUI_GETFONTINFO       * pfGetFontInfo;
  GUI_ISINFONT          * pfIsInFont;
  GUI_GETCHARINFO       * pfGetCharInfo;
  const tGUI_ENC_APIList* pafEncode;
} tGUI_SIF_APIList;

#line 521 "..\\STemWin\\inc\\GUI_Type.h"




 
typedef int GUI_XBF_GET_DATA_FUNC(unsigned long Off, unsigned short NumBytes, void * pVoid, void * pBuffer);

typedef struct {
  unsigned short First;                          
  unsigned short Last;                           
  void * pVoid;                       
  GUI_XBF_GET_DATA_FUNC * pfGetData;  
} GUI_XBF_DATA;

typedef struct tGUI_XBF_APIList_struct {
  GUI_DISPCHAR          * pfDispChar;
  GUI_GETCHARDISTX      * pfGetCharDistX;
  GUI_GETFONTINFO       * pfGetFontInfo;
  GUI_ISINFONT          * pfIsInFont;
  GUI_GETCHARINFO       * pfGetCharInfo;
  const tGUI_ENC_APIList* pafEncode;
} tGUI_XBF_APIList;

#line 550 "..\\STemWin\\inc\\GUI_Type.h"




 
typedef struct {
  const void * pData;       
  unsigned long NumBytes;             
} GUI_TTF_DATA;

typedef struct {
  GUI_TTF_DATA * pTTF;      
  unsigned long aImageTypeBuffer[4];  
  int PixelHeight;         


 
  int FaceIndex;           

 
} GUI_TTF_CS;




 
typedef void (* GUI_SIGNAL_EVENT_FUNC)    (void);
typedef void (* GUI_WAIT_EVENT_FUNC)      (void);
typedef void (* GUI_WAIT_EVENT_TIMED_FUNC)(int Period);




 




typedef     signed long      GUI_HWIN;
typedef     signed long      GUI_HSPRITE;




 




typedef struct {
  signed long x;
  signed long y;
  unsigned long Id;
  unsigned short Flags;
} GUI_MTOUCH_INPUT;

typedef struct {
  int            LayerIndex;
  unsigned       NumPoints;
  int TimeStamp;
  signed long       hInput;
} GUI_MTOUCH_EVENT;




typedef struct {
  unsigned char  Layer;
  unsigned char  NumPoints;
  signed short ax[5];
  signed short ay[5];
  unsigned short aId[5];
  unsigned char  aFlags[5];
} GUI_MTOUCH_STATE;

typedef void (* T_GUI_MTOUCH_STOREEVENT)(GUI_MTOUCH_EVENT *, GUI_MTOUCH_INPUT * pInput);




 
typedef struct {
  
  
  
  void (* pfWrite8_A0)  (unsigned char Data);
  void (* pfWrite8_A1)  (unsigned char Data);
  void (* pfWriteM8_A0) (unsigned char * pData, int NumItems);
  void (* pfWriteM8_A1) (unsigned char * pData, int NumItems);
  unsigned char   (* pfRead8_A0)   (void);
  unsigned char   (* pfRead8_A1)   (void);
  void (* pfReadM8_A0)  (unsigned char * pData, int NumItems);
  void (* pfReadM8_A1)  (unsigned char * pData, int NumItems);
  
  
  
  void (* pfWrite16_A0) (unsigned short Data);
  void (* pfWrite16_A1) (unsigned short Data);
  void (* pfWriteM16_A0)(unsigned short * pData, int NumItems);
  void (* pfWriteM16_A1)(unsigned short * pData, int NumItems);
  unsigned short  (* pfRead16_A0)  (void);
  unsigned short  (* pfRead16_A1)  (void);
  void (* pfReadM16_A0) (unsigned short * pData, int NumItems);
  void (* pfReadM16_A1) (unsigned short * pData, int NumItems);
  
  
  
  void (* pfWrite32_A0) (unsigned long Data);
  void (* pfWrite32_A1) (unsigned long Data);
  void (* pfWriteM32_A0)(unsigned long * pData, int NumItems);
  void (* pfWriteM32_A1)(unsigned long * pData, int NumItems);
  unsigned long  (* pfRead32_A0)  (void);
  unsigned long  (* pfRead32_A1)  (void);
  void (* pfReadM32_A0) (unsigned long * pData, int NumItems);
  void (* pfReadM32_A1) (unsigned long * pData, int NumItems);
  
  
  
  void (* pfSetCS)      (unsigned char NotActive);
  
  
  
  void (* pfFlushBuffer)(void);
} GUI_PORT_API;




 
typedef int    (* GUI_tSend)  (const unsigned char * pData, int len, void * p);
typedef int    (* GUI_tRecv)  (      unsigned char * pData, int len, void * p);




 
typedef void * (* GUI_tMalloc)(unsigned int);
typedef void   (* GUI_tFree)  (void *);



 
#line 59 "..\\STemWin\\inc\\GUI.h"
#line 1 "..\\STemWin\\inc\\GUI_Version.h"
































 


















 
  







 
#line 60 "..\\STemWin\\inc\\GUI.h"








 









 








 
typedef struct GUI_CONTEXT GUI_CONTEXT;

#line 108 "..\\STemWin\\inc\\GUI.h"




 
struct GUI_DEVICE_API {
  
  
  
  int DeviceClassIndex;
  
  
  
  void     (* pfDrawBitmap   )(GUI_DEVICE *  pDevice,  int x0, int y0, int xsize, int ysize, int BitsPerPixel, int BytesPerLine, const unsigned char * pData, int Diff, const unsigned long * pTrans);
  void     (* pfDrawHLine    )(GUI_DEVICE *  pDevice,  int x0, int y0,  int x1);
  void     (* pfDrawVLine    )(GUI_DEVICE *  pDevice,  int x , int y0,  int y1);
  void     (* pfFillRect     )(GUI_DEVICE *  pDevice,  int x0, int y0, int x1, int y1);
  unsigned (* pfGetPixelIndex)(GUI_DEVICE *  pDevice,  int x, int y);
  void     (* pfSetPixelIndex)(GUI_DEVICE *  pDevice,  int x, int y, int ColorIndex);
  void     (* pfXorPixel     )(GUI_DEVICE *  pDevice,  int x, int y);
  
  
  
  void     (* pfSetOrg       )(GUI_DEVICE *  pDevice,  int x, int y);
  
  
  
  void   (*(* pfGetDevFunc)   (GUI_DEVICE ** ppDevice, int Index))(void);
  signed long      (* pfGetDevProp   )(GUI_DEVICE *  pDevice,  int Index);
  void    *(* pfGetDevData   )(GUI_DEVICE *  pDevice,  int Index);
  void     (* pfGetRect      )(GUI_DEVICE *  pDevice,  LCD_RECT * pRect);
};




 
typedef enum {
  DEVICE_CLASS_DRIVER = 0,
  DEVICE_CLASS_DRIVER_MODIFIER,   
  DEVICE_CLASS_VNC,
  DEVICE_CLASS_SPRITE,
  DEVICE_CLASS_MEMDEV,
  DEVICE_CLASS_ALPHA,
  DEVICE_CLASS_AUTOALPHA,
  DEVICE_CLASS_MEASDEV
} DEVICE_CLASS;






 



extern const GUI_DEVICE_API GUIDRV_Win_API;

extern const GUI_DEVICE_API GUIDRV_Template_API;



















 
struct GUI_DEVICE {
  
  
  
  GUI_DEVICE * pNext;
  GUI_DEVICE * pPrev;
  
  
  
  union {
    signed long hContext; 
    void   * pContext; 
  } u;
  
  
  
  const GUI_DEVICE_API     * pDeviceAPI;
  const LCD_API_COLOR_CONV * pColorConvAPI;
  unsigned short Flags;
  int LayerIndex;
};

extern const GUI_DEVICE_API GUI_MEMDEV_DEVICE_1;
extern const GUI_DEVICE_API GUI_MEMDEV_DEVICE_8;
extern const GUI_DEVICE_API GUI_MEMDEV_DEVICE_16;
extern const GUI_DEVICE_API GUI_MEMDEV_DEVICE_32;







 
typedef union {
  unsigned char  aColorIndex8[2];
  unsigned short aColorIndex16[2];
  unsigned long aColorIndex32[2];
} LCD_COLORINDEX_UNION;

struct GUI_CONTEXT {
  
  
  
  LCD_COLORINDEX_UNION uLCD;
  LCD_RECT       ClipRect;
  unsigned char             DrawMode;
  unsigned char             SelLayer;
  unsigned char             TextStyle;
  
  
  
  GUI_RECT * pClipRect_HL;                 
  unsigned char         PenSize;
  unsigned char         PenShape;
  unsigned char         LineStyle;
  
  
  
  const GUI_FONT * pAFont;
  signed short LBorder;
  signed short DispPosX, DispPosY;
  signed short DrawPosX, DrawPosY;
  signed short TextMode, TextAlign;
  GUI_COLOR Color, BkColor;                
  
  
  
  unsigned long * LCD_pBkColorIndex;
  unsigned long * LCD_pColorIndex;
  unsigned long * LCD_pAColorIndex;
  
  
  

    const GUI_RECT * WM__pUserClipRect;
    GUI_HWIN hAWin;
    int xOff, yOff;
    unsigned char WM_IsActive;

  
  
  
  
  GUI_DEVICE * apDriver[2];
  
  
  
  signed long    hDevData;
  
  
  
  const tLCD_HL_APIList * pLCD_HL;       
  unsigned char AA_Factor;
  unsigned char AA_HiResEnable;
  void (* AA_pfSetPixelAA)(int x, int y, unsigned char Intens); 
};

 









 
GUI_DEVICE * GUI_DEVICE_Create       (const GUI_DEVICE_API * pDeviceAPI, const LCD_API_COLOR_CONV * pColorConvAPI, unsigned short Flags, int LayerIndex);
GUI_DEVICE * GUI_DEVICE_CreateAndLink(const GUI_DEVICE_API * pDeviceAPI, const LCD_API_COLOR_CONV * pColorConvAPI, unsigned short Flags, int LayerIndex);
void         GUI_DEVICE_Delete       (GUI_DEVICE * pDevice);
int          GUI_DEVICE_Link         (GUI_DEVICE * pDevice);
void         GUI_DEVICE_Unlink       (GUI_DEVICE * pDevice);
GUI_DEVICE * GUI_DEVICE__GetpDriver  (int LayerIndex);
GUI_DEVICE * GUI_DEVICE__GetpDevice  (int LayerIndex, int DeviceClass);

GUI_DEVICE * GUI_DEVICE_UnlinkTaskDevices(void);
void         GUI_DEVICE_LinkDevices      (GUI_DEVICE * pDevice);

void _RegisterExit(void);



 
typedef struct {
  void * pData;         
  int    x0, y0;        
  int    xSize, ySize;  
  int    LineOff;       
  int    BytesPerPixel; 
  int    IsDirty;       
} GUI_DIRTYDEVICE_INFO;

int GUI_DIRTYDEVICE_Create      (void);
int GUI_DIRTYDEVICE_CreateEx    (int LayerIndex);
int GUI_DIRTYDEVICE_CreateExInfo(GUI_DIRTYDEVICE_INFO * pInfo, int LayerIndex);
int GUI_DIRTYDEVICE_Delete      (void);
int GUI_DIRTYDEVICE_DeleteEx    (int LayerIndex);
int GUI_DIRTYDEVICE_Fetch       (GUI_DIRTYDEVICE_INFO * pInfo);
int GUI_DIRTYDEVICE_FetchEx     (GUI_DIRTYDEVICE_INFO * pInfo, int LayerIndex);




 
typedef struct {
  int xPos;
  int yPos;
  int xSize;
  int ySize;
  int Visible;
} GUI_SOFTLAYER_CONFIG;

int  GUI_SOFTLAYER_Enable           (GUI_SOFTLAYER_CONFIG * pConfig, int NumLayers, GUI_COLOR CompositeColor);
int  GUI_SOFTLAYER_Refresh          (void);
void GUI_SOFTLAYER_SetCompositeColor(unsigned long Color);
int  GUI_SOFTLAYER_MULTIBUF_Enable  (int OnOff);




 
int          GUI_Init             (void);
int          GUI_IsInitialized    (void);
void         GUI_Exit             (void);
void         GUI_SetDefaultFont   (const GUI_FONT * pFont);
void         GUI_SetDefault       (void);
GUI_DRAWMODE GUI_SetDrawMode      (GUI_DRAWMODE dm);
const char * GUI_GetVersionString (void);
void         GUI_SaveContext_W      (      GUI_CONTEXT * pContext);
void         GUI_RestoreContext   (const GUI_CONTEXT * pContext);
void         GUI_SetScreenSizeX   (int xSize);
void         GUI_SetScreenSizeY   (int ySize);
int          GUI_GetScreenSizeX   (void);
int          GUI_GetScreenSizeY   (void);
const GUI_RECT * GUI_SetClipRect  (const GUI_RECT * pRect);
void         GUI_SetRefreshHook   (void (* pFunc)(void));
void         GUI_SetControlHook   (void (* pFunc)(int LayerIndex, int Cmd));
void         MainTask             (void);




 
int  GUI_RectsIntersect(const GUI_RECT * pr0, const GUI_RECT * pr1);
void GUI_MoveRect       (GUI_RECT * pRect, int x, int y);
void GUI_MergeRect      (GUI_RECT * pDest, const GUI_RECT * pr0, const GUI_RECT * pr1);
int  GUI__IntersectRects(GUI_RECT * pDest, const GUI_RECT * pr0, const GUI_RECT * pr1);
void GUI__IntersectRect (GUI_RECT * pDest, const GUI_RECT * pr0);
void GUI__ReduceRect    (GUI_RECT * pDest, const GUI_RECT * pRect, int Dist);




 
signed long  GUI__ATan2(signed long x, signed long y, signed long * ph);
signed long  GUI__ASinHQ(signed long SinHQ);
int  GUI__CompactPixelIndices  (unsigned long * pBuffer, int NumPixels, int BitsPerPixel);
int  GUI__CompactPixelIndicesEx(unsigned long * pBuffer, int NumPixels, int BitsPerPixel, const LCD_API_COLOR_CONV * pColorConvAPI);
int  GUI__ConvertColor2Index   (unsigned long * pBuffer, int NumPixels, int BitsPerPixel, const LCD_API_COLOR_CONV * pColorConvAPI, void * pResult);
void GUI__Config(void);
signed long  GUI__CosHQ(signed long Ang1000);
int  GUI__DivideRound     (int a, int b);
signed long  GUI__DivideRound32   (signed long a, signed long b);
void GUI__ExpandPixelIndices   (void * pBuffer, int NumPixels, int BitsPerPixel);
void GUI__ExpandPixelIndicesEx (void * pBuffer, int NumPixels, int BitsPerPixel, const LCD_API_COLOR_CONV * pColorConvAPI);
void GUI__memcpy(void * pDest, const void * pSrc, int NumBytes);
int  GUI__SetText(signed long * phText, const char * s);
signed long  GUI__SinHQ(signed long Ang1000);
signed long  GUI__sqrt32(signed long Square);
void GUI__DrawTwinArc2(int xl, int xr, int y0,         int r, GUI_COLOR ColorR0, GUI_COLOR ColorR1, GUI_COLOR ColorFill);
void GUI__DrawTwinArc4(int x0, int y0, int x1, int y1, int r, GUI_COLOR ColorR0, GUI_COLOR ColorR1, GUI_COLOR ColorFill);
void GUI__FillTrippleArc(int x0, int y0, int Size, GUI_COLOR ColorR0, GUI_COLOR ColorR1, GUI_COLOR ColorR2, GUI_COLOR ColorFill);
void GUI__RegisterExit(GUI_REGISTER_EXIT * pRegisterExit);




 
GUI_COLOR GUI_GetBkColor     (void);
int       GUI_GetBkColorIndex(void);
GUI_COLOR GUI_GetColor       (void);
int       GUI_GetColorIndex  (void);
unsigned char        GUI_GetLineStyle   (void);
unsigned char        GUI_GetPenSize     (void);
unsigned char        GUI_GetPenShape    (void);
unsigned  GUI_GetPixelIndex  (int x, int y);

void      GUI_SetBkColor   (GUI_COLOR);
void      GUI_SetColor     (GUI_COLOR);
void      GUI_SetBkColorIndex(int Index);
void      GUI_SetColorIndex(int Index);

unsigned char        GUI_SetPenSize   (unsigned char Size);
unsigned char        GUI_SetPenShape  (unsigned char Shape);
unsigned char        GUI_SetLineStyle (unsigned char Style);

 
char      GUI_GetDecChar(void);
char      GUI_SetDecChar(char c);




 
int       GUI_Color2Index(GUI_COLOR color);
GUI_COLOR GUI_Color2VisColor(GUI_COLOR color);
char      GUI_ColorIsAvailable(GUI_COLOR color);
GUI_COLOR GUI_Index2Color(int Index);
unsigned long       GUI_CalcColorDist (GUI_COLOR Color0, GUI_COLOR  Color1);
unsigned long       GUI_CalcVisColorError(GUI_COLOR color);




 
void GUI_SetOnErrorFunc(void (* pFunc)(const char * s));




 
void GUI_Log      (const char * s);
void GUI_Log1     (const char * s, signed long p0);
void GUI_Log2     (const char * s, signed long p0, signed long p1);
void GUI_Log3     (const char * s, signed long p0, signed long p1, signed long p2);
void GUI_Log4     (const char * s, signed long p0, signed long p1, signed long p2,signed long p3);
void GUI_Warn     (const char * s);
void GUI_Warn1    (const char * s, signed long p0);
void GUI_Warn2    (const char * s, signed long p0, signed long p1);
void GUI_Warn3    (const char * s, signed long p0, signed long p1, signed long p2);
void GUI_Warn4    (const char * s, signed long p0, signed long p1, signed long p2, signed long p3);
void GUI_ErrorOut (const char * s);
void GUI_ErrorOut1(const char * s, signed long p0);
void GUI_ErrorOut2(const char * s, signed long p0, signed long p1);
void GUI_ErrorOut3(const char * s, signed long p0, signed long p1, signed long p2);
void GUI_ErrorOut4(const char * s, signed long p0, signed long p1, signed long p2, signed long p3);




 
void GUI_Clear            (void);
void GUI_ClearRect        (int x0, int y0, int x1, int y1);
void GUI_ClearRectEx      (const GUI_RECT * pRect);
void GUI_CopyRect         (int x0, int y0, int x1, int y1, int dx, int dy);
void GUI_DrawArc          (int x0, int y0, int rx, int ry, int a0, int a1);
void GUI_DrawBitmap       (const GUI_BITMAP * pBM, int x0, int y0);
void GUI_DrawBitmapMag    (const GUI_BITMAP * pBM, int x0, int y0, int XMul, int YMul);
void GUI_DrawBitmapEx     (const GUI_BITMAP * pBM, int x0, int y0, int xCenter, int yCenter, int xMag, int yMag);
void GUI_DrawBitmapExp    (int x0, int y0, int XSize, int YSize, int XMul,  int YMul, int BitsPerPixel, int BytesPerLine, const unsigned char * pData, const GUI_LOGPALETTE * pPal);
void GUI_DrawBitmapHWAlpha(const GUI_BITMAP * pBM, int x0, int y0);
void GUI_DrawCircle       (int x0, int y0, int r);
void GUI_DrawEllipse      (int x0, int y0, int rx, int ry);
void GUI_DrawGradientH    (int x0, int y0, int x1, int y1, GUI_COLOR Color0, GUI_COLOR Color1);
void GUI_DrawGradientV    (int x0, int y0, int x1, int y1, GUI_COLOR Color0, GUI_COLOR Color1);
void GUI_DrawGradientRoundedH(int x0, int y0, int x1, int y1, int rd, GUI_COLOR Color0, GUI_COLOR Color1);
void GUI_DrawGradientRoundedV(int x0, int y0, int x1, int y1, int rd, GUI_COLOR Color0, GUI_COLOR Color1);
void GUI_DrawGraph        (signed short * pay, int NumPoints, int x0, int y0);
void GUI_DrawGraphEx      (signed short * pay, int NumPoints, int x0, int y0, int Numerator, int Denominator, int MirrorX);
void GUI_DrawHLine        (int y0, int x0, int x1);
void GUI_DrawLine         (int x0, int y0, int x1, int y1);
void GUI_DrawLineRel      (int dx, int dy);
void GUI_DrawLineTo       (int x, int y);
void GUI_DrawPie          (int x0, int y0, int r, int a0, int a1, int Type);
void GUI_DrawPixel        (int x, int y);
void GUI_DrawPoint        (int x, int y);
void GUI_DrawPolygon      (const GUI_POINT * pPoints, int NumPoints, int x0, int y0);
void GUI_DrawPolyLine     (const GUI_POINT * pPoints, int NumPoints, int x0, int y0);
void GUI_DrawFocusRect    (const GUI_RECT  * pRect, int Dist);
void GUI_DrawRect         (int x0, int y0, int x1, int y1);
void GUI_DrawRectEx       (const GUI_RECT * pRect);
void GUI_DrawRoundedFrame (int x0, int y0, int x1, int y1, int r, int w);
void GUI_DrawRoundedRect  (int x0, int y0, int x1, int y1, int r);
void GUI_DrawVLine        (int x0, int y0, int y1);
void GUI_FillCircle       (int x0, int y0, int r);
void GUI_FillEllipse      (int x0, int y0, int rx, int ry);
void GUI_FillPolygon      (const GUI_POINT * pPoints, int NumPoints, int x0, int y0);
void GUI_FillRect         (int x0, int y0, int x1, int y1);
void GUI_FillRectEx       (const GUI_RECT * pRect);
void GUI_FillRoundedFrame (int x0, int y0, int x1, int y1, int r, int w);
void GUI_FillRoundedRect  (int x0, int y0, int x1, int y1, int r);
void GUI_FillRoundedRectB (int x0, int y0, int x1, int y1, int r);
void GUI_FillRoundedRectT (int x0, int y0, int x1, int y1, int r);
void GUI_GetClientRect    (GUI_RECT * pRect);
void GUI_InvertRect       (int x0, int y0, int x1, int y1);
void GUI_MoveRel          (int dx, int dy);
void GUI_MoveTo           (int x, int y);
void GUI_SetAlphaMask8888 (unsigned long OrMask, unsigned long AndMask);




 
typedef int GUI_GET_DATA_FUNC(void * p, const unsigned char ** ppData, unsigned NumBytes, unsigned long Off);




 
int GUI_GIF_Draw           (const void * pGIF, unsigned long NumBytes,         int x0, int y0);
int GUI_GIF_DrawEx         (GUI_GET_DATA_FUNC * pfGetData, void * p, int x0, int y0);
int GUI_GIF_DrawSub        (const void * pGIF, unsigned long NumBytes,         int x0, int y0, int Index);
int GUI_GIF_DrawSubEx      (GUI_GET_DATA_FUNC * pfGetData, void * p, int x0, int y0, int Index);
int GUI_GIF_DrawSubScaled  (const void * pGIF, unsigned long NumBytes,         int x0, int y0, int Index, int Num, int Denom);
int GUI_GIF_DrawSubScaledEx(GUI_GET_DATA_FUNC * pfGetData, void * p, int x0, int y0, int Index, int Num, int Denom);
int GUI_GIF_GetComment     (const void * pGIF, unsigned long NumBytes,         unsigned char * pBuffer, int MaxSize, int Index);
int GUI_GIF_GetCommentEx   (GUI_GET_DATA_FUNC * pfGetData, void * p, unsigned char * pBuffer, int MaxSize, int Index);
int GUI_GIF_GetImageInfo   (const void * pGIF, unsigned long NumBytes,         GUI_GIF_IMAGE_INFO * pInfo, int Index);
int GUI_GIF_GetImageInfoEx (GUI_GET_DATA_FUNC * pfGetData, void * p, GUI_GIF_IMAGE_INFO * pInfo, int Index);
int GUI_GIF_GetInfo        (const void * pGIF, unsigned long NumBytes,         GUI_GIF_INFO * pInfo);
int GUI_GIF_GetInfoEx      (GUI_GET_DATA_FUNC * pfGetData, void * p, GUI_GIF_INFO * pInfo);
int GUI_GIF_GetXSize       (const void * pGIF);
int GUI_GIF_GetXSizeEx     (GUI_GET_DATA_FUNC * pfGetData, void * p);
int GUI_GIF_GetYSize       (const void * pGIF);
int GUI_GIF_GetYSizeEx     (GUI_GET_DATA_FUNC * pfGetData, void * p);
int GUI_GIF_SetFillTrans   (int OnOff);




 
int  GUI_BMP_Draw        (const void * pFileData,                  int x0, int y0);
int  GUI_BMP_DrawEx      (GUI_GET_DATA_FUNC * pfGetData, void * p, int x0, int y0);
int  GUI_BMP_DrawScaled  (const void * pFileData,                  int x0, int y0, int Num, int Denom);
int  GUI_BMP_DrawScaledEx(GUI_GET_DATA_FUNC * pfGetData, void * p, int x0, int y0, int Num, int Denom);
int  GUI_BMP_GetXSize    (const void * pFileData);
int  GUI_BMP_GetXSizeEx  (GUI_GET_DATA_FUNC * pfGetData, void * p);
int  GUI_BMP_GetYSize    (const void * pFileData);
int  GUI_BMP_GetYSizeEx  (GUI_GET_DATA_FUNC * pfGetData, void * p);
void GUI_BMP_EnableAlpha (void);
void GUI_BMP_DisableAlpha(void);




 
int GUI_PNG_Draw      (const void * pFileData, int DataSize, int x0, int y0);
int GUI_PNG_DrawEx    (GUI_GET_DATA_FUNC * pfGetData, void * p, int x0, int y0);
int GUI_PNG_GetXSize  (const void * pFileData, int FileSize);
int GUI_PNG_GetXSizeEx(GUI_GET_DATA_FUNC * pfGetData, void * p);
int GUI_PNG_GetYSize  (const void * pFileData, int FileSize);
int GUI_PNG_GetYSizeEx(GUI_GET_DATA_FUNC * pfGetData, void * p);




 
typedef struct {
  int XSize;
  int YSize;
} GUI_JPEG_INFO;

int GUI_JPEG_Draw        (const void * pFileData, int DataSize,    int x0, int y0);
int GUI_JPEG_DrawEx      (GUI_GET_DATA_FUNC * pfGetData, void * p, int x0, int y0);
int GUI_JPEG_DrawScaled  (const void * pFileData, int DataSize,    int x0, int y0, int Num, int Denom);
int GUI_JPEG_DrawScaledEx(GUI_GET_DATA_FUNC * pfGetData, void * p, int x0, int y0, int Num, int Denom);
int GUI_JPEG_GetInfo     (const void * pFileData, int DataSize,    GUI_JPEG_INFO * pInfo);
int GUI_JPEG_GetInfoEx   (GUI_GET_DATA_FUNC * pfGetData, void * p, GUI_JPEG_INFO * pInfo);




 






typedef signed long GUI_MOVIE_HANDLE;

typedef void GUI_MOVIE_FUNC(GUI_MOVIE_HANDLE hMovie, int Notification, unsigned long CurrentFrame);

typedef struct {
  int xSize;         
  int ySize;         
  int msPerFrame;    
  unsigned long NumFrames;     
} GUI_MOVIE_INFO;

GUI_MOVIE_HANDLE GUI_MOVIE_Create       (const void * pFileData, unsigned long FileSize, GUI_MOVIE_FUNC * pfNotify);
GUI_MOVIE_HANDLE GUI_MOVIE_CreateEx     (GUI_GET_DATA_FUNC * pfGetData, void * pParam, GUI_MOVIE_FUNC * pfNotify);
int              GUI_MOVIE_Delete       (GUI_MOVIE_HANDLE hMovie);
unsigned long              GUI_MOVIE_GetFrameIndex(GUI_MOVIE_HANDLE hMovie);
int              GUI_MOVIE_GetInfo      (const void * pFileData, unsigned long FileSize, GUI_MOVIE_INFO * pInfo);
int              GUI_MOVIE_GetInfoEx    (GUI_GET_DATA_FUNC * pfGetData, void * pParam, GUI_MOVIE_INFO * pInfo);
int              GUI_MOVIE_GetPos       (GUI_MOVIE_HANDLE hMovie, int * pxPos, int * pyPos, int * pxSize, int * pySize);
int              GUI_MOVIE_GotoFrame    (GUI_MOVIE_HANDLE hMovie, unsigned long Frame);
int              GUI_MOVIE_Pause        (GUI_MOVIE_HANDLE hMovie);
int              GUI_MOVIE_Play         (GUI_MOVIE_HANDLE hMovie);
int              GUI_MOVIE_SetPeriod    (GUI_MOVIE_HANDLE hMovie, unsigned Period);
int              GUI_MOVIE_SetPos       (GUI_MOVIE_HANDLE hMovie, int xPos, int yPos);
int              GUI_MOVIE_ShowScaled   (GUI_MOVIE_HANDLE hMovie, int xPos, int yPos, int num, int denom, int DoLoop);
int              GUI_MOVIE_Show         (GUI_MOVIE_HANDLE hMovie, int xPos, int yPos, int DoLoop);




 



typedef struct {
  const GUI_BITMAP  * pBitmap;
  int                 xHot;
  int                 yHot;
} GUI_CURSOR;

typedef struct {
  const GUI_BITMAP ** ppBm;
  int                 xHot;
  int                 yHot;
  unsigned            Period;
  const unsigned    * pPeriod;
  int                 NumItems;
} GUI_CURSOR_ANIM;


  int                GUI_CURSOR_GetState     (void);
  int                GUI_CURSOR_GetStateEx   (int Layer);
  void               GUI_CURSOR_Hide         (void);
  void               GUI_CURSOR_HideEx       (int Layer);
  const GUI_CURSOR * GUI_CURSOR_Select       (const GUI_CURSOR * pCursor);
  const GUI_CURSOR * GUI_CURSOR_SelectEx     (const GUI_CURSOR * pCursor, int Layer);
  int                GUI_CURSOR_SelectAnim   (const GUI_CURSOR_ANIM * pCursorAnim);
  int                GUI_CURSOR_SelectAnimEx (const GUI_CURSOR_ANIM * pCursorAnim, int LayerIndex);
  int                GUI_CURSOR_SetBitmap    (const GUI_BITMAP * pBM);
  int                GUI_CURSOR_SetBitmapEx  (const GUI_BITMAP * pBM, int Layer);
  void               GUI_CURSOR_SetPosition  (int x, int y);
  void               GUI_CURSOR_SetPositionEx(int xNewPos, int yNewPos, int Layer);
  void               GUI_CURSOR_Show         (void);
  void               GUI_CURSOR_ShowEx       (int Layer);
  GUI_HSPRITE        GUI_CURSOR__GetSpriteEx (int LayerIndex, int * pxPos, int * pyPos);
  void               GUI_CURSOR__SetSpriteEx (GUI_HSPRITE hSprite, const GUI_CURSOR * pCursor, int LayerIndex);








 






GUI_HSPRITE GUI_SPRITE__CreateEx           (const GUI_BITMAP * pBM, int x, int y, int Layer, unsigned short Flags);  
void        GUI_SPRITE__SetCallback        (GUI_HSPRITE hSprite, signed long hContext, void (* pCB)(GUI_HSPRITE, int));
GUI_HSPRITE GUI_SPRITE_Create              (const GUI_BITMAP * pBM, int x, int y);
GUI_HSPRITE GUI_SPRITE_CreateAnim          (const GUI_BITMAP ** ppBm, int x, int y, unsigned Period, const unsigned * pPeriod, int NumItems);
GUI_HSPRITE GUI_SPRITE_CreateEx            (const GUI_BITMAP * pBM, int x, int y, int Layer);
GUI_HSPRITE GUI_SPRITE_CreateExAnim        (const GUI_BITMAP ** ppBm, int x, int y, unsigned Period, const unsigned * pPeriod, int NumItems, int LayerIndex);
GUI_HSPRITE GUI_SPRITE_CreateHidden        (const GUI_BITMAP * pBM, int x, int y);
GUI_HSPRITE GUI_SPRITE_CreateHiddenEx      (const GUI_BITMAP * pBM, int x, int y, int Layer);
void        GUI_SPRITE_Delete              (GUI_HSPRITE hSprite);
int         GUI_SPRITE_GetState            (GUI_HSPRITE hSprite);
void        GUI_SPRITE_Hide                (GUI_HSPRITE hSprite);
int         GUI_SPRITE_SetBitmap           (GUI_HSPRITE hSprite, const GUI_BITMAP * pBM);
int         GUI_SPRITE_SetBitmapAndPosition(GUI_HSPRITE hSprite, const GUI_BITMAP * pBM, int x, int y);
int         GUI_SPRITE_SetLoop             (GUI_HSPRITE hSprite, int OnOff);
void        GUI_SPRITE_SetPosition         (GUI_HSPRITE hSprite, int x, int y);
int         GUI_SPRITE_StartAnim           (GUI_HSPRITE hSprite);
int         GUI_SPRITE_StopAnim            (GUI_HSPRITE hSprite);
void        GUI_SPRITE_Show                (GUI_HSPRITE hSprite);




 
extern const GUI_CURSOR GUI_CursorArrowS,  GUI_CursorArrowSI;
extern const GUI_CURSOR GUI_CursorArrowM,  GUI_CursorArrowMI;
extern const GUI_CURSOR GUI_CursorArrowL,  GUI_CursorArrowLI;
extern const GUI_CURSOR GUI_CursorCrossS,  GUI_CursorCrossSI;
extern const GUI_CURSOR GUI_CursorCrossM,  GUI_CursorCrossMI;
extern const GUI_CURSOR GUI_CursorCrossL,  GUI_CursorCrossLI;
extern const GUI_CURSOR GUI_CursorHeaderM, GUI_CursorHeaderMI;

extern const GUI_BITMAP GUI_BitmapArrowS, GUI_BitmapArrowSI;
extern const GUI_BITMAP GUI_BitmapArrowM, GUI_BitmapArrowMI;
extern const GUI_BITMAP GUI_BitmapArrowL, GUI_BitmapArrowLI;
extern const GUI_BITMAP GUI_BitmapCrossS, GUI_BitmapCrossSI;
extern const GUI_BITMAP GUI_BitmapCrossM, GUI_BitmapCrossMI;
extern const GUI_BITMAP GUI_BitmapCrossL, GUI_BitmapCrossLI;

extern const GUI_CURSOR_ANIM GUI_CursorAnimHourglassM;




 
typedef enum { GUI_WRAPMODE_NONE, GUI_WRAPMODE_WORD, GUI_WRAPMODE_CHAR } GUI_WRAPMODE;




 
void  GUI_DispCEOL             (void);
void  GUI_DispChar             (unsigned short c);
void  GUI_DispCharAt           (unsigned short c, signed short x, signed short y);
void  GUI_DispChars            (unsigned short c, int Cnt);
void  GUI_DispNextLine         (void);
void  GUI_DispString           (const char * s);
void  GUI_DispStringAt         (const char * s, int x, int y);
void  GUI_DispStringAtCEOL     (const char * s, int x, int y);
void  GUI_DispStringHCenterAt  (const char * s, int x, int y);
void  GUI__DispStringInRect    (const char * s, GUI_RECT * pRect, int TextAlign, int MaxNumChars);
void  GUI_DispStringInRect     (const char * s, GUI_RECT * pRect, int TextAlign);

  void  GUI_DispStringInRectEx (const char * s, GUI_RECT * pRect, int TextAlign, int MaxLen, const tLCD_APIList * pLCD_Api);

void  GUI_DispStringInRectMax  (const char * s, GUI_RECT * pRect, int TextAlign, int MaxLen);  
void  GUI_DispStringInRectWrap (const char * s, GUI_RECT * pRect, int TextAlign, GUI_WRAPMODE WrapMode);  
void  GUI_DispStringLen        (const char * s, int Len);
void  GUI_GetTextExtend        (GUI_RECT* pRect, const char * s, int Len);
int   GUI_GetYAdjust           (void);
int   GUI_GetDispPosX          (void);
int   GUI_GetDispPosY          (void);
const GUI_FONT * GUI_GetFont(void);
int   GUI_GetCharDistX         (unsigned short c);
int   GUI_GetCharDistXEx       (unsigned short c, int * pSizeX);
int   GUI_GetStringDistX       (const char * s);
GUI_DRAWMODE GUI_GetDrawMode   (void);
int   GUI_GetFontDistY         (void);
int   GUI_GetFontSizeY         (void);
void  GUI_GetFontInfo          (const GUI_FONT * pFont, GUI_FONTINFO * pfi);
void  GUI_GetOrg               (int * px, int * py);
int   GUI_GetYSizeOfFont       (const GUI_FONT * pFont);
int   GUI_GetYDistOfFont       (const GUI_FONT * pFont);
int   GUI_GetTextAlign         (void);
int   GUI_GetTextMode          (void);
char  GUI_IsInFont             (const GUI_FONT * pFont, unsigned short c);
int   GUI_SetTextAlign         (int Align);
int   GUI_SetTextMode          (int Mode);
char  GUI_SetTextStyle         (char Style);
int   GUI_SetLBorder           (int x);
const GUI_FONT * GUI_SetFont(const GUI_FONT * pNewFont);
char  GUI_GotoXY               (int x, int y);
char  GUI_GotoX                (int x);
char  GUI_GotoY                (int y);
int   GUI_WrapGetNumLines      (const char * pText, int xSize, GUI_WRAPMODE WrapMode);

int   GUI_GetLeadingBlankCols (unsigned short c);
int   GUI_GetTrailingBlankCols(unsigned short c);





 
void GUI_SIF_CreateFont(const void * pFontData, GUI_FONT * pFont, const tGUI_SIF_APIList * pFontType);
void GUI_SIF_DeleteFont(GUI_FONT * pFont);




 
int  GUI_XBF_CreateFont(GUI_FONT * pFont, GUI_XBF_DATA * pXBF, const tGUI_XBF_APIList * pFontType, GUI_XBF_GET_DATA_FUNC * pfGetData, void * pVoid);
void GUI_XBF_DeleteFont(GUI_FONT * pFont);




 
int  GUI_TTF_CreateFont   (GUI_FONT * pFont, GUI_TTF_CS * pCS);
int  GUI_TTF_CreateFontAA (GUI_FONT * pFont, GUI_TTF_CS * pCS);
void GUI_TTF_DestroyCache (void);
void GUI_TTF_Done         (void);
int  GUI_TTF_GetFamilyName(GUI_FONT * pFont, char * pBuffer, int NumBytes);
int  GUI_TTF_GetStyleName (GUI_FONT * pFont, char * pBuffer, int NumBytes);
void GUI_TTF_SetCacheSize (unsigned MaxFaces, unsigned MaxSizes, unsigned long MaxBytes);




 
int          GUI_LANG_GetLang          (void);
int          GUI_LANG_GetNumItems      (int IndexLang);
const char * GUI_LANG_GetText          (int IndexText);
int          GUI_LANG_GetTextBuffered  (int IndexText, char * pBuffer, int SizeOfBuffer);
int          GUI_LANG_GetTextBufferedEx(int IndexText, int IndexLang, char * pBuffer, int SizeOfBuffer);
const char * GUI_LANG_GetTextEx        (int IndexText, int IndexLang);
int          GUI_LANG_LoadCSV          (unsigned char * pFileData, unsigned long FileSize);
int          GUI_LANG_LoadCSVEx        (GUI_GET_DATA_FUNC * pfGetData, void * p);
int          GUI_LANG_LoadText         (unsigned char * pFileData, unsigned long FileSize, int IndexLang);
int          GUI_LANG_LoadTextEx       (GUI_GET_DATA_FUNC * pfGetData, void * p, int IndexLang);
int          GUI_LANG_SetLang          (int IndexLang);
unsigned     GUI_LANG_SetMaxNumLang    (unsigned MaxNumLang);
unsigned short          GUI_LANG_SetSep           (unsigned short Sep);




 
int   GUI_UC_ConvertUC2UTF8   (const unsigned short * s, int Len, char * pBuffer, int BufferSize);
int   GUI_UC_ConvertUTF82UC   (const char * s, int Len, unsigned short * pBuffer, int BufferSize);
int   GUI_UC_Encode           (char * s, unsigned short Char);
int   GUI_UC_GetCharSize      (const char * s);
unsigned short   GUI_UC_GetCharCode      (const char * s);
void  GUI_UC_SetEncodeNone    (void);
void  GUI_UC_SetEncodeUTF8    (void);
int   GUI_UC_EnableBIDI       (int OnOff);

void GUI_UC_DispString(const unsigned short * s);
void GUI_UC2DB (unsigned short Code, unsigned char * pOut);
unsigned short  GUI_DB2UC (unsigned char Byte0, unsigned char Byte1);




 
void GUI_DispBin  (unsigned long  v, unsigned char Len);
void GUI_DispBinAt(unsigned long  v, signed short x, signed short y, unsigned char Len);
void GUI_DispDec  (signed long v, unsigned char Len);
void GUI_DispDecAt (signed long v, signed short x, signed short y, unsigned char Len);
void GUI_DispDecMin(signed long v);
void GUI_DispDecShift(signed long v, unsigned char Len, unsigned char Shift);
void GUI_DispDecSpace(signed long v, unsigned char MaxDigits);
void GUI_DispHex  (unsigned long v, unsigned char Len);
void GUI_DispHexAt(unsigned long v, signed short x, signed short y, unsigned char Len);
void GUI_DispSDec(signed long v, unsigned char Len);
void GUI_DispSDecShift(signed long v, unsigned char Len, unsigned char Shift);




 
void GUI_DispFloat    (float v, char Len);
void GUI_DispFloatFix (float v, char Len, char Fract);
void GUI_DispFloatMin (float v, char Fract);
void GUI_DispSFloatFix(float v, char Len, char Fract);
void GUI_DispSFloatMin(float v, char Fract);




 
typedef struct {
  unsigned long TotalBytes;
  unsigned long FreeBytes;
  unsigned long UsedBytes;
  unsigned long AllocSize;
  unsigned long NumFixedBytes;
  unsigned long MaxUsedBytes;
} GUI_ALLOC_INFO;

signed long GUI_ALLOC_GetNumFreeBlocks(void);
signed long GUI_ALLOC_GetNumFreeBytes (void);
signed long GUI_ALLOC_GetNumUsedBlocks(void);
signed long GUI_ALLOC_GetNumUsedBytes (void);
signed long GUI_ALLOC_GetMaxUsedBytes (void);

void GUI_ALLOC_GetMemInfo  (GUI_ALLOC_INFO * pInfo);
void GUI_ALLOC_SuppressPeak(int OnOff);

signed long           GUI_ALLOC_AllocInit       (const void * pInitData, signed long Size);
signed long           GUI_ALLOC_AllocNoInit     (signed long size);
signed long           GUI_ALLOC_AllocZero       (signed long size);
void               GUI_ALLOC_AssignMemory    (void * p, unsigned long NumBytes);
void               GUI_ALLOC_Free            (signed long  hMem);
void               GUI_ALLOC_FreeFixedBlock  (void * p);
void               GUI_ALLOC_FreePtrArray    (signed long * pArray, int NumElems);
void               GUI_ALLOC_FreePtr         (signed long * phMem);
void *             GUI_ALLOC_GetFixedBlock   (signed long Size);
signed long GUI_ALLOC_GetMaxSize      (void);
signed long GUI_ALLOC_GetSize         (signed long  hMem);
void *             GUI_ALLOC_h2p             (signed long  hMem);
signed long           GUI_ALLOC_p2h             (void * p);
void               GUI_ALLOC_Init            (void);
void               GUI_ALLOC_Lock            (void);
void *             GUI_ALLOC_LockH           (signed long  hMem);
signed long           GUI_ALLOC_Realloc         (signed long hOld, int NewSize);
signed long GUI_ALLOC_RequestSize     (void);
void               GUI_ALLOC_SetAvBlockSize  (unsigned long BlockSize);
void               GUI_ALLOC_Unlock          (void);
void *             GUI_ALLOC_UnlockH         (void ** pp);
int                GUI_ALLOC_SetMaxPercentage(int MaxPercentage);




 



typedef signed long GUI_MEMDEV_Handle;
typedef void     GUI_CALLBACK_VOID_P        (void * p);
typedef int      GUI_ANIMATION_CALLBACK_FUNC(int TimeRem, void * pVoid);
typedef void     GUI_DRAWMEMDEV_16BPP_FUNC  (void * pDst, const void * pSrc, int xSize, int ySize, int BytesPerLineDst, int BytesPerLineSrc);

extern GUI_ANIMATION_CALLBACK_FUNC * GUI_MEMDEV__pCbAnimation;
extern void                        * GUI_MEMDEV__pVoid;

typedef struct {
  GUI_RECT rView, rPrev;
  char FirstCall;
} GUI_AUTODEV;

typedef struct {
  char DrawFixed;
  char IsMeasurement;
} GUI_AUTODEV_INFO;

int  GUI_MEMDEV_CreateAuto(GUI_AUTODEV * pAutoDev);
void GUI_MEMDEV_DeleteAuto(GUI_AUTODEV * pAutoDev);
int  GUI_MEMDEV_DrawAuto  (GUI_AUTODEV * pAutoDev, GUI_AUTODEV_INFO * pAutoDevInfo, GUI_CALLBACK_VOID_P * pfDraw, void * pData);

 
GUI_MEMDEV_Handle GUI_MEMDEV_Create       (int x0, int y0, int xSize, int ySize);
GUI_MEMDEV_Handle GUI_MEMDEV_CreateEx     (int x0, int y0, int xSize, int ySize, int Flags);
GUI_MEMDEV_Handle GUI_MEMDEV_CreateFixed  (int x0, int y0, int xSize, int ySize, int Flags,
                                           const GUI_DEVICE_API     * pDeviceAPI,
                                           const LCD_API_COLOR_CONV * pColorConvAPI);
GUI_MEMDEV_Handle GUI_MEMDEV_CreateFixed32(int x0, int y0, int xSize, int ySize);

void GUI_MEMDEV_Clear                (GUI_MEMDEV_Handle hMem);
int  GUI_MEMDEV_ClearAlpha           (GUI_MEMDEV_Handle hMemData, GUI_MEMDEV_Handle hMemMask);
void GUI_MEMDEV_CopyFromLCD          (GUI_MEMDEV_Handle hMem);
void GUI_MEMDEV_CopyFromLCDAA        (GUI_MEMDEV_Handle hMem);
void GUI_MEMDEV_CopyToLCD            (GUI_MEMDEV_Handle hMem);
void GUI_MEMDEV_CopyToLCDAA          (GUI_MEMDEV_Handle hMem);
void GUI_MEMDEV_CopyToLCDAt          (GUI_MEMDEV_Handle hMem, int x, int y);
int  GUI_MEMDEV_CompareWithLCD       (GUI_MEMDEV_Handle hMem, int * px, int * py, int * pExp, int * pAct);
void GUI_MEMDEV_Delete               (GUI_MEMDEV_Handle MemDev);
void GUI_MEMDEV_DrawPerspectiveX     (GUI_MEMDEV_Handle hMem, int x, int y, int h0, int h1, int dx, int dy);
int  GUI_MEMDEV_GetXPos              (GUI_MEMDEV_Handle hMem);
int  GUI_MEMDEV_GetXSize             (GUI_MEMDEV_Handle hMem);
int  GUI_MEMDEV_GetYPos              (GUI_MEMDEV_Handle hMem);
int  GUI_MEMDEV_GetYSize             (GUI_MEMDEV_Handle hMem);
void GUI_MEMDEV_MarkDirty            (GUI_MEMDEV_Handle hMem, int x0, int y0, int x1, int y1);
void GUI_MEMDEV_ReduceYSize          (GUI_MEMDEV_Handle hMem, int YSize);
void GUI_MEMDEV_Rotate               (GUI_MEMDEV_Handle hSrc, GUI_MEMDEV_Handle hDst, int dx, int dy, int a, int Mag);
void GUI_MEMDEV_RotateAlpha          (GUI_MEMDEV_Handle hSrc, GUI_MEMDEV_Handle hDst, int dx, int dy, int a, int Mag, unsigned char Alpha);
void GUI_MEMDEV_RotateHR             (GUI_MEMDEV_Handle hSrc, GUI_MEMDEV_Handle hDst, signed long dx, signed long dy, int a, int Mag);
void GUI_MEMDEV__Rotate              (GUI_MEMDEV_Handle hSrc, GUI_MEMDEV_Handle hDst, int dx, int dy, int a, int Mag, unsigned long Mask);
void GUI_MEMDEV__RotateHR            (GUI_MEMDEV_Handle hSrc, GUI_MEMDEV_Handle hDst, signed long dx, signed long dy, int a, int Mag, unsigned long Mask);
void GUI_MEMDEV_RotateHQ             (GUI_MEMDEV_Handle hSrc, GUI_MEMDEV_Handle hDst, int dx, int dy, int a, int Mag);
void GUI_MEMDEV_RotateHQAlpha        (GUI_MEMDEV_Handle hSrc, GUI_MEMDEV_Handle hDst, int dx, int dy, int a, int Mag, unsigned char Alpha);
void GUI_MEMDEV_RotateHQHR           (GUI_MEMDEV_Handle hSrc, GUI_MEMDEV_Handle hDst, signed long dx, signed long dy, int a, int Mag);
void GUI_MEMDEV_RotateHQT            (GUI_MEMDEV_Handle hSrc, GUI_MEMDEV_Handle hDst, int dx, int dy, int a, int Mag);
void GUI_MEMDEV_RotateHQTI           (GUI_MEMDEV_Handle hSrc, GUI_MEMDEV_Handle hDst, int dx, int dy, int a, int Mag);
GUI_MEMDEV_Handle GUI_MEMDEV_Select  (GUI_MEMDEV_Handle hMem);   
void  GUI_MEMDEV_SetOrg              (GUI_MEMDEV_Handle hMem, int x0, int y0);
void  GUI_MEMDEV_WriteAt             (GUI_MEMDEV_Handle hMem, int x, int y);
void  GUI_MEMDEV_Write               (GUI_MEMDEV_Handle hMem);
void  GUI_MEMDEV_WriteAlphaAt        (GUI_MEMDEV_Handle hMem, int Alpha, int x, int y);
void  GUI_MEMDEV_WriteAlpha          (GUI_MEMDEV_Handle hMem, int Alpha);
void  GUI_MEMDEV_WriteExAt           (GUI_MEMDEV_Handle hMem, int x, int y, int xMag, int yMag, int Alpha);
void  GUI_MEMDEV_WriteEx             (GUI_MEMDEV_Handle hMem, int xMag, int yMag, int Alpha);
void  GUI_MEMDEV_WriteOpaque         (GUI_MEMDEV_Handle hMem);
void  GUI_MEMDEV_WriteOpaqueAt       (GUI_MEMDEV_Handle hMem, int x, int y);
int   GUI_MEMDEV_Draw                (GUI_RECT * pRect, GUI_CALLBACK_VOID_P * pfDraw, void * pData, int NumLines, int Flags);
void* GUI_MEMDEV_GetDataPtr          (GUI_MEMDEV_Handle hMem);
void  GUI_MEMDEV_SetColorConv        (GUI_MEMDEV_Handle hMem, const LCD_API_COLOR_CONV * pColorConvAPI);
const LCD_API_COLOR_CONV * GUI_MEMDEV_GetColorConv(GUI_MEMDEV_Handle hMemDev);
int   GUI_MEMDEV_GetBitsPerPixel     (GUI_MEMDEV_Handle hMemDev);
int   GUI_MEMDEV_FadeInDevices       (GUI_MEMDEV_Handle hMem0, GUI_MEMDEV_Handle hMem1, int Period);
int   GUI_MEMDEV_FadeOutDevices      (GUI_MEMDEV_Handle hMem0, GUI_MEMDEV_Handle hMem1, int Period);
void  GUI_MEMDEV_SerializeBMP        (GUI_MEMDEV_Handle hDev, GUI_CALLBACK_VOID_U8_P * pfSerialize, void * p);
void  GUI_MEMDEV_SetAnimationCallback(GUI_ANIMATION_CALLBACK_FUNC * pCbAnimation, void * pVoid);
void  GUI_MEMDEV__FadeDevice         (GUI_MEMDEV_Handle hMemWin, GUI_MEMDEV_Handle hMemBk, GUI_MEMDEV_Handle hMemDst, unsigned char Intens);
void  GUI_MEMDEV__FadeDeviceEx       (GUI_MEMDEV_Handle hMemWin, GUI_MEMDEV_Handle hMemBk, GUI_MEMDEV_Handle hMemDst, unsigned char Intens, int xPosWin, int yPosWin);
int   GUI_MEMDEV_PunchOutDevice      (GUI_MEMDEV_Handle hMemData, GUI_MEMDEV_Handle hMemMask);
void  GUI_MEMDEV_SetTimePerFrame     (unsigned TimePerFrame);

void  GUI_SelectLCD(void);

 
GUI_MEMDEV_Handle GUI_MEMDEV_CreateBlurredDevice32  (GUI_MEMDEV_Handle hMem, unsigned char Depth);
GUI_MEMDEV_Handle GUI_MEMDEV_CreateBlurredDevice32HQ(GUI_MEMDEV_Handle hMem, unsigned char Depth);
GUI_MEMDEV_Handle GUI_MEMDEV_CreateBlurredDevice32LQ(GUI_MEMDEV_Handle hMem, unsigned char Depth);
void              GUI_MEMDEV_SetBlurHQ              (void);
void              GUI_MEMDEV_SetBlurLQ              (void);
int               GUI_MEMDEV_BlendColor32           (GUI_MEMDEV_Handle hMem, unsigned long BlendColor, unsigned char BlendIntens);
int               GUI_MEMDEV_Dither32               (GUI_MEMDEV_Handle hMem, const LCD_API_COLOR_CONV * pColorConvAPI);

 
void GUI_MEMDEV_SetDrawMemdev16bppFunc(GUI_DRAWMEMDEV_16BPP_FUNC * pfDrawMemdev16bppFunc);




 
typedef struct {
  unsigned long UserAlpha;
} GUI_ALPHA_STATE;



unsigned GUI_EnableAlpha         (unsigned OnOff);
unsigned long      GUI_RestoreUserAlpha    (GUI_ALPHA_STATE * pAlphaState);
unsigned GUI_SetAlpha            (unsigned char Alpha);
unsigned long      GUI_SetUserAlpha        (GUI_ALPHA_STATE * pAlphaState, unsigned long UserAlpha);
void     GUI_SetFuncAlphaBlending(void (* pfAlphaBlending)(LCD_COLOR *, LCD_COLOR *, LCD_COLOR *, unsigned long));
void     GUI_SetFuncMixColors    (LCD_COLOR (* pFunc)(LCD_COLOR Color, LCD_COLOR BkColor, unsigned char Intens));
void     GUI_SetFuncMixColorsBulk(void (* pFunc)(unsigned long * pFG, unsigned long * pBG, unsigned long * pDst, unsigned OffFG, unsigned OffBG, unsigned OffDest, unsigned xSize, unsigned ySize, unsigned char Intens));
unsigned GUI_PreserveTrans       (unsigned OnOff);




 
unsigned GUI_SelectLayer(unsigned Index);
unsigned GUI_GetSelLayer(void);

int  GUI_SetLayerPosEx  (unsigned Index, int xPos, int yPos);
int  GUI_SetLayerSizeEx (unsigned Index, int xSize, int ySize);
int  GUI_SetLayerVisEx  (unsigned Index, int OnOff);
int  GUI_SetLayerAlphaEx(unsigned Index, int Alpha);
void GUI_GetLayerPosEx  (unsigned Index, int * pxPos, int * pyPos);

void     GUI_AssignCursorLayer(unsigned Index, unsigned CursorLayer);
unsigned GUI_GetCursorLayer   (unsigned Index);




 
void GUI_SetOrg(int x, int y);

void GUI_MULTIBUF_Begin          (void);
void GUI_MULTIBUF_BeginEx        (int LayerIndex);
void GUI_MULTIBUF_End            (void);
void GUI_MULTIBUF_EndEx          (int LayerIndex);
void GUI_MULTIBUF_Config         (int NumBuffers);
void GUI_MULTIBUF_ConfigEx       (int LayerIndex, int NumBuffers);
void GUI_MULTIBUF_Confirm        (int Index);
void GUI_MULTIBUF_ConfirmEx      (int LayerIndex, int BufferIndex);
int  GUI_MULTIBUF_GetNumBuffers  (void);
int  GUI_MULTIBUF_GetNumBuffersEx(int LayerIndex);
void GUI_MULTIBUF_UseSingleBuffer(void);




 
int  GUI_SPY_Process      (GUI_tSend pfSend, GUI_tRecv pfRecv, void * pConnectInfo);
void GUI_SPY_SetMemHandler(GUI_tMalloc pMalloc, GUI_tFree pFree);
int  GUI_SPY_StartServer  (void);
int  GUI_SPY_X_StartServer(void);




 













typedef signed long GUI_ANIM_HANDLE;

typedef signed long (* GUI_ANIM_GETPOS_FUNC)(int ts, int te, int tNow);

typedef struct {
  int Pos;
  int State;
  GUI_ANIM_HANDLE hAnim;
  int Period;
} GUI_ANIM_INFO;

typedef void GUI_ANIMATION_FUNC(GUI_ANIM_INFO * pInfo, void * pVoid);

signed long GUI_ANIM__Linear    (int ts, int te, int tNow);
signed long GUI_ANIM__Decel     (int ts, int te, int tNow);
signed long GUI_ANIM__Accel     (int ts, int te, int tNow);
signed long GUI_ANIM__AccelDecel(int ts, int te, int tNow);

int             GUI_ANIM_AddItem(GUI_ANIM_HANDLE hAnim, int ts, int te, GUI_ANIM_GETPOS_FUNC pfGetPos, void * pVoid, GUI_ANIMATION_FUNC * pfAnim);
GUI_ANIM_HANDLE GUI_ANIM_Create (int Period, unsigned MinTimePerFrame, void * pVoid, void (* pfSliceInfo)(int State, void * pVoid));
void            GUI_ANIM_Delete (GUI_ANIM_HANDLE hAnim);
int             GUI_ANIM_Exec   (GUI_ANIM_HANDLE hAnim);
void            GUI_ANIM_Start  (GUI_ANIM_HANDLE hAnim);




 



 
typedef struct {
  void     (* pfDrawBitmap   )(GUI_DEVICE * pDevice, int x0, int y0, int xsize, int ysize, int BitsPerPixel, int BytesPerLine, const unsigned char * pData, int Diff, const unsigned long * pTrans);
  void     (* pfDrawHLine    )(GUI_DEVICE * pDevice, int x0, int y0,  int x1);
  void     (* pfDrawVLine    )(GUI_DEVICE * pDevice, int x , int y0,  int y1);
  void     (* pfFillRect     )(GUI_DEVICE * pDevice, int x0, int y0, int x1, int y1);
  unsigned (* pfGetPixelIndex)(GUI_DEVICE * pDevice, int x, int y);
  void     (* pfSetPixelIndex)(GUI_DEVICE * pDevice, int x, int y, int ColorIndex);
  void     (* pfXorPixel     )(GUI_DEVICE * pDevice, int x, int y);
  int      BytesPerPixel;
} GUI_ORIENTATION_API;

extern const GUI_ORIENTATION_API GUI_OrientationAPI_C0;
extern const GUI_ORIENTATION_API GUI_OrientationAPI_C8;
extern const GUI_ORIENTATION_API GUI_OrientationAPI_C16;
extern const GUI_ORIENTATION_API GUI_OrientationAPI_C32;






int GUI_SetOrientation        (int Orientation);
int GUI_SetOrientationEx      (int Orientation, int LayerIndex);
int GUI_SetOrientationExCached(int Orientation, int LayerIndex, const GUI_ORIENTATION_API * pAPI);




 
typedef signed long GUI_MEASDEV_Handle;

GUI_MEASDEV_Handle GUI_MEASDEV_Create (void);
void               GUI_MEASDEV_Delete (GUI_MEASDEV_Handle hMemDev);
void               GUI_MEASDEV_Select (GUI_MEASDEV_Handle hMem);
void               GUI_MEASDEV_GetRect(GUI_MEASDEV_Handle hMem, GUI_RECT * pRect);
void               GUI_MEASDEV_ClearRect(GUI_MEASDEV_Handle hMem);




 
void GUI_RotatePolygon (GUI_POINT * pDest, const GUI_POINT * pSrc, int NumPoints, float Angle);
void GUI_MagnifyPolygon(GUI_POINT * pDest, const GUI_POINT * pSrc, int NumPoints, int Mag);
void GUI_EnlargePolygon(GUI_POINT * pDest, const GUI_POINT * pSrc, int NumPoints, int Len);




 






int GUI_CreateBitmapFromStreamIDX(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamRLE4(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamRLE8(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStream565(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamM565(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStream555(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamM555(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamA565(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamAM565(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamA555(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamAM555(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamRLE16(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamRLEM16(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStream24(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamAlpha(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamM8888I(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamRLEAlpha(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamRLE32(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStream444_12(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamM444_12(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStream444_12_1(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamM444_12_1(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStream444_16(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
int GUI_CreateBitmapFromStreamM444_16(GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);

int  GUI_CreateBitmapFromStream   (GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const void * p);
void GUI_DrawStreamedBitmap       (const void * p, int x, int y);
void GUI_DrawStreamedBitmapAuto   (const void * p, int x, int y);
int  GUI_DrawStreamedBitmapEx     (GUI_GET_DATA_FUNC * pfGetData, const void * p, int x, int y);
int  GUI_DrawStreamedBitmapExAuto (GUI_GET_DATA_FUNC * pfGetData, const void * p, int x, int y);
int  GUI_DrawStreamedBitmap555Ex  (GUI_GET_DATA_FUNC * pfGetData, const void * p, int x, int y);
int  GUI_DrawStreamedBitmapM555Ex (GUI_GET_DATA_FUNC * pfGetData, const void * p, int x, int y);
int  GUI_DrawStreamedBitmap565Ex  (GUI_GET_DATA_FUNC * pfGetData, const void * p, int x, int y);
int  GUI_DrawStreamedBitmapM565Ex (GUI_GET_DATA_FUNC * pfGetData, const void * p, int x, int y);
int  GUI_DrawStreamedBitmapA555Ex (GUI_GET_DATA_FUNC * pfGetData, const void * p, int x, int y);
int  GUI_DrawStreamedBitmapAM555Ex(GUI_GET_DATA_FUNC * pfGetData, const void * p, int x, int y);
int  GUI_DrawStreamedBitmapA565Ex (GUI_GET_DATA_FUNC * pfGetData, const void * p, int x, int y);
int  GUI_DrawStreamedBitmapAM565Ex(GUI_GET_DATA_FUNC * pfGetData, const void * p, int x, int y);
int  GUI_DrawStreamedBitmap24Ex   (GUI_GET_DATA_FUNC * pfGetData, const void * p, int x, int y);
void GUI_GetStreamedBitmapInfo    (const void * p, GUI_BITMAPSTREAM_INFO * pInfo);
int  GUI_GetStreamedBitmapInfoEx  (GUI_GET_DATA_FUNC * pfGetData, const void * p, GUI_BITMAPSTREAM_INFO * pInfo);
void GUI_SetStreamedBitmapHook    (GUI_BITMAPSTREAM_CALLBACK pfStreamedBitmapHook);

void LCD__RLE4_SetFunc (GUI_GET_DATA_FUNC * pfGetData, void * pVoid, unsigned long Off, const LCD_LOGPALETTE * pLogPal);
void LCD__RLE8_SetFunc (GUI_GET_DATA_FUNC * pfGetData, void * pVoid, unsigned long Off, const LCD_LOGPALETTE * pLogPal);
void LCD__RLE16_SetFunc(GUI_GET_DATA_FUNC * pfGetData, void * pVoid, unsigned long Off);
void LCD__RLE32_SetFunc(GUI_GET_DATA_FUNC * pfGetData, void * pVoid, unsigned long Off);




 
void GUI_BMP_Serialize     (GUI_CALLBACK_VOID_U8_P * pfSerialize, void * p);
void GUI_BMP_SerializeEx   (GUI_CALLBACK_VOID_U8_P * pfSerialize, int x0, int y0, int xSize, int ySize, void * p);
void GUI_BMP_SerializeExBpp(GUI_CALLBACK_VOID_U8_P * pfSerialize, int x0, int y0, int xSize, int ySize, void * p, int BitsPerPixel);




 
void           GUI_Delay  (int Period);
int GUI_GetTime(void);
int            GUI_Exec(void);          
int            GUI_Exec1(void);         




 
int     GUI_MessageBox   (const char * sMessage, const char * sCaption, int Flags);









 



typedef signed long GUI_TIMER_HANDLE;

typedef struct {
  int   Time;
  unsigned long              Context;
  GUI_TIMER_HANDLE hTimer;
} GUI_TIMER_MESSAGE;

typedef void GUI_TIMER_CALLBACK(  GUI_TIMER_MESSAGE* pTM);

GUI_TIMER_HANDLE GUI_TIMER_Create   (GUI_TIMER_CALLBACK * cb, int Time, unsigned long Context, unsigned short Flags);
void             GUI_TIMER_Delete   (GUI_TIMER_HANDLE hObj);

 
int GUI_TIMER_GetPeriod(GUI_TIMER_HANDLE hObj);
void           GUI_TIMER_SetPeriod(GUI_TIMER_HANDLE hObj, int Period);
void           GUI_TIMER_SetTime  (GUI_TIMER_HANDLE hObj, int Period);
void           GUI_TIMER_SetDelay (GUI_TIMER_HANDLE hObj, int Delay);
void           GUI_TIMER_Restart  (GUI_TIMER_HANDLE hObj);
int            GUI_TIMER_GetFlag  (GUI_TIMER_HANDLE hObj, int Flag);  
int            GUI_TIMER_Exec     (void);




 



void GUI_AA_DisableHiRes     (void);
void GUI_AA_EnableHiRes      (void);
int  GUI_AA_GetFactor        (void);
void GUI_AA_SetFactor        (int Factor);
void GUI_AA_DrawArc          (int x0, int y0, int rx, int ry, int a0, int a1);
void GUI_AA_DrawLine         (int x0, int y0, int x1, int y1);
void GUI_AA_DrawPolyOutline  (const GUI_POINT * pSrc, int NumPoints, int Thickness, int x, int y);
void GUI_AA_DrawPolyOutlineEx(const GUI_POINT * pSrc, int NumPoints, int Thickness, int x, int y, GUI_POINT * pBuffer);
void GUI_AA_DrawRoundedRect  (int x0, int y0, int x1, int y1, int r);
void GUI_AA_DrawRoundedRectEx(GUI_RECT * pRect, int r);
void GUI_AA_FillCircle       (int x0, int y0, int r);
void GUI_AA_FillEllipse      (int x0, int y0, int rx, int ry);
void GUI_AA_FillPolygon      (GUI_POINT * pPoints, int NumPoints, int x0, int y0);
void GUI_AA_FillRoundedRect  (int x0, int y0, int x1, int y1, int r);
void GUI_AA_FillRoundedRectEx(GUI_RECT * pRect, int r);

int  GUI_AA_SetDrawMode      (int Mode);
void GUI_AA_SetpfDrawCharAA4 (int (* pfDrawChar)(int LayerIndex, int x, int y, unsigned char const * p, int xSize, int ySize, int BytesPerLine));






 
 
void GUI_StoreKeyMsg(int Key, int Pressed);
void GUI_SendKeyMsg (int Key, int Pressed);
int  GUI_PollKeyMsg (void);
void GUI_GetKeyState(GUI_KEY_STATE * pState);

void GUI_KEY__SetHook(void (* pfHook)(const GUI_KEY_STATE *));

 
int  GUI_GetKey(void);
int  GUI_WaitKey(void);
void GUI_StoreKey(int c);
void GUI_ClearKeyBuffer(void);




 
void GUI_WaitEvent            (void);
void GUI_SignalEvent          (void);
void GUI_SetSignalEventFunc   (GUI_SIGNAL_EVENT_FUNC     pfSignalEvent);
void GUI_SetWaitEventFunc     (GUI_WAIT_EVENT_FUNC       pfWaitEvent);
void GUI_SetWaitEventTimedFunc(GUI_WAIT_EVENT_TIMED_FUNC pfWaitEventTimed);




 
void GUI_JOYSTICK_StoreState(const GUI_PID_STATE * pState);




 
void GUI_PID_StoreState     (const GUI_PID_STATE * pState);
int  GUI_PID_GetState       (      GUI_PID_STATE * pState);
void GUI_PID_GetCurrentState(      GUI_PID_STATE * pState);
int  GUI_PID_IsEmpty        (void);
int  GUI_PID_IsPressed      (void);
void GUI_PID__SetHook       (void (* pfHook)(const GUI_PID_STATE *));




 
int  GUI_MOUSE_GetState  (      GUI_PID_STATE * pState);
void GUI_MOUSE_StoreState(const GUI_PID_STATE * pState);




 
int  GUI_TOUCH_GetLayer     (void);
int  GUI_TOUCH_GetState     (GUI_PID_STATE * pState);
void GUI_TOUCH_GetUnstable  (int * px, int * py);   
void GUI_TOUCH_SetLayer     (int Layer);
void GUI_TOUCH_StoreState   (int x, int y);
void GUI_TOUCH_StoreStateEx (const GUI_PID_STATE * pState);
void GUI_TOUCH_StoreUnstable(int x, int y);




 
void GUI_MOUSE_DRIVER_PS2_Init(void);                
void GUI_MOUSE_DRIVER_PS2_OnRx(unsigned char Data);




 
int  GUI_TOUCH_CalcCoefficients (int NumPoints, int * pxRef, int * pyRef, int * pxSample, int * pySample, int xSize, int ySize);
int  GUI_TOUCH_Calibrate        (int Coord, int Log0, int Log1, int Phys0, int Phys1);
int  GUI_TOUCH_CalibratePoint   (int * px, int * py);
void GUI_TOUCH_EnableCalibration(int OnOff);
void GUI_TOUCH_Exec             (void);
int  GUI_TOUCH_GetxPhys         (void);     
int  GUI_TOUCH_GetyPhys         (void);     
void GUI_TOUCH_SetCalibration   (int (* pFunc)(int *, int *));  
void GUI_TOUCH_SetOrientation   (unsigned Orientation);
int  GUI_TOUCH_TransformPoint   (int * px, int * py);           








 
void GUI_TOUCH_X_ActivateX(void);
void GUI_TOUCH_X_ActivateY(void);
void GUI_TOUCH_X_Disable  (void);
int  GUI_TOUCH_X_MeasureX (void);
int  GUI_TOUCH_X_MeasureY (void);














 



void GUI_X_Config(void);
void GUI_X_Init  (void);




int GUI_X_GetTime(void);
void           GUI_X_Delay  (int Period);




void GUI_X_Unlock   (void);
void GUI_X_Lock     (void);
unsigned long  GUI_X_GetTaskId(void);
void GUI_X_InitOS   (void);




void GUI_X_ExecIdle      (void);
void GUI_X_WaitEvent     (void);
void GUI_X_WaitEventTimed(int Period);
void GUI_X_SignalEvent   (void);




void GUI_X_Log     (const char * s);
void GUI_X_Warn    (const char * s);
void GUI_X_ErrorOut(const char * s);




 
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsRLE4;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsRLE4Ex;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsRLE8;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsRLE8Ex;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsRLE16;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsRLE16Ex;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsRLEM16;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsRLEM16Ex;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsRLE32;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsRLE32Ex;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsRLEAlpha;

extern const GUI_BITMAP_METHODS GUI_BitmapMethods444_12;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsM444_12;
extern const GUI_BITMAP_METHODS GUI_BitmapMethods444_12_1;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsM444_12_1;
extern const GUI_BITMAP_METHODS GUI_BitmapMethods444_16;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsM444_16;
extern const GUI_BITMAP_METHODS GUI_BitmapMethods555;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsM555;
extern const GUI_BITMAP_METHODS GUI_BitmapMethods565;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsM565;
extern const GUI_BITMAP_METHODS GUI_BitmapMethods24;
extern const GUI_BITMAP_METHODS GUI_BitmapMethods888;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsM888;
extern const GUI_BITMAP_METHODS GUI_BitmapMethods8888;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsM8888I;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsA565;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsAM565;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsA555;
extern const GUI_BITMAP_METHODS GUI_BitmapMethodsAM555;




#line 1504 "..\\STemWin\\inc\\GUI.h"

#line 1524 "..\\STemWin\\inc\\GUI.h"

extern const tGUI_SIF_APIList GUI_SIF_APIList_Prop;
extern const tGUI_SIF_APIList GUI_SIF_APIList_Prop_Ext;
extern const tGUI_SIF_APIList GUI_SIF_APIList_Prop_Frm;
extern const tGUI_SIF_APIList GUI_SIF_APIList_Prop_AA2;
extern const tGUI_SIF_APIList GUI_SIF_APIList_Prop_AA4;
extern const tGUI_SIF_APIList GUI_SIF_APIList_Prop_AA2_EXT;
extern const tGUI_SIF_APIList GUI_SIF_APIList_Prop_AA4_EXT;

extern const tGUI_XBF_APIList GUI_XBF_APIList_Prop;
extern const tGUI_XBF_APIList GUI_XBF_APIList_Prop_Ext;
extern const tGUI_XBF_APIList GUI_XBF_APIList_Prop_Frm;
extern const tGUI_XBF_APIList GUI_XBF_APIList_Prop_AA2_Ext;
extern const tGUI_XBF_APIList GUI_XBF_APIList_Prop_AA4_Ext;









 
#line 1566 "..\\STemWin\\inc\\GUI.h"












 
#line 1587 "..\\STemWin\\inc\\GUI.h"




#line 1601 "..\\STemWin\\inc\\GUI.h"

#line 1612 "..\\STemWin\\inc\\GUI.h"

#line 1623 "..\\STemWin\\inc\\GUI.h"

#line 1634 "..\\STemWin\\inc\\GUI.h"






#line 1648 "..\\STemWin\\inc\\GUI.h"

#line 1659 "..\\STemWin\\inc\\GUI.h"

#line 1670 "..\\STemWin\\inc\\GUI.h"














































#line 1726 "..\\STemWin\\inc\\GUI.h"

#line 1737 "..\\STemWin\\inc\\GUI.h"



#line 1750 "..\\STemWin\\inc\\GUI.h"










 








 








 









 







 
#line 1864 "..\\STemWin\\inc\\GUI.h"

#line 1878 "..\\STemWin\\inc\\GUI.h"



#line 1888 "..\\STemWin\\inc\\GUI.h"











 










extern T_GUI_MTOUCH_STOREEVENT GUI_MTOUCH__pStoreEvent;




void GUI_MTOUCH_Enable          (int OnOff);
int  GUI_MTOUCH_GetEvent        (GUI_MTOUCH_EVENT * pEvent);
int  GUI_MTOUCH_GetTouchInput   (GUI_MTOUCH_EVENT * pEvent, GUI_MTOUCH_INPUT * pBuffer, unsigned Index);
int  GUI_MTOUCH_IsEmpty         (void);
void GUI_MTOUCH_SetOrientation  (int Orientation);
void GUI_MTOUCH_SetOrientationEx(int Orientation, int LayerIndex);
void GUI_MTOUCH_StoreEvent      (GUI_MTOUCH_EVENT * pEvent, GUI_MTOUCH_INPUT * pInput);




 






 



extern const GUI_FONT GUI_Font8_ASCII,        GUI_Font8_1;
extern const GUI_FONT GUI_Font10S_ASCII,      GUI_Font10S_1;
extern const GUI_FONT GUI_Font10_ASCII,       GUI_Font10_1;
extern const GUI_FONT GUI_Font13_ASCII,       GUI_Font13_1;
extern const GUI_FONT GUI_Font13B_ASCII,      GUI_Font13B_1;
extern const GUI_FONT GUI_Font13H_ASCII,      GUI_Font13H_1;
extern const GUI_FONT GUI_Font13HB_ASCII,     GUI_Font13HB_1;
extern const GUI_FONT GUI_Font16_ASCII,       GUI_Font16_1,       GUI_Font16_HK,    GUI_Font16_1HK;
extern const GUI_FONT GUI_Font16B_ASCII,      GUI_Font16B_1;
extern const GUI_FONT GUI_Font20_ASCII,       GUI_Font20_1;
extern const GUI_FONT GUI_Font20B_ASCII,      GUI_Font20B_1;
extern const GUI_FONT GUI_Font24_ASCII,       GUI_Font24_1;
extern const GUI_FONT GUI_Font24B_ASCII,      GUI_Font24B_1;
extern const GUI_FONT GUI_Font32_ASCII,       GUI_Font32_1;
extern const GUI_FONT GUI_Font32B_ASCII,      GUI_Font32B_1;




extern const GUI_FONT GUI_Font20F_ASCII;




extern const GUI_FONT GUI_Font4x6;
extern const GUI_FONT GUI_Font6x8,            GUI_Font6x9;
extern const GUI_FONT GUI_Font6x8_ASCII,      GUI_Font6x8_1;
extern const GUI_FONT GUI_Font8x8,            GUI_Font8x9;
extern const GUI_FONT GUI_Font8x8_ASCII,      GUI_Font8x8_1;
extern const GUI_FONT GUI_Font8x10_ASCII;
extern const GUI_FONT GUI_Font8x12_ASCII;
extern const GUI_FONT GUI_Font8x13_ASCII,     GUI_Font8x13_1;
extern const GUI_FONT GUI_Font8x15B_ASCII,    GUI_Font8x15B_1;
extern const GUI_FONT GUI_Font8x16,           GUI_Font8x17,       GUI_Font8x18;
extern const GUI_FONT GUI_Font8x16x1x2,       GUI_Font8x16x2x2,   GUI_Font8x16x3x3;
extern const GUI_FONT GUI_Font8x16_ASCII,     GUI_Font8x16_1;




extern const GUI_FONT GUI_FontD24x32;
extern const GUI_FONT GUI_FontD32;
extern const GUI_FONT GUI_FontD36x48;
extern const GUI_FONT GUI_FontD48;
extern const GUI_FONT GUI_FontD48x64;
extern const GUI_FONT GUI_FontD64;
extern const GUI_FONT GUI_FontD60x80;
extern const GUI_FONT GUI_FontD80;




extern const GUI_FONT GUI_FontComic18B_ASCII, GUI_FontComic18B_1;
extern const GUI_FONT GUI_FontComic24B_ASCII, GUI_FontComic24B_1;




 



#line 2031 "..\\STemWin\\inc\\GUI.h"









#line 2063 "..\\STemWin\\inc\\GUI.h"




#line 2075 "..\\STemWin\\inc\\GUI.h"
















 
#line 2100 "..\\STemWin\\inc\\GUI.h"

#line 2109 "..\\STemWin\\inc\\GUI.h"

 






 






 









 
#line 2146 "..\\STemWin\\inc\\GUI.h"




 
#line 2407 "..\\STemWin\\inc\\GUI.h"




 
#line 2423 "..\\STemWin\\inc\\GUI.h"



 
#line 20 "..\\User\\emWinTask\\MainTask.h"
#line 1 "..\\STemWin\\inc\\DIALOG.h"
































 


















 
  



#line 1 "..\\STemWin\\inc\\WM.h"
































 


















 
  




#line 59 "..\\STemWin\\inc\\WM.h"
#line 60 "..\\STemWin\\inc\\WM.h"
#line 1 "..\\STemWin\\inc\\WM_GUI.h"
































 


















 
  







int       WM__InitIVRSearch(const GUI_RECT* pMaxRect);
int       WM__GetNextIVR   (void);
int       WM__GetOrgX_AA(void);
int       WM__GetOrgY_AA(void);










#line 84 "..\\STemWin\\inc\\WM_GUI.h"








 
#line 61 "..\\STemWin\\inc\\WM.h"
#line 62 "..\\STemWin\\inc\\WM.h"





 




 





 
#line 86 "..\\STemWin\\inc\\WM.h"




 






 








 




 




#line 123 "..\\STemWin\\inc\\WM.h"




 








 
typedef struct WM_WINDOW_INFO WM_WINDOW_INFO;

struct WM_WINDOW_INFO {
  signed long hWin;
  signed long hParent;
  signed long hFirstChild;
  signed long hNext;
  GUI_RECT Rect;
  unsigned long      Status;
  unsigned long      DebugId;
  WM_WINDOW_INFO * pNext;
};

typedef struct {
  int Key, PressedCnt;
} WM_KEY_INFO;

typedef struct {
  int NumItems, v, PageSize;
} WM_SCROLL_STATE;

typedef struct {
  int Done;
  int ReturnValue;
} WM_DIALOG_STATUS;

typedef struct {
  int x,y;
  unsigned char  State;
  unsigned char  StatePrev;
} WM_PID_STATE_CHANGED_INFO;

typedef struct {
  int Cmd;
  int dx, dy, da;
  int xPos, yPos;
  int Period;
  int SnapX;
  int SnapY;
  int FinalMove;
  unsigned long Flags;
  GUI_PID_STATE * pState;
  signed long hContext;
} WM_MOTION_INFO;

typedef struct {
  signed long       FactorMin;   
  signed long       FactorMax;   
  unsigned long       xSize;       
  unsigned long       ySize;       
  unsigned long       xSizeParent; 
  unsigned long       ySizeParent; 
  signed long       Factor0;     
  int       xPos0;       
  int       yPos0;       
  GUI_POINT Center0;     
} WM_ZOOM_INFO;

typedef struct {
  int            Flags;     
  GUI_POINT      Point;     
  GUI_POINT      Center;    
  signed long            Angle;     
  signed long            Factor;    
  WM_ZOOM_INFO * pZoomInfo; 
} WM_GESTURE_INFO;

typedef struct {
  int dx, dy;
} WM_MOVE_INFO;




 











 







































#line 271 "..\\STemWin\\inc\\WM.h"

#line 279 "..\\STemWin\\inc\\WM.h"

















 








 








 
#line 325 "..\\STemWin\\inc\\WM.h"







 









 










 
#line 361 "..\\STemWin\\inc\\WM.h"

 




 




























 
typedef struct WM_Obj     WM_Obj;
typedef struct WM_MESSAGE WM_MESSAGE;

typedef void WM_CALLBACK( WM_MESSAGE * pMsg);

struct WM_MESSAGE {
  int MsgId;             
  GUI_HWIN hWin;          
  GUI_HWIN hWinSrc;       
  union {
    const void * p;             
    int v;
    GUI_COLOR Color;
  } Data;
};

struct WM_Obj {
  GUI_RECT Rect;         
  GUI_RECT InvalidRect;  
  WM_CALLBACK* cb;       
  GUI_HWIN hNextLin;      
  GUI_HWIN hParent;
  GUI_HWIN hFirstChild;
  GUI_HWIN hNext;

    GUI_MEMDEV_Handle hMem;  

  unsigned long Status;            



};

typedef void WM_tfPollPID(void);
typedef void WM_tfForEach(GUI_HWIN hWin, void * pData);

typedef void (* WM_tfInvalidateParent)  (const GUI_RECT * pInvalidRect, GUI_HWIN hParent, GUI_HWIN hStop);
typedef void (* WM_tfInvalidateDrawFunc)(GUI_HWIN hWin);
typedef void (* WM_tfPaint1Func)        (GUI_HWIN hWin);

typedef struct {
  signed long  hTimer;
  GUI_HWIN  hWin;
  int      UserId;
} WM_TIMER_OBJ;




 
void WM_Activate  (void);
void WM_Deactivate(void);
void WM_Init      (void);
int  WM_Exec      (void);     
unsigned long  WM_SetCreateFlags(unsigned long Flags);
WM_tfPollPID * WM_SetpfPollPID(WM_tfPollPID * pf);




 
void    WM_AttachWindow              (GUI_HWIN hWin, GUI_HWIN hParent);
void    WM_AttachWindowAt            (GUI_HWIN hWin, GUI_HWIN hParent, int x, int y);
int     WM_CheckScrollPos            (WM_SCROLL_STATE * pScrollState, int Pos, int LowerDist, int UpperDist);  
void    WM_ClrHasTrans               (GUI_HWIN hWin);
GUI_HWIN WM_CreateWindow              (int x0, int y0, int xSize, int ySize, unsigned long Style, WM_CALLBACK * cb, int NumExtraBytes);
GUI_HWIN WM_CreateWindowAsChild       (int x0, int y0, int xSize, int ySize, GUI_HWIN hWinParent, unsigned long Style, WM_CALLBACK* cb, int NumExtraBytes);
void    WM_DeleteWindow              (GUI_HWIN hWin);
void    WM_DetachWindow              (GUI_HWIN hWin);
void    WM_EnableGestures            (GUI_HWIN hWin, int OnOff);
int     WM_GetHasTrans               (GUI_HWIN hWin);
GUI_HWIN WM_GetFocussedWindow         (void);
int     WM_GetInvalidRect            (GUI_HWIN hWin, GUI_RECT * pRect);
int     WM_GetStayOnTop              (GUI_HWIN hWin);
void    WM_HideWindow                (GUI_HWIN hWin);
void    WM_InvalidateArea            (const GUI_RECT * pRect);
void    WM_InvalidateRect            (GUI_HWIN hWin, const GUI_RECT * pRect);
void    WM_InvalidateWindow          (GUI_HWIN hWin);
void    WM_InvalidateWindowAndDescsEx(GUI_HWIN hWin, const GUI_RECT * pInvalidRect, unsigned short Flags);
void    WM_InvalidateWindowAndDescs  (GUI_HWIN hWin);     
int     WM_IsEnabled                 (GUI_HWIN hObj);
char    WM_IsCompletelyCovered       (GUI_HWIN hWin);     
char    WM_IsCompletelyVisible       (GUI_HWIN hWin);     
int     WM_IsFocussable              (GUI_HWIN hWin);
int     WM_IsVisible                 (GUI_HWIN hWin);
int     WM_IsWindow                  (GUI_HWIN hWin);     
void    WM_SetAnchor                 (GUI_HWIN hWin, unsigned short AnchorFlags);
void    WM_SetHasTrans               (GUI_HWIN hWin);
void    WM_SetId                     (GUI_HWIN hObj, int Id);
void    WM_SetStayOnTop              (GUI_HWIN hWin, int OnOff);
void    WM_SetTransState             (GUI_HWIN hWin, unsigned State);
void    WM_ShowWindow                (GUI_HWIN hWin);
void    WM_ValidateRect              (GUI_HWIN hWin, const GUI_RECT * pRect);
void    WM_ValidateWindow            (GUI_HWIN hWin);

 
void WM_GESTURE_Enable  (int OnOff);
int  WM_GESTURE_EnableEx(int OnOff, int MaxFactor);
void WM_GESTURE_Exec    (void);
signed long  WM_GESTURE_SetThresholdAngle(signed long ThresholdAngle);
signed long  WM_GESTURE_SetThresholdDist (signed long ThresholdDist);

 
void     WM_MOTION_Enable          (int OnOff);
void     WM_MOTION_SetMovement     (GUI_HWIN hWin, int Axis, signed long Velocity, signed long Dist);
void     WM_MOTION_SetMotion       (GUI_HWIN hWin, int Axis, signed long Velocity, signed long Deceleration);
void     WM_MOTION_SetMoveable     (GUI_HWIN hWin, unsigned long Flags, int OnOff);
void     WM_MOTION_SetDeceleration (GUI_HWIN hWin, int Axis, signed long Deceleration);
unsigned WM_MOTION_SetDefaultPeriod(unsigned Period);
void     WM_MOTION_SetSpeed        (GUI_HWIN hWin, int Axis, signed long Velocity);

 
signed long WM_MOTION__CreateContext(void);
void    WM_MOTION__DeleteContext(signed long hContext);

 
void     WM__SetMotionCallback (void(* cbMotion) (GUI_PID_STATE * pState, void * p));

 






  int               GUI_MEMDEV_BlendWinBk       (GUI_HWIN hWin, int Period, unsigned long BlendColor, unsigned char BlendIntens);
  int               GUI_MEMDEV_BlurAndBlendWinBk(GUI_HWIN hWin, int Period, unsigned char BlurDepth, unsigned long BlendColor, unsigned char BlendIntens);
  int               GUI_MEMDEV_BlurWinBk        (GUI_HWIN hWin, int Period, unsigned char BlurDepth);
  void              GUI_MEMDEV_CreateStatic     (GUI_HWIN hWin);
  GUI_MEMDEV_Handle GUI_MEMDEV_CreateWindowDevice(GUI_HWIN hWin);
  int               GUI_MEMDEV_FadeInWindow     (GUI_HWIN hWin, int Period);
  int               GUI_MEMDEV_FadeOutWindow    (GUI_HWIN hWin, int Period);
  GUI_MEMDEV_Handle GUI_MEMDEV_GetStaticDevice  (GUI_HWIN hWin);
  GUI_MEMDEV_Handle GUI_MEMDEV_GetWindowDevice  (GUI_HWIN hWin);
  int               GUI_MEMDEV_MoveInWindow     (GUI_HWIN hWin, int x, int y, int a180, int Period);
  int               GUI_MEMDEV_MoveOutWindow    (GUI_HWIN hWin, int x, int y, int a180, int Period);
  void              GUI_MEMDEV_Paint1Static     (GUI_HWIN hWin);                                      
  int               GUI_MEMDEV_ShiftInWindow    (GUI_HWIN hWin, int Period, int Direction);
  int               GUI_MEMDEV_ShiftOutWindow   (GUI_HWIN hWin, int Period, int Direction);
  int               GUI_MEMDEV_SwapWindow       (GUI_HWIN hWin, int Period, int Edge);

  void              GUI_MEMDEV__CreateStatic    (GUI_HWIN hWin);


 
void WM_MoveWindow                (GUI_HWIN hWin, int dx, int dy);
void WM_ResizeWindow              (GUI_HWIN hWin, int dx, int dy);
void WM_MoveTo                    (GUI_HWIN hWin, int x, int y);
void WM_MoveChildTo               (GUI_HWIN hWin, int x, int y);
void WM_SetSize                   (GUI_HWIN hWin, int XSize, int YSize);
void WM_SetWindowPos              (GUI_HWIN hWin, int xPos, int yPos, int xSize, int ySize);
int  WM_SetXSize                  (GUI_HWIN hWin, int xSize);
int  WM_SetYSize                  (GUI_HWIN hWin, int ySize);
int  WM_SetScrollbarH             (GUI_HWIN hWin, int OnOff);  
int  WM_SetScrollbarV             (GUI_HWIN hWin, int OnOff);  

 








typedef signed long WM_TOOLTIP_HANDLE;

typedef struct {
  int          Id;
  const char * pText;
} TOOLTIP_INFO;

int               WM_TOOLTIP_AddTool         (WM_TOOLTIP_HANDLE hToolTip, GUI_HWIN hTool, const char * pText);
WM_TOOLTIP_HANDLE WM_TOOLTIP_Create          (GUI_HWIN hDlg, const TOOLTIP_INFO * pInfo, unsigned NumItems);
void              WM_TOOLTIP_Delete          (WM_TOOLTIP_HANDLE hToolTip);
GUI_COLOR         WM_TOOLTIP_SetDefaultColor (unsigned Index, GUI_COLOR Color);
const GUI_FONT *  WM_TOOLTIP_SetDefaultFont  (const GUI_FONT * pFont);
unsigned          WM_TOOLTIP_SetDefaultPeriod(unsigned Index, unsigned Period);

 
void WM__SetToolTipCallback(void(* cbToolTip)(GUI_PID_STATE * pState, GUI_HWIN));

 




  signed long WM_CreateTimer (GUI_HWIN hWin, int UserID, int Period, int Mode);  
  void    WM_DeleteTimer (signed long hTimer);  
  void    WM_RestartTimer(signed long hTimer, int Period);

int WM_GetTimerId(signed long hTimer);

 
int WM_GetNumWindows(void);
int WM_GetNumInvalidWindows(void);

 
void WM_CheckScrollBounds(WM_SCROLL_STATE * pScrollState);  
int  WM_GetScrollPosH    (GUI_HWIN hWin);
int  WM_GetScrollPosV    (GUI_HWIN hWin);
void WM_SetScrollPosH    (GUI_HWIN hWin, unsigned ScrollPos);
void WM_SetScrollPosV    (GUI_HWIN hWin, unsigned ScrollPos);
int  WM_SetScrollValue   (WM_SCROLL_STATE * pScrollState, int v);  

 
WM_CALLBACK * WM_SetCallback(GUI_HWIN hWin, WM_CALLBACK * cb);
WM_CALLBACK * WM_GetCallback(GUI_HWIN hWin);

 
void      WM_GetClientRect           (GUI_RECT * pRect);
void      WM_GetClientRectEx         (GUI_HWIN hWin, GUI_RECT * pRect);
void      WM_GetInsideRect           (GUI_RECT * pRect);
void      WM_GetInsideRectEx         (GUI_HWIN hWin, GUI_RECT * pRect);
void      WM_GetInsideRectExScrollbar(GUI_HWIN hWin, GUI_RECT * pRect);  
void      WM_GetWindowRect           (GUI_RECT * pRect);
void      WM_GetWindowRectEx         (GUI_HWIN hWin, GUI_RECT * pRect);
int       WM_GetOrgX                 (void);
int       WM_GetOrgY                 (void);
int       WM_GetWindowOrgX           (GUI_HWIN hWin);
int       WM_GetWindowOrgY           (GUI_HWIN hWin);
int       WM_GetWindowSizeX          (GUI_HWIN hWin);
int       WM_GetWindowSizeY          (GUI_HWIN hWin);
GUI_HWIN   WM_GetFirstChild           (GUI_HWIN hWin);
GUI_HWIN   WM_GetNextSibling          (GUI_HWIN hWin);
GUI_HWIN   WM_GetParent               (GUI_HWIN hWin);
GUI_HWIN   WM_GetPrevSibling          (GUI_HWIN hWin);
int       WM_GetId                   (GUI_HWIN hWin);
GUI_HWIN   WM_GetScrollbarV           (GUI_HWIN hWin);
GUI_HWIN   WM_GetScrollbarH           (GUI_HWIN hWin);
GUI_HWIN   WM_GetScrollPartner        (GUI_HWIN hWin);
GUI_HWIN   WM_GetClientWindow         (GUI_HWIN hObj);
GUI_COLOR WM_GetBkColor              (GUI_HWIN hObj);

 
void WM_BringToBottom(GUI_HWIN hWin);
void WM_BringToTop(GUI_HWIN hWin);

GUI_COLOR WM_SetDesktopColor  (GUI_COLOR Color);
GUI_COLOR WM_SetDesktopColorEx(GUI_COLOR Color, unsigned int LayerIndex);
void      WM_SetDesktopColors (GUI_COLOR Color);

 
GUI_HWIN WM_SelectWindow           (GUI_HWIN  hWin);
GUI_HWIN WM_GetActiveWindow        (void);
void    WM_Paint                  (GUI_HWIN hObj);
void    WM_Update                 (GUI_HWIN hWin);
void    WM_PaintWindowAndDescs    (GUI_HWIN hWin);
void    WM_UpdateWindowAndDescs   (GUI_HWIN hWin);

 
GUI_HWIN WM_GetDesktopWindow  (void);
GUI_HWIN WM_GetDesktopWindowEx(unsigned int LayerIndex);

 
const GUI_RECT * WM_SetUserClipRect(const GUI_RECT * pRect);
void             WM_SetDefault     (void);

 
void WM_EnableMemdev              (GUI_HWIN hWin);
void WM_DisableMemdev             (GUI_HWIN hWin);

 
int WM_MULTIBUF_Enable(int OnOff);

extern const GUI_MULTIBUF_API * WM_MULTIBUF__pAPI;

typedef void (* T_WM_EXEC_GESTURE)(void);

extern T_WM_EXEC_GESTURE WM__pExecGestures;

 
int WM_OnKey(int Key, int Pressed);
void WM_MakeModal(GUI_HWIN hWin);
int WM_SetModalLayer(int LayerIndex);
int WM_GetModalLayer(void);







 
void      WM_NotifyParent         (GUI_HWIN hWin, int Notification);
void      WM_SendMessage          (GUI_HWIN hWin, WM_MESSAGE * p);
void      WM_SendMessageNoPara    (GUI_HWIN hWin, int MsgId);              
void      WM_DefaultProc          (WM_MESSAGE * pMsg);
int       WM_BroadcastMessage     (WM_MESSAGE * pMsg);
void      WM_SetScrollState       (GUI_HWIN hWin, const WM_SCROLL_STATE * pState);
void      WM_SetEnableState       (GUI_HWIN hItem, int State);
void      WM_SendToParent         (GUI_HWIN hWin, WM_MESSAGE * pMsg);
int       WM_HasFocus             (GUI_HWIN hWin);
int       WM_SetFocus             (GUI_HWIN hWin);
GUI_HWIN   WM_SetFocusOnNextChild  (GUI_HWIN hParent);      
GUI_HWIN   WM_SetFocusOnPrevChild  (GUI_HWIN hParent);      
GUI_HWIN   WM_GetDialogItem        (GUI_HWIN hWin, int Id);
void      WM_EnableWindow         (GUI_HWIN hWin);
void      WM_DisableWindow        (GUI_HWIN hWin);
void      WM_GetScrollState       (GUI_HWIN hObj, WM_SCROLL_STATE * pScrollState);




 
int       WM_GetUserData   (GUI_HWIN hWin, void * pDest, int SizeOfBuffer);
int       WM_SetUserData   (GUI_HWIN hWin, const void * pSrc, int SizeOfBuffer);
int       WM__GetUserDataEx(GUI_HWIN hWin, void * pDest, int NumBytes, int SizeOfObject);
int       WM__SetUserDataEx(GUI_HWIN hWin, const void * pSrc, int NumBytes, int SizeOfObject);




 
int  WM_HasCaptured   (GUI_HWIN hWin);
void WM_SetCapture    (GUI_HWIN hObj, int AutoRelease);
void WM_SetCaptureMove(GUI_HWIN hWin, const GUI_PID_STATE * pState, int MinVisibility, int LimitTop);  
void WM_ReleaseCapture(void);




 
int       WM_HandlePID      (void);
GUI_HWIN   WM_Screen2hWin    (int x, int y);
GUI_HWIN   WM_Screen2hWinEx  (GUI_HWIN hStop, int x, int y);
void      WM_ForEachDesc    (GUI_HWIN hWin, WM_tfForEach * pcb, void * pData);
void      WM_SetScreenSize  (int xSize, int ySize);
int       WM_PollSimMsg     (void);
int       WM_GetWindowInfo  (WM_WINDOW_INFO * pInfo, int FirstWindow);




 







 












#line 765 "..\\STemWin\\inc\\WM.h"

















 
#line 58 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\BUTTON.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\BUTTON.h"
#line 1 "..\\STemWin\\inc\\DIALOG_Intern.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\DIALOG_Intern.h"












 
typedef struct  GUI_WIDGET_CREATE_INFO_struct GUI_WIDGET_CREATE_INFO;
typedef GUI_HWIN GUI_WIDGET_CREATE_FUNC        (const GUI_WIDGET_CREATE_INFO * pCreate, GUI_HWIN hWin, int x0, int y0, WM_CALLBACK * cb);






 
struct GUI_WIDGET_CREATE_INFO_struct {
  GUI_WIDGET_CREATE_FUNC * pfCreateIndirect;
  const char             * pName;            
  signed short                      Id;               
  signed short                      x0;               
  signed short                      y0;               
  signed short                      xSize;            
  signed short                      ySize;            
  unsigned short                      Flags;            
  signed long                      Para;             
  unsigned long                      NumExtraBytes;    
};






 
GUI_HWIN            GUI_CreateDialogBox   (const GUI_WIDGET_CREATE_INFO * paWidget, int NumWidgets, WM_CALLBACK * cb, GUI_HWIN hParent, int x0, int y0);
void               GUI_EndDialog         (GUI_HWIN hWin, int r);
int                GUI_ExecDialogBox     (const GUI_WIDGET_CREATE_INFO * paWidget, int NumWidgets, WM_CALLBACK * cb, GUI_HWIN hParent, int x0, int y0);
int                GUI_ExecCreatedDialog (GUI_HWIN hDialog);
WM_DIALOG_STATUS * GUI_GetDialogStatusPtr(GUI_HWIN hDialog);                                    
void               GUI_SetDialogStatusPtr(GUI_HWIN hDialog, WM_DIALOG_STATUS * pDialogStatus);  




 
LCD_COLOR          DIALOG_GetBkColor(void);
LCD_COLOR          DIALOG_SetBkColor(LCD_COLOR BkColor);








 
#line 59 "..\\STemWin\\inc\\BUTTON.h"
#line 1 "..\\STemWin\\inc\\WIDGET.h"
































 


















 
  







#line 1 "..\\STemWin\\inc\\WM_Intern.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\WM_Intern.h"
#line 1 "..\\STemWin\\inc\\GUI_Private.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\GUI_Private.h"
#line 1 "..\\STemWin\\inc\\LCD_Protected.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\LCD_Protected.h"








 
typedef struct {
  LCD_COLOR * paColor;
  signed short         NumEntries;
} LCD_LUT_INFO;

typedef struct {
  tLCDDEV_DrawPixel  * pfDrawPixel;
  tLCDDEV_DrawHLine  * pfDrawHLine;
  tLCDDEV_DrawVLine  * pfDrawVLine;
  tLCDDEV_FillRect   * pfFillRect;
  tLCDDEV_DrawBitmap * pfDrawBitmap;
} LCD_API_LIST;




 
extern const unsigned char LCD_aMirror[256];
extern unsigned long * LCD__aConvTable;




 
void LCD_UpdateColorIndices   (void);
int  LCD_PassingBitmapsAllowed(void);
void LCD_EnableCursor         (int OnOff);
void LCD_SelectLCD            (void);

void LCD_DrawBitmap(int x0,    int y0,
                    int xsize, int ysize,
                    int xMul,  int yMul,
                    int BitsPerPixel,
                    int BytesPerLine,
                    const unsigned char * pPixel,
                    const unsigned long * pTrans);

void LCD__DrawBitmap_1bpp(int x0,    int y0,
                          int xsize, int ysize,
                          int xMul,  int yMul,
                          int BitsPerPixel,
                          int BytesPerLine,
                          const unsigned char * pPixel,
                          const unsigned long * pTrans,
                          int OffData);




 
tLCDDEV_Index2Color LCD_Index2Color_444_12;
tLCDDEV_Index2Color LCD_Index2Color_M444_12;
tLCDDEV_Index2Color LCD_Index2Color_444_12_1;
tLCDDEV_Index2Color LCD_Index2Color_M444_12_1;
tLCDDEV_Index2Color LCD_Index2Color_444_16;
tLCDDEV_Index2Color LCD_Index2Color_M444_16;
tLCDDEV_Index2Color LCD_Index2Color_555;
tLCDDEV_Index2Color LCD_Index2Color_565;
tLCDDEV_Index2Color LCD_Index2Color_8666;
tLCDDEV_Index2Color LCD_Index2Color_888;
tLCDDEV_Index2Color LCD_Index2Color_8888;
tLCDDEV_Index2Color LCD_Index2Color_M8888I;
tLCDDEV_Index2Color LCD_Index2Color_M555;
tLCDDEV_Index2Color LCD_Index2Color_M565;
tLCDDEV_Index2Color LCD_Index2Color_M888;

tLCDDEV_Color2Index LCD_Color2Index_8666;







 

#line 59 "..\\STemWin\\inc\\GUI_Private.h"
#line 1 "..\\STemWin\\inc\\GUI_Debug.h"



































 


















 
  



#line 1 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"
 






 

 
 
 





 





#line 34 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"




  typedef signed int ptrdiff_t;



  



    typedef unsigned int size_t;    
#line 57 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



   



      typedef unsigned short wchar_t;  
#line 82 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



    




   




  typedef long double max_align_t;









#line 114 "C:\\Keil_v521a\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



 

#line 61 "..\\STemWin\\inc\\GUI_Debug.h"

#line 63 "..\\STemWin\\inc\\GUI_Debug.h"

#line 70 "..\\STemWin\\inc\\GUI_Debug.h"

#line 78 "..\\STemWin\\inc\\GUI_Debug.h"









 












 

 

#line 127 "..\\STemWin\\inc\\GUI_Debug.h"






 

 

#line 160 "..\\STemWin\\inc\\GUI_Debug.h"






 
 

#line 192 "..\\STemWin\\inc\\GUI_Debug.h"






 











 
#line 60 "..\\STemWin\\inc\\GUI_Private.h"
#line 62 "..\\STemWin\\inc\\GUI_Private.h"














 








 

























 











 
#line 133 "..\\STemWin\\inc\\GUI_Private.h"






 
extern const unsigned char GUI__aConvert_15_255[(1 << 4)];
extern const unsigned char GUI__aConvert_31_255[(1 << 5)];
extern const unsigned char GUI__aConvert_63_255[(1 << 6)];
extern const unsigned char GUI__aConvert_255_15[(1 << 8)];
extern const unsigned char GUI__aConvert_255_31[(1 << 8)];
extern const unsigned char GUI__aConvert_255_63[(1 << 8)];






 
typedef signed long GUI_USAGE_Handle;
typedef struct tsUSAGE_APIList tUSAGE_APIList;
typedef struct GUI_Usage GUI_USAGE;




typedef GUI_USAGE_Handle tUSAGE_CreateCompatible(GUI_USAGE * p);
typedef void        tUSAGE_AddPixel        (GUI_USAGE * p, int x, int y);
typedef void        tUSAGE_AddHLine        (GUI_USAGE * p, int x0, int y0, int len);
typedef void        tUSAGE_Clear           (GUI_USAGE * p);
typedef void        tUSAGE_Delete          (GUI_USAGE_Handle h);
typedef int         tUSAGE_GetNextDirty    (GUI_USAGE * p, int * pxOff, int yOff);



void GUI_USAGE_DecUseCnt(GUI_USAGE_Handle  hUsage);

GUI_USAGE_Handle GUI_USAGE_BM_Create(int x0, int y0, int xsize, int ysize, int Flags);
void    GUI_USAGE_Select(GUI_USAGE_Handle hUsage);
void    GUI_USAGE_AddRect(GUI_USAGE * pUsage, int x0, int y0, int xSize, int ySize);






struct tsUSAGE_APIList {
  tUSAGE_AddPixel         * pfAddPixel;
  tUSAGE_AddHLine         * pfAddHLine;
  tUSAGE_Clear            * pfClear;
  tUSAGE_CreateCompatible * pfCreateCompatible;
  tUSAGE_Delete           * pfDelete;
  tUSAGE_GetNextDirty     * pfGetNextDirty;
} ;

struct GUI_Usage {
  signed short x0, y0, XSize, YSize;
  const tUSAGE_APIList * pAPI;
  signed short UseCnt;
};







 


typedef struct {
  GUI_DEVICE * pDevice;
  signed short                   x0, y0, XSize, YSize;
  unsigned               BytesPerLine;
  unsigned               BitsPerPixel;
  signed long               hUsage;
} GUI_MEMDEV;



void         GUI_MEMDEV__CopyFromLCD (GUI_MEMDEV_Handle hMem);
void         GUI_MEMDEV__GetRect     (GUI_RECT * pRect);
unsigned     GUI_MEMDEV__Color2Index (LCD_COLOR Color);
LCD_COLOR    GUI_MEMDEV__Index2Color (int Index);
unsigned int GUI_MEMDEV__GetIndexMask(void);
void         GUI_MEMDEV__SetAlphaCallback(unsigned(* pcbSetAlpha)(unsigned char));

GUI_MEMDEV_Handle GUI_MEMDEV__CreateFixed(int x0, int y0, int xSize, int ySize, int Flags,
                                          const GUI_DEVICE_API     * pDeviceAPI,
                                          const LCD_API_COLOR_CONV * pColorConvAPI);

void              GUI_MEMDEV__DrawSizedAt        (GUI_MEMDEV_Handle hMem, int xPos, int yPos, int xSize, int ySize);
GUI_MEMDEV_Handle GUI_MEMDEV__GetEmptyCopy32     (GUI_MEMDEV_Handle hMem, int * pxSize, int * pySize, int * pxPos, int * pyPos);
void              GUI_MEMDEV__ReadLine           (int x0, int y, int x1, unsigned long * pBuffer);
void              GUI_MEMDEV__WriteToActiveAlpha (GUI_MEMDEV_Handle hMem,int x, int y);
void              GUI_MEMDEV__WriteToActiveAt    (GUI_MEMDEV_Handle hMem,int x, int y);
void              GUI_MEMDEV__WriteToActiveOpaque(GUI_MEMDEV_Handle hMem,int x, int y);
void            * GUI_MEMDEV__XY2PTR             (int x,int y);
void            * GUI_MEMDEV__XY2PTREx           (GUI_MEMDEV * pDev, int x,int y);
void              GUI_MEMDEV__BlendColor32       (GUI_MEMDEV_Handle hMem, unsigned long BlendColor, unsigned char BlendIntens);

unsigned GUI__AlphaPreserveTrans(int OnOff);

extern unsigned GUI_MEMDEV__TimePerFrame;













 









 

int  GUI_cos(int angle);
int  GUI_sin(int angle);
extern const unsigned long GUI_Pow10[10];

 
void GUI_MTOUCH__ManagePID(int OnOff);

 
int  GUI_AA_Init       (int x0, int x1);
int  GUI_AA_Init_HiRes (int x0, int x1);
void GUI_AA_Exit       (void);
signed short  GUI_AA_HiRes2Pixel(int HiRes);

void GL_FillCircleAA_HiRes (int x0, int y0, int r);
void GL_FillEllipseAA_HiRes(int x0, int y0, int rx, int ry);

void GUI_AA__DrawCharAA2(int x0, int y0, int XSize, int YSize, int BytesPerLine, const unsigned char * pData);
void GUI_AA__DrawCharAA4(int x0, int y0, int XSize, int YSize, int BytesPerLine, const unsigned char * pData);
void GUI_AA__DrawCharAA8(int x0, int y0, int XSize, int YSize, int BytesPerLine, const unsigned char * pData);

 


int      GUI__GetAlphaBuffer    (unsigned long ** ppCurrent, unsigned long ** ppConvert, unsigned long ** ppData, int * pVXSizeMax);
int      GUI__AllocAlphaBuffer  (int AllocDataBuffer);
unsigned long    * GUI__DoAlphaBlending   (int x, int y, unsigned long * pData, int xSize, tLCDDEV_Index2Color * pfIndex2Color_DEV, int * pDone);
unsigned GUI__SetAlphaBufferSize(int xSize);

 
int        GUI_SIF__GetCharDistX       (unsigned short c, int * pSizeX);
void       GUI_SIF__GetFontInfo        (const GUI_FONT * pFont, GUI_FONTINFO * pfi);
char       GUI_SIF__IsInFont           (const GUI_FONT * pFont, unsigned short c);
const unsigned char * GUI_SIF__GetpCharInfo       (const GUI_FONT * pFont, unsigned short c, unsigned SizeOfCharInfo);
int        GUI_SIF__GetNumCharAreas    (const GUI_FONT * pFont);
int        GUI_SIF__GetCharDistX_ExtFrm(unsigned short c, int * pSizeX);
void       GUI_SIF__GetFontInfo_ExtFrm (const GUI_FONT * pFont, GUI_FONTINFO * pfi);
char       GUI_SIF__IsInFont_ExtFrm    (const GUI_FONT * pFont, unsigned short c);
int        GUI_SIF__GetCharInfo_ExtFrm (unsigned short c, GUI_CHARINFO_EXT * pInfo);
void       GUI_SIF__ClearLine_ExtFrm   (const char * s, int Len);

 
int        GUI_XBF__GetOff       (const GUI_XBF_DATA * pXBF_Data, unsigned c, unsigned long * pOff);
int        GUI_XBF__GetOffAndSize(const GUI_XBF_DATA * pXBF_Data, unsigned c, unsigned long * pOff, unsigned short * pSize);
int        GUI_XBF__GetCharDistX (unsigned short c, int * pSizeX);
void       GUI_XBF__GetFontInfo  (const GUI_FONT * pFont, GUI_FONTINFO * pInfo);
char       GUI_XBF__IsInFont     (const GUI_FONT * pFont, unsigned short c);
int        GUI_XBF__GetCharInfo  (unsigned short c, GUI_CHARINFO_EXT * pInfo);
void       GUI_XBF__ClearLine    (const char * s, int Len);

 
void GUI_AddHex     (unsigned long v, unsigned char Len, char ** ps);
void GUI_AddBin     (unsigned long v, unsigned char Len, char ** ps);
void GUI_AddDecMin  (signed long v, char ** ps);
void GUI_AddDec     (signed long v, unsigned char Len, char ** ps);
void GUI_AddDecShift(signed long v, unsigned char Len, unsigned char Shift, char ** ps);
long GUI_AddSign    (long v, char ** ps);
int  GUI_Long2Len   (signed long v);




int   GUI_UC__CalcSizeOfChar   (unsigned short Char);
unsigned short   GUI_UC__GetCharCodeInc   (const char ** ps);
int   GUI_UC__NumChars2NumBytes(const char * s, int NumChars);
int   GUI_UC__NumBytes2NumChars(const char * s, int NumBytes);

int  GUI__GetLineNumChars  (const char * s, int MaxNumChars);
int  GUI__GetNumChars      (const char * s);
int  GUI__GetOverlap       (unsigned short Char);
int  GUI__GetLineDistX     (const char * s, int Len);
int  GUI__GetFontSizeY     (void);
int  GUI__HandleEOLine     (const char ** ps);
void GUI__DispLine         (const char * s, int Len, const GUI_RECT * pr);
void GUI__AddSpaceHex      (unsigned long v, unsigned char Len, char ** ps);
void GUI__CalcTextRect     (const char * pText, const GUI_RECT * pTextRectIn, GUI_RECT * pTextRectOut, int TextAlign);

void GUI__ClearTextBackground(int xDist, int yDist);

int  GUI__WrapGetNumCharsDisp       (const char * pText, int xSize, GUI_WRAPMODE WrapMode);
int  GUI__WrapGetNumCharsToNextLine (const char * pText, int xSize, GUI_WRAPMODE WrapMode);
int  GUI__WrapGetNumBytesToNextLine (const char * pText, int xSize, GUI_WRAPMODE WrapMode);
void GUI__memset    (unsigned char  * p, unsigned char Fill, int NumBytes);
void GUI__memset16  (unsigned short * p, unsigned short Fill, int NumWords);
int  GUI__strlen    (const char * s);
int  GUI__strcmp    (const char * s0, const char * s1);
int  GUI__strcmp_hp (signed long hs0, const char * s1);

 
int  GUI__GetCursorPosX     (const char * s, int Index, int MaxNumChars);
int  GUI__GetCursorPosChar  (const char * s, int x, int NumCharsToNextLine);
unsigned short  GUI__GetCursorCharacter(const char * s, int Index, int MaxNumChars, int * pIsRTL);

 
unsigned short  GUI__GetPresentationForm     (unsigned short Char, unsigned short Next, unsigned short Prev, int * pIgnoreNext, const char * s);
int  GUI__IsArabicCharacter       (unsigned short c);

 
int  GUI__BIDI_Log2Vis           (const char * s, int NumChars, char * pBuffer, int BufferSize);
int  GUI__BIDI_GetCursorPosX     (const char * s, int NumChars, int Index);
int  GUI__BIDI_GetCursorPosChar  (const char * s, int NumChars, int x);
unsigned short  GUI__BIDI_GetLogChar        (const char * s, int NumChars, int Index);
int  GUI__BIDI_GetCharDir        (const char * s, int NumChars, int Index);
int  GUI__BIDI_IsNSM             (unsigned short Char);
unsigned short  GUI__BIDI_GetCursorCharacter(const char * s, int Index, int MaxNumChars, int * pIsRTL);
int  GUI__BIDI_GetWordWrap       (const char * s, int xSize, int * pxDist);
int  GUI__BIDI_GetCharWrap       (const char * s, int xSize);

const char * GUI__BIDI_Log2VisBuffered(const char * s, int * pMaxNumChars);

extern int GUI__BIDI_Enabled;

extern int (* _pfGUI__BIDI_Log2Vis         )(const char * s, int NumChars, char * pBuffer, int BufferSize);
extern int (* _pfGUI__BIDI_GetCursorPosX   )(const char * s, int NumChars, int Index);
extern int (* _pfGUI__BIDI_GetCursorPosChar)(const char * s, int NumChars, int x);
extern unsigned short (* _pfGUI__BIDI_GetLogChar      )(const char * s, int NumChars, int Index);
extern int (* _pfGUI__BIDI_GetCharDir      )(const char * s, int NumChars, int Index);
extern int (* _pfGUI__BIDI_IsNSM           )(unsigned short Char);

 
extern const char * (* GUI_CharLine_pfLog2Vis)(const char * s, int * pMaxNumChars);

extern int (* GUI__GetCursorPos_pfGetPosX)     (const char * s, int MaxNumChars, int Index);
extern int (* GUI__GetCursorPos_pfGetPosChar)  (const char * s, int MaxNumChars, int x);
extern unsigned short (* GUI__GetCursorPos_pfGetCharacter)(const char * s, int MaxNumChars, int Index, int * pIsRTL);

extern int (* GUI__Wrap_pfGetWordWrap)(const char * s, int xSize, int * pxDist);
extern int (* GUI__Wrap_pfGetCharWrap)(const char * s, int xSize);

 
const GUI_FONT_PROP * GUIPROP__FindChar(const GUI_FONT_PROP * pProp, unsigned short c);

 
const GUI_FONT_PROP_EXT * GUIPROP_EXT__FindChar(const GUI_FONT_PROP_EXT * pPropExt, unsigned short c);
void  GUIPROP_EXT__DispLine      (const char * s, int Len);
void  GUIPROP_EXT__ClearLine     (const char * s, int Len);
void  GUIPROP_EXT__SetfpClearLine(void (* fpClearLine)(const char * s, int Len));

 
unsigned short GUI__Read16(const unsigned char ** ppData);
unsigned long GUI__Read32(const unsigned char ** ppData);

 
void GUI__GetOrg(int * px, int * py);
void GUI__SetOrgHook(void(* pfHook)(int x, int y));

 
int              GUI_TIMER__IsActive       (void);
int   GUI_TIMER__GetPeriod      (void);
GUI_TIMER_HANDLE GUI_TIMER__GetNextTimer   (GUI_TIMER_HANDLE hTimer, unsigned long * pContext);
GUI_TIMER_HANDLE GUI_TIMER__GetFirstTimer  (unsigned long * pContext);
GUI_TIMER_HANDLE GUI_TIMER__GetNextTimerLin(GUI_TIMER_HANDLE hTimer, unsigned long * pContext);

 
tLCDDEV_Index2Color * GUI_GetpfIndex2ColorEx(int LayerIndex);
tLCDDEV_Color2Index * GUI_GetpfColor2IndexEx(int LayerIndex);

int GUI_GetBitsPerPixelEx(int LayerIndex);

unsigned long * LCD_GetpPalConvTable        (const LCD_LOGPALETTE * pLogPal);
unsigned long * LCD_GetpPalConvTableUncached(const LCD_LOGPALETTE * pLogPal);
unsigned long * LCD_GetpPalConvTableBM      (const LCD_LOGPALETTE * pLogPal, const GUI_BITMAP * pBitmap, int LayerIndex);

 
void GUI_SetFuncGetpPalConvTable(unsigned long * (* pFunc)(const LCD_LOGPALETTE * pLogPal, const GUI_BITMAP * pBitmap, int LayerIndex));







 
#line 459 "..\\STemWin\\inc\\GUI_Private.h"


void GUI__ReadHeaderFromStream  (GUI_BITMAP_STREAM * pBitmapHeader, const unsigned char * pData);
void GUI__CreateBitmapFromStream(const GUI_BITMAP_STREAM * pBitmapHeader, const void * pData, GUI_BITMAP * pBMP, GUI_LOGPALETTE * pPAL, const GUI_BITMAP_METHODS * pMethods);

 
int GUI__ManageCache  (int Cmd);
int GUI__ManageCacheEx(int LayerIndex, int Cmd);






 
void GL_DispChar         (unsigned short c);
void GL_DrawArc          (int x0, int y0, int rx, int ry, int a0, int a1);
void GL_DrawBitmap       (const GUI_BITMAP * pBM, int x0, int y0);
void GL_DrawCircle       (int x0, int y0, int r);
void GL_DrawEllipse      (int x0, int y0, int rx, int ry, int w);
void GL_DrawHLine        (int y0, int x0, int x1);
void GL_DrawPolygon      (const GUI_POINT * pPoints, int NumPoints, int x0, int y0);
void GL_DrawPoint        (int x,  int y);
void GL_DrawLine1        (int x0, int y0, int x1, int y1);
void GL_DrawLine1Ex      (int x0, int y0, int x1, int y1, unsigned * pPixelCnt);
void GL_DrawLineRel      (int dx, int dy);
void GL_DrawLineTo       (int x,  int y);
void GL_DrawLineToEx     (int x,  int y, unsigned * pPixelCnt);
void GL_DrawLine         (int x0, int y0, int x1, int y1);
void GL_DrawLineEx       (int x0, int y0, int x1, int y1, unsigned * pPixelCnt);
void GL_MoveTo           (int x,  int y);
void GL_FillCircle       (int x0, int y0, int r);
void GL_FillCircleAA     (int x0, int y0, int r);
void GL_FillEllipse      (int x0, int y0, int rx, int ry);
void GL_FillPolygon      (const GUI_POINT * pPoints, int NumPoints, int x0, int y0);
void GL_SetDefault       (void);








 
typedef int  GUI_tfTimer(void);
typedef int  WM_tfHandlePID(void);







 
extern const unsigned char  GUI_Pixels_ArrowS[45];
extern const unsigned char  GUI_Pixels_ArrowM[60];
extern const unsigned char  GUI_Pixels_ArrowL[150];
extern const unsigned char  GUI_Pixels_CrossS[33];
extern const unsigned char  GUI_Pixels_CrossM[126];
extern const unsigned char  GUI_Pixels_CrossL[248];
extern const unsigned char  GUI_PixelsHeaderM[5 * 17];

extern const GUI_LOGPALETTE GUI_CursorPal;
extern const GUI_LOGPALETTE GUI_CursorPalI;







 
extern GUI_RECT  GUI_RectDispString;  






 
extern unsigned char GUI__CharHasTrans;






 
extern int GUITASK__EntranceCnt;






 

int       GUI_GetBitmapPixelIndex(const GUI_BITMAP * pBMP, unsigned x, unsigned y);
GUI_COLOR GUI_GetBitmapPixelColor(const GUI_BITMAP * pBMP, unsigned x, unsigned y);
int       GUI_GetBitmapPixelIndexEx(int BitsPerPixel, int BytesPerLine, const unsigned char * pData, unsigned x, unsigned y);

void      GUI__DrawBitmap16bpp (int x0, int y0, int xsize, int ysize, const unsigned char * pPixel, const LCD_LOGPALETTE * pLogPal, int xMag, int yMag, tLCDDEV_Index2Color * pfIndex2Color, const LCD_API_COLOR_CONV * pColorConvAPI);
void      GUI__DrawBitmapA16bpp(int x0, int y0, int xSize, int ySize, const unsigned char * pPixel, const LCD_LOGPALETTE * pLogPal, int xMag, int yMag, tLCDDEV_Index2Color * pfIndex2Color);
void      GUI__SetPixelAlpha   (int x, int y, unsigned char Alpha, LCD_COLOR Color);
LCD_COLOR GUI__MixColors       (LCD_COLOR Color, LCD_COLOR BkColor, unsigned char Intens);
void      GUI__MixColorsBulk   (unsigned long * pFG, unsigned long * pBG, unsigned long * pDst, unsigned OffFG, unsigned OffBG, unsigned OffDest, unsigned xSize, unsigned ySize, unsigned char Intens);

extern const GUI_UC_ENC_APILIST GUI_UC_None;






 



#line 589 "..\\STemWin\\inc\\GUI_Private.h"

void LCD_ReadRect  (int x0, int y0, int x1, int y1, unsigned long * pBuffer, GUI_DEVICE * pDevice);
void GUI_ReadRect  (int x0, int y0, int x1, int y1, unsigned long * pBuffer, GUI_DEVICE * pDevice);
void GUI_ReadRectEx(int x0, int y0, int x1, int y1, unsigned long * pBuffer, GUI_DEVICE * pDevice);

void LCD_ReadRectNoClip(int x0, int y0, int x1, int y1, unsigned long * pBuffer, GUI_DEVICE * pDevice);






 
typedef struct {
  void         (* pfSetColor)   (LCD_COLOR Index);
  void         (* pfSetBkColor) (LCD_COLOR Index);
  LCD_DRAWMODE (* pfSetDrawMode)(LCD_DRAWMODE dm);
} LCD_SET_COLOR_API;

extern const LCD_SET_COLOR_API * LCD__pSetColorAPI;






 








 
extern const GUI_FONT * GUI__pFontDefault;

extern  GUI_CONTEXT * GUI_pContext;

extern GUI_DEVICE * GUI__apDevice[2];




extern unsigned long * (* GUI_pfGetpPalConvTable)(const LCD_LOGPALETTE * pLogPal, const GUI_BITMAP * pBitmap, int LayerIndex);




extern LCD_COLOR (* GUI__pfMixColors)(LCD_COLOR Color, LCD_COLOR BkColor, unsigned char Intens);




extern void (* GUI__pfMixColorsBulk)(unsigned long * pFG, unsigned long * pBG, unsigned long * pDst, unsigned OffFG, unsigned OffBG, unsigned OffDest, unsigned xSize, unsigned ySize, unsigned char Intens);




extern const GUI_MULTIBUF_API    GUI_MULTIBUF_APIList;
extern const GUI_MULTIBUF_API_EX GUI_MULTIBUF_APIListEx;







extern   int  (* GUI_pfUpdateSoftLayer)(void);





extern void (* GUI_pfHookMTOUCH)(const GUI_MTOUCH_STATE * pState);

extern const GUI_UC_ENC_APILIST * GUI_pUC_API;  

extern  char             GUI_DecChar;
extern           GUI_tfTimer    * GUI_pfTimerExec;
extern           WM_tfHandlePID * WM_pfHandlePID;
extern   void (* GUI_pfDispCharStyle)(unsigned short Char);

extern           int GUI__BufferSize; 
extern           int GUI_AA__ClipX0;  

extern           signed char  GUI__aNumBuffers[2]; 
extern           unsigned char  GUI__PreserveTrans;
extern           unsigned char  GUI__IsInitialized;


  extern const tLCD_APIList * GUI_pLCD_APIList;  


extern signed short GUI_OrgX, GUI_OrgY;









 
#line 59 "..\\STemWin\\inc\\WM_Intern.h"

















 


 






#line 92 "..\\STemWin\\inc\\WM_Intern.h"








#line 107 "..\\STemWin\\inc\\WM_Intern.h"






 
typedef struct {
  GUI_HWIN hOld;
  GUI_HWIN hNew;
} WM_NOTIFY_CHILD_HAS_FOCUS_INFO;

typedef struct WM_CRITICAL_HANDLE {
  struct  WM_CRITICAL_HANDLE * pNext;
  volatile GUI_HWIN hWin;
} WM_CRITICAL_HANDLE;






 
extern unsigned long            WM__CreateFlags;
extern GUI_HWIN        WM__ahCapture[2];
extern GUI_HWIN        WM__ahWinFocus[2];
extern char           WM__CaptureReleaseAuto;
extern WM_tfPollPID * WM_pfPollPID;
extern unsigned char             WM__PaintCallbackCnt;       
extern GUI_HWIN        WM__hCreateStatic;


  extern int     WM__TransWindowCnt;
  extern GUI_HWIN WM__hATransWindow;






extern WM_CRITICAL_HANDLE     WM__aCHWinModal[2];
extern WM_CRITICAL_HANDLE     WM__aCHWinLast[2];
extern int                    WM__ModalLayer;


  extern WM_CRITICAL_HANDLE   WM__aCHWinMouseOver[2];









  extern unsigned                  WM__TouchedLayer;





extern unsigned short     WM__NumWindows;
extern unsigned short     WM__NumInvalidWindows;
extern GUI_HWIN WM__FirstWin;
extern WM_CRITICAL_HANDLE * WM__pFirstCriticalHandle;

extern GUI_HWIN   WM__ahDesktopWin[2];
extern GUI_COLOR WM__aBkColor[2];








 
void    WM__ActivateClipRect        (void);
int     WM__ClipAtParentBorders     (GUI_RECT * pRect, GUI_HWIN hWin);
void    WM__Client2Screen           (const WM_Obj * pWin, GUI_RECT * pRect);
void    WM__DeleteAssocTimer        (GUI_HWIN hWin);
void    WM__DeleteSecure            (GUI_HWIN hWin);
void    WM__DetachWindow            (GUI_HWIN hChild);
void    WM__ForEachDesc             (GUI_HWIN hWin, WM_tfForEach * pcb, void * pData);
void    WM__GetClientRectWin        (const WM_Obj * pWin, GUI_RECT * pRect);
void    WM__GetClientRectEx         (GUI_HWIN hWin, GUI_RECT * pRect);
GUI_HWIN WM__GetFirstSibling         (GUI_HWIN hWin);
GUI_HWIN WM__GetFocussedChild        (GUI_HWIN hWin);
int     WM__GetHasFocus             (GUI_HWIN hWin);
GUI_HWIN WM__GetLastSibling          (GUI_HWIN hWin);
GUI_HWIN WM__GetPrevSibling          (GUI_HWIN hWin);
int     WM__GetTopLevelLayer        (GUI_HWIN hWin);
int     WM__GetWindowSizeX          (const WM_Obj * pWin);
int     WM__GetWindowSizeY          (const WM_Obj * pWin);
void    WM__InsertWindowIntoList    (GUI_HWIN hWin, GUI_HWIN hParent);
void    WM__Invalidate1Abs          (GUI_HWIN hWin, const GUI_RECT * pRect);
void    WM__InvalidateAreaBelow     (const GUI_RECT * pRect, GUI_HWIN StopWin);
void    WM__InvalidateRectEx        (const GUI_RECT * pInvalidRect, GUI_HWIN hParent, GUI_HWIN hStop);
void    WM__InvalidateTransAreaAbove(const GUI_RECT * pRect, GUI_HWIN StopWin);
int     WM__IntersectRect           (GUI_RECT * pDest, const GUI_RECT * pr0, const GUI_RECT * pr1);
int     WM__IsAncestor              (GUI_HWIN hChild, GUI_HWIN hParent);
int     WM__IsAncestorOrSelf        (GUI_HWIN hChild, GUI_HWIN hParent);
int     WM__IsChild                 (GUI_HWIN hWin, GUI_HWIN hParent);
int     WM__IsEnabled               (GUI_HWIN hWin);
int     WM__IsInModalArea           (GUI_HWIN hWin);
int     WM__IsInWindow              (WM_Obj * pWin, int x, int y);
int     WM__IsWindow                (GUI_HWIN hWin);
void    WM__LeaveIVRSearch          (void);
void    WM__MoveTo                  (GUI_HWIN hWin, int x, int y);
void    WM__MoveWindow              (GUI_HWIN hWin, int dx, int dy);
void    WM__NotifyVisChanged        (GUI_HWIN hWin, GUI_RECT * pRect);
int     WM__RectIsNZ                (const GUI_RECT * pr);
void    WM__RemoveWindowFromList    (GUI_HWIN hWin);
void    WM__Screen2Client           (const WM_Obj * pWin, GUI_RECT * pRect);
void    WM__SelectTopLevelLayer     (GUI_HWIN  hWin);
void    WM__SendMsgNoData           (GUI_HWIN hWin, unsigned char MsgId);
void    WM__SendMessage             (GUI_HWIN hWin, WM_MESSAGE * pm);
void    WM__SendMessageIfEnabled    (GUI_HWIN hWin, WM_MESSAGE * pm);
void    WM__SendMessageNoPara       (GUI_HWIN hWin, int MsgId);
void    WM__SendPIDMessage          (GUI_HWIN hWin, WM_MESSAGE * pMsg);
int     WM__SetScrollbarH           (GUI_HWIN hWin, int OnOff);
int     WM__SetScrollbarV           (GUI_HWIN hWin, int OnOff);
void    WM__UpdateChildPositions    (WM_Obj * pObj, int dx0, int dy0, int dx1, int dy1);
void    WM_PID__GetPrevState        (GUI_PID_STATE * pPrevState, int Layer);
void    WM_PID__SetPrevState        (GUI_PID_STATE * pPrevState, int Layer);
void    WM__SendTouchMessage        (GUI_HWIN hWin, WM_MESSAGE * pMsg);

unsigned short     WM_GetFlags                 (GUI_HWIN hWin);
int     WM__Paint                   (GUI_HWIN hWin);
void    WM__Paint1                  (GUI_HWIN hWin);
void    WM__AddCriticalHandle       (WM_CRITICAL_HANDLE * pCH);
void    WM__RemoveCriticalHandle    (WM_CRITICAL_HANDLE * pCH);
void    WM__SetLastTouched          (GUI_HWIN hWin);


  void    WM__InvalidateDrawAndDescs(GUI_HWIN hWin);




 

  typedef struct {
    int xSize, ySize; 
  } EFFECT_CONTEXT;

  int  GUI_MEMDEV__CalcParaFadeIn    (int Period, int TimeUsed);
  void GUI_MEMDEV__ClipBK            (EFFECT_CONTEXT * pContext);
  void GUI_MEMDEV__RemoveStaticDevice(GUI_HWIN hWin);
  void GUI_MEMDEV__UndoClipBK        (EFFECT_CONTEXT * pContext);


void WM__InvalidateParent(const GUI_RECT * pInvalidRect, GUI_HWIN hParent, GUI_HWIN hStop);
void WM__InvalidateRect  (const GUI_RECT * pInvalidRect, GUI_HWIN hParent, GUI_HWIN hStop, unsigned short Flags);

WM_tfInvalidateParent   WM__SetInvalidateParentFunc(WM_tfInvalidateParent pfInvalidateParentFunc);
WM_tfInvalidateDrawFunc WM__SetInvalidateDrawFunc  (WM_tfInvalidateDrawFunc pfInvalidateDrawFunc);
WM_tfPaint1Func         WM__SetPaint1Func          (WM_tfPaint1Func pfPaint1Func);









 
#line 62 "..\\STemWin\\inc\\WIDGET.h"








 
typedef struct {
  GUI_HWIN    hWin;
  int        Cmd;          
  int        ItemIndex;
  int        Col;
  int        x0, y0, x1, y1;
  void     * p;
} WIDGET_ITEM_DRAW_INFO;

typedef int  WIDGET_DRAW_ITEM_FUNC(const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
typedef void WIDGET_PAINT         (GUI_HWIN hObj);
typedef void WIDGET_CREATE        (GUI_HWIN hObj);

typedef struct {
  WIDGET_PAINT  * pfPaint;
  WIDGET_CREATE * pfCreate;
  void          * pSkinPrivate;
} WIDGET_SKIN;







 
#line 1 "..\\STemWin\\inc\\SCROLLBAR.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\SCROLLBAR.h"
#line 59 "..\\STemWin\\inc\\SCROLLBAR.h"
#line 1 "..\\STemWin\\inc\\WIDGET.h"
































 


















 
  
#line 396 "..\\STemWin\\inc\\WIDGET.h"




#line 60 "..\\STemWin\\inc\\SCROLLBAR.h"












 







 





 






 








 
typedef signed long SCROLLBAR_Handle;

typedef struct {
  GUI_COLOR aColorFrame[3];
  GUI_COLOR aColorUpper[2];
  GUI_COLOR aColorLower[2];
  GUI_COLOR aColorShaft[2];
  GUI_COLOR ColorArrow;
  GUI_COLOR ColorGrasp;
} SCROLLBAR_SKINFLEX_PROPS;

typedef struct {
  int IsVertical;
  int State;
} SCROLLBAR_SKINFLEX_INFO;






 
SCROLLBAR_Handle SCROLLBAR_Create        (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int WinFlags, int SpecialFlags);
SCROLLBAR_Handle SCROLLBAR_CreateAttached(GUI_HWIN hParent, int SpecialFlags);
SCROLLBAR_Handle SCROLLBAR_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
SCROLLBAR_Handle SCROLLBAR_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
SCROLLBAR_Handle SCROLLBAR_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void SCROLLBAR_Callback(WM_MESSAGE * pMsg);






 

 

void      SCROLLBAR_AddValue   (SCROLLBAR_Handle hObj, int Add);
void      SCROLLBAR_Dec        (SCROLLBAR_Handle hObj);
void      SCROLLBAR_Inc        (SCROLLBAR_Handle hObj);
int       SCROLLBAR_GetUserData(SCROLLBAR_Handle hObj, void * pDest, int NumBytes);
GUI_COLOR SCROLLBAR_SetColor   (SCROLLBAR_Handle hObj, int Index, GUI_COLOR Color);
void      SCROLLBAR_SetNumItems(SCROLLBAR_Handle hObj, int NumItems);
void      SCROLLBAR_SetPageSize(SCROLLBAR_Handle hObj, int PageSize);
void      SCROLLBAR_SetValue   (SCROLLBAR_Handle hObj, int v);
int       SCROLLBAR_SetWidth   (SCROLLBAR_Handle hObj, int Width);
void      SCROLLBAR_SetState   (SCROLLBAR_Handle hObj, const WM_SCROLL_STATE* pState);
int       SCROLLBAR_SetUserData(SCROLLBAR_Handle hObj, const void * pSrc, int NumBytes);






 
void SCROLLBAR_GetSkinFlexProps     (SCROLLBAR_SKINFLEX_PROPS * pProps, int Index);
void SCROLLBAR_SetSkinClassic       (SCROLLBAR_Handle hObj);
void SCROLLBAR_SetSkin              (SCROLLBAR_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
int  SCROLLBAR_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void SCROLLBAR_SetSkinFlexProps     (const SCROLLBAR_SKINFLEX_PROPS * pProps, int Index);
void SCROLLBAR_SetDefaultSkinClassic(void);
WIDGET_DRAW_ITEM_FUNC * SCROLLBAR_SetDefaultSkin(WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);








 
int       SCROLLBAR_GetDefaultWidth(void);
GUI_COLOR SCROLLBAR_SetDefaultColor(GUI_COLOR Color, unsigned int Index);  
int       SCROLLBAR_SetDefaultWidth(int DefaultWidth);






 
int       SCROLLBAR_GetThumbSizeMin(void);
int       SCROLLBAR_SetThumbSizeMin(int ThumbSizeMin);






 
int SCROLLBAR_GetNumItems(SCROLLBAR_Handle hObj);
int SCROLLBAR_GetPageSize(SCROLLBAR_Handle hObj);
int SCROLLBAR_GetValue   (SCROLLBAR_Handle hObj);






 











 
#line 98 "..\\STemWin\\inc\\WIDGET.h"






 



 
#line 136 "..\\STemWin\\inc\\WIDGET.h"






 

#line 167 "..\\STemWin\\inc\\WIDGET.h"




 












 
#line 215 "..\\STemWin\\inc\\WIDGET.h"
                                           







 






 








 
typedef struct {
  int EffectSize;
  void (* pfDrawUp)      (void);
  void (* pfDrawUpRect)  (const GUI_RECT * pRect);
  void (* pfDrawDown)    (void);
  void (* pfDrawDownRect)(const GUI_RECT * pRect);
  void (* pfDrawFlat)    (void);
  void (* pfDrawFlatRect)(const GUI_RECT * pRect);
} WIDGET_EFFECT;

typedef struct {
  WM_Obj      Win;
  const WIDGET_EFFECT* pEffect;
  signed short Id;
  unsigned short State;



} WIDGET;









 
 
typedef struct GUI_DRAW GUI_DRAW;
typedef void   GUI_DRAW_SELF_CB (GUI_HWIN hWin);
typedef signed long GUI_DRAW_HANDLE;

 
typedef struct {
  void (* pfDraw)    (GUI_DRAW_HANDLE hDrawObj, GUI_HWIN hObj, int x, int y);
  int  (* pfGetXSize)(GUI_DRAW_HANDLE hDrawObj);
  int  (* pfGetYSize)(GUI_DRAW_HANDLE hDrawObj);
} GUI_DRAW_CONSTS;

 
struct GUI_DRAW {
  const GUI_DRAW_CONSTS* pConsts;
  union {
    const void * pData;
    GUI_DRAW_SELF_CB* pfDraw;
  } Data;
  signed short xOff, yOff;
};

 
void GUI_DRAW__Draw    (GUI_DRAW_HANDLE hDrawObj, GUI_HWIN hObj, int x, int y);
int  GUI_DRAW__GetXSize(GUI_DRAW_HANDLE hDrawObj);
int  GUI_DRAW__GetYSize(GUI_DRAW_HANDLE hDrawObj);

 
signed long GUI_DRAW_BITMAP_Create  (const GUI_BITMAP* pBitmap, int x, int y);
signed long GUI_DRAW_BMP_Create     (const void* pBMP, int x, int y);
signed long GUI_DRAW_STREAMED_Create(const GUI_BITMAP_STREAM * pBitmap, int x, int y);
signed long GUI_DRAW_SELF_Create(GUI_DRAW_SELF_CB* pfDraw, int x, int y);






 

extern const WIDGET_EFFECT WIDGET_Effect_3D;
extern const WIDGET_EFFECT WIDGET_Effect_3D1L;
extern const WIDGET_EFFECT WIDGET_Effect_3D2L;
extern const WIDGET_EFFECT WIDGET_Effect_None;
extern const WIDGET_EFFECT WIDGET_Effect_Simple;






 

void      WIDGET__DrawFocusRect      (WIDGET * pWidget, const GUI_RECT * pRect, int Dist);
void      WIDGET__DrawHLine          (WIDGET * pWidget, int y, int x0, int x1);
void      WIDGET__DrawTriangle       (WIDGET * pWidget, int x, int y, int Size, int Inc);
void      WIDGET__DrawVLine          (WIDGET * pWidget, int x, int y0, int y1);
void      WIDGET__EFFECT_DrawDownRect(WIDGET * pWidget, GUI_RECT * pRect);
void      WIDGET__EFFECT_DrawDown    (WIDGET * pWidget);
void      WIDGET__EFFECT_DrawUpRect  (WIDGET * pWidget, GUI_RECT * pRect);
void      WIDGET__FillRectEx         (WIDGET * pWidget, const GUI_RECT * pRect);
int       WIDGET__GetWindowSizeX     (GUI_HWIN hWin);
GUI_COLOR WIDGET__GetBkColor         (GUI_HWIN hObj);
int       WIDGET__GetXSize           (const WIDGET * pWidget);
int       WIDGET__GetYSize           (const WIDGET * pWidget);
void      WIDGET__GetClientRect      (WIDGET * pWidget, GUI_RECT * pRect);
void      WIDGET__GetInsideRect      (WIDGET * pWidget, GUI_RECT * pRect);
void      WIDGET__Init               (WIDGET * pWidget, int Id, unsigned short State);
void      WIDGET__RotateRect90       (WIDGET * pWidget, GUI_RECT * pDest, const GUI_RECT * pRect);
void      WIDGET__SetScrollState     (GUI_HWIN hWin, const WM_SCROLL_STATE * pVState, const WM_SCROLL_STATE * pState);
void      WIDGET__FillStringInRect   (const char * pText, const GUI_RECT * pFillRect, const GUI_RECT * pTextRectMax, const GUI_RECT * pTextRectAct);






 
void  WIDGET_SetState     (GUI_HWIN hObj, int State);
void  WIDGET_AndState     (GUI_HWIN hObj, int State);
void  WIDGET_OrState      (GUI_HWIN hObj, int State);
int   WIDGET_HandleActive (GUI_HWIN hObj, WM_MESSAGE* pMsg);
int   WIDGET_GetState     (GUI_HWIN hObj);
int   WIDGET_SetWidth     (GUI_HWIN hObj, int Width);

void  WIDGET_EFFECT_3D_DrawUp(void);

const WIDGET_EFFECT* WIDGET_SetDefaultEffect(const WIDGET_EFFECT* pEffect);

void  WIDGET_SetEffect              (GUI_HWIN hObj, const WIDGET_EFFECT* pEffect);

const WIDGET_EFFECT* WIDGET_GetDefaultEffect(void);

void WIDGET_EFFECT_3D_SetColor    (unsigned Index, GUI_COLOR Color);
void WIDGET_EFFECT_3D1L_SetColor  (unsigned Index, GUI_COLOR Color);
void WIDGET_EFFECT_3D2L_SetColor  (unsigned Index, GUI_COLOR Color);
void WIDGET_EFFECT_Simple_SetColor(unsigned Index, GUI_COLOR Color);

GUI_COLOR WIDGET_EFFECT_3D_GetColor    (unsigned Index);
GUI_COLOR WIDGET_EFFECT_3D1L_GetColor  (unsigned Index);
GUI_COLOR WIDGET_EFFECT_3D2L_GetColor  (unsigned Index);
GUI_COLOR WIDGET_EFFECT_Simple_GetColor(unsigned Index);

int WIDGET_EFFECT_3D_GetNumColors(void);
int WIDGET_EFFECT_3D1L_GetNumColors(void);
int WIDGET_EFFECT_3D2L_GetNumColors(void);
int WIDGET_EFFECT_Simple_GetNumColors(void);






 

















#line 60 "..\\STemWin\\inc\\BUTTON.h"










 
 







 







 







 






 










 
typedef signed long BUTTON_Handle;

typedef struct {
  GUI_COLOR aColorFrame[3];
  GUI_COLOR aColorUpper[2];
  GUI_COLOR aColorLower[2];
  int Radius;
} BUTTON_SKINFLEX_PROPS;







 

BUTTON_Handle BUTTON_Create        (int x0, int y0, int xSize, int ySize, int ID, int Flags);
BUTTON_Handle BUTTON_CreateAsChild (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int Flags);
BUTTON_Handle BUTTON_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
BUTTON_Handle BUTTON_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
BUTTON_Handle BUTTON_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);






 
GUI_COLOR        BUTTON_GetDefaultBkColor   (unsigned Index);
const GUI_FONT * BUTTON_GetDefaultFont      (void);
int              BUTTON_GetDefaultTextAlign (void);
GUI_COLOR        BUTTON_GetDefaultTextColor (unsigned Index);
void             BUTTON_SetDefaultBkColor   (GUI_COLOR Color, unsigned Index);
GUI_COLOR        BUTTON_SetDefaultFocusColor(GUI_COLOR Color);
void             BUTTON_SetDefaultFont      (const GUI_FONT * pFont);
void             BUTTON_SetDefaultTextAlign (int Align);
void             BUTTON_SetDefaultTextColor (GUI_COLOR Color, unsigned Index);







 
void BUTTON_Callback(WM_MESSAGE *pMsg);






 
GUI_COLOR          BUTTON_GetBkColor         (BUTTON_Handle hObj, unsigned int Index);
const GUI_BITMAP * BUTTON_GetBitmap(BUTTON_Handle hObj,unsigned int Index);
const GUI_FONT   * BUTTON_GetFont  (BUTTON_Handle hObj);
GUI_COLOR          BUTTON_GetFrameColor      (BUTTON_Handle hObj);
WIDGET           * BUTTON_GetpWidget         (BUTTON_Handle hObj);
void               BUTTON_GetText            (BUTTON_Handle hObj, char * pBuffer, int MaxLen);
GUI_COLOR          BUTTON_GetTextColor       (BUTTON_Handle hObj, unsigned int Index);
int                BUTTON_GetTextAlign       (BUTTON_Handle hObj);
int                BUTTON_GetUserData        (BUTTON_Handle hObj, void * pDest, int NumBytes);
unsigned           BUTTON_IsPressed          (BUTTON_Handle hObj);
void               BUTTON_SetBitmap          (BUTTON_Handle hObj, unsigned int Index, const GUI_BITMAP * pBitmap);
void               BUTTON_SetBitmapEx        (BUTTON_Handle hObj, unsigned int Index, const GUI_BITMAP * pBitmap, int x, int y);
void               BUTTON_SetBkColor         (BUTTON_Handle hObj, unsigned int Index, GUI_COLOR Color);
void               BUTTON_SetBMP             (BUTTON_Handle hObj, unsigned int Index, const void * pBitmap);
void               BUTTON_SetBMPEx           (BUTTON_Handle hObj, unsigned int Index, const void * pBitmap, int x, int y);
void               BUTTON_SetFont            (BUTTON_Handle hObj, const GUI_FONT * pfont);
void               BUTTON_SetFrameColor      (BUTTON_Handle hObj, GUI_COLOR Color);
void               BUTTON_SetState           (BUTTON_Handle hObj, int State);                                     
void               BUTTON_SetPressed         (BUTTON_Handle hObj, int State);
GUI_COLOR          BUTTON_SetFocusColor      (BUTTON_Handle hObj, GUI_COLOR Color);
void               BUTTON_SetFocussable      (BUTTON_Handle hObj, int State);
void               BUTTON_SetStreamedBitmap  (BUTTON_Handle hObj, unsigned int Index, const GUI_BITMAP_STREAM * pBitmap);
void               BUTTON_SetStreamedBitmapEx(BUTTON_Handle hObj, unsigned int Index, const GUI_BITMAP_STREAM * pBitmap, int x, int y);
int                BUTTON_SetText            (BUTTON_Handle hObj, const char* s);
void               BUTTON_SetTextAlign       (BUTTON_Handle hObj, int Align);
void               BUTTON_SetTextColor       (BUTTON_Handle hObj, unsigned int Index, GUI_COLOR Color);
void               BUTTON_SetTextOffset      (BUTTON_Handle hObj, int xPos, int yPos);
void               BUTTON_SetSelfDrawEx      (BUTTON_Handle hObj, unsigned int Index, GUI_DRAW_SELF_CB * pDraw, int x, int y);  
void               BUTTON_SetSelfDraw        (BUTTON_Handle hObj, unsigned int Index, GUI_DRAW_SELF_CB * pDraw);                
void               BUTTON_SetReactOnLevel    (void);
void               BUTTON_SetReactOnTouch    (void);
int                BUTTON_SetUserData        (BUTTON_Handle hObj, const void * pSrc, int NumBytes);






 
void BUTTON_GetSkinFlexProps     (BUTTON_SKINFLEX_PROPS * pProps, int Index);
void BUTTON_SetSkinClassic       (BUTTON_Handle hObj);
void BUTTON_SetSkin              (BUTTON_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
int  BUTTON_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void BUTTON_SetSkinFlexProps     (const BUTTON_SKINFLEX_PROPS * pProps, int Index);
void BUTTON_SetDefaultSkinClassic(void);
WIDGET_DRAW_ITEM_FUNC * BUTTON_SetDefaultSkin(WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);










 
#line 59 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\CALENDAR.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\CALENDAR.h"












 






















 








 



 
typedef struct {
  int Year;
  int Month;
  int Day;
} CALENDAR_DATE;




 
typedef struct {
  GUI_COLOR aColorFrame[3]; 
  GUI_COLOR aColorUpper[2]; 
  GUI_COLOR aColorLower[2]; 
  GUI_COLOR ColorArrow;     
} CALENDAR_SKINFLEX_PROPS;






 
GUI_HWIN CALENDAR_Create           (GUI_HWIN hParent, int xPos, int yPos, unsigned Year, unsigned Month, unsigned Day, unsigned FirstDayOfWeek, int Id, int Flags);
void    CALENDAR_GetDate          (GUI_HWIN hWin, CALENDAR_DATE * pDate);
void    CALENDAR_GetSel           (GUI_HWIN hWin, CALENDAR_DATE * pDate);
void    CALENDAR_SetDate          (GUI_HWIN hWin, CALENDAR_DATE * pDate);
void    CALENDAR_SetSel           (GUI_HWIN hWin, CALENDAR_DATE * pDate);
void    CALENDAR_ShowDate         (GUI_HWIN hWin, CALENDAR_DATE * pDate);




 
void    CALENDAR_SetDefaultBkColor(unsigned Index, GUI_COLOR Color);
void    CALENDAR_SetDefaultColor  (unsigned Index, GUI_COLOR Color);
void    CALENDAR_SetDefaultDays   (const char ** apDays);
void    CALENDAR_SetDefaultFont   (unsigned Index, const GUI_FONT * pFont);
void    CALENDAR_SetDefaultMonths (const char ** apMonths);
void    CALENDAR_SetDefaultSize   (unsigned Index, unsigned Size);




 
void    CALENDAR_GetSkinFlexProps (CALENDAR_SKINFLEX_PROPS * pProps, int Index);
void    CALENDAR_SetSkinFlexProps (const CALENDAR_SKINFLEX_PROPS * pProps, int Index);







 
void CALENDAR_Callback(WM_MESSAGE * pMsg);








 
#line 60 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\CHECKBOX.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\CHECKBOX.h"
#line 59 "..\\STemWin\\inc\\CHECKBOX.h"
#line 60 "..\\STemWin\\inc\\CHECKBOX.h"












 




 






 
#line 91 "..\\STemWin\\inc\\CHECKBOX.h"




 








 
typedef signed long CHECKBOX_Handle;

typedef struct {
  GUI_COLOR aColorFrame[3];
  GUI_COLOR aColorInner[2];
  GUI_COLOR ColorCheck;
  int       ButtonSize;
} CHECKBOX_SKINFLEX_PROPS;






 
CHECKBOX_Handle CHECKBOX_Create        (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int Flags);
CHECKBOX_Handle CHECKBOX_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
CHECKBOX_Handle CHECKBOX_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
CHECKBOX_Handle CHECKBOX_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void CHECKBOX_Callback(WM_MESSAGE * pMsg);






 

int              CHECKBOX_GetDefaultAlign     (void);
GUI_COLOR        CHECKBOX_GetDefaultBkColor   (void);
const GUI_FONT * CHECKBOX_GetDefaultFont      (void);
int              CHECKBOX_GetDefaultSpacing   (void);
int              CHECKBOX_GetDefaultTextAlign (void);
GUI_COLOR        CHECKBOX_GetDefaultTextColor (void);
int              CHECKBOX_GetUserData         (CHECKBOX_Handle hObj, void * pDest, int NumBytes);
void             CHECKBOX_SetDefaultAlign     (int Align);
void             CHECKBOX_SetDefaultBkColor   (GUI_COLOR Color);
GUI_COLOR        CHECKBOX_SetDefaultFocusColor(GUI_COLOR Color);
void             CHECKBOX_SetDefaultFont      (const GUI_FONT * pFont);
void             CHECKBOX_SetDefaultImage     (const GUI_BITMAP * pBitmap, unsigned int Index);
void             CHECKBOX_SetDefaultSpacing   (int Spacing);
void             CHECKBOX_SetDefaultTextAlign (int Align);
void             CHECKBOX_SetDefaultTextColor (GUI_COLOR Color);






 

int       CHECKBOX_GetState     (CHECKBOX_Handle hObj);
int       CHECKBOX_GetText      (CHECKBOX_Handle hObj, char * pBuffer, int MaxLen);
int       CHECKBOX_IsChecked    (CHECKBOX_Handle hObj);
void      CHECKBOX_SetBkColor   (CHECKBOX_Handle hObj, GUI_COLOR Color);
GUI_COLOR CHECKBOX_SetBoxBkColor(CHECKBOX_Handle hObj, GUI_COLOR Color, int Index);
GUI_COLOR CHECKBOX_SetFocusColor(CHECKBOX_Handle hObj, GUI_COLOR Color);
void      CHECKBOX_SetFont      (CHECKBOX_Handle hObj, const GUI_FONT * pFont);
void      CHECKBOX_SetImage     (CHECKBOX_Handle hObj, const GUI_BITMAP * pBitmap, unsigned int Index);
void      CHECKBOX_SetNumStates (CHECKBOX_Handle hObj, unsigned NumStates);
void      CHECKBOX_SetSpacing   (CHECKBOX_Handle hObj, unsigned Spacing);
void      CHECKBOX_SetState     (CHECKBOX_Handle hObj, unsigned State);
void      CHECKBOX_SetText      (CHECKBOX_Handle hObj, const char * pText);
void      CHECKBOX_SetTextAlign (CHECKBOX_Handle hObj, int Align);
void      CHECKBOX_SetTextColor (CHECKBOX_Handle hObj, GUI_COLOR Color);
int       CHECKBOX_SetUserData  (CHECKBOX_Handle hObj, const void * pSrc, int NumBytes);






 
void CHECKBOX_GetSkinFlexProps      (CHECKBOX_SKINFLEX_PROPS * pProps, int Index);
void CHECKBOX_SetSkinClassic        (CHECKBOX_Handle hObj);
void CHECKBOX_SetSkin               (CHECKBOX_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
int  CHECKBOX_DrawSkinFlex          (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void CHECKBOX_SetSkinFlexProps      (const CHECKBOX_SKINFLEX_PROPS * pProps, int Index);
void CHECKBOX_SetDefaultSkinClassic (void);
int  CHECKBOX_GetSkinFlexButtonSize (CHECKBOX_Handle hObj);
void CHECKBOX_SetSkinFlexButtonSize (CHECKBOX_Handle hObj, int ButtonSize);
WIDGET_DRAW_ITEM_FUNC * CHECKBOX_SetDefaultSkin(WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);








 














 
#line 61 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\CHOOSECOLOR.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\CHOOSECOLOR.h"












 










 



 
typedef struct {
  unsigned  aBorder[2];
  unsigned  aSpace[2];
  unsigned  aButtonSize[2];
  GUI_COLOR aColor[2];
} CHOOSECOLOR_PROPS;




 
typedef struct {
  unsigned long               LastColor;
  const GUI_COLOR * pColor;
  unsigned          NumColors;
  unsigned          NumColorsPerLine;
  int               SelOld;
  int               Sel;
  GUI_HWIN           hParent;
  CHOOSECOLOR_PROPS Props;
} CHOOSECOLOR_CONTEXT;






 
GUI_HWIN CHOOSECOLOR_Create(GUI_HWIN           hParent,
                           int               xPos,
                           int               yPos,
                           int               xSize,
                           int               ySize,
                           const GUI_COLOR * pColor,
                           unsigned          NumColors,
                           unsigned          NumColorsPerLine,
                           int               Sel,
                           const char      * sCaption,
                           int               Flags);

int  CHOOSECOLOR_GetSel(GUI_HWIN hObj);
void CHOOSECOLOR_SetSel(GUI_HWIN hObj, int Sel);

void CHOOSECOLOR_SetDefaultColor     (unsigned Index, GUI_COLOR Color);
void CHOOSECOLOR_SetDefaultSpace     (unsigned Index, unsigned Space);
void CHOOSECOLOR_SetDefaultBorder    (unsigned Index, unsigned Border);
void CHOOSECOLOR_SetDefaultButtonSize(unsigned Index, unsigned ButtonSize);







 
void CHOOSECOLOR_Callback(WM_MESSAGE * pMsg);







#line 62 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\CHOOSEFILE.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\CHOOSEFILE.h"












 


















 



 
typedef struct CHOOSEFILE_INFO CHOOSEFILE_INFO;

struct CHOOSEFILE_INFO {
  int               Cmd;                                 
  int               Id;                                  
  const char      * pMask;                               
  char            * pName;                               
  char            * pExt;                                
  char            * pAttrib;                             
  WM_TOOLTIP_HANDLE hToolTip;                            
  unsigned long               SizeL;                               
  unsigned long               SizeH;                               
  unsigned long               Flags;                               
  char              pRoot[256];            
  int            (* pfGetData)(CHOOSEFILE_INFO * pInfo); 
};






 
GUI_HWIN CHOOSEFILE_Create(GUI_HWIN           hParent,  
                          int               xPos,     
                          int               yPos,     
                          int               xSize,    
                          int               ySize,    
                          const char      * apRoot[], 
                          int               NumRoot,  
                          int               SelRoot,  
                          const char      * sCaption, 
                          int               Flags,    
                          CHOOSEFILE_INFO * pInfo     
                          );

void    CHOOSEFILE_Callback            (WM_MESSAGE * pMsg);
void    CHOOSEFILE_EnableToolTips      (void);
void    CHOOSEFILE_SetButtonText       (GUI_HWIN hWin, unsigned ButtonIndex, const char * pText);
void    CHOOSEFILE_SetDefaultButtonText(unsigned ButtonIndex, const char * pText);
void    CHOOSEFILE_SetDelim            (char Delim);
void    CHOOSEFILE_SetToolTips         (const TOOLTIP_INFO * pInfo, int NumItems);
void    CHOOSEFILE_SetTopMode          (unsigned OnOff);







#line 63 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\DROPDOWN.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\DROPDOWN.h"
#line 59 "..\\STemWin\\inc\\DROPDOWN.h"
#line 1 "..\\STemWin\\inc\\LISTBOX.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\LISTBOX.h"
#line 59 "..\\STemWin\\inc\\LISTBOX.h"
#line 60 "..\\STemWin\\inc\\LISTBOX.h"












 






 








 
typedef signed long LISTBOX_Handle;







 





 
#line 112 "..\\STemWin\\inc\\LISTBOX.h"






 

LISTBOX_Handle LISTBOX_Create        (const GUI_ConstString * ppText, int x0, int y0, int xSize, int ySize, int Flags);
LISTBOX_Handle LISTBOX_CreateAsChild (const GUI_ConstString * ppText, GUI_HWIN hWinParent, int x0, int y0, int xSize, int ySize, int Flags);
LISTBOX_Handle LISTBOX_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, const GUI_ConstString * ppText);
LISTBOX_Handle LISTBOX_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, const GUI_ConstString * ppText, int NumExtraBytes);
LISTBOX_Handle LISTBOX_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void LISTBOX_Callback(WM_MESSAGE * pMsg);






 

int          LISTBOX_AddKey          (LISTBOX_Handle hObj, int Key);
void         LISTBOX_AddString       (LISTBOX_Handle hObj, const char * s);
void         LISTBOX_AddStringH      (LISTBOX_Handle hObj, signed long hString);  
void         LISTBOX_DecSel          (LISTBOX_Handle hObj);
void         LISTBOX_DeleteItem      (LISTBOX_Handle hObj, unsigned int Index);
void         LISTBOX_EnableWrapMode  (LISTBOX_Handle hObj, int OnOff);
unsigned     LISTBOX_GetItemSpacing  (LISTBOX_Handle hObj);
unsigned     LISTBOX_GetNumItems     (LISTBOX_Handle hObj);
int          LISTBOX_GetSel          (LISTBOX_Handle hObj);
const GUI_FONT * LISTBOX_GetFont     (LISTBOX_Handle hObj);
int          LISTBOX_GetItemDisabled (LISTBOX_Handle hObj, unsigned Index);
int          LISTBOX_GetItemSel      (LISTBOX_Handle hObj, unsigned Index);
void         LISTBOX_GetItemText     (LISTBOX_Handle hObj, unsigned Index, char * pBuffer, int MaxSize);
int          LISTBOX_GetMulti        (LISTBOX_Handle hObj);
int          LISTBOX_GetScrollStepH  (LISTBOX_Handle hObj);
int          LISTBOX_GetTextAlign    (LISTBOX_Handle hObj);
int          LISTBOX_GetUserData     (LISTBOX_Handle hObj, void * pDest, int NumBytes);
void         LISTBOX_IncSel          (LISTBOX_Handle hObj);
void         LISTBOX_InsertString    (LISTBOX_Handle hObj, const char * s, unsigned int Index);
void         LISTBOX_InvalidateItem  (LISTBOX_Handle hObj, int Index);
int          LISTBOX_OwnerDraw       (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void         LISTBOX_SetAutoScrollH  (LISTBOX_Handle hObj, int OnOff);
void         LISTBOX_SetAutoScrollV  (LISTBOX_Handle hObj, int OnOff);
void         LISTBOX_SetBkColor      (LISTBOX_Handle hObj, unsigned int Index, GUI_COLOR color);
void         LISTBOX_SetFont         (LISTBOX_Handle hObj, const GUI_FONT * pFont);
void         LISTBOX_SetItemDisabled (LISTBOX_Handle hObj, unsigned Index, int OnOff);
void         LISTBOX_SetItemSel      (LISTBOX_Handle hObj, unsigned Index, int OnOff);
void         LISTBOX_SetItemSpacing  (LISTBOX_Handle hObj, unsigned Value);
void         LISTBOX_SetMulti        (LISTBOX_Handle hObj, int Mode);
void         LISTBOX_SetOwner        (LISTBOX_Handle hObj, GUI_HWIN hOwner);
void         LISTBOX_SetOwnerDraw    (LISTBOX_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawItem);
void         LISTBOX_SetScrollStepH  (LISTBOX_Handle hObj, int Value);
void         LISTBOX_SetSel          (LISTBOX_Handle hObj, int Sel);
void         LISTBOX_SetScrollbarColor(LISTBOX_Handle hObj, unsigned Index, GUI_COLOR Color);
void         LISTBOX_SetScrollbarWidth(LISTBOX_Handle hObj, unsigned Width);
void         LISTBOX_SetString       (LISTBOX_Handle hObj, const char * s, unsigned int Index);
void         LISTBOX_SetText         (LISTBOX_Handle hObj, const GUI_ConstString * ppText);
void         LISTBOX_SetTextAlign    (LISTBOX_Handle hObj, int Align);
GUI_COLOR    LISTBOX_SetTextColor    (LISTBOX_Handle hObj, unsigned int Index, GUI_COLOR Color);
int          LISTBOX_SetUserData     (LISTBOX_Handle hObj, const void * pSrc, int NumBytes);
int          LISTBOX_UpdateScrollers (LISTBOX_Handle hObj);






 

const GUI_FONT * LISTBOX_GetDefaultFont(void);
int              LISTBOX_GetDefaultScrollStepH (void);
GUI_COLOR        LISTBOX_GetDefaultBkColor     (unsigned Index);
int              LISTBOX_GetDefaultTextAlign   (void);
GUI_COLOR        LISTBOX_GetDefaultTextColor   (unsigned Index);
void             LISTBOX_SetDefaultFont        (const GUI_FONT * pFont);
void             LISTBOX_SetDefaultScrollStepH (int Value);
void             LISTBOX_SetDefaultBkColor     (unsigned Index, GUI_COLOR Color);
void             LISTBOX_SetDefaultTextAlign   (int Align);
void             LISTBOX_SetDefaultTextColor   (unsigned Index, GUI_COLOR Color);






 











 
#line 60 "..\\STemWin\\inc\\DROPDOWN.h"










 






 










 










 
typedef signed long DROPDOWN_Handle;

typedef struct {
  GUI_COLOR aColorFrame[3];
  GUI_COLOR aColorUpper[2];
  GUI_COLOR aColorLower[2];
  GUI_COLOR ColorArrow;
  GUI_COLOR ColorText;
  GUI_COLOR ColorSep;
  int Radius;
} DROPDOWN_SKINFLEX_PROPS;






 
DROPDOWN_Handle DROPDOWN_Create        (GUI_HWIN hWinParent, int x0, int y0, int xSize, int ySize, int Flags);
DROPDOWN_Handle DROPDOWN_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
DROPDOWN_Handle DROPDOWN_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
DROPDOWN_Handle DROPDOWN_CreateIndirect(const GUI_WIDGET_CREATE_INFO* pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK* cb);







 
void DROPDOWN_Callback(WM_MESSAGE * pMsg);






 
void     DROPDOWN_AddKey           (DROPDOWN_Handle hObj, int Key);
void     DROPDOWN_AddString        (DROPDOWN_Handle hObj, const char* s);
void     DROPDOWN_Collapse         (DROPDOWN_Handle hObj);
void     DROPDOWN_DecSel           (DROPDOWN_Handle hObj);
void     DROPDOWN_DecSelExp        (DROPDOWN_Handle hObj);
void     DROPDOWN_DeleteItem       (DROPDOWN_Handle hObj, unsigned int Index);
void     DROPDOWN_Expand           (DROPDOWN_Handle hObj);
unsigned DROPDOWN_GetItemDisabled  (DROPDOWN_Handle hObj, unsigned Index);
unsigned DROPDOWN_GetItemSpacing   (DROPDOWN_Handle hObj);
int      DROPDOWN_GetItemText      (DROPDOWN_Handle hObj, unsigned Index, char * pBuffer, int MaxSize);
LISTBOX_Handle DROPDOWN_GetListbox (DROPDOWN_Handle hObj);
int      DROPDOWN_GetNumItems      (DROPDOWN_Handle hObj);
int      DROPDOWN_GetSel           (DROPDOWN_Handle hObj);
int      DROPDOWN_GetSelExp        (DROPDOWN_Handle hObj);
int      DROPDOWN_GetUserData      (DROPDOWN_Handle hObj, void * pDest, int NumBytes);
void     DROPDOWN_IncSel           (DROPDOWN_Handle hObj);
void     DROPDOWN_IncSelExp        (DROPDOWN_Handle hObj);
void     DROPDOWN_InsertString     (DROPDOWN_Handle hObj, const char* s, unsigned int Index);
void     DROPDOWN_SetAutoScroll    (DROPDOWN_Handle hObj, int OnOff);
void     DROPDOWN_SetBkColor       (DROPDOWN_Handle hObj, unsigned int Index, GUI_COLOR color);
void     DROPDOWN_SetColor         (DROPDOWN_Handle hObj, unsigned int Index, GUI_COLOR Color);
void     DROPDOWN_SetFont          (DROPDOWN_Handle hObj, const GUI_FONT * pfont);
void     DROPDOWN_SetItemDisabled  (DROPDOWN_Handle hObj, unsigned Index, int OnOff);
void     DROPDOWN_SetItemSpacing   (DROPDOWN_Handle hObj, unsigned Value);
int      DROPDOWN_SetListHeight    (DROPDOWN_Handle hObj, unsigned Height);
void     DROPDOWN_SetScrollbarColor(DROPDOWN_Handle hObj, unsigned Index, GUI_COLOR Color);
void     DROPDOWN_SetScrollbarWidth(DROPDOWN_Handle hObj, unsigned Width);
void     DROPDOWN_SetSel           (DROPDOWN_Handle hObj, int Sel);
void     DROPDOWN_SetSelExp        (DROPDOWN_Handle hObj, int Sel);
void     DROPDOWN_SetTextAlign     (DROPDOWN_Handle hObj, int Align);
void     DROPDOWN_SetTextColor     (DROPDOWN_Handle hObj, unsigned int index, GUI_COLOR color);
void     DROPDOWN_SetTextHeight    (DROPDOWN_Handle hObj, unsigned TextHeight);
int      DROPDOWN_SetUpMode        (DROPDOWN_Handle hObj, int OnOff);
int      DROPDOWN_SetUserData      (DROPDOWN_Handle hObj, const void * pSrc, int NumBytes);






 
void DROPDOWN_GetSkinFlexProps     (DROPDOWN_SKINFLEX_PROPS * pProps, int Index);
void DROPDOWN_SetSkinClassic       (DROPDOWN_Handle hObj);
void DROPDOWN_SetSkin              (DROPDOWN_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
int  DROPDOWN_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void DROPDOWN_SetSkinFlexProps     (const DROPDOWN_SKINFLEX_PROPS * pProps, int Index);
void DROPDOWN_SetDefaultSkinClassic(void);
WIDGET_DRAW_ITEM_FUNC * DROPDOWN_SetDefaultSkin(WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);








 
GUI_COLOR        DROPDOWN_GetDefaultBkColor       (int Index);
GUI_COLOR        DROPDOWN_GetDefaultColor         (int Index);
const GUI_FONT * DROPDOWN_GetDefaultFont          (void);
GUI_COLOR        DROPDOWN_GetDefaultScrollbarColor(int Index);
void             DROPDOWN_SetDefaultFont          (const GUI_FONT * pFont);
GUI_COLOR        DROPDOWN_SetDefaultBkColor       (int Index, GUI_COLOR Color);
GUI_COLOR        DROPDOWN_SetDefaultColor         (int Index, GUI_COLOR Color);
GUI_COLOR        DROPDOWN_SetDefaultScrollbarColor(int Index, GUI_COLOR Color);








 
#line 64 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\EDIT.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\EDIT.h"
#line 59 "..\\STemWin\\inc\\EDIT.h"















 





 



#line 90 "..\\STemWin\\inc\\EDIT.h"






































 
typedef signed long EDIT_Handle;
typedef void tEDIT_AddKeyEx    (EDIT_Handle hObj, int Key);
typedef void tEDIT_UpdateBuffer(EDIT_Handle hObj);




 
EDIT_Handle EDIT_Create        (int x0, int y0, int xSize, int ySize, int Id, int MaxLen, int Flags);
EDIT_Handle EDIT_CreateAsChild (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int Flags, int MaxLen);
EDIT_Handle EDIT_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int MaxLen);
EDIT_Handle EDIT_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int MaxLen, int NumExtraBytes);
EDIT_Handle EDIT_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void EDIT_Callback(WM_MESSAGE * pMsg);






 
void EDIT_SetDefaultBkColor  (unsigned int Index, GUI_COLOR Color);
void EDIT_SetDefaultFont     (const GUI_FONT * pFont);
void EDIT_SetDefaultTextAlign(int Align);
void EDIT_SetDefaultTextColor(unsigned int Index, GUI_COLOR Color);




 



GUI_COLOR        EDIT_GetDefaultBkColor(unsigned int Index);
const GUI_FONT * EDIT_GetDefaultFont(void);
int              EDIT_GetDefaultTextAlign(void);
GUI_COLOR        EDIT_GetDefaultTextColor(unsigned int Index);



void EDIT_AddKey           (EDIT_Handle hObj, int Key);
void EDIT_EnableBlink      (EDIT_Handle hObj, int Period, int OnOff);
GUI_COLOR EDIT_GetBkColor  (EDIT_Handle hObj, unsigned int Index);
void EDIT_SetBkColor       (EDIT_Handle hObj, unsigned int Index, GUI_COLOR color);
void EDIT_SetCursorAtChar  (EDIT_Handle hObj, int Pos);
void EDIT_SetCursorAtPixel (EDIT_Handle hObj, int xPos);
void EDIT_SetFocussable    (EDIT_Handle hObj, int State);
void EDIT_SetFont          (EDIT_Handle hObj, const GUI_FONT * pFont);
int  EDIT_SetInsertMode    (EDIT_Handle hObj, int OnOff);
void EDIT_SetMaxLen        (EDIT_Handle hObj, int MaxLen);
void EDIT_SetpfAddKeyEx    (EDIT_Handle hObj, tEDIT_AddKeyEx * pfAddKeyEx);
void EDIT_SetpfUpdateBuffer(EDIT_Handle hObj, tEDIT_UpdateBuffer * pfUpdateBuffer);
void EDIT_SetText          (EDIT_Handle hObj, const char * s);
void EDIT_SetTextAlign     (EDIT_Handle hObj, int Align);
GUI_COLOR EDIT_GetTextColor(EDIT_Handle hObj, unsigned int Index);
void EDIT_SetTextColor     (EDIT_Handle hObj, unsigned int Index, GUI_COLOR Color);
void EDIT_SetSel           (EDIT_Handle hObj, int FirstChar, int LastChar);
int  EDIT_SetUserData      (EDIT_Handle hObj, const void * pSrc, int NumBytes);
int  EDIT_EnableInversion  (EDIT_Handle hObj, int OnOff);



int   EDIT_GetCursorCharPos  (EDIT_Handle hObj);
void  EDIT_GetCursorPixelPos (EDIT_Handle hObj, int * pxPos, int * pyPos);
float EDIT_GetFloatValue     (EDIT_Handle hObj);
const GUI_FONT * EDIT_GetFont(EDIT_Handle hObj);
int   EDIT_GetNumChars       (EDIT_Handle hObj);
void  EDIT_GetText           (EDIT_Handle hObj, char * sDest, int MaxLen);
signed long   EDIT_GetValue          (EDIT_Handle hObj);
void  EDIT_SetFloatValue     (EDIT_Handle hObj, float Value);
int   EDIT_GetUserData       (EDIT_Handle hObj, void * pDest, int NumBytes);
void  EDIT_SetValue          (EDIT_Handle hObj, signed long Value);






 
void  EDIT_SetHexMode  (EDIT_Handle hEdit, unsigned long Value, unsigned long Min, unsigned long Max);
void  EDIT_SetBinMode  (EDIT_Handle hEdit, unsigned long Value, unsigned long Min, unsigned long Max);
void  EDIT_SetDecMode  (EDIT_Handle hEdit, signed long Value, signed long Min, signed long Max, int Shift, unsigned char Flags);
void  EDIT_SetFloatMode(EDIT_Handle hEdit, float Value, float Min, float Max, int Shift, unsigned char Flags);
void  EDIT_SetTextMode (EDIT_Handle hEdit);
void  EDIT_SetUlongMode(EDIT_Handle hEdit, unsigned long Value, unsigned long Min, unsigned long Max);

unsigned long   GUI_EditHex      (unsigned long Value, unsigned long Min, unsigned long Max, int Len, int xSize);
unsigned long   GUI_EditBin      (unsigned long Value, unsigned long Min, unsigned long Max, int Len, int xSize);
signed long   GUI_EditDec      (signed long Value, signed long Min, signed long Max, int Len, int xSize, int Shift, unsigned char Flags);
float GUI_EditFloat    (float Value, float Min, float Max, int Len, int xSize, int Shift, unsigned char Flags);
void  GUI_EditString   (char * pString, int Len, int xSize);








 
#line 65 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\FRAMEWIN.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\FRAMEWIN.h"
#line 59 "..\\STemWin\\inc\\FRAMEWIN.h"
#line 61 "..\\STemWin\\inc\\FRAMEWIN.h"










 



 







 






 
#line 97 "..\\STemWin\\inc\\FRAMEWIN.h"

#line 104 "..\\STemWin\\inc\\FRAMEWIN.h"




 






 






 










 
typedef signed long FRAMEWIN_Handle;

typedef struct {
  GUI_COLOR aColorFrame[3];
  GUI_COLOR aColorTitle[2];
  int Radius;
  int SpaceX;
  int BorderSizeL;
  int BorderSizeR;
  int BorderSizeT;
  int BorderSizeB;
} FRAMEWIN_SKINFLEX_PROPS;






 
FRAMEWIN_Handle FRAMEWIN_Create        (const char * pTitle, WM_CALLBACK * cb, int Flags, int x0, int y0, int xSize, int ySize);
FRAMEWIN_Handle FRAMEWIN_CreateAsChild (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, const char * pText, WM_CALLBACK * cb, int Flags);
FRAMEWIN_Handle FRAMEWIN_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, const char * pTitle, WM_CALLBACK * cb);
FRAMEWIN_Handle FRAMEWIN_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, const char * pTitle, WM_CALLBACK * cb, int NumExtraBytes);
FRAMEWIN_Handle FRAMEWIN_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void FRAMEWIN_Callback(WM_MESSAGE * pMsg);






 
GUI_HWIN FRAMEWIN_AddButton     (FRAMEWIN_Handle hObj, int Flags, int Off, int Id);
GUI_HWIN FRAMEWIN_AddCloseButton(FRAMEWIN_Handle hObj, int Flags, int Off);
GUI_HWIN FRAMEWIN_AddMaxButton  (FRAMEWIN_Handle hObj, int Flags, int Off);
void    FRAMEWIN_AddMenu       (FRAMEWIN_Handle hObj, GUI_HWIN hMenu);
GUI_HWIN FRAMEWIN_AddMinButton  (FRAMEWIN_Handle hObj, int Flags, int Off);
void    FRAMEWIN_Minimize      (FRAMEWIN_Handle hObj);
void    FRAMEWIN_Maximize      (FRAMEWIN_Handle hObj);
void    FRAMEWIN_Restore       (FRAMEWIN_Handle hObj);
void    FRAMEWIN_SetActive     (FRAMEWIN_Handle hObj, int State);
void    FRAMEWIN_SetBarColor   (FRAMEWIN_Handle hObj, unsigned Index, GUI_COLOR Color);
void    FRAMEWIN_SetBorderSize (FRAMEWIN_Handle hObj, unsigned Size);
void    FRAMEWIN_SetClientColor(FRAMEWIN_Handle hObj, GUI_COLOR Color);
void    FRAMEWIN_SetFont       (FRAMEWIN_Handle hObj, const GUI_FONT * pFont);
void    FRAMEWIN_SetMoveable   (FRAMEWIN_Handle hObj, int State);
void    FRAMEWIN_SetOwnerDraw  (FRAMEWIN_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawItem);
void    FRAMEWIN_SetResizeable (FRAMEWIN_Handle hObj, int State);
void    FRAMEWIN_SetText       (FRAMEWIN_Handle hObj, const char* s);
void    FRAMEWIN_SetTextAlign  (FRAMEWIN_Handle hObj, int Align);
void    FRAMEWIN_SetTextColor  (FRAMEWIN_Handle hObj, GUI_COLOR Color);
void    FRAMEWIN_SetTextColorEx(FRAMEWIN_Handle hObj, unsigned Index, GUI_COLOR Color);
void    FRAMEWIN_SetTitleVis   (FRAMEWIN_Handle hObj, int Show);
int     FRAMEWIN_SetTitleHeight(FRAMEWIN_Handle hObj, int Height);
int     FRAMEWIN_SetUserData   (FRAMEWIN_Handle hObj, const void * pSrc, int NumBytes);






 
void FRAMEWIN_GetSkinFlexProps     (FRAMEWIN_SKINFLEX_PROPS * pProps, int Index);
void FRAMEWIN_SetSkinClassic       (FRAMEWIN_Handle hObj);
void FRAMEWIN_SetSkin              (FRAMEWIN_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
int  FRAMEWIN_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void FRAMEWIN_SetSkinFlexProps     (const FRAMEWIN_SKINFLEX_PROPS * pProps, int Index);
void FRAMEWIN_SetDefaultSkinClassic(void);
WIDGET_DRAW_ITEM_FUNC * FRAMEWIN_SetDefaultSkin(WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);








 
const GUI_FONT * FRAMEWIN_GetFont(FRAMEWIN_Handle hObj);

int       FRAMEWIN_GetActive      (FRAMEWIN_Handle hObj);
int       FRAMEWIN_GetTitleHeight (FRAMEWIN_Handle hObj);
GUI_COLOR FRAMEWIN_GetBarColor    (FRAMEWIN_Handle hObj, unsigned Index);
int       FRAMEWIN_GetBorderSize  (FRAMEWIN_Handle hObj);
int       FRAMEWIN_GetBorderSizeEx(FRAMEWIN_Handle hObj, unsigned Edge);
void      FRAMEWIN_GetText        (FRAMEWIN_Handle hObj, char * pBuffer, int MaxLen);
int       FRAMEWIN_GetTextAlign   (FRAMEWIN_Handle hObj);
int       FRAMEWIN_GetUserData    (FRAMEWIN_Handle hObj, void * pDest, int NumBytes);
int       FRAMEWIN_IsMinimized    (FRAMEWIN_Handle hObj);
int       FRAMEWIN_IsMaximized    (FRAMEWIN_Handle hObj);






 
GUI_COLOR        FRAMEWIN_GetDefaultBarColor   (unsigned Index);
int              FRAMEWIN_GetDefaultBorderSize (void);
int              FRAMEWIN_GetDefaultTitleHeight(void);
GUI_COLOR        FRAMEWIN_GetDefaultClientColor(void);
const GUI_FONT * FRAMEWIN_GetDefaultFont       (void);
GUI_COLOR        FRAMEWIN_GetDefaultTextColor  (unsigned Index);
int              FRAMEWIN_OwnerDraw            (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void             FRAMEWIN_SetDefaultBarColor   (unsigned Index, GUI_COLOR Color);
void             FRAMEWIN_SetDefaultBorderSize (int DefaultBorderSize);
void             FRAMEWIN_SetDefaultTitleHeight(int DefaultTitleHeight);
void             FRAMEWIN_SetDefaultClientColor(GUI_COLOR Color);
void             FRAMEWIN_SetDefaultFont       (const GUI_FONT * pFont);
int              FRAMEWIN_SetDefaultTextAlign  (int TextAlign);
void             FRAMEWIN_SetDefaultTextColor  (unsigned Index, GUI_COLOR Color);






 
#line 266 "..\\STemWin\\inc\\FRAMEWIN.h"








 
#line 66 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\GRAPH.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\GRAPH.h"
#line 59 "..\\STemWin\\inc\\GRAPH.h"
#line 60 "..\\STemWin\\inc\\GRAPH.h"












 




































 
typedef signed long GRAPH_Handle;
typedef signed long GRAPH_DATA_Handle;
typedef signed long GRAPH_SCALE_Handle;






 

GRAPH_Handle GRAPH_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
GRAPH_Handle GRAPH_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
GRAPH_Handle GRAPH_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);

GRAPH_DATA_Handle  GRAPH_DATA_XY_Create(GUI_COLOR Color, unsigned MaxNumItems, GUI_POINT * pData, unsigned NumItems);
GRAPH_DATA_Handle  GRAPH_DATA_YT_Create(GUI_COLOR Color, unsigned MaxNumItems, signed short * pData, unsigned NumItems);
GRAPH_SCALE_Handle GRAPH_SCALE_Create  (int Pos, int TextAlign, unsigned Flags, unsigned TickDist);







 
void GRAPH_Callback(WM_MESSAGE * pMsg);






 
void      GRAPH_AttachData             (GRAPH_Handle hObj, GRAPH_DATA_Handle hData);
void      GRAPH_AttachScale            (GRAPH_Handle hObj, GRAPH_SCALE_Handle hScale);
void      GRAPH_DetachData             (GRAPH_Handle hObj, GRAPH_DATA_Handle hData);
void      GRAPH_DetachScale            (GRAPH_Handle hObj, GRAPH_SCALE_Handle hScale);
signed long       GRAPH_GetScrollValue         (GRAPH_Handle hObj, unsigned char Coord);
int       GRAPH_GetUserData            (GRAPH_Handle hObj, void * pDest, int NumBytes);
void      GRAPH_SetAutoScrollbar       (GRAPH_Handle hObj, unsigned char Coord, unsigned char OnOff);
void      GRAPH_SetBorder              (GRAPH_Handle hObj, unsigned BorderL, unsigned BorderT, unsigned BorderR, unsigned BorderB);
GUI_COLOR GRAPH_SetColor               (GRAPH_Handle hObj, GUI_COLOR Color, unsigned Index);
unsigned  GRAPH_SetGridFixedX          (GRAPH_Handle hObj, unsigned OnOff);
unsigned  GRAPH_SetGridOffY            (GRAPH_Handle hObj, unsigned Value);
unsigned  GRAPH_SetGridVis             (GRAPH_Handle hObj, unsigned OnOff);
unsigned  GRAPH_SetGridDistX           (GRAPH_Handle hObj, unsigned Value);
unsigned  GRAPH_SetGridDistY           (GRAPH_Handle hObj, unsigned Value);
unsigned char        GRAPH_SetLineStyleH          (GRAPH_Handle hObj, unsigned char Value);
unsigned char        GRAPH_SetLineStyleV          (GRAPH_Handle hObj, unsigned char Value);
void      GRAPH_SetLineStyle           (GRAPH_Handle hObj, unsigned char Value);
void      GRAPH_SetScrollValue         (GRAPH_Handle hObj, unsigned char Coord, unsigned long Value);
unsigned  GRAPH_SetVSizeX              (GRAPH_Handle hObj, unsigned Value);
unsigned  GRAPH_SetVSizeY              (GRAPH_Handle hObj, unsigned Value);
int       GRAPH_SetUserData            (GRAPH_Handle hObj, const void * pSrc, int NumBytes);
void      GRAPH_SetUserDraw            (GRAPH_Handle hObj, void (* pOwnerDraw)(GUI_HWIN, int));

void      GRAPH_DATA_YT_AddValue       (GRAPH_DATA_Handle hDataObj, signed short Value);
void      GRAPH_DATA_YT_Clear          (GRAPH_DATA_Handle hDataObj);
void      GRAPH_DATA_YT_Delete         (GRAPH_DATA_Handle hDataObj);
void      GRAPH_DATA_YT_SetAlign       (GRAPH_DATA_Handle hDataObj, int Align);
void      GRAPH_DATA_YT_SetOffY        (GRAPH_DATA_Handle hDataObj, int Off);
void      GRAPH_DATA_YT_MirrorX        (GRAPH_DATA_Handle hDataObj, int OnOff);

void      GRAPH_DATA_XY_AddPoint       (GRAPH_DATA_Handle hDataObj, GUI_POINT * pPoint);
void      GRAPH_DATA_XY_Clear          (GRAPH_DATA_Handle hDataObj);
void      GRAPH_DATA_XY_Delete         (GRAPH_DATA_Handle hDataObj);
unsigned  GRAPH_DATA_XY_GetLineVis     (GRAPH_DATA_Handle hDataObj);
unsigned  GRAPH_DATA_XY_GetPointVis    (GRAPH_DATA_Handle hDataObj);
void      GRAPH_DATA_XY_SetLineStyle   (GRAPH_DATA_Handle hDataObj, unsigned char LineStyle);
unsigned  GRAPH_DATA_XY_SetLineVis     (GRAPH_DATA_Handle hDataObj, unsigned OnOff);
void      GRAPH_DATA_XY_SetOffX        (GRAPH_DATA_Handle hDataObj, int Off);
void      GRAPH_DATA_XY_SetOffY        (GRAPH_DATA_Handle hDataObj, int Off);
void      GRAPH_DATA_XY_SetPenSize     (GRAPH_DATA_Handle hDataObj, unsigned char PenSize);
void      GRAPH_DATA_XY_SetPointSize   (GRAPH_DATA_Handle hDataObj, unsigned PointSize);
unsigned  GRAPH_DATA_XY_SetPointVis    (GRAPH_DATA_Handle hDataObj, unsigned OnOff);
void      GRAPH_DATA_XY_SetOwnerDraw   (GRAPH_DATA_Handle hDataObj, WIDGET_DRAW_ITEM_FUNC * pOwnerDraw);

void             GRAPH_SCALE_Delete      (GRAPH_SCALE_Handle hScaleObj);
float            GRAPH_SCALE_SetFactor   (GRAPH_SCALE_Handle hScaleObj, float Factor);
const GUI_FONT * GRAPH_SCALE_SetFont     (GRAPH_SCALE_Handle hScaleObj, const GUI_FONT * pFont);
int              GRAPH_SCALE_SetNumDecs  (GRAPH_SCALE_Handle hScaleObj, int NumDecs);
int              GRAPH_SCALE_SetOff      (GRAPH_SCALE_Handle hScaleObj, int Off);
int              GRAPH_SCALE_SetPos      (GRAPH_SCALE_Handle hScaleObj, int Pos);
GUI_COLOR        GRAPH_SCALE_SetTextColor(GRAPH_SCALE_Handle hScaleObj, GUI_COLOR Color);
unsigned         GRAPH_SCALE_SetTickDist (GRAPH_SCALE_Handle hScaleObj, unsigned Value);








 
#line 67 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\HEADER.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\HEADER.h"
#line 59 "..\\STemWin\\inc\\HEADER.h"
#line 60 "..\\STemWin\\inc\\HEADER.h"












 

typedef signed long HEADER_Handle;

typedef struct {
  GUI_COLOR aColorFrame[2];
  GUI_COLOR aColorUpper[2];
  GUI_COLOR aColorLower[2];
  GUI_COLOR ColorArrow;
} HEADER_SKINFLEX_PROPS;






 

HEADER_Handle HEADER_Create        (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int Flags, int SpecialFlags);
HEADER_Handle HEADER_CreateAttached(GUI_HWIN hParent, int Id, int SpecialFlags);
HEADER_Handle HEADER_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
HEADER_Handle HEADER_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
HEADER_Handle HEADER_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void HEADER_Callback(WM_MESSAGE * pMsg);






 
 
GUI_COLOR          HEADER_SetDefaultArrowColor(GUI_COLOR Color);
GUI_COLOR          HEADER_SetDefaultBkColor   (GUI_COLOR Color);
const GUI_CURSOR * HEADER_SetDefaultCursor    (const GUI_CURSOR * pCursor);
const GUI_FONT *   HEADER_SetDefaultFont      (const GUI_FONT * pFont);
int                HEADER_SetDefaultBorderH   (int Spacing);
int                HEADER_SetDefaultBorderV   (int Spacing);
GUI_COLOR          HEADER_SetDefaultTextColor (GUI_COLOR Color);

 
GUI_COLOR          HEADER_GetDefaultArrowColor(void);
GUI_COLOR          HEADER_GetDefaultBkColor   (void);
const GUI_CURSOR * HEADER_GetDefaultCursor    (void);
const GUI_FONT *   HEADER_GetDefaultFont      (void);
int                HEADER_GetDefaultBorderH   (void);
int                HEADER_GetDefaultBorderV   (void);
GUI_COLOR          HEADER_GetDefaultTextColor (void);






 
void      HEADER_AddItem            (HEADER_Handle hObj, int Width, const char * s, int Align);
void      HEADER_DeleteItem         (HEADER_Handle hObj, unsigned Index);
GUI_COLOR HEADER_GetArrowColor      (HEADER_Handle hObj);
GUI_COLOR HEADER_GetBkColor         (HEADER_Handle hObj);
int       HEADER_GetHeight          (HEADER_Handle hObj);
int       HEADER_GetItemWidth       (HEADER_Handle hObj, unsigned int Index);
int       HEADER_GetNumItems        (HEADER_Handle hObj);
int       HEADER_GetSel             (HEADER_Handle hObj);
GUI_COLOR HEADER_GetTextColor       (HEADER_Handle hObj);
int       HEADER_GetUserData        (HEADER_Handle hObj, void * pDest, int NumBytes);
void      HEADER_SetArrowColor      (HEADER_Handle hObj, GUI_COLOR Color);
void      HEADER_SetBitmap          (HEADER_Handle hObj, unsigned int Index, const GUI_BITMAP * pBitmap);
void      HEADER_SetBitmapEx        (HEADER_Handle hObj, unsigned int Index, const GUI_BITMAP * pBitmap, int x, int y);
void      HEADER_SetBkColor         (HEADER_Handle hObj, GUI_COLOR Color);
void      HEADER_SetBMP             (HEADER_Handle hObj, unsigned int Index, const void * pBitmap);
void      HEADER_SetBMPEx           (HEADER_Handle hObj, unsigned int Index, const void * pBitmap, int x, int y);
void      HEADER_SetDirIndicator    (HEADER_Handle hObj, int Column, int Reverse);  
void      HEADER_SetDragLimit       (HEADER_Handle hObj, unsigned DragLimit);
unsigned  HEADER_SetFixed           (HEADER_Handle hObj, unsigned Fixed);
void      HEADER_SetFont            (HEADER_Handle hObj, const GUI_FONT * pFont);
void      HEADER_SetHeight          (HEADER_Handle hObj, int Height);
void      HEADER_SetTextAlign       (HEADER_Handle hObj, unsigned int Index, int Align);
void      HEADER_SetItemText        (HEADER_Handle hObj, unsigned int Index, const char * s);
void      HEADER_SetItemWidth       (HEADER_Handle hObj, unsigned int Index, int Width);
void      HEADER_SetScrollPos       (HEADER_Handle hObj, int ScrollPos);
void      HEADER_SetStreamedBitmap  (HEADER_Handle hObj, unsigned int Index, const GUI_BITMAP_STREAM * pBitmap);
void      HEADER_SetStreamedBitmapEx(HEADER_Handle hObj, unsigned int Index, const GUI_BITMAP_STREAM * pBitmap, int x, int y);
void      HEADER_SetTextColor       (HEADER_Handle hObj, GUI_COLOR Color);
int       HEADER_SetUserData        (HEADER_Handle hObj, const void * pSrc, int NumBytes);






 
void HEADER_GetSkinFlexProps     (HEADER_SKINFLEX_PROPS * pProps, int Index);
void HEADER_SetSkinClassic       (HEADER_Handle hObj);
void HEADER_SetSkin              (HEADER_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
int  HEADER_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void HEADER_SetSkinFlexProps     (const HEADER_SKINFLEX_PROPS * pProps, int Index);
void HEADER_SetDefaultSkinClassic(void);
WIDGET_DRAW_ITEM_FUNC * HEADER_SetDefaultSkin(WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);








 

#line 198 "..\\STemWin\\inc\\HEADER.h"








 
#line 68 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\ICONVIEW.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\ICONVIEW.h"
#line 59 "..\\STemWin\\inc\\ICONVIEW.h"
#line 60 "..\\STemWin\\inc\\ICONVIEW.h"












 

































 
typedef signed long ICONVIEW_Handle;






 
ICONVIEW_Handle ICONVIEW_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int xSizeItems, int ySizeItems);
ICONVIEW_Handle ICONVIEW_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int xSizeItems, int ySizeItems, int NumExtraBytes);
ICONVIEW_Handle ICONVIEW_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);

int  ICONVIEW_AddBitmapItem           (ICONVIEW_Handle hObj, const GUI_BITMAP * pBitmap, const char * pText);
int  ICONVIEW_AddBMPItem              (ICONVIEW_Handle hObj, const unsigned char * pBMP, const char * pText);
int  ICONVIEW_AddBMPItemEx            (ICONVIEW_Handle hObj, const void * pBMP, GUI_GET_DATA_FUNC * pfGetData, const char * pText);
int  ICONVIEW_AddStreamedBitmapItem   (ICONVIEW_Handle hObj, const void * pStreamedBitmap, const char * pText);
void ICONVIEW_DeleteItem              (ICONVIEW_Handle hObj, unsigned Index);
void ICONVIEW_EnableStreamAuto        (void);
unsigned long  ICONVIEW_GetItemUserData         (ICONVIEW_Handle hObj, int Index);
int  ICONVIEW_GetNumItems             (ICONVIEW_Handle hObj);
int  ICONVIEW_GetItemText             (ICONVIEW_Handle hObj, int Index, char * pBuffer, int MaxSize);
int  ICONVIEW_GetSel                  (ICONVIEW_Handle hObj);
int  ICONVIEW_GetUserData             (ICONVIEW_Handle hObj, void * pDest, int NumBytes);
int  ICONVIEW_InsertBitmapItem        (ICONVIEW_Handle hObj, const GUI_BITMAP * pBitmap, const char * pText, int Index);
int  ICONVIEW_InsertBMPItem           (ICONVIEW_Handle hObj, const unsigned char * pBMP, const char * pText, int Index);
int  ICONVIEW_InsertBMPItemEx         (ICONVIEW_Handle hObj, const void * pBMP, GUI_GET_DATA_FUNC * pfGetData, const char * pText, int Index);
int  ICONVIEW_InsertStreamedBitmapItem(ICONVIEW_Handle hObj, const void * pStreamedBitmap, const char * pText, int Index);
int  ICONVIEW_SetBitmapItem           (ICONVIEW_Handle hObj, int Index, const GUI_BITMAP * pBitmap);
void ICONVIEW_SetBkColor              (ICONVIEW_Handle hObj, int Index, GUI_COLOR Color);
int  ICONVIEW_SetBMPItem              (ICONVIEW_Handle hObj, const unsigned char * pBMP, int Index);
int  ICONVIEW_SetBMPItemEx            (ICONVIEW_Handle hObj, const void * pBMP, GUI_GET_DATA_FUNC * pfGetData, int Index);
void ICONVIEW_SetFont                 (ICONVIEW_Handle hObj, const GUI_FONT * pFont);
void ICONVIEW_SetFrame                (ICONVIEW_Handle hObj, int Coord, int Value);
void ICONVIEW_SetItemText             (ICONVIEW_Handle hObj, int Index, const char * pText);
void ICONVIEW_SetItemUserData         (ICONVIEW_Handle hObj, int Index, unsigned long UserData);
void ICONVIEW_SetSel                  (ICONVIEW_Handle hObj, int Sel);
void ICONVIEW_SetSpace                (ICONVIEW_Handle hObj, int Coord, int Value);
int  ICONVIEW_SetStreamedBitmapItem   (ICONVIEW_Handle hObj, int Index, const void * pStreamedBitmap);
void ICONVIEW_SetIconAlign            (ICONVIEW_Handle hObj, int IconAlign);
void ICONVIEW_SetTextAlign            (ICONVIEW_Handle hObj, int TextAlign);
void ICONVIEW_SetTextColor            (ICONVIEW_Handle hObj, int Index, GUI_COLOR Color);
int  ICONVIEW_SetUserData             (ICONVIEW_Handle hObj, const void * pSrc, int NumBytes);
void ICONVIEW_SetWrapMode             (ICONVIEW_Handle hObj, GUI_WRAPMODE WrapMode);

void ICONVIEW_Callback(WM_MESSAGE * pMsg);








 
#line 69 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\IMAGE.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\IMAGE.h"
#line 59 "..\\STemWin\\inc\\IMAGE.h"
#line 60 "..\\STemWin\\inc\\IMAGE.h"












 











 
typedef signed long IMAGE_Handle;






 
IMAGE_Handle IMAGE_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
IMAGE_Handle IMAGE_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
IMAGE_Handle IMAGE_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);

void IMAGE_Callback(WM_MESSAGE * pMsg);






 
int  IMAGE_GetUserData(IMAGE_Handle hObj, void * pDest, int NumBytes);
void IMAGE_SetBitmap  (IMAGE_Handle hWin, const GUI_BITMAP * pBitmap);
void IMAGE_SetBMP     (IMAGE_Handle hObj, const void * pData, unsigned long FileSize);
void IMAGE_SetBMPEx   (IMAGE_Handle hObj, GUI_GET_DATA_FUNC * pfGetData, void * pVoid);
void IMAGE_SetDTA     (IMAGE_Handle hObj, const void * pData, unsigned long FileSize);
void IMAGE_SetDTAEx   (IMAGE_Handle hObj, GUI_GET_DATA_FUNC * pfGetData, void * pVoid);
void IMAGE_SetGIF     (IMAGE_Handle hObj, const void * pData, unsigned long FileSize);
void IMAGE_SetGIFEx   (IMAGE_Handle hObj, GUI_GET_DATA_FUNC * pfGetData, void * pVoid);
void IMAGE_SetJPEG    (IMAGE_Handle hObj, const void * pData, unsigned long FileSize);
void IMAGE_SetJPEGEx  (IMAGE_Handle hObj, GUI_GET_DATA_FUNC * pfGetData, void * pVoid);
void IMAGE_SetPNG     (IMAGE_Handle hObj, const void * pData, unsigned long FileSize);
void IMAGE_SetPNGEx   (IMAGE_Handle hObj, GUI_GET_DATA_FUNC * pfGetData, void * pVoid);
int  IMAGE_SetUserData(IMAGE_Handle hObj, const void * pSrc, int NumBytes);









 
#line 70 "..\\STemWin\\inc\\DIALOG.h"
#line 71 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\LISTVIEW.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\LISTVIEW.h"
#line 59 "..\\STemWin\\inc\\LISTVIEW.h"
#line 60 "..\\STemWin\\inc\\LISTVIEW.h"
#line 61 "..\\STemWin\\inc\\LISTVIEW.h"










 





 








 











 
typedef signed long LISTVIEW_Handle;






 
LISTVIEW_Handle LISTVIEW_Create        (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int Flags, int SpecialFlags);
LISTVIEW_Handle LISTVIEW_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
LISTVIEW_Handle LISTVIEW_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
LISTVIEW_Handle LISTVIEW_CreateAttached(GUI_HWIN hParent, int Id, int SpecialFlags);
LISTVIEW_Handle LISTVIEW_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void LISTVIEW_Callback(WM_MESSAGE * pMsg);






 
int              LISTVIEW_AddColumn           (LISTVIEW_Handle hObj, int Width, const char * s, int Align);
int              LISTVIEW_AddRow              (LISTVIEW_Handle hObj, const GUI_ConstString * ppText);
int              LISTVIEW_CompareText         (const void * p0, const void * p1);
int              LISTVIEW_CompareDec          (const void * p0, const void * p1);
void             LISTVIEW_DecSel              (LISTVIEW_Handle hObj);
void             LISTVIEW_DeleteAllRows       (LISTVIEW_Handle hObj);
void             LISTVIEW_DeleteColumn        (LISTVIEW_Handle hObj, unsigned Index);
void             LISTVIEW_DeleteRow           (LISTVIEW_Handle hObj, unsigned Index);
void             LISTVIEW_DeleteRowSorted     (LISTVIEW_Handle hObj, int Row);
void             LISTVIEW_DisableRow          (LISTVIEW_Handle hObj, unsigned Row);
void             LISTVIEW_DisableSort         (LISTVIEW_Handle hObj);
void             LISTVIEW_EnableCellSelect    (LISTVIEW_Handle hObj, unsigned OnOff);  
void             LISTVIEW_EnableRow           (LISTVIEW_Handle hObj, unsigned Row);
void             LISTVIEW_EnableSort          (LISTVIEW_Handle hObj);
GUI_COLOR        LISTVIEW_GetBkColor          (LISTVIEW_Handle hObj, unsigned Index);
const GUI_FONT * LISTVIEW_GetFont             (LISTVIEW_Handle hObj);
HEADER_Handle    LISTVIEW_GetHeader           (LISTVIEW_Handle hObj);
void             LISTVIEW_GetItemRect         (LISTVIEW_Handle hObj, unsigned long Col, unsigned long Row, GUI_RECT * pRect);
void             LISTVIEW_GetItemText         (LISTVIEW_Handle hObj, unsigned Column, unsigned Row, char * pBuffer, unsigned MaxSize);
unsigned         LISTVIEW_GetItemTextLen      (LISTVIEW_Handle hObj, unsigned Column, unsigned Row);
void             LISTVIEW_GetItemTextSorted   (LISTVIEW_Handle hObj, unsigned Column, unsigned Row, char * pBuffer, unsigned MaxSize);
unsigned         LISTVIEW_GetLBorder          (LISTVIEW_Handle hObj);
unsigned         LISTVIEW_GetNumColumns       (LISTVIEW_Handle hObj);
unsigned         LISTVIEW_GetNumRows          (LISTVIEW_Handle hObj);
unsigned         LISTVIEW_GetRBorder          (LISTVIEW_Handle hObj);
int              LISTVIEW_GetSel              (LISTVIEW_Handle hObj);
int              LISTVIEW_GetSelCol           (LISTVIEW_Handle hObj);
int              LISTVIEW_GetSelUnsorted      (LISTVIEW_Handle hObj);
int              LISTVIEW_GetTextAlign        (LISTVIEW_Handle hObj, unsigned ColIndex);
GUI_COLOR        LISTVIEW_GetTextColor        (LISTVIEW_Handle hObj, unsigned Index);
int              LISTVIEW_GetUserData         (LISTVIEW_Handle hObj, void * pDest, int NumBytes);
unsigned long              LISTVIEW_GetUserDataRow      (LISTVIEW_Handle hObj, unsigned Row);
GUI_WRAPMODE     LISTVIEW_GetWrapMode         (LISTVIEW_Handle hObj);
void             LISTVIEW_IncSel              (LISTVIEW_Handle hObj);
int              LISTVIEW_InsertRow           (LISTVIEW_Handle hObj, unsigned Index, const GUI_ConstString * ppText);
int              LISTVIEW_OwnerDraw           (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
unsigned         LISTVIEW_RowIsDisabled       (LISTVIEW_Handle hObj, unsigned Row);
void             LISTVIEW_SetAutoScrollH      (LISTVIEW_Handle hObj, int OnOff);
void             LISTVIEW_SetAutoScrollV      (LISTVIEW_Handle hObj, int OnOff);
void             LISTVIEW_SetItemBitmap       (LISTVIEW_Handle hObj, unsigned Column, unsigned Row, int xOff, int yOff, const GUI_BITMAP * pBitmap);
void             LISTVIEW_SetBkColor          (LISTVIEW_Handle hObj, unsigned int Index, GUI_COLOR Color);
void             LISTVIEW_SetColumnWidth      (LISTVIEW_Handle hObj, unsigned int Index, int Width);
void             LISTVIEW_SetCompareFunc      (LISTVIEW_Handle hObj, unsigned Column, int (* fpCompare)(const void * p0, const void * p1));
unsigned         LISTVIEW_SetFixed            (LISTVIEW_Handle hObj, unsigned Fixed);
void             LISTVIEW_SetFont             (LISTVIEW_Handle hObj, const GUI_FONT * pFont);
int              LISTVIEW_SetGridVis          (LISTVIEW_Handle hObj, int Show);
void             LISTVIEW_SetHeaderHeight     (LISTVIEW_Handle hObj, unsigned HeaderHeight);
void             LISTVIEW_SetItemBkColor      (LISTVIEW_Handle hObj, unsigned Column, unsigned Row, unsigned int Index, GUI_COLOR Color);
void             LISTVIEW_SetItemText         (LISTVIEW_Handle hObj, unsigned Column, unsigned Row, const char * s);
void             LISTVIEW_SetItemTextColor    (LISTVIEW_Handle hObj, unsigned Column, unsigned Row, unsigned int Index, GUI_COLOR Color);
void             LISTVIEW_SetItemTextSorted   (LISTVIEW_Handle hObj, unsigned Column, unsigned Row, const char * pText);
void             LISTVIEW_SetLBorder          (LISTVIEW_Handle hObj, unsigned BorderSize);
void             LISTVIEW_SetOwnerDraw        (LISTVIEW_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawItem);
void             LISTVIEW_SetRBorder          (LISTVIEW_Handle hObj, unsigned BorderSize);
unsigned         LISTVIEW_SetRowHeight        (LISTVIEW_Handle hObj, unsigned RowHeight);
void             LISTVIEW_SetSel              (LISTVIEW_Handle hObj, int Sel);
void             LISTVIEW_SetSelCol           (LISTVIEW_Handle hObj, int NewCol);
void             LISTVIEW_SetSelUnsorted      (LISTVIEW_Handle hObj, int Sel);
unsigned         LISTVIEW_SetSort             (LISTVIEW_Handle hObj, unsigned Column, unsigned Reverse);
void             LISTVIEW_SetTextAlign        (LISTVIEW_Handle hObj, unsigned int Index, int Align);
void             LISTVIEW_SetTextColor        (LISTVIEW_Handle hObj, unsigned int Index, GUI_COLOR Color);
int              LISTVIEW_SetUserData         (LISTVIEW_Handle hObj, const void * pSrc, int NumBytes);
void             LISTVIEW_SetUserDataRow      (LISTVIEW_Handle hObj, unsigned Row, unsigned long UserData);
void             LISTVIEW_SetWrapMode         (LISTVIEW_Handle hObj, GUI_WRAPMODE WrapMode);






 

GUI_COLOR        LISTVIEW_SetDefaultBkColor  (unsigned  Index, GUI_COLOR Color);
const GUI_FONT * LISTVIEW_SetDefaultFont     (const GUI_FONT * pFont);
GUI_COLOR        LISTVIEW_SetDefaultGridColor(GUI_COLOR Color);
GUI_COLOR        LISTVIEW_SetDefaultTextColor(unsigned  Index, GUI_COLOR Color);








 
#line 72 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\LISTWHEEL.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\LISTWHEEL.h"
#line 59 "..\\STemWin\\inc\\LISTWHEEL.h"
#line 60 "..\\STemWin\\inc\\LISTWHEEL.h"












 








 
typedef signed long LISTWHEEL_Handle;






 





 
LISTWHEEL_Handle LISTWHEEL_Create        (const GUI_ConstString * ppText, int x0, int y0, int xSize, int ySize, int Flags);
LISTWHEEL_Handle LISTWHEEL_CreateAsChild (const GUI_ConstString * ppText, GUI_HWIN hWinParent, int x0, int y0, int xSize, int ySize, int Flags);
LISTWHEEL_Handle LISTWHEEL_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);
LISTWHEEL_Handle LISTWHEEL_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent,
                                          int WinFlags, int ExFlags, int Id, const GUI_ConstString * ppText);
LISTWHEEL_Handle LISTWHEEL_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent,
                                          int WinFlags, int ExFlags, int Id, const GUI_ConstString * ppText, int NumExtraBytes);







 
void LISTWHEEL_Callback(WM_MESSAGE * pMsg);






 
void      LISTWHEEL_AddString      (LISTWHEEL_Handle hObj, const char * s);
void *    LISTWHEEL_GetItemData    (LISTWHEEL_Handle hObj, unsigned Index);  
void      LISTWHEEL_GetItemText    (LISTWHEEL_Handle hObj, unsigned Index, char * pBuffer, int MaxSize);
int       LISTWHEEL_GetItemFromPos (LISTWHEEL_Handle hObj, int yPos);
int       LISTWHEEL_GetLBorder     (LISTWHEEL_Handle hObj);
unsigned  LISTWHEEL_GetLineHeight  (LISTWHEEL_Handle hObj);
int       LISTWHEEL_GetNumItems    (LISTWHEEL_Handle hObj);
int       LISTWHEEL_GetPos         (LISTWHEEL_Handle hObj);
int       LISTWHEEL_GetRBorder     (LISTWHEEL_Handle hObj);
int       LISTWHEEL_GetSel         (LISTWHEEL_Handle hObj);
int       LISTWHEEL_GetSnapPosition(LISTWHEEL_Handle hObj);
int       LISTWHEEL_GetTextAlign   (LISTWHEEL_Handle hObj);
int       LISTWHEEL_GetUserData    (LISTWHEEL_Handle hObj, void * pDest, int NumBytes);
int       LISTWHEEL_IsMoving       (LISTWHEEL_Handle hObj);
void      LISTWHEEL_MoveToPos      (LISTWHEEL_Handle hObj, unsigned int Index);
int       LISTWHEEL_OwnerDraw      (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void      LISTWHEEL_SetBkColor     (LISTWHEEL_Handle hObj, unsigned int Index, GUI_COLOR Color);
void      LISTWHEEL_SetDeceleration(LISTWHEEL_Handle hObj, unsigned Deceleration);
void      LISTWHEEL_SetFont        (LISTWHEEL_Handle hObj, const GUI_FONT * pFont);
void      LISTWHEEL_SetItemData    (LISTWHEEL_Handle hObj, unsigned Index, void * pData);  
void      LISTWHEEL_SetLBorder     (LISTWHEEL_Handle hObj, unsigned BorderSize);
void      LISTWHEEL_SetLineHeight  (LISTWHEEL_Handle hObj, unsigned LineHeight);
void      LISTWHEEL_SetOwnerDraw   (LISTWHEEL_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfOwnerDraw);
void      LISTWHEEL_SetPos         (LISTWHEEL_Handle hObj, unsigned int Index);
void      LISTWHEEL_SetRBorder     (LISTWHEEL_Handle hObj, unsigned BorderSize);
void      LISTWHEEL_SetSel         (LISTWHEEL_Handle hObj, int Sel);
void      LISTWHEEL_SetSnapPosition(LISTWHEEL_Handle hObj, int SnapPosition);
void      LISTWHEEL_SetText        (LISTWHEEL_Handle hObj, const GUI_ConstString * ppText);
void      LISTWHEEL_SetTextAlign   (LISTWHEEL_Handle hObj, int Align);
void      LISTWHEEL_SetTextColor   (LISTWHEEL_Handle hObj, unsigned int Index, GUI_COLOR Color);
void      LISTWHEEL_SetTimerPeriod (LISTWHEEL_Handle hObj, int TimerPeriod);
int       LISTWHEEL_SetUserData    (LISTWHEEL_Handle hObj, const void * pSrc, int NumBytes);
void      LISTWHEEL_SetVelocity    (LISTWHEEL_Handle hObj, int Velocity);

const GUI_FONT * LISTWHEEL_GetFont(LISTWHEEL_Handle hObj);








 
#line 73 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\MENU.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\MENU.h"
#line 59 "..\\STemWin\\inc\\MENU.h"
#line 60 "..\\STemWin\\inc\\MENU.h"












 





 









 






 









 








 
#line 121 "..\\STemWin\\inc\\MENU.h"
                                            






 











 

typedef signed long MENU_Handle;

typedef struct {
  
  
  
  GUI_COLOR aBkColorH[2];
  GUI_COLOR BkColorV;
  GUI_COLOR FrameColorH;
  GUI_COLOR FrameColorV;
  
  
  
  GUI_COLOR aSelColorH[2];
  GUI_COLOR aSelColorV[2];
  GUI_COLOR FrameColorSelH;
  GUI_COLOR FrameColorSelV;
  
  
  
  GUI_COLOR aSepColorH[2];
  GUI_COLOR aSepColorV[2];
  
  
  
  GUI_COLOR ArrowColor;
  
  
  
  GUI_COLOR TextColor;
} MENU_SKINFLEX_PROPS;




 
typedef struct {
  unsigned short MsgType;
  unsigned short ItemId;
} MENU_MSG_DATA;




 
typedef struct {
  const char  * pText;
  unsigned short           Id;
  unsigned short           Flags;
  MENU_Handle   hSubmenu;
} MENU_ITEM_DATA;






 
MENU_Handle MENU_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);
MENU_Handle MENU_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
MENU_Handle MENU_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);







 
void MENU_Callback(WM_MESSAGE * pMsg);






 
void      MENU_AddItem      (MENU_Handle hObj, const MENU_ITEM_DATA * pItemData);
void      MENU_Attach       (MENU_Handle hObj, GUI_HWIN hDestWin, int x, int y, int xSize, int ySize, int Flags);
void      MENU_DeleteItem   (MENU_Handle hObj, unsigned short ItemId);
void      MENU_DisableItem  (MENU_Handle hObj, unsigned short ItemId);
void      MENU_EnableItem   (MENU_Handle hObj, unsigned short ItemId);
void      MENU_GetItem      (MENU_Handle hObj, unsigned short ItemId, MENU_ITEM_DATA * pItemData);
void      MENU_GetItemText  (MENU_Handle hObj, unsigned short ItemId, char * pBuffer, unsigned BufferSize);
unsigned  MENU_GetNumItems  (MENU_Handle hObj);
GUI_HWIN   MENU_GetOwner     (MENU_Handle hObj);
int       MENU_GetUserData  (MENU_Handle hObj, void * pDest, int NumBytes);
void      MENU_InsertItem   (MENU_Handle hObj, unsigned short ItemId, const MENU_ITEM_DATA * pItemData);
void      MENU_Popup        (MENU_Handle hObj, GUI_HWIN hDestWin, int x, int y, int xSize, int ySize, int Flags);
void      MENU_SetBkColor   (MENU_Handle hObj, unsigned ColorIndex, GUI_COLOR Color);
void      MENU_SetBorderSize(MENU_Handle hObj, unsigned BorderIndex, unsigned char BorderSize);
void      MENU_SetFont      (MENU_Handle hObj, const GUI_FONT * pFont);
void      MENU_SetItem      (MENU_Handle hObj, unsigned short ItemId, const MENU_ITEM_DATA * pItemData);
void      MENU_SetOwner     (MENU_Handle hObj, GUI_HWIN hOwner);
int       MENU_SetSel       (MENU_Handle hObj, int Sel);
void      MENU_SetTextColor (MENU_Handle hObj, unsigned ColorIndex, GUI_COLOR Color);
int       MENU_SetUserData  (MENU_Handle hObj, const void * pSrc, int NumBytes);






 
GUI_COLOR             MENU_GetDefaultTextColor  (unsigned ColorIndex);
GUI_COLOR             MENU_GetDefaultBkColor    (unsigned ColorIndex);
unsigned char                    MENU_GetDefaultBorderSize (unsigned BorderIndex);
const WIDGET_EFFECT * MENU_GetDefaultEffect     (void);
const GUI_FONT      * MENU_GetDefaultFont       (void);
void                  MENU_SetDefaultTextColor  (unsigned ColorIndex, GUI_COLOR Color);
void                  MENU_SetDefaultBkColor    (unsigned ColorIndex, GUI_COLOR Color);
void                  MENU_SetDefaultBorderSize (unsigned BorderIndex, unsigned char BorderSize);
void                  MENU_SetDefaultEffect     (const WIDGET_EFFECT * pEffect);
void                  MENU_SetDefaultFont       (const GUI_FONT * pFont);






 
int                     MENU_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void                    MENU_GetSkinFlexProps     (MENU_SKINFLEX_PROPS * pProps, int Index);
WIDGET_DRAW_ITEM_FUNC * MENU_SetDefaultSkin       (WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
void                    MENU_SetDefaultSkinClassic(void);
void                    MENU_SetSkinClassic       (MENU_Handle hObj);
void                    MENU_SetSkin              (MENU_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
void                    MENU_SetSkinFlexProps     (const MENU_SKINFLEX_PROPS * pProps, int Index);
void                    MENU_SkinEnableArrow      (MENU_Handle hObj, int OnOff);








 
#line 74 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\MULTIEDIT.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\MULTIEDIT.h"
#line 59 "..\\STemWin\\inc\\MULTIEDIT.h"






















 








 

typedef signed long MULTIEDIT_HANDLE;






 
MULTIEDIT_HANDLE MULTIEDIT_Create        (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int Flags, int ExFlags, const char * pText, int MaxLen);
MULTIEDIT_HANDLE MULTIEDIT_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int BufferSize, const char * pText);
MULTIEDIT_HANDLE MULTIEDIT_CreateIndirect(const GUI_WIDGET_CREATE_INFO* pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);
MULTIEDIT_HANDLE MULTIEDIT_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int BufferSize, const char * pText, int NumExtraBytes);







 
void MULTIEDIT_Callback(WM_MESSAGE * pMsg);






 

int  MULTIEDIT_AddKey           (MULTIEDIT_HANDLE hObj, unsigned short Key);
int  MULTIEDIT_AddText          (MULTIEDIT_HANDLE hObj, const char * s);
void MULTIEDIT_EnableBlink      (MULTIEDIT_HANDLE hObj, int Period, int OnOff);
int  MULTIEDIT_GetCursorCharPos (MULTIEDIT_HANDLE hObj);
void MULTIEDIT_GetCursorPixelPos(MULTIEDIT_HANDLE hObj, int * pxPos, int * pyPos);
void MULTIEDIT_GetPrompt        (MULTIEDIT_HANDLE hObj, char* sDest, int MaxNumChars);
int  MULTIEDIT_GetTextSize      (MULTIEDIT_HANDLE hObj);
void MULTIEDIT_GetText          (MULTIEDIT_HANDLE hObj, char* sDest, int MaxNumChars);
int  MULTIEDIT_GetUserData      (MULTIEDIT_HANDLE hObj, void * pDest, int NumBytes);
void MULTIEDIT_SetTextAlign     (MULTIEDIT_HANDLE hObj, int Align);
void MULTIEDIT_SetAutoScrollH   (MULTIEDIT_HANDLE hObj, int OnOff);
void MULTIEDIT_SetAutoScrollV   (MULTIEDIT_HANDLE hObj, int OnOff);
void MULTIEDIT_SetBkColor       (MULTIEDIT_HANDLE hObj, unsigned Index, GUI_COLOR color);
void MULTIEDIT_SetCursorCharPos (MULTIEDIT_HANDLE hObj, int x, int y);        
void MULTIEDIT_SetCursorPixelPos(MULTIEDIT_HANDLE hObj, int x, int y);        
void MULTIEDIT_SetCursorOffset  (MULTIEDIT_HANDLE hObj, int Offset);
void MULTIEDIT_SetHBorder       (MULTIEDIT_HANDLE hObj, unsigned HBorder);
void MULTIEDIT_SetFocussable    (MULTIEDIT_HANDLE hObj, int State);
void MULTIEDIT_SetFont          (MULTIEDIT_HANDLE hObj, const GUI_FONT * pFont);
void MULTIEDIT_SetInsertMode    (MULTIEDIT_HANDLE hObj, int OnOff);
void MULTIEDIT_SetBufferSize    (MULTIEDIT_HANDLE hObj, int BufferSize);
void MULTIEDIT_SetMaxNumChars   (MULTIEDIT_HANDLE hObj, unsigned MaxNumChars);
void MULTIEDIT_SetPrompt        (MULTIEDIT_HANDLE hObj, const char* sPrompt);
void MULTIEDIT_SetReadOnly      (MULTIEDIT_HANDLE hObj, int OnOff);
void MULTIEDIT_SetPasswordMode  (MULTIEDIT_HANDLE hObj, int OnOff);
void MULTIEDIT_SetText          (MULTIEDIT_HANDLE hObj, const char* s);
void MULTIEDIT_SetTextColor     (MULTIEDIT_HANDLE hObj, unsigned Index, GUI_COLOR color);
int  MULTIEDIT_SetUserData      (MULTIEDIT_HANDLE hObj, const void * pSrc, int NumBytes);
void MULTIEDIT_SetWrapNone      (MULTIEDIT_HANDLE hObj);
void MULTIEDIT_SetWrapChar      (MULTIEDIT_HANDLE hObj);
void MULTIEDIT_SetWrapWord      (MULTIEDIT_HANDLE hObj);






 











 
#line 75 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\MULTIPAGE.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\MULTIPAGE.h"
#line 1 "..\\STemWin\\inc\\DIALOG.h"
































 


















 
  
#line 112 "..\\STemWin\\inc\\DIALOG.h"

 
#line 59 "..\\STemWin\\inc\\MULTIPAGE.h"












 



 
































 
typedef signed long MULTIPAGE_Handle;

typedef struct {
  GUI_COLOR BkColor;
  GUI_COLOR aBkUpper[2];
  GUI_COLOR aBkLower[2];
  GUI_COLOR FrameColor;
  GUI_COLOR TextColor;
} MULTIPAGE_SKINFLEX_PROPS;

typedef struct {
  unsigned char  SelSideBorderInc;         
  unsigned char  SelTopBorderInc;          
} MULTIPAGE_SKIN_PROPS;

typedef struct {

    tLCD_APIList  * pRotation;

  unsigned          Align;
  int               Sel;
  unsigned short               State;
  unsigned char                FrameFlags;    
  unsigned char                PageStatus;
  GUI_DRAW_HANDLE * pDrawObj;
} MULTIPAGE_SKIN_INFO;






 
MULTIPAGE_Handle MULTIPAGE_Create        (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int Flags, int SpecialFlags);
MULTIPAGE_Handle MULTIPAGE_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
MULTIPAGE_Handle MULTIPAGE_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
MULTIPAGE_Handle MULTIPAGE_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void MULTIPAGE_Callback(WM_MESSAGE * pMsg);






 
void             MULTIPAGE_AddEmptyPage   (MULTIPAGE_Handle hObj, GUI_HWIN hWin ,const char * pText);
void             MULTIPAGE_AddPage        (MULTIPAGE_Handle hObj, GUI_HWIN hWin ,const char * pText);
GUI_HWIN          MULTIPAGE_AttachWindow   (MULTIPAGE_Handle hObj, unsigned Index, GUI_HWIN hWin);
void             MULTIPAGE_DeletePage     (MULTIPAGE_Handle hObj, unsigned Index, int Delete);
void             MULTIPAGE_DisablePage    (MULTIPAGE_Handle hObj, unsigned Index);
void             MULTIPAGE_EnablePage     (MULTIPAGE_Handle hObj, unsigned Index);
void             MULTIPAGE_EnableScrollbar(MULTIPAGE_Handle hObj, unsigned OnOff);
const GUI_FONT * MULTIPAGE_GetFont        (MULTIPAGE_Handle hObj);
int              MULTIPAGE_GetSelection   (MULTIPAGE_Handle hObj);
int              MULTIPAGE_GetPageText    (MULTIPAGE_Handle hObj, unsigned Index, char * pBuffer, int MaxLen);
int              MULTIPAGE_GetUserData    (MULTIPAGE_Handle hObj, void * pDest, int NumBytes);
GUI_HWIN          MULTIPAGE_GetWindow      (MULTIPAGE_Handle hObj, unsigned Index);
int              MULTIPAGE_IsPageEnabled  (MULTIPAGE_Handle hObj, unsigned Index);
void             MULTIPAGE_SelectPage     (MULTIPAGE_Handle hObj, unsigned Index);
void             MULTIPAGE_SetAlign       (MULTIPAGE_Handle hObj, unsigned Align);
int              MULTIPAGE_SetBitmapEx    (MULTIPAGE_Handle hObj, const GUI_BITMAP * pBitmap, int x, int y, int Index, int State);
int              MULTIPAGE_SetBitmap      (MULTIPAGE_Handle hObj, const GUI_BITMAP * pBitmap, int Index, int State);
void             MULTIPAGE_SetBkColor     (MULTIPAGE_Handle hObj, GUI_COLOR Color, unsigned Index);
void             MULTIPAGE_SetFont        (MULTIPAGE_Handle hObj, const GUI_FONT * pFont);
void             MULTIPAGE_SetRotation    (MULTIPAGE_Handle hObj, unsigned Rotation);
void             MULTIPAGE_SetTabWidth    (MULTIPAGE_Handle hObj, int Width, int Index);
void             MULTIPAGE_SetTabHeight   (MULTIPAGE_Handle hObj, int Height);
void             MULTIPAGE_SetTextAlign   (MULTIPAGE_Handle hObj, unsigned Align);
void             MULTIPAGE_SetText        (MULTIPAGE_Handle hObj, const char * pText, unsigned Index);
void             MULTIPAGE_SetTextColor   (MULTIPAGE_Handle hObj, GUI_COLOR Color, unsigned Index);
int              MULTIPAGE_SetUserData    (MULTIPAGE_Handle hObj, const void * pSrc, int NumBytes);






 
unsigned         MULTIPAGE_GetDefaultAlign      (void);
GUI_COLOR        MULTIPAGE_GetDefaultBkColor    (unsigned Index);
const GUI_FONT * MULTIPAGE_GetDefaultFont       (void);
GUI_COLOR        MULTIPAGE_GetDefaultTextColor  (unsigned Index);

void             MULTIPAGE_SetDefaultAlign      (unsigned Align);
void             MULTIPAGE_SetDefaultBkColor    (GUI_COLOR Color, unsigned Index);
void             MULTIPAGE_SetDefaultBorderSizeX(unsigned Size);
void             MULTIPAGE_SetDefaultBorderSizeY(unsigned Size);
void             MULTIPAGE_SetDefaultFont       (const GUI_FONT * pFont);
void             MULTIPAGE_SetDefaultTextColor  (GUI_COLOR Color, unsigned Index);

void             MULTIPAGE_SetEffectColor       (unsigned Index, GUI_COLOR Color);
GUI_COLOR        MULTIPAGE_GetEffectColor       (unsigned Index);
int              MULTIPAGE_GetNumEffectColors   (void);






 
int                     MULTIPAGE_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void                    MULTIPAGE_GetSkinFlexProps     (MULTIPAGE_SKINFLEX_PROPS * pProps, int Index);
WIDGET_DRAW_ITEM_FUNC * MULTIPAGE_SetDefaultSkin       (WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
void                    MULTIPAGE_SetDefaultSkinClassic(void);
void                    MULTIPAGE_SetSkinClassic       (MULTIPAGE_Handle hObj);
void                    MULTIPAGE_SetSkin              (MULTIPAGE_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
void                    MULTIPAGE_SetSkinFlexProps     (const MULTIPAGE_SKINFLEX_PROPS * pProps, int Index);










 
#line 76 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\PROGBAR.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\PROGBAR.h"
#line 59 "..\\STemWin\\inc\\PROGBAR.h"
#line 60 "..\\STemWin\\inc\\PROGBAR.h"












 



 







 








 
typedef signed long PROGBAR_Handle;

typedef struct {
  GUI_COLOR aColorUpperL[2];
  GUI_COLOR aColorLowerL[2];
  GUI_COLOR aColorUpperR[2];
  GUI_COLOR aColorLowerR[2];
  GUI_COLOR ColorFrame;
  GUI_COLOR ColorText;
} PROGBAR_SKINFLEX_PROPS;

typedef struct {
  int IsVertical;
  int Index;
  const char * pText;
} PROGBAR_SKINFLEX_INFO;






 

PROGBAR_Handle PROGBAR_Create        (int x0, int y0, int xSize, int ySize, int Flags);
PROGBAR_Handle PROGBAR_CreateAsChild (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int Flags);
PROGBAR_Handle PROGBAR_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
PROGBAR_Handle PROGBAR_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
PROGBAR_Handle PROGBAR_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void PROGBAR_Callback(WM_MESSAGE * pMsg);






 

void PROGBAR_GetMinMax   (PROGBAR_Handle hObj, int * pMin, int * pMax);
int  PROGBAR_GetUserData (PROGBAR_Handle hObj, void * pDest, int NumBytes);
int  PROGBAR_GetValue    (PROGBAR_Handle hObj);
void PROGBAR_SetBarColor (PROGBAR_Handle hObj, unsigned int index, GUI_COLOR color);
void PROGBAR_SetFont     (PROGBAR_Handle hObj, const GUI_FONT * pfont);
void PROGBAR_SetMinMax   (PROGBAR_Handle hObj, int Min, int Max);
void PROGBAR_SetText     (PROGBAR_Handle hObj, const char* s);
void PROGBAR_SetTextAlign(PROGBAR_Handle hObj, int Align);
void PROGBAR_SetTextColor(PROGBAR_Handle hObj, unsigned int index, GUI_COLOR color);
void PROGBAR_SetTextPos  (PROGBAR_Handle hObj, int XOff, int YOff);
void PROGBAR_SetValue    (PROGBAR_Handle hObj, int v);
int  PROGBAR_SetUserData (PROGBAR_Handle hObj, const void * pSrc, int NumBytes);






 
void PROGBAR_GetSkinFlexProps     (PROGBAR_SKINFLEX_PROPS * pProps, int Index);
void PROGBAR_SetSkinClassic       (PROGBAR_Handle hObj);
void PROGBAR_SetSkin              (PROGBAR_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
int  PROGBAR_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void PROGBAR_SetSkinFlexProps     (const PROGBAR_SKINFLEX_PROPS * pProps, int Index);
void PROGBAR_SetDefaultSkinClassic(void);
WIDGET_DRAW_ITEM_FUNC * PROGBAR_SetDefaultSkin(WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);










 
#line 77 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\RADIO.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\RADIO.h"
#line 59 "..\\STemWin\\inc\\RADIO.h"
#line 60 "..\\STemWin\\inc\\RADIO.h"










 







 











 






 








 
typedef signed long RADIO_Handle;

typedef struct {
  GUI_COLOR aColorButton[4];
  int       ButtonSize;
} RADIO_SKINFLEX_PROPS;






 

RADIO_Handle RADIO_Create        (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int Flags, unsigned Para);
RADIO_Handle RADIO_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumItems, int Spacing);
RADIO_Handle RADIO_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumItems, int Spacing, int NumExtraBytes);
RADIO_Handle RADIO_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void RADIO_Callback(WM_MESSAGE * pMsg);






 

void             RADIO_SetDefaultFont      (const GUI_FONT * pFont);
GUI_COLOR        RADIO_SetDefaultFocusColor(GUI_COLOR Color);
void             RADIO_SetDefaultImage     (const GUI_BITMAP * pBitmap, unsigned int Index);
void             RADIO_SetDefaultTextColor (GUI_COLOR TextColor);
const GUI_FONT * RADIO_GetDefaultFont      (void);
GUI_COLOR        RADIO_GetDefaultTextColor (void);






 

void      RADIO_AddValue     (RADIO_Handle hObj, int Add);
void      RADIO_Dec          (RADIO_Handle hObj);
int       RADIO_GetText      (RADIO_Handle hObj, unsigned Index, char * pBuffer, int MaxLen);
int       RADIO_GetUserData  (RADIO_Handle hObj, void * pDest, int NumBytes);
void      RADIO_Inc          (RADIO_Handle hObj);
void      RADIO_SetBkColor   (RADIO_Handle hObj, GUI_COLOR Color);
GUI_COLOR RADIO_SetFocusColor(RADIO_Handle hObj, GUI_COLOR Color);
void      RADIO_SetFont      (RADIO_Handle hObj, const GUI_FONT * pFont);
void      RADIO_SetGroupId   (RADIO_Handle hObj, unsigned char GroupId);
void      RADIO_SetImage     (RADIO_Handle hObj, const GUI_BITMAP * pBitmap, unsigned int Index);
void      RADIO_SetText      (RADIO_Handle hObj, const char* pText, unsigned Index);
void      RADIO_SetTextColor (RADIO_Handle hObj, GUI_COLOR Color);
void      RADIO_SetValue     (RADIO_Handle hObj, int v);
int       RADIO_SetUserData  (RADIO_Handle hObj, const void * pSrc, int NumBytes);

const GUI_BITMAP * RADIO_GetImage(RADIO_Handle hObj, unsigned int Index);






 
void RADIO_GetSkinFlexProps     (RADIO_SKINFLEX_PROPS * pProps, int Index);
void RADIO_SetSkinClassic       (RADIO_Handle hObj);
void RADIO_SetSkin              (RADIO_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
int  RADIO_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void RADIO_SetSkinFlexProps     (const RADIO_SKINFLEX_PROPS * pProps, int Index);
void RADIO_SetDefaultSkinClassic(void);
WIDGET_DRAW_ITEM_FUNC * RADIO_SetDefaultSkin(WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);








 
int RADIO_GetValue(RADIO_Handle hObj);








 
#line 78 "..\\STemWin\\inc\\DIALOG.h"
#line 79 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\SLIDER.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\SLIDER.h"
#line 59 "..\\STemWin\\inc\\SLIDER.h"
#line 60 "..\\STemWin\\inc\\SLIDER.h"












 



 





 






 








 
typedef signed long SLIDER_Handle;

typedef struct {
  GUI_COLOR aColorFrame[2];
  GUI_COLOR aColorInner[2];
  GUI_COLOR aColorShaft[3];
  GUI_COLOR ColorTick;
  GUI_COLOR ColorFocus;
  int TickSize;
  int ShaftSize;
} SLIDER_SKINFLEX_PROPS;

typedef struct {
  int Width;
  int NumTicks;
  int Size;
  int IsPressed;
  int IsVertical;
} SLIDER_SKINFLEX_INFO;






 
SLIDER_Handle SLIDER_Create        (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int WinFlags, int SpecialFlags);
SLIDER_Handle SLIDER_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
SLIDER_Handle SLIDER_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
SLIDER_Handle SLIDER_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void SLIDER_Callback(WM_MESSAGE * pMsg);






 
void      SLIDER_Dec            (SLIDER_Handle hObj);
void      SLIDER_EnableFocusRect(SLIDER_Handle hObj, int OnOff);
GUI_COLOR SLIDER_GetBarColor    (SLIDER_Handle hObj);
GUI_COLOR SLIDER_GetBkColor     (SLIDER_Handle hObj);
unsigned char        SLIDER_GetFlag        (SLIDER_Handle hObj, unsigned char Flag);
GUI_COLOR SLIDER_GetFocusColor  (SLIDER_Handle hObj);
void      SLIDER_GetRange       (SLIDER_Handle hObj, int * pMin, int * pMax);
GUI_COLOR SLIDER_GetTickColor   (SLIDER_Handle hObj);
int       SLIDER_GetUserData    (SLIDER_Handle hObj, void * pDest, int NumBytes);
int       SLIDER_GetValue       (SLIDER_Handle hObj);
void      SLIDER_Inc            (SLIDER_Handle hObj);
void      SLIDER_SetBarColor    (SLIDER_Handle hObj, GUI_COLOR Color);
void      SLIDER_SetBkColor     (SLIDER_Handle hObj, GUI_COLOR Color);
GUI_COLOR SLIDER_SetFocusColor  (SLIDER_Handle hObj, GUI_COLOR Color);
void      SLIDER_SetNumTicks    (SLIDER_Handle hObj, int NumTicks);
void      SLIDER_SetRange       (SLIDER_Handle hObj, int Min, int Max);
void      SLIDER_SetTickColor   (SLIDER_Handle hObj, GUI_COLOR Color);
int       SLIDER_SetUserData    (SLIDER_Handle hObj, const void * pSrc, int NumBytes);
void      SLIDER_SetValue       (SLIDER_Handle hObj, int v);
void      SLIDER_SetWidth       (SLIDER_Handle hObj, int Width);






 
void SLIDER_GetSkinFlexProps     (SLIDER_SKINFLEX_PROPS * pProps, int Index);
void SLIDER_SetSkinClassic       (SLIDER_Handle hObj);
void SLIDER_SetSkin              (SLIDER_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
int  SLIDER_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void SLIDER_SetSkinFlexProps     (const SLIDER_SKINFLEX_PROPS * pProps, int Index);
void SLIDER_SetDefaultSkinClassic(void);
WIDGET_DRAW_ITEM_FUNC * SLIDER_SetDefaultSkin(WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);








 
GUI_COLOR SLIDER_GetDefaultBkColor   (void);
GUI_COLOR SLIDER_GetDefaultBarColor  (void);
GUI_COLOR SLIDER_GetDefaultFocusColor(void);
GUI_COLOR SLIDER_GetDefaultTickColor (void);
void      SLIDER_SetDefaultBkColor   (GUI_COLOR Color);
void      SLIDER_SetDefaultBarColor  (GUI_COLOR Color);
GUI_COLOR SLIDER_SetDefaultFocusColor(GUI_COLOR Color);
void      SLIDER_SetDefaultTickColor (GUI_COLOR Color);








 
#line 80 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\SPINBOX.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\SPINBOX.h"
#line 59 "..\\STemWin\\inc\\SPINBOX.h"
#line 60 "..\\STemWin\\inc\\SPINBOX.h"
#line 61 "..\\STemWin\\inc\\SPINBOX.h"
#line 62 "..\\STemWin\\inc\\SPINBOX.h"












 



 

















 







 













 
typedef signed long SPINBOX_Handle;

typedef struct {
  GUI_COLOR aColorFrame[2];   
  GUI_COLOR aColorUpper[2];   
  GUI_COLOR aColorLower[2];   
  GUI_COLOR ColorArrow;       
  GUI_COLOR ColorBk;          
  GUI_COLOR ColorText;        
  GUI_COLOR ColorButtonFrame; 
} SPINBOX_SKINFLEX_PROPS;






 



 
SPINBOX_Handle SPINBOX_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int Id, int Min, int Max);
SPINBOX_Handle SPINBOX_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int Id, int Min, int Max, int NumExtraBytes);
SPINBOX_Handle SPINBOX_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);




 
void           SPINBOX_Callback(WM_MESSAGE * pMsg);




 
void        SPINBOX_EnableBlink     (SPINBOX_Handle hObj, int Period, int OnOff);
GUI_COLOR   SPINBOX_GetBkColor      (SPINBOX_Handle hObj, unsigned int Index);
GUI_COLOR   SPINBOX_GetButtonBkColor(SPINBOX_Handle hObj, unsigned int Index);
EDIT_Handle SPINBOX_GetEditHandle   (SPINBOX_Handle hObj);
int         SPINBOX_GetUserData     (SPINBOX_Handle hObj, void * pDest, int NumBytes);
signed long         SPINBOX_GetValue        (SPINBOX_Handle hObj);
void        SPINBOX_SetBkColor      (SPINBOX_Handle hObj, unsigned int Index, GUI_COLOR Color);
void        SPINBOX_SetButtonBkColor(SPINBOX_Handle hObj, unsigned int Index, GUI_COLOR Color);
void        SPINBOX_SetButtonSize   (SPINBOX_Handle hObj, unsigned ButtonSize);
void        SPINBOX_SetEdge         (SPINBOX_Handle hObj, unsigned char Edge);
void        SPINBOX_SetEditMode     (SPINBOX_Handle hObj, unsigned char EditMode);
void        SPINBOX_SetFont         (SPINBOX_Handle hObj, const GUI_FONT * pFont);
void        SPINBOX_SetRange        (SPINBOX_Handle hObj, signed long Min, signed long Max);
unsigned short         SPINBOX_SetStep         (SPINBOX_Handle hObj, unsigned short Step);
void        SPINBOX_SetTextColor    (SPINBOX_Handle hObj, unsigned int Index, GUI_COLOR Color);
int         SPINBOX_SetUserData     (SPINBOX_Handle hObj, const void * pSrc, int NumBytes);
void        SPINBOX_SetValue        (SPINBOX_Handle hObj, signed long Value);




 
unsigned short  SPINBOX_GetDefaultButtonSize(void);
void SPINBOX_SetDefaultButtonSize(unsigned short ButtonSize);




 
void                    SPINBOX_GetSkinFlexProps     (SPINBOX_SKINFLEX_PROPS * pProps, int Index);
void                    SPINBOX_SetSkinClassic       (SPINBOX_Handle hObj);
void                    SPINBOX_SetSkin              (SPINBOX_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
int                     SPINBOX_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void                    SPINBOX_SetSkinFlexProps     (const SPINBOX_SKINFLEX_PROPS * pProps, int Index);
void                    SPINBOX_SetDefaultSkinClassic(void);
WIDGET_DRAW_ITEM_FUNC * SPINBOX_SetDefaultSkin(WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);








 
#line 81 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\TEXT.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\TEXT.h"
#line 59 "..\\STemWin\\inc\\TEXT.h"
#line 60 "..\\STemWin\\inc\\TEXT.h"
#line 61 "..\\STemWin\\inc\\TEXT.h"












 




 















 
typedef signed long TEXT_Handle;






 
TEXT_Handle TEXT_Create        (int x0, int y0, int xSize, int ySize, int Id, int Flags, const char * s, int Align);
TEXT_Handle TEXT_CreateAsChild (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int Id, int Flags, const char * s, int Align);
TEXT_Handle TEXT_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, const char * pText);
TEXT_Handle TEXT_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, const char * pText, int NumExtraBytes);
TEXT_Handle TEXT_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void TEXT_Callback(WM_MESSAGE * pMsg);






 

 

GUI_COLOR        TEXT_GetBkColor  (TEXT_Handle hObj); 
const GUI_FONT * TEXT_GetFont     (TEXT_Handle hObj);
int              TEXT_GetNumLines (TEXT_Handle hObj);
int              TEXT_GetText     (TEXT_Handle hObj, char * pDest, unsigned long BufferSize);
int              TEXT_GetTextAlign(TEXT_Handle hObj);
GUI_COLOR        TEXT_GetTextColor(TEXT_Handle hObj);
int              TEXT_GetUserData (TEXT_Handle hObj, void * pDest, int NumBytes);
GUI_WRAPMODE     TEXT_GetWrapMode (TEXT_Handle hObj);
void             TEXT_SetBkColor  (TEXT_Handle hObj, GUI_COLOR Color);
void             TEXT_SetFont     (TEXT_Handle hObj, const GUI_FONT * pFont);
int              TEXT_SetText     (TEXT_Handle hObj, const char * s);
void             TEXT_SetTextAlign(TEXT_Handle hObj, int Align);
void             TEXT_SetTextColor(TEXT_Handle hObj, GUI_COLOR Color);
int              TEXT_SetUserData (TEXT_Handle hObj, const void * pSrc, int NumBytes);
void             TEXT_SetWrapMode (TEXT_Handle hObj, GUI_WRAPMODE WrapMode);






 

const GUI_FONT * TEXT_GetDefaultFont     (void);
GUI_COLOR        TEXT_GetDefaultTextColor(void);
GUI_WRAPMODE     TEXT_GetDefaultWrapMode (void);
void             TEXT_SetDefaultFont     (const GUI_FONT * pFont);
void             TEXT_SetDefaultTextColor(GUI_COLOR Color);
GUI_WRAPMODE     TEXT_SetDefaultWrapMode (GUI_WRAPMODE WrapMode);








 
#line 82 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\TREEVIEW.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\TREEVIEW.h"
#line 59 "..\\STemWin\\inc\\TREEVIEW.h"
#line 60 "..\\STemWin\\inc\\TREEVIEW.h"












 
 
#line 82 "..\\STemWin\\inc\\TREEVIEW.h"

 
#line 90 "..\\STemWin\\inc\\TREEVIEW.h"

 




 




 
#line 108 "..\\STemWin\\inc\\TREEVIEW.h"

 



 








 
typedef signed long TREEVIEW_Handle;
typedef signed long TREEVIEW_ITEM_Handle;

typedef struct {
  int IsNode;
  int IsExpanded;
  int HasLines;
  int HasRowSelect;
  int Level;
} TREEVIEW_ITEM_INFO;

typedef struct {
  GUI_COLOR ColorBk;
  GUI_COLOR ColorText;
  GUI_COLOR ColorTextBk;
  GUI_COLOR ColorLines;
  GUI_RECT rText;
  TREEVIEW_ITEM_Handle hItem;
  const GUI_FONT * pFont;
  char * pText;
  unsigned char NumLines;
  signed short ax0[3];
  signed short ay0[3];
  signed short ax1[3];
  signed short ay1[3];
  unsigned char NumConnectors;
  signed short axc[16];
  const GUI_BITMAP * pBmPM;
  const GUI_BITMAP * pBmOCL;
  signed short xPosPM, xPosOCL;
  unsigned char IndexPM;
  unsigned char IndexOCL;
} TREEVIEW_ITEM_DRAW_INFO;






 
TREEVIEW_Handle      TREEVIEW_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id);
TREEVIEW_Handle      TREEVIEW_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, int NumExtraBytes);
TREEVIEW_Handle      TREEVIEW_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void TREEVIEW_Callback(WM_MESSAGE * pMsg);






 
int                  TREEVIEW_AttachItem     (TREEVIEW_Handle hObj, TREEVIEW_ITEM_Handle hItem, TREEVIEW_ITEM_Handle hItemAt, int Position);
void                 TREEVIEW_DecSel         (TREEVIEW_Handle hObj);
TREEVIEW_ITEM_Handle TREEVIEW_GetItem        (TREEVIEW_Handle hObj, TREEVIEW_ITEM_Handle hItem, int Flags);
TREEVIEW_ITEM_Handle TREEVIEW_GetSel         (TREEVIEW_Handle hObj);
int                  TREEVIEW_GetUserData    (TREEVIEW_Handle hObj, void * pDest, int NumBytes);
void                 TREEVIEW_IncSel         (TREEVIEW_Handle hObj);
TREEVIEW_ITEM_Handle TREEVIEW_InsertItem     (TREEVIEW_Handle hObj, int IsNode, TREEVIEW_ITEM_Handle hItemPrev, int Position, const char * s);
int                  TREEVIEW_OwnerDraw      (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void                 TREEVIEW_ScrollToSel    (TREEVIEW_Handle hObj);
void                 TREEVIEW_SetAutoScrollH (TREEVIEW_Handle hObj, int State);
void                 TREEVIEW_SetAutoScrollV (TREEVIEW_Handle hObj, int State);
void                 TREEVIEW_SetBitmapOffset(TREEVIEW_Handle hObj, int Index, int xOff, int yOff);
void                 TREEVIEW_SetBkColor     (TREEVIEW_Handle hObj, int Index, GUI_COLOR Color);
void                 TREEVIEW_SetFont        (TREEVIEW_Handle hObj, const GUI_FONT * pFont);
void                 TREEVIEW_SetHasLines    (TREEVIEW_Handle hObj, int State);
void                 TREEVIEW_SetImage       (TREEVIEW_Handle hObj, int Index, const GUI_BITMAP * pBitmap);
int                  TREEVIEW_SetIndent      (TREEVIEW_Handle hObj, int Indent);
void                 TREEVIEW_SetLineColor   (TREEVIEW_Handle hObj, int Index, GUI_COLOR Color);
void                 TREEVIEW_SetOwnerDraw   (TREEVIEW_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawItem);
void                 TREEVIEW_SetSel         (TREEVIEW_Handle hObj, TREEVIEW_ITEM_Handle hItem);
void                 TREEVIEW_SetSelMode     (TREEVIEW_Handle hObj, int Mode);
void                 TREEVIEW_SetTextColor   (TREEVIEW_Handle hObj, int Index, GUI_COLOR Color);
int                  TREEVIEW_SetTextIndent  (TREEVIEW_Handle hObj, int TextIndent);
int                  TREEVIEW_SetUserData    (TREEVIEW_Handle hObj, const void * pSrc, int NumBytes);







 
void                 TREEVIEW_ITEM_Collapse   (TREEVIEW_ITEM_Handle hItem);
void                 TREEVIEW_ITEM_CollapseAll(TREEVIEW_ITEM_Handle hItem);
TREEVIEW_ITEM_Handle TREEVIEW_ITEM_Create     (int IsNode, const char * s, unsigned long UserData);
void                 TREEVIEW_ITEM_Delete     (TREEVIEW_ITEM_Handle hItem);
void                 TREEVIEW_ITEM_Detach     (TREEVIEW_ITEM_Handle hItem);
void                 TREEVIEW_ITEM_Expand     (TREEVIEW_ITEM_Handle hItem);
void                 TREEVIEW_ITEM_ExpandAll  (TREEVIEW_ITEM_Handle hItem);
void                 TREEVIEW_ITEM_GetInfo    (TREEVIEW_ITEM_Handle hItem, TREEVIEW_ITEM_INFO * pInfo);
void                 TREEVIEW_ITEM_GetText    (TREEVIEW_ITEM_Handle hItem, unsigned char * pBuffer, int MaxNumBytes);
unsigned long                  TREEVIEW_ITEM_GetUserData(TREEVIEW_ITEM_Handle hItem);
void                 TREEVIEW_ITEM_SetImage   (TREEVIEW_ITEM_Handle hItem, int Index, const GUI_BITMAP * pBitmap);
TREEVIEW_ITEM_Handle TREEVIEW_ITEM_SetText    (TREEVIEW_ITEM_Handle hItem, const char * s);
void                 TREEVIEW_ITEM_SetUserData(TREEVIEW_ITEM_Handle hItem, unsigned long UserData);






 
GUI_COLOR        TREEVIEW_GetDefaultBkColor  (int Index);
const GUI_FONT * TREEVIEW_GetDefaultFont     (void);
GUI_COLOR        TREEVIEW_GetDefaultLineColor(int Index);
GUI_COLOR        TREEVIEW_GetDefaultTextColor(int Index);
void             TREEVIEW_SetDefaultBkColor  (int Index, GUI_COLOR Color);
void             TREEVIEW_SetDefaultFont     (const GUI_FONT * pFont);
void             TREEVIEW_SetDefaultLineColor(int Index, GUI_COLOR Color);
void             TREEVIEW_SetDefaultTextColor(int Index, GUI_COLOR Color);








 
#line 83 "..\\STemWin\\inc\\DIALOG.h"
#line 1 "..\\STemWin\\inc\\KNOB.h"
































 


















 
  



#line 58 "..\\STemWin\\inc\\KNOB.h"
#line 59 "..\\STemWin\\inc\\KNOB.h"
#line 60 "..\\STemWin\\inc\\KNOB.h"
#line 61 "..\\STemWin\\inc\\KNOB.h"












 
typedef signed long KNOB_Handle;






 
KNOB_Handle KNOB_CreateEx      (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int Id);
KNOB_Handle KNOB_CreateUser    (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int Id, int NumExtraBytes);
KNOB_Handle KNOB_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);







 
void KNOB_Callback(WM_MESSAGE * pMsg);






 
void KNOB_AddValue   (KNOB_Handle hObj, signed long Value);
int  KNOB_GetUserData(KNOB_Handle hObj, void * pDest, int NumBytes);               
signed long  KNOB_GetValue   (KNOB_Handle hObj);                                           
void KNOB_SetBkColor (KNOB_Handle hObj, GUI_COLOR Color);                          
void KNOB_SetBkDevice(KNOB_Handle hObj, GUI_MEMDEV_Handle hMemBk);                 
void KNOB_SetDevice  (KNOB_Handle hObj, GUI_MEMDEV_Handle hMemSrc);                
void KNOB_SetKeyValue(KNOB_Handle hObj, signed long KeyValue);                             
void KNOB_SetOffset  (KNOB_Handle hObj, signed long Offset);                               
void KNOB_SetPeriod  (KNOB_Handle hObj, signed long Period);                               
void KNOB_SetPos     (KNOB_Handle hObj, signed long Pos);                                  
void KNOB_SetRange   (KNOB_Handle hObj, signed long MinRange, signed long MaxRange);               
void KNOB_SetSnap    (KNOB_Handle hObj, signed long Snap);                                 
void KNOB_SetTickSize(KNOB_Handle hObj, signed long TickSize);                             
int  KNOB_SetUserData(KNOB_Handle hObj, const void * pSrc, int NumBytes);          






 








 
#line 84 "..\\STemWin\\inc\\DIALOG.h"










 
GUI_HWIN   WINDOW_CreateEx         (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, WM_CALLBACK * cb);
GUI_HWIN   WINDOW_CreateUser       (int x0, int y0, int xSize, int ySize, GUI_HWIN hParent, int WinFlags, int ExFlags, int Id, WM_CALLBACK * cb, int NumExtraBytes);
GUI_HWIN   WINDOW_CreateIndirect   (const GUI_WIDGET_CREATE_INFO * pCreateInfo, GUI_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);
GUI_COLOR WINDOW_GetDefaultBkColor(void);
int       WINDOW_GetUserData      (GUI_HWIN hObj, void * pDest, int NumBytes);
void      WINDOW_SetBkColor       (GUI_HWIN hObj, GUI_COLOR Color);
void      WINDOW_SetDefaultBkColor(GUI_COLOR Color);
int       WINDOW_SetUserData      (GUI_HWIN hObj, const void * pSrc, int NumBytes);

void WINDOW_Callback(WM_MESSAGE * pMsg);








 
#line 21 "..\\User\\emWinTask\\MainTask.h"
#line 22 "..\\User\\emWinTask\\MainTask.h"
#line 23 "..\\User\\emWinTask\\MainTask.h"
#line 24 "..\\User\\emWinTask\\MainTask.h"
#line 25 "..\\User\\emWinTask\\MainTask.h"
#line 26 "..\\User\\emWinTask\\MainTask.h"
#line 27 "..\\User\\emWinTask\\MainTask.h"
#line 28 "..\\User\\emWinTask\\MainTask.h"
#line 29 "..\\User\\emWinTask\\MainTask.h"
#line 30 "..\\User\\emWinTask\\MainTask.h"
#line 31 "..\\User\\emWinTask\\MainTask.h"
#line 32 "..\\User\\emWinTask\\MainTask.h"
#line 33 "..\\User\\emWinTask\\MainTask.h"
#line 34 "..\\User\\emWinTask\\MainTask.h"
#line 35 "..\\User\\emWinTask\\MainTask.h"
#line 36 "..\\User\\emWinTask\\MainTask.h"
#line 37 "..\\User\\emWinTask\\MainTask.h"
#line 38 "..\\User\\emWinTask\\MainTask.h"
#line 39 "..\\User\\emWinTask\\MainTask.h"
#line 40 "..\\User\\emWinTask\\MainTask.h"

#line 1 "..\\FatFS\\src\\ff.h"

















 









#line 1 "..\\FatFS\\src\\integer.h"
 
 
 




#line 16 "..\\FatFS\\src\\integer.h"

 
typedef int				INT;
typedef unsigned int	UINT;

 
typedef unsigned char	BYTE;

 
typedef short			SHORT;
typedef unsigned short	WORD;
typedef unsigned short	WCHAR;

 
typedef long			LONG;
typedef unsigned long	DWORD;

 
typedef unsigned long long QWORD;



#line 29 "..\\FatFS\\src\\ff.h"
#line 1 "..\\FatFS\\src\\ffconf.h"


 





 





 









 








 




 



 



 



 




 




 



 




 



























 

















 





 











 








 




 


 








 








 









 





 











 





 





 





 













 











 




















 

 


 
#line 30 "..\\FatFS\\src\\ff.h"







 

#line 51 "..\\FatFS\\src\\ff.h"





 

#line 69 "..\\FatFS\\src\\ff.h"
typedef char TCHAR;







 

#line 85 "..\\FatFS\\src\\ff.h"
typedef DWORD FSIZE_t;




 

typedef struct {
	BYTE	fs_type;		 
	BYTE	drv;			 
	BYTE	n_fats;			 
	BYTE	wflag;			 
	BYTE	fsi_flag;		 
	WORD	id;				 
	WORD	n_rootdir;		 
	WORD	csize;			 




	WCHAR*	lfnbuf;			 
#line 114 "..\\FatFS\\src\\ff.h"
	DWORD	last_clst;		 
	DWORD	free_clst;		 
#line 125 "..\\FatFS\\src\\ff.h"
	DWORD	n_fatent;		 
	DWORD	fsize;			 
	DWORD	volbase;		 
	DWORD	fatbase;		 
	DWORD	dirbase;		 
	DWORD	database;		 
	DWORD	winsect;		 
	BYTE	win[512];	 
} FATFS;



 

typedef struct {
	FATFS*	fs;			 
	WORD	id;			 
	BYTE	attr;		 
	BYTE	stat;		 
	DWORD	sclust;		 
	FSIZE_t	objsize;	 
#line 155 "..\\FatFS\\src\\ff.h"
} _FDID;



 

typedef struct {
	_FDID	obj;			 
	BYTE	flag;			 
	BYTE	err;			 
	FSIZE_t	fptr;			 
	DWORD	clust;			 
	DWORD	sect;			 

	DWORD	dir_sect;		 
	BYTE*	dir_ptr;		 





	BYTE	buf[512];	 

} FIL;



 

typedef struct {
	_FDID	obj;			 
	DWORD	dptr;			 
	DWORD	clust;			 
	DWORD	sect;			 
	BYTE*	dir;			 
	BYTE	fn[12];			 

	DWORD	blk_ofs;		 




} DIR;



 

typedef struct {
	FSIZE_t	fsize;			 
	WORD	fdate;			 
	WORD	ftime;			 
	BYTE	fattrib;		 

	TCHAR	altname[13];			 
	TCHAR	fname[255 + 1];	 



} FILINFO;



 

typedef enum {
	FR_OK = 0,				 
	FR_DISK_ERR,			 
	FR_INT_ERR,				 
	FR_NOT_READY,			 
	FR_NO_FILE,				 
	FR_NO_PATH,				 
	FR_INVALID_NAME,		 
	FR_DENIED,				 
	FR_EXIST,				 
	FR_INVALID_OBJECT,		 
	FR_WRITE_PROTECTED,		 
	FR_INVALID_DRIVE,		 
	FR_NOT_ENABLED,			 
	FR_NO_FILESYSTEM,		 
	FR_MKFS_ABORTED,		 
	FR_TIMEOUT,				 
	FR_LOCKED,				 
	FR_NOT_ENOUGH_CORE,		 
	FR_TOO_MANY_OPEN_FILES,	 
	FR_INVALID_PARAMETER	 
} FRESULT;



 
 

FRESULT f_open (FIL* fp, const TCHAR* path, BYTE mode);				 
FRESULT f_close (FIL* fp);											 
FRESULT f_read (FIL* fp, void* buff, UINT btr, UINT* br);			 
FRESULT f_write (FIL* fp, const void* buff, UINT btw, UINT* bw);	 
FRESULT f_lseek (FIL* fp, FSIZE_t ofs);								 
FRESULT f_truncate (FIL* fp);										 
FRESULT f_sync (FIL* fp);											 
FRESULT f_opendir (DIR* dp, const TCHAR* path);						 
FRESULT f_closedir (DIR* dp);										 
FRESULT f_readdir (DIR* dp, FILINFO* fno);							 
FRESULT f_findfirst (DIR* dp, FILINFO* fno, const TCHAR* path, const TCHAR* pattern);	 
FRESULT f_findnext (DIR* dp, FILINFO* fno);							 
FRESULT f_mkdir (const TCHAR* path);								 
FRESULT f_unlink (const TCHAR* path);								 
FRESULT f_rename (const TCHAR* path_old, const TCHAR* path_new);	 
FRESULT f_stat (const TCHAR* path, FILINFO* fno);					 
FRESULT f_chmod (const TCHAR* path, BYTE attr, BYTE mask);			 
FRESULT f_utime (const TCHAR* path, const FILINFO* fno);			 
FRESULT f_chdir (const TCHAR* path);								 
FRESULT f_chdrive (const TCHAR* path);								 
FRESULT f_getcwd (TCHAR* buff, UINT len);							 
FRESULT f_getfree (const TCHAR* path, DWORD* nclst, FATFS** fatfs);	 
FRESULT f_getlabel (const TCHAR* path, TCHAR* label, DWORD* vsn);	 
FRESULT f_setlabel (const TCHAR* label);							 
FRESULT f_forward (FIL* fp, UINT(*func)(const BYTE*,UINT), UINT btf, UINT* bf);	 
FRESULT f_expand (FIL* fp, FSIZE_t szf, BYTE opt);					 
FRESULT f_mount (FATFS* fs, const TCHAR* path, BYTE opt);			 
FRESULT f_mkfs (const TCHAR* path, BYTE opt, DWORD au, void* work, UINT len);	 
FRESULT f_fdisk (BYTE pdrv, const DWORD* szt, void* work);			 
int f_putc (TCHAR c, FIL* fp);										 
int f_puts (const TCHAR* str, FIL* cp);								 
int f_printf (FIL* fp, const TCHAR* str, ...);						 
TCHAR* f_gets (TCHAR* buff, int len, FIL* fp);						 

#line 288 "..\\FatFS\\src\\ff.h"








 
 

 

DWORD get_fattime (void);


 

WCHAR ff_convert (WCHAR chr, UINT dir);	 
WCHAR ff_wtoupper (WCHAR chr);			 






 
#line 321 "..\\FatFS\\src\\ff.h"




 
 


 
#line 337 "..\\FatFS\\src\\ff.h"

 


 






 





 











#line 42 "..\\User\\emWinTask\\MainTask.h"
#line 1 "..\\FatFS\\src\\diskio.h"


 








#line 13 "..\\FatFS\\src\\diskio.h"


 
typedef BYTE	DSTATUS;

 
typedef enum {
	RES_OK = 0,		 
	RES_ERROR,		 
	RES_WRPRT,		 
	RES_NOTRDY,		 
	RES_PARERR		 
} DRESULT;


 
 


DSTATUS disk_initialize (BYTE pdrv);
DSTATUS disk_status (BYTE pdrv);
DRESULT disk_read (BYTE pdrv, BYTE* buff, DWORD sector, UINT count);
DRESULT disk_write (BYTE pdrv, const BYTE* buff, DWORD sector, UINT count);
DRESULT disk_ioctl (BYTE pdrv, BYTE cmd, void* buff);


 






 

 






 





 
#line 70 "..\\FatFS\\src\\diskio.h"

 








#line 43 "..\\User\\emWinTask\\MainTask.h"





 
extern FRESULT result;
extern FIL file;
extern FILINFO finfo;
extern DIR DirInf;
extern UINT bw;
extern FATFS fs;

extern void _WriteByte2File(unsigned char Data, void * p); 




 
extern const  GUI_FONT GUI_FontHZ_Song_12;
extern const  GUI_FONT GUI_FontHZ_FangSong_16;
extern const  GUI_FONT GUI_FontHZ_Song_16;
extern const  GUI_FONT GUI_FontHZ_Hei_24;
extern const  GUI_FONT GUI_FontHZ_Kai_24;
extern const  GUI_FONT GUI_FontHZ_Song_24;
extern const  GUI_FONT GUI_FontHZ_SimSun_1616;
extern const  GUI_FONT GUI_FontHZ_SimSun_2424;



 
#line 17 "..\\User\\emWinTask\\MainTask.c"






 
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
  { FRAMEWIN_CreateIndirect, "Counting...",        0,      30,  80, 400, 190, (1<<4) },
  { TEXT_CreateIndirect,     "00",        0x160,    10,  10, 120,  80 },
  { RADIO_CreateIndirect,    "",          0x150,  150,  10, 180,  120, 0, 0x1904, 0},
  { BUTTON_CreateIndirect,   "OK",        1,       19, 121, 114, 32, 0, 0x0, 0 },
  { BUTTON_CreateIndirect,   "Cancel",    2,  252, 120, 116, 35, 0, 0x0, 0 },
};





 
static const char * _apLabel[] = {
  "GUI_FontFD32",
  "GUI_FontFD48",
  "GUI_FontFD64",
  "GUI_FontFD80",
};

static const GUI_FONT * _apFont[] = {
  &GUI_FontD32,
  &GUI_FontD48,
  &GUI_FontD64,
  &GUI_FontD80
};

static const char * _asExplain[] = {
  "Please use the RADIO buttons to select",
  "the big digit font used for counting."
};








 
static void _SetFont(GUI_HWIN hDlg) 
{
	GUI_HWIN hItem;
	int Index;
	hItem = WM_GetDialogItem(hDlg, 0x150);
	Index = RADIO_GetValue(hItem);
	hItem = WM_GetDialogItem(hDlg, 0x160);
	TEXT_SetFont(hItem, _apFont[Index]);
}








 
static void _cbBkWindow(WM_MESSAGE * pMsg) 
{
	int i;
	
	switch (pMsg->MsgId) 
	{
		 
		case 0x000F:
			GUI_SetBkColor(0x00FF0000);
			GUI_Clear();
			GUI_SetColor(0x00FFFFFF);
			GUI_SetFont(&GUI_Font24_ASCII);
			GUI_DispStringHCenterAt("Counting Sample", 160, 5);
			GUI_SetFont(&GUI_Font8x16);
			for (i = 0; i < (sizeof(_asExplain) / sizeof(_asExplain[0])); i++) 
			{
				GUI_DispStringAt(_asExplain[i], 5, 40 + i * 16);
			}
			
		default:
			WM_DefaultProc(pMsg);
	}
}








 
static void _cbCallback(WM_MESSAGE * pMsg) 
{
	int i;
	int NCode, Id;
	GUI_HWIN hDlg, hItem;
	hDlg = pMsg->hWin;
	switch (pMsg->MsgId) 
	{
		case 29:
			hItem = WM_GetDialogItem(hDlg, 0x150);
			for (i = 0; i < (sizeof(_apLabel) / sizeof(_apLabel[0])); i++) 
			{
				RADIO_SetText(hItem, _apLabel[i], i);
			}
			_SetFont(hDlg);
			break;
			
		case 38:
			Id    = WM_GetId(pMsg->hWinSrc);    
			NCode = pMsg->Data.v;             
			switch (NCode) 
			{
				case 5: 
					_SetFont(hDlg);
					break;
				
				 
				case 2:     
					if (Id == 1) 
					{            
						GUI_EndDialog(hDlg, 0);
					}
					if (Id == 2) 
					{       
						GUI_EndDialog(hDlg, 1);
					}
					break;
			}
			break;
			
		default:
		WM_DefaultProc(pMsg);
	}
}








 
void MainTask(void) 
{
	int Value = 0;
	GUI_HWIN hDlgFrame;
	
	 
	GUI_Init();
    
	
	WM_SetCallback(WM_GetDesktopWindow(), _cbBkWindow);  
	WM_SetCreateFlags((1UL << 2));   
	hDlgFrame = 0;
	
	while(1) 
	{
		GUI_HWIN hDlg, hText;
		char acText[3] = {0};
		
		GUI_Delay(100);
		 
		if (!WM_IsWindow(hDlgFrame)) 
		{
			Value = 0;
			hDlgFrame = GUI_CreateDialogBox(_aDialogCreate, (sizeof(_aDialogCreate) / sizeof(_aDialogCreate[0])), &_cbCallback, 0, 0, 0);
		}
		
		Value = (Value + 1) % 100;
		acText[0] = '0' + Value / 10;
		acText[1] = '0' + Value % 10;
		hDlg = WM_GetClientWindow(hDlgFrame);
		hText = WM_GetDialogItem(hDlg, 0x160);
		TEXT_SetText(hText, acText);
	}
}

 
